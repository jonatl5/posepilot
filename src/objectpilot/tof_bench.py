"""Guided, motors-off bench session for the TT forward ToF sensor.

The operator places the aircraft on a stand at each planned tape distance, surface and
angle; the script polls the range query, labels every reply with that ground truth and
writes raw samples plus per-point accuracy, validity and freshness summaries. It uses the
ground-only GroundLink allowlist: no takeoff, landing, motor or RC command can be sent.
"""
import argparse
import csv
import json
import math
import platform
import socket
import statistics
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

from .core import CustomRangeParser, parse_ext_tof, repository_revision

IDENTITY_QUERIES = ("sdk?", "sn?", "hardware?", "battery?", "wifi?", "EXT version?")
# Reporting thresholds for the "usable range" line; a summary, not a flight qualification.
USABLE_VALID_FRACTION = 0.9
USABLE_ABS_BIAS_M = 0.03
USABLE_P95_ERROR_M = 0.05
POINT_FIELDS = ("point_id", "surface", "distance_m", "angle_deg", "attempt", "queries", "replies",
                "valid_fraction", "out_of_range_fraction", "failed_fraction", "median_m", "bias_m",
                "mae_m", "p95_abs_error_m", "std_m", "changes_per_s", "longest_repeat_s",
                "rtt_median_ms", "rtt_p95_ms", "battery_pct", "temph_c")


def load_plan(path, surfaces=None, seconds=None):
    """Expand the plan blocks into ordered points; optionally keep only some surfaces."""
    plan = json.loads(Path(path).read_text(encoding="utf-8"))
    keep = set(surfaces) if surfaces else None
    points, seen = [], set()
    for block in plan["blocks"]:
        if keep is not None and block["surface"] not in keep:
            continue
        for angle in block.get("angles_deg", [0]):
            for distance in block["distances_m"]:
                if distance is not None and not (math.isfinite(distance) and distance > 0):
                    raise ValueError(f"Invalid distance {distance!r} in plan")
                label = "none" if distance is None else f"{distance:.2f}m"
                point_id = f"{block['surface']}-{label}-{int(angle)}deg"
                if point_id in seen:
                    raise ValueError(f"Duplicate plan point {point_id}")
                seen.add(point_id)
                points.append(dict(point_id=point_id, surface=block["surface"], distance_m=distance,
                                   angle_deg=angle, note=block.get("note", ""),
                                   seconds=float(seconds or plan.get("seconds_per_point", 10))))
    if not points:
        raise ValueError("The plan selects no points")
    return plan, points


class StateListener:
    """Collect the aircraft's UDP state string (battery, board temperature, downward ToF)."""

    def __init__(self, port=8890):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("", port))
        self.socket.settimeout(.5)
        self.latest = {}
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while not self.stop.is_set():
            try:
                raw, _ = self.socket.recvfrom(2048)
            except (socket.timeout, OSError):
                continue
            fields = {}
            for item in raw.decode("ascii", errors="replace").strip().split(";"):
                key, _, value = item.partition(":")
                if key and value:
                    fields[key.strip()] = value.strip()
            if fields:
                fields["received_monotonic"] = time.monotonic()
                self.latest = fields

    def snapshot(self):
        state = self.latest
        return {k: state.get(k) for k in ("bat", "temph", "templ", "tof", "h")}

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1.)
        self.socket.close()


def board_temperature(state):
    """Highest main-board temperature from the state stream in degrees C, or None."""
    try:
        return float(state.snapshot().get("temph")) if state else None
    except (TypeError, ValueError):
        return None


def collect_point(link, point, interval, clock, sleep, custom=False, state=None, config=None, timeout=1.5,
                  stop_temp=None):
    """Poll the range query for the point's duration; every attempt becomes one sample row.

    Stops early, marking the last sample `stopped_hot`, once the board reaches `stop_temp`;
    on the bench the TT powered itself off at about 99 degrees C.
    """
    parser = CustomRangeParser(config) if custom else None
    command = "EXT oprange?" if custom else "EXT tof?"
    samples = []
    start = clock()
    while clock()-start < point["seconds"]:
        request_start = clock()
        reply, status, distance, rtt = None, "query_failed", None, None
        try:
            reply, sent, received = link.request(command, timeout)
            rtt = received-sent
            sample = parser.parse(reply, sent, received) if custom else parse_ext_tof(reply, sent)
            status, distance = sample.status, sample.distance_m
        except TimeoutError:
            status = "timeout"
        except (RuntimeError, OSError) as error:
            status, reply = "error", str(error)
        mm = None
        if reply and reply.split()[:1] in (["tof"], ["oprange"]):
            try:
                mm = int(reply.split()[-3 if custom else -1])
            except (ValueError, IndexError):
                mm = None
        samples.append(dict(t_s=round(request_start-start, 4), reply=reply, status=status,
                            measured_m=distance, raw_mm=mm,
                            rtt_ms=None if rtt is None else round(1000*rtt, 2),
                            state=state.snapshot() if state else None,
                            late_lines=list(getattr(link, "last_flushed", [])),
                            other_lines=list(getattr(link, "last_unmatched", []))))
        hot = board_temperature(state)
        if stop_temp is not None and hot is not None and hot >= stop_temp:
            samples[-1]["stopped_hot"] = True
            break
        wait = interval-(clock()-request_start)
        if wait > 0:
            sleep(wait)
    return samples


def _percentile(values, q):
    ordered = sorted(values)
    return ordered[min(len(ordered)-1, max(0, math.ceil(q*len(ordered))-1))]


def summarize_point(samples, truth_m):
    queries = len(samples)
    replies = [s for s in samples if s["status"] not in ("timeout", "error", "query_failed")]
    valid = [s for s in samples if s["measured_m"] is not None]
    out_of_range = [s for s in samples if s["status"] == "out_of_range"]
    result = dict(queries=queries, replies=len(replies),
                  valid_fraction=round(len(valid)/queries, 4) if queries else 0.,
                  out_of_range_fraction=round(len(out_of_range)/queries, 4) if queries else 0.,
                  failed_fraction=round(1-len(replies)/queries, 4) if queries else 0.)
    for key in ("median_m", "bias_m", "mae_m", "p95_abs_error_m", "std_m", "changes_per_s",
                "longest_repeat_s", "rtt_median_ms", "rtt_p95_ms"):
        result[key] = None
    if valid:
        values = [s["measured_m"] for s in valid]
        result["median_m"] = round(statistics.median(values), 4)
        result["std_m"] = round(statistics.pstdev(values), 4)
        if truth_m is not None:
            errors = [v-truth_m for v in values]
            result["bias_m"] = round(statistics.median(errors), 4)
            result["mae_m"] = round(statistics.fmean(abs(e) for e in errors), 4)
            result["p95_abs_error_m"] = round(_percentile([abs(e) for e in errors], .95), 4)
        # Freshness proxy for stock replies without a sequence number: how often the value changes.
        duration = max(samples[-1]["t_s"]-samples[0]["t_s"], 1e-9)
        changes = sum(a["raw_mm"] != b["raw_mm"] for a, b in zip(valid, valid[1:]))
        result["changes_per_s"] = round(changes/duration, 3)
        longest, run_start = 0., valid[0]["t_s"]
        for a, b in zip(valid, valid[1:]):
            if b["raw_mm"] != a["raw_mm"]:
                run_start = b["t_s"]
            longest = max(longest, b["t_s"]-run_start)
        result["longest_repeat_s"] = round(longest, 3)
    rtts = [s["rtt_ms"] for s in samples if s["rtt_ms"] is not None]
    if rtts:
        result["rtt_median_ms"] = round(statistics.median(rtts), 2)
        result["rtt_p95_ms"] = round(_percentile(rtts, .95), 2)
    return result


def usable_range(rows):
    """Per surface, the largest head-on distance from 0.40 m upward where every tested point passes."""
    usable = {}
    for surface in sorted({r["surface"] for r in rows if r["distance_m"] is not None}):
        tested = sorted((r for r in rows if r["surface"] == surface and r["angle_deg"] == 0
                         and r["distance_m"] is not None and r["distance_m"] >= .4),
                        key=lambda r: r["distance_m"])
        best = None
        for r in tested:
            ok = (r["valid_fraction"] >= USABLE_VALID_FRACTION and r["bias_m"] is not None
                  and abs(r["bias_m"]) <= USABLE_ABS_BIAS_M and r["p95_abs_error_m"] <= USABLE_P95_ERROR_M)
            if not ok:
                break
            best = r["distance_m"]
        usable[surface] = best
    return usable


def _describe(point):
    where = ("no tape distance" if point["distance_m"] is None
             else f"sensor window {point['distance_m']:.2f} m from the surface")
    return f"{point['surface']}, {where}, angle {point['angle_deg']} deg. {point['note']}"


def _line(summary):
    def fmt(value, scale=1., unit="", sign=False):
        return "n/a" if value is None else f"{value*scale:{'+' if sign else ''}.3f}{unit}"
    return (f"valid {summary['valid_fraction']:.0%}  out-of-range {summary['out_of_range_fraction']:.0%}  "
            f"failed {summary['failed_fraction']:.0%}  median {fmt(summary['median_m'], unit=' m')}  "
            f"bias {fmt(summary['bias_m'], unit=' m', sign=True)}  p95 err {fmt(summary['p95_abs_error_m'], unit=' m')}  "
            f"changes {summary['changes_per_s'] if summary['changes_per_s'] is not None else 'n/a'}/s  "
            f"rtt {summary['rtt_median_ms'] if summary['rtt_median_ms'] is not None else 'n/a'} ms")


def _query_identity(link):
    identity = {}
    for query in IDENTITY_QUERIES:
        try:
            identity[query] = link.request(query, 2.)[0]
        except (TimeoutError, RuntimeError, OSError) as error:
            identity[query] = f"unavailable: {error}"
    return identity


def run_session(link, plan, points, out, *, interval, custom=False, state=None, timeout=1.5,
                max_temp=90., stop_temp=97.,
                input_fn=input, print_fn=print, clock=time.monotonic, sleep=time.sleep, argv=None):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=False)
    session = dict(started_utc=datetime.now(timezone.utc).isoformat(timespec="seconds"),
                   plan_name=plan.get("name"), reference=plan.get("reference"),
                   range_query="EXT oprange?" if custom else "EXT tof?", interval_s=interval,
                   argv=argv, git_revision=repository_revision(), python=sys.version.split()[0],
                   platform=platform.platform(), identity=_query_identity(link), points_planned=len(points))
    print_fn("Aircraft identity: " + json.dumps(session["identity"]))
    (out/"session.json").write_text(json.dumps(session, indent=2), encoding="utf-8")
    accepted, overheated = [], None
    with (out/"samples.jsonl").open("x", encoding="utf-8") as raw:
        for index, point in enumerate(points, 1):
            print_fn(f"\n[{index}/{len(points)}] {point['point_id']}\n  {_describe(point)}")
            hot = board_temperature(state)
            if hot is not None:
                print_fn(f"  board {hot:.0f} C, battery {state.snapshot().get('bat')}%")
            if hot is not None and hot >= max_temp:
                print_fn(f"  Board is at {hot:.0f} C (limit {max_temp:.0f} C); the TT shuts down near 99 C.")
                if input_fn("  c = measure anyway, any other key = stop and cool down > ").strip().lower() != "c":
                    overheated = point["point_id"]
                    break
            answer = input_fn("  Enter = measure, s = skip, q = quit > ").strip().lower()
            if answer == "q":
                break
            if answer == "s":
                continue
            attempt = 0
            while True:
                attempt += 1
                print_fn(f"  measuring for {point['seconds']:.0f} s ... keep everything still")
                samples = collect_point(link, point, interval, clock, sleep, custom, state, timeout=timeout,
                                        stop_temp=stop_temp)
                for s in samples:
                    raw.write(json.dumps(dict(point_id=point["point_id"], surface=point["surface"],
                                              distance_m=point["distance_m"], angle_deg=point["angle_deg"],
                                              attempt=attempt, **s)) + "\n")
                raw.flush()
                if samples[-1].get("stopped_hot"):
                    overheated = point["point_id"]
                    print_fn(f"  Stopped: board reached {stop_temp:.0f} C. This point was not accepted.")
                    break
                summary = summarize_point(samples, point["distance_m"])
                print_fn("  " + _line(summary))
                if input_fn("  Enter = accept, r = redo > ").strip().lower() != "r":
                    break
            if overheated:
                break
            last_state = samples[-1]["state"] or {}
            accepted.append(dict(point_id=point["point_id"], surface=point["surface"],
                                 distance_m=point["distance_m"], angle_deg=point["angle_deg"],
                                 attempt=attempt, battery_pct=last_state.get("bat"),
                                 temph_c=last_state.get("temph"), **summary))
            _write_points(out/"points.csv", accepted)
    usable = usable_range(accepted)
    session.update(finished_utc=datetime.now(timezone.utc).isoformat(timespec="seconds"),
                   points_accepted=len(accepted), usable_range_m=usable, stopped_for_heat_at=overheated,
                   usable_rule=dict(valid_fraction_min=USABLE_VALID_FRACTION, abs_bias_max_m=USABLE_ABS_BIAS_M,
                                    p95_abs_error_max_m=USABLE_P95_ERROR_M),
                   battery_end=_query_identity_one(link, "battery?"))
    (out/"session.json").write_text(json.dumps(session, indent=2), encoding="utf-8")
    _write_points(out/"points.csv", accepted)
    print_fn(f"\n{len(accepted)} points accepted. Usable head-on range by surface "
             f"(valid >= {USABLE_VALID_FRACTION:.0%}, |bias| <= {USABLE_ABS_BIAS_M} m, "
             f"p95 <= {USABLE_P95_ERROR_M} m): {json.dumps(usable)}")
    print_fn(f"Files: {out/'session.json'}, {out/'points.csv'}, {out/'samples.jsonl'}")
    if overheated:
        print_fn(f"Power the aircraft off and let it cool below about 70 C, then continue with "
                 f"--from {overheated} and a new --out directory.")
    return session, accepted


def _query_identity_one(link, query):
    try:
        return link.request(query, 2.)[0]
    except (TimeoutError, RuntimeError, OSError) as error:
        return f"unavailable: {error}"


def _write_points(path, rows):
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=POINT_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k) for k in POINT_FIELDS})


def _probe(link, out, command, rounds=5, seconds=3.):
    """Print and save every line received after each query, with milliseconds since sending."""
    out.mkdir(parents=True, exist_ok=False)
    record = []
    for query in ["battery?"] + [command]*rounds + ["EXT version?"]:
        lines = link.listen(query, seconds)
        record.append(dict(query=query, lines=lines))
        print(f"{query!r}: " + ("no reply" if not lines else
                                "; ".join(f"+{ms} ms {text!r}" for ms, _, text in lines)))
    (out/"probe.json").write_text(json.dumps(record, indent=2), encoding="utf-8")
    print(f"Saved {out/'probe.json'}")
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--plan", default="configs/tof_bench_plan.json")
    p.add_argument("--out", required=True, help="New directory for this session")
    p.add_argument("--surfaces", help="Comma-separated subset of plan surfaces, e.g. whiteboard,cup_print")
    p.add_argument("--seconds", type=float, help="Override seconds per point")
    p.add_argument("--interval", type=float, help="Seconds between queries (default from plan)")
    p.add_argument("--timeout", type=float, default=1.5, help="Seconds to wait for each range reply")
    p.add_argument("--from", dest="start_from", metavar="POINT_ID", help="Start at this point of the selected plan")
    p.add_argument("--max-temp", type=float, default=90., help="Ask before measuring above this board temperature (C)")
    p.add_argument("--stop-temp", type=float, default=97., help="Abort a point at this board temperature (C)")
    p.add_argument("--probe", action="store_true",
                   help="Diagnose: send a few range queries and save every line the aircraft returns")
    p.add_argument("--range-mode", choices=("stock", "custom"), default="stock",
                   help="custom only after the EXT oprange? firmware has been qualified")
    p.add_argument("--host", default="192.168.10.1")
    p.add_argument("--no-state", action="store_true", help="Do not listen for the UDP 8890 state stream")
    p.add_argument("--dry-list", action="store_true", help="Print the selected points and exit")
    a = p.parse_args(argv)
    if not (math.isfinite(a.timeout) and 0 < a.timeout <= 5):
        p.error("--timeout must be in (0, 5] seconds")
    if a.seconds is not None and not (math.isfinite(a.seconds) and a.seconds > 0):
        p.error("--seconds must be positive")
    plan, points = load_plan(a.plan, a.surfaces.split(",") if a.surfaces else None, a.seconds)
    if a.start_from:
        ids = [x["point_id"] for x in points]
        if a.start_from not in ids:
            p.error(f"--from {a.start_from} is not in the selected plan; use --dry-list to see point IDs")
        points = points[ids.index(a.start_from):]
    interval = a.interval if a.interval is not None else float(plan.get("interval_s", .1))
    if not (math.isfinite(interval) and 0 < interval <= 1):
        p.error("--interval must be in (0, 1] seconds")
    if a.dry_list:
        for point in points:
            print(f"{point['point_id']:<32} {point['seconds']:.0f} s  {point['note']}")
        print(f"{len(points)} points, about {sum(x['seconds'] for x in points)/60:.1f} min of measurement")
        return 0
    if Path(a.out).exists():
        p.error("Output exists; choose a new session directory")
    from .ground_link import GroundLink
    link = GroundLink(a.host)
    state = None
    try:
        link.connect(range_mode="none", video=False)
        if a.probe:
            return _probe(link, Path(a.out), "EXT oprange?" if a.range_mode == "custom" else "EXT tof?")
        if not a.no_state:
            try:
                state = StateListener()
            except OSError as error:
                print(f"State stream unavailable ({error}); continuing without battery/temperature logging")
        run_session(link, plan, points, a.out, interval=interval, custom=a.range_mode == "custom", timeout=a.timeout,
                    max_temp=a.max_temp, stop_temp=a.stop_temp,
                    state=state, argv=sys.argv[1:] if argv is None else list(argv))
    finally:
        if state:
            state.close()
        link.close()
    print("No flight, motor or RC command was sent.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
