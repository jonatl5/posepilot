import csv
import json

import pytest

from objectpilot.ground_link import GroundLink
from objectpilot.tof_bench import collect_point, load_plan, main, run_session, summarize_point, usable_range


class FakeClock:
    def __init__(self):
        self.now = 0.

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class FakeLink:
    """Scripted stock replies first, then the current `truth_mm` (8192 when unset)."""

    def __init__(self, clock, readings_mm, rtt=.02):
        self.clock, self.readings, self.rtt = clock, list(readings_mm), rtt
        self.truth_mm = 8192
        self.sent = []

    def request(self, command, timeout=.5):
        self.sent.append(command)
        start = self.clock()
        self.clock.now += self.rtt
        if command == "EXT tof?":
            value = self.readings.pop(0) if self.readings else self.truth_mm
            if value is None:
                raise TimeoutError("No matching reply")
            return f"tof {value}", start, self.clock()
        return {"sdk?": "30", "hardware?": "RMTT", "battery?": "87", "EXT version?": "esp32v1.0.0.0"}.get(
            command, "ok"), start, self.clock()


def test_plan_expands_blocks_filters_surfaces_and_rejects_duplicates(tmp_path):
    plan, points = load_plan("configs/tof_bench_plan.json")
    ids = [p["point_id"] for p in points]
    assert len(ids) == len(set(ids)) == 31
    assert "whiteboard-0.65m-20deg" in ids and "open_space-none-0deg" in ids
    _, subset = load_plan("configs/tof_bench_plan.json", ["cup_print"], seconds=3)
    assert {p["surface"] for p in subset} == {"cup_print"} and all(p["seconds"] == 3 for p in subset)
    bad = tmp_path/"plan.json"
    bad.write_text(json.dumps({"blocks": [{"surface": "a", "distances_m": [1, 1], "angles_deg": [0]}]}))
    with pytest.raises(ValueError):
        load_plan(bad)


def test_summary_reports_bias_validity_freshness_and_rtt():
    samples = [dict(t_s=i*.1, reply=f"tof {mm}", status="sample_age_unknown", measured_m=mm/1000,
                    raw_mm=mm, rtt_ms=20., state=None) for i, mm in enumerate([660, 660, 662, 662, 664])]
    samples.append(dict(t_s=.5, reply="tof 8192", status="out_of_range", measured_m=None, raw_mm=8192,
                        rtt_ms=20., state=None))
    samples.append(dict(t_s=.6, reply=None, status="timeout", measured_m=None, raw_mm=None, rtt_ms=None, state=None))
    s = summarize_point(samples, .65)
    assert s["queries"] == 7 and s["replies"] == 6
    assert s["valid_fraction"] == pytest.approx(5/7, abs=1e-3)
    assert s["out_of_range_fraction"] == pytest.approx(1/7, abs=1e-3)
    assert s["bias_m"] == pytest.approx(.012) and s["median_m"] == pytest.approx(.662)
    assert s["longest_repeat_s"] == pytest.approx(.1)
    assert s["changes_per_s"] == pytest.approx(2/.6, abs=1e-2)
    assert s["rtt_median_ms"] == 20.


def test_collect_point_paces_queries_and_records_failures():
    clock = FakeClock()
    link = FakeLink(clock, [650, None, 651])
    samples = collect_point(link, dict(seconds=.5), .1, clock, clock.sleep)
    assert [s["status"] for s in samples[:3]] == ["sample_age_unknown", "timeout", "sample_age_unknown"]
    assert samples[3]["status"] == "out_of_range"
    assert 4 <= len(samples) <= 6 and set(link.sent) == {"EXT tof?"}


def test_usable_range_stops_at_first_failing_distance():
    row = lambda d, valid, bias: dict(surface="w", angle_deg=0, distance_m=d, valid_fraction=valid,
                                      bias_m=bias, p95_abs_error_m=abs(bias)+.01)
    rows = [row(.25, 0., 0.), row(.4, 1., .01), row(.8, 1., .02), row(1.2, .5, .02), row(1.5, 1., .01)]
    assert usable_range(rows) == {"w": .8}


def test_session_flow_writes_files_and_sends_only_ground_queries(tmp_path):
    clock = FakeClock()
    link = FakeLink(clock, [])
    plan = dict(name="t", reference="r")
    points = [dict(point_id="w-0.40m-0deg", surface="w", distance_m=.4, angle_deg=0, note="", seconds=1.),
              dict(point_id="w-0.65m-0deg", surface="w", distance_m=.65, angle_deg=0, note="", seconds=1.),
              dict(point_id="w-0.80m-0deg", surface="w", distance_m=.8, angle_deg=0, note="", seconds=1.)]
    # The operator "places" the stand when asked to measure; 0.80 m is quit before measuring.
    answers = iter([("", 400), ("", None), ("", 655), ("r", None), ("", None), ("q", None)])

    def operator(_prompt):
        answer, truth = next(answers)
        if truth is not None:
            link.truth_mm = truth
        return answer

    session, rows = run_session(link, plan, points, tmp_path/"s", interval=.1, input_fn=operator,
                                print_fn=lambda *_: None, clock=clock, sleep=clock.sleep)
    assert [r["point_id"] for r in rows] == ["w-0.40m-0deg", "w-0.65m-0deg"] and rows[1]["attempt"] == 2
    assert session["identity"]["hardware?"] == "RMTT" and session["usable_range_m"] == {"w": .65}
    assert set(link.sent) <= GroundLink.ALLOWED
    lines = (tmp_path/"s/samples.jsonl").read_text().splitlines()
    assert {json.loads(x)["attempt"] for x in lines} == {1, 2}
    with (tmp_path/"s/points.csv").open() as f:
        assert len(list(csv.DictReader(f))) == 2
    with pytest.raises(FileExistsError):
        run_session(link, plan, points, tmp_path/"s", interval=.1, input_fn=lambda _: "q",
                    print_fn=lambda *_: None, clock=clock, sleep=clock.sleep)


class HeatingState:
    """Board temperature rises one degree per snapshot, like the idle TT on the bench."""

    def __init__(self, start):
        self.temph = start

    def snapshot(self):
        self.temph += 1
        return {"bat": "60", "temph": str(self.temph), "templ": "80", "tof": "10", "h": "0"}


def test_session_stops_a_point_before_thermal_shutdown_and_names_the_resume_point(tmp_path):
    clock = FakeClock()
    link = FakeLink(clock, [])
    link.truth_mm = 400
    points = [dict(point_id="w-0.40m-0deg", surface="w", distance_m=.4, angle_deg=0, note="", seconds=10.),
              dict(point_id="w-0.50m-0deg", surface="w", distance_m=.5, angle_deg=0, note="", seconds=10.)]
    printed = []
    session, rows = run_session(link, dict(name="t"), points, tmp_path/"hot", interval=.1, state=HeatingState(80),
                                max_temp=90., stop_temp=97., input_fn=lambda _: "", print_fn=printed.append,
                                clock=clock, sleep=clock.sleep)
    assert rows == [] and session["stopped_for_heat_at"] == "w-0.40m-0deg"
    assert any("--from w-0.40m-0deg" in line for line in printed)
    samples = [json.loads(x) for x in (tmp_path/"hot/samples.jsonl").read_text().splitlines()]
    assert samples[-1]["stopped_hot"] and len(samples) < 20

    _, rows = run_session(link, dict(name="t"), points, tmp_path/"warm", interval=.1, state=HeatingState(95),
                          input_fn=lambda _: "", print_fn=lambda *_: None, clock=clock, sleep=clock.sleep)
    assert rows == []


def test_from_selects_the_resume_point(capsys):
    assert main(["--out", "unused", "--dry-list", "--surfaces", "whiteboard", "--from", "whiteboard-1.20m-0deg"]) == 0
    listed = capsys.readouterr().out
    assert "whiteboard-1.00m-0deg " not in listed and listed.splitlines()[0].startswith("whiteboard-1.20m-0deg")
    with pytest.raises(SystemExit):
        main(["--out", "unused", "--dry-list", "--from", "nope"])


def test_ground_link_skips_forwarded_keepalive_rejections_over_udp():
    import socket
    import threading

    drone = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    drone.bind(("127.0.0.1", 0))
    drone.settimeout(2.)

    def serve():
        for _ in range(4):
            data, peer = drone.recvfrom(2048)
            command = data.decode()
            drone.sendto(b"unknown command: keepalive", peer)
            if command == "EXT tof?":
                drone.sendto(b"tof 650\r\n", peer)
            elif command == "sdk?":
                drone.sendto(b"30", peer)
            else:
                drone.sendto(f"unknown command: {command}".encode(), peer)

    thread = threading.Thread(target=serve, daemon=True)
    thread.start()
    link = GroundLink("127.0.0.1", local_port=0, remote_port=drone.getsockname()[1])
    try:
        assert link.request("EXT tof?", 1.)[0] == "tof 650"
        assert link.request("sdk?", 1.)[0] == "30"
        with pytest.raises(RuntimeError, match="EXT version"):
            link.request("EXT version?", 1.)
        assert link.ignored_lines == 3
        assert link.last_unmatched == []
        heard = [text for _, _, text in link.listen("EXT tof?", .3)]
        assert heard == ["unknown command: keepalive", "tof 650"]
        with pytest.raises(ValueError):
            link.listen("takeoff", .1)
    finally:
        link.socket.close()
        drone.close()
        thread.join(timeout=2.)


def test_ground_link_allows_read_only_identity_but_no_flight_commands():
    assert {"sdk?", "sn?", "hardware?", "battery?", "wifi?"} <= GroundLink.ALLOWED
    for word in ("takeoff", "land", "rc", "motoron", "emergency", "go", "throwfly"):
        assert not any(c.split()[0] == word for c in GroundLink.ALLOWED)


def test_dry_list_needs_no_aircraft(capsys):
    assert main(["--out", "unused", "--dry-list", "--surfaces", "blocked"]) == 0
    assert "blocked-none-0deg" in capsys.readouterr().out
