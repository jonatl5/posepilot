import json
import subprocess
import sys

from objectpilot.demo import run_demo
import pytest


def test_offline_cli_produces_a_replayable_pid_episode(tmp_path):
    out = tmp_path / "demo"
    subprocess.run([sys.executable, "-m", "objectpilot.demo", "--out", str(out)], check=True, capture_output=True)
    summary = json.loads((out / "summary.json").read_text())
    rows = [json.loads(line) for line in (out / "trajectory.jsonl").read_text().splitlines()]
    html = (out / "demo.html").read_text(encoding="utf-8")
    assert summary["success"] and summary["seed"] == 10000
    assert rows[-1]["outcome"] == summary["outcome"]
    assert rows[-1]["t"] == summary["time_s"]
    assert all(len(row["observation"]) == 64 for row in rows)
    assert all(row["rc"][0] == row["rc"][2] == 0 for row in rows)
    payload = html.split('<script id="recording" type="application/json">')[1].split('</script>')[0]
    assert json.loads(payload)["samples"] == rows
    assert "__OBJECTPILOT_TRACE__" not in html


def test_demo_preserves_an_existing_output(tmp_path):
    marker = tmp_path / "summary.json"
    marker.write_text("existing experiment")
    with pytest.raises(FileExistsError, match="choose a new"):
        run_demo(tmp_path)
    assert marker.read_text() == "existing experiment"
