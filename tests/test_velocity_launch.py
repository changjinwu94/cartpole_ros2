"""Exercise the host launchers with a fake Docker CLI, including fail-fast collection."""
import csv
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
DOCKER = '''#!/usr/bin/env python3
import json, os, pathlib, sys, time
cmd = sys.argv[1:]
root = pathlib.Path(os.environ["CARTPOLE_REPO"])
with (root / "calls").open("a") as out:
    print(json.dumps(cmd), file=out)
if "mktemp" in cmd:
    print("/tmp/cartpole-velocity.Xa1234")
if "--check-only" in cmd and os.environ.get("FAIL_CHECK"):
    sys.exit(1)
if "--execute" in cmd or "run" in cmd:
    vel = "--execute" in cmd
    cfg = {k: float(cmd[cmd.index("--"+k)+1]) for k in ("speed","acceleration")} if vel else {}
    path = root / "logs" / ("trial_" + str(time.time_ns()) + ".json")
    report = dict(summary=dict(completed=True, motion_attempted=True,
        stop_confirmed=not os.environ.get("FAIL_STOP"), outcome="bounded_time_limit",
        control_mode="velocity_final_push_swing_only" if vel else "final_push_swing_only", settings=cfg),
        events=[dict(kind="velocity_command", measured_m_s=-.031)])
    path.write_text(json.dumps(report))
    print("RECORDING: /logs/" + path.name)
'''


class LaunchTests(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)
        for sub in ("cartpole_batch", "experiments", "tools"):
            shutil.copytree(ROOT / sub, self.root / sub)
        (self.root / "logs").mkdir()
        for name, script in (("docker", DOCKER), ("hostname", "#!/bin/sh\necho robot01\n")):
            exe = self.root / name
            exe.write_text(script)
            exe.chmod(0o755)
        self.env = os.environ | dict(PATH=f"{self.root}:{os.environ['PATH']}",
                                   XDG_RUNTIME_DIR=tmp.name, CARTPOLE_REPO=tmp.name, CARTPOLE_BATCH="1")

    def launch(self, script, *args, **flags):
        return subprocess.run(["bash", str(self.root / "cartpole_batch" / script), *args],
                              env=self.env | flags, stdin=subprocess.DEVNULL,
                              capture_output=True, text=True, timeout=30)

    def calls(self):
        return [json.loads(line) for line in (self.root / "calls").read_text().splitlines()]

    def test_velocity_check_only_skips_preparation_and_enable(self):
        p = self.launch("cartpole_final_push_trial.sh", "--velocity", "--check-only")
        self.assertEqual(p.returncode, 0, p.stderr)
        calls = self.calls()
        self.assertTrue(any("--check-only" in c for c in calls))
        self.assertFalse(any("up" in c or "stop" in c or "--execute" in c or "run" in c for c in calls))
        self.assertFalse(any("prepare_cart.py" in " ".join(c) for c in calls))

    def test_invalid_parameters_and_failed_preflight_block_execution(self):
        script = "cartpole_final_push_trial.sh"
        p = self.launch(script, "--velocity", "0.3044", ".83", "8", ".276")
        self.assertNotEqual(p.returncode, 0)
        self.assertFalse((self.root / "calls").exists())
        p = self.launch(script, "--velocity", FAIL_CHECK="1")
        self.assertNotEqual(p.returncode, 0)
        self.assertFalse(any("--execute" in c for c in self.calls()))

    def test_collect_both_modes_and_preserve_stop_failure(self):
        script = "cartpole_collect_pairs.sh"
        for args, prefix in ((["1"], "final_push_pairs"), (["--velocity","1",".07",".5"], "velocity_final_push_pairs")):
            p = self.launch(script, *args)
            self.assertEqual(p.returncode, 0, p.stdout + p.stderr)
            result = next((self.root / "logs").glob(prefix + ".*/results.csv"))
            with result.open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual([r["fraction"] for r in rows], ["0.3044", "0.3043"])
            self.assertTrue(all((result.parent / r["recording"]).is_file() for r in rows))
            if args[0] == "--velocity":
                self.assertEqual(float(rows[0]["speed_cap_m_s"]), .07)
                self.assertEqual(float(rows[0]["accel_cap_m_s2"]), .5)
                self.assertEqual(float(rows[0]["peak_measured_speed_m_s"]), .031)
        previous = len(self.calls())
        p = self.launch(script, "--velocity", "2", FAIL_STOP="1")
        self.assertNotEqual(p.returncode, 0)
        calls = self.calls()[previous:]
        self.assertEqual(sum("--execute" in c for c in calls), 1)
        self.assertFalse(any("stop" in c or "run" in c for c in calls))
