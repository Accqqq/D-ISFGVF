#!/usr/bin/env python3
"""ROS-free checks: never import or execute the ROS scenario runners."""
import ast
import io
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock

RUNNERS = ("batch10_scenarios.py", "pillar_left_to_right_stability_test.py")


def helpers(name, log_dir):
    path = Path(__file__).with_name(name)
    source = path.read_text()
    compile(source, str(path), "exec")  # syntax-check the whole runner, without running it
    tree = ast.parse(source, filename=str(path))
    allowed = {"Proc", "stop_owned_processes", "run_scenario", "run_ablation", "run_once"}
    selected = [n for n in tree.body if isinstance(n, (ast.ClassDef, ast.FunctionDef))
                and n.name in allowed]
    namespace = dict(os=os, signal=signal, subprocess=subprocess, sys=sys,
                     time=time, LOG_DIR=log_dir, _owned_processes=[])
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(path), "exec"), namespace)
    return source, namespace


class ScopedCleanupTest(unittest.TestCase):
    def test_only_tracked_process_is_stopped(self):
        # This test-owned outsider deliberately has roslaunch in its argv.
        outsider = subprocess.Popen(
            ["roslaunch", "-c", "import time; time.sleep(30)"],
            executable=sys.executable, start_new_session=True)
        try:
            with tempfile.TemporaryDirectory() as directory:
                for name in RUNNERS:
                    with self.subTest(runner=name):
                        _, ns = helpers(name, directory)
                        proc = ns["Proc"]([sys.executable, "-c", "import time; time.sleep(30)"], "owned")
                        try:
                            with mock.patch.object(subprocess, "check_output", side_effect=AssertionError("no global scan")):
                                ns["stop_owned_processes"]()
                                proc.stop()  # idempotent
                            self.assertIsNotNone(proc.proc.poll())
                            self.assertTrue(proc.log.closed)
                            self.assertEqual(ns["_owned_processes"], [])
                            self.assertIsNone(outsider.poll())
                        finally:
                            proc.stop()
        finally:
            outsider.terminate()
            outsider.wait(timeout=5)

    def test_spawn_failure_closes_log(self):
        with tempfile.TemporaryDirectory() as directory:
            for name in RUNNERS:
                _, ns = helpers(name, directory)
                handle = io.StringIO()
                with mock.patch("builtins.open", return_value=handle), \
                     mock.patch.object(subprocess, "Popen", side_effect=OSError("test failure")):
                    with self.assertRaises(OSError):
                        ns["Proc"](["never-executed"], "failed")
                self.assertTrue(handle.closed)
                self.assertEqual(ns["_owned_processes"], [])

    def test_busy_port_refuses_without_starting_or_killing(self):
        with tempfile.TemporaryDirectory() as directory:
            for name in RUNNERS:
                source, ns = helpers(name, directory)
                self.assertNotIn("os.kill(", source)
                self.assertNotIn("port_owner_pid", source)
                self.assertNotIn("kill_leftover_ros", source)
                ns["port_open"] = lambda: True
                ns["Proc"] = mock.Mock(side_effect=AssertionError("occupied port must not spawn"))
                if "run_scenario" in ns:
                    self.assertFalse(ns["run_scenario"]("busy", [], 0))
                    self.assertIsNone(ns["run_ablation"](0))
                else:
                    self.assertEqual(ns["run_once"](0, 0, "busy")["error"], "master busy")
                ns["Proc"].assert_not_called()


if __name__ == "__main__":
    unittest.main()
