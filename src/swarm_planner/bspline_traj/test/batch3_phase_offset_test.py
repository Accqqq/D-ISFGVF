#!/usr/bin/env python3
"""Batch 3 ROS integration test: UAV converges to the ACTIVE reference.

With enable_phase_offset=true and a fixed phase_offset/test_delta, the UAV
must fly on r = p + N*delta instead of being pulled back to the base path p:
  delta = +0.5 -> steady-state lateral offset ~= +0.5 m
  delta = -0.5 -> steady-state lateral offset ~= -0.5 m
  delta =  0.0 -> stays on the centerline (legacy behavior)
  enable_phase_offset=false -> identical to legacy

Run with a sourced ROS environment:
  python3 src/swarm_planner/bspline_traj/test/batch3_phase_offset_test.py
"""

import json
import os
import re
import signal
import socket
import subprocess
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray

MASTER_URI = "http://localhost:11311"
LOG_DIR = "/tmp/gvf_ws_ros_log"
WS = "/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws"
os.environ["ROS_MASTER_URI"] = MASTER_URI
os.environ["ROS_LOG_DIR"] = LOG_DIR


class Proc:
    def __init__(self, argv, logname):
        self.argv = argv
        os.makedirs(LOG_DIR, exist_ok=True)
        self.log = open(os.path.join(LOG_DIR, logname + ".log"), "a")
        self.proc = subprocess.Popen(
            argv, env=dict(os.environ), stdout=self.log,
            stderr=subprocess.STDOUT, start_new_session=True)

    def alive(self):
        return self.proc.poll() is None

    def stop(self):
        if self.proc.poll() is None:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
            try:
                self.proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
                self.proc.wait(timeout=5)
        self.log.close()


def port_open():
    s = socket.socket()
    s.settimeout(0.3)
    try:
        s.connect(("127.0.0.1", 11311))
        s.close()
        return True
    except Exception:
        return False


def port_owner_pid():
    try:
        out = subprocess.check_output(["ss", "-tlnp"], text=True,
                                      stderr=subprocess.DEVNULL)
        for line in out.splitlines():
            if ":11311" in line:
                m = re.search(r"pid=(\d+)", line)
                if m:
                    return int(m.group(1))
    except Exception:
        pass
    return None


def wait_master(timeout=25.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            rospy.get_master().getPid()
            return True
        except Exception:
            time.sleep(0.2)
    return False


def wait_topic(topic, timeout=25.0, msg_type=Odometry):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            rospy.wait_for_message(topic, msg_type, timeout=1.0)
            return True
        except Exception:
            time.sleep(0.2)
    return False


def run_case(delta, enabled, goal=(8.0, 0.0)):
    """Launch the single-agent sim, send a goal, measure lateral offset."""
    roscore = Proc(["roscore"], "b3_roscore")
    if not wait_master():
        roscore.stop()
        return None
    rospy.init_node("batch3_test", anonymous=True)
    time.sleep(1.0)
    argv = ["roslaunch", "bspline_race", "phase_offset_single_sim.launch",
            "enable_phase_offset:=%s" % ("true" if enabled else "false")]
    if enabled and delta is not None:
        argv.append("test_delta:=%.2f" % delta)
    launch = Proc(argv, "b3_launch")
    ok_topics = wait_topic("/sim/odom", timeout=30.0)
    if not ok_topics:
        launch.stop()
        roscore.stop()
        return {"error": "no odom"}

    pub = rospy.Publisher("/uav_0/goal", PoseStamped, queue_size=1)
    t_end = time.time() + 5.0
    while time.time() < t_end and pub.get_num_connections() == 0:
        time.sleep(0.05)
    g = PoseStamped()
    g.header.frame_id = "world"
    g.pose.position.x = goal[0]
    g.pose.position.y = goal[1]
    g.pose.position.z = 1.0
    g.pose.orientation.w = 1.0
    pub.publish(g)

    # Wait for the UAV to reach the terminal region, then measure the
    # steady-state lateral offset relative to the base path (y for a +x path).
    time.sleep(14.0)
    ys = []
    xs = []
    t_end = time.time() + 3.0
    while time.time() < t_end:
        try:
            m = rospy.wait_for_message("/sim/odom", Odometry, timeout=1.0)
            ys.append(m.pose.pose.position.y)
            xs.append(m.pose.pose.position.x)
        except rospy.ROSException:
            pass
        time.sleep(0.05)

    vis_ok = wait_topic("/particle0/phase_offset/active_path_vis",
                        timeout=5.0, msg_type=MarkerArray)
    result = {
        "delta": delta,
        "enabled": enabled,
        "mean_y": round(sum(ys) / max(1, len(ys)), 3) if ys else None,
        "mean_x": round(sum(xs) / max(1, len(xs)), 3) if xs else None,
        "active_ref_vis": vis_ok,
    }
    launch.stop()
    roscore.stop()
    t_end = time.time() + 8.0
    while time.time() < t_end and port_open():
        pid = port_owner_pid()
        if pid is not None:
            try:
                os.kill(pid, signal.SIGTERM)
            except OSError:
                pass
        time.sleep(0.2)
    return result


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--case":
        # child mode: run one case in its own process
        name = sys.argv[2]
        delta = float(sys.argv[3]) if sys.argv[3] != "None" else None
        enabled = sys.argv[4] == "true"
        r = run_case(delta, enabled)
        print(json.dumps(r))
        return 0

    if port_open():
        print("master already running; abort")
        return 2

    cases = [
        ("legacy_off", None, False),
        ("delta_0", 0.0, True),
        ("delta_plus", 0.5, True),
        ("delta_minus", -0.5, True),
    ]
    results = {}
    ok = True
    for name, delta, enabled in cases:
        out = subprocess.check_output(
            [sys.executable, __file__, "--case", name,
             "None" if delta is None else "%.2f" % delta,
             "true" if enabled else "false"],
            text=True)
        r = json.loads(out.strip().splitlines()[-1])
        results[name] = r
        print(name, json.dumps(r))
        if r is None or "error" in r:
            ok = False
            continue
        if name == "legacy_off" or name == "delta_0":
            ok = ok and r["mean_y"] is not None and abs(r["mean_y"]) < 0.12
        elif name == "delta_plus":
            ok = ok and r["mean_y"] is not None and \
                abs(r["mean_y"] - 0.5) < 0.15
        elif name == "delta_minus":
            ok = ok and r["mean_y"] is not None and \
                abs(r["mean_y"] + 0.5) < 0.15
        if enabled:
            ok = ok and bool(r.get("active_ref_vis"))
        else:
            ok = ok and not r.get("active_ref_vis")
        time.sleep(2.0)

    results["ok"] = ok
    print(json.dumps(results, indent=2, sort_keys=True))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
