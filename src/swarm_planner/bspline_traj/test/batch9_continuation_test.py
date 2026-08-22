#!/usr/bin/env python3
"""Batch 9 ROS integration test: C2 update / delta continuation.

Part A (single UAV, open map, test_delta=0.3):
  - mid-flight goal change triggers an asynchronous replan (C2 install);
  - for every continuation event: delta_jump == 0, r jump < 1e-3 m,
    r_w jump < 1e-2, delta stays at 0.3 (never clipped/reset);
  - active reference stays finite; no allocator EMERGENCY storm.

Part B (N=2 isolation):
  - changing ONLY uav_0's goal mid-flight must not reset uav_1:
    uav_1 keeps 100 Hz odom, its own continuation events keep delta_jump=0,
    no NaN in its state; exactly one multi simulator process.
"""

import json
import math
import os
import re
import signal
import socket
import subprocess
import sys
import time

import rospy
from common_msgs.msg import PhaseOffsetDebug
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

MASTER_URI = "http://localhost:11311"
LOG_DIR = "/tmp/gvf_ws_ros_log"
WS = "/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws"
os.environ["ROS_MASTER_URI"] = MASTER_URI
os.environ["ROS_LOG_DIR"] = LOG_DIR


class Proc:
    def __init__(self, argv, logname):
        os.makedirs(LOG_DIR, exist_ok=True)
        stamp = time.strftime("%Y%m%d-%H%M%S")
        self.log = open(os.path.join(LOG_DIR,
                                     logname + "_" + stamp + ".log"), "a")
        self.proc = subprocess.Popen(
            argv, env=dict(os.environ), stdout=self.log,
            stderr=subprocess.STDOUT, start_new_session=True)

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


def wait_topic(topic, timeout=40.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            rospy.wait_for_message(topic, Odometry, timeout=1.0)
            return True
        except Exception:
            time.sleep(0.2)
    return False


def publish_goal(topic, x, y, z=1.0):
    pub = rospy.Publisher(topic, PoseStamped, queue_size=1)
    t_end = time.time() + 5.0
    while time.time() < t_end and pub.get_num_connections() == 0:
        time.sleep(0.05)
    g = PoseStamped()
    g.header.stamp = rospy.Time.now()
    g.header.frame_id = "world"
    g.pose.position.x = x
    g.pose.position.y = y
    g.pose.position.z = z
    g.pose.orientation.w = 1.0
    pub.publish(g)


class DebugCollector:
    """Persistent subscriber so no debug message is lost between polls."""

    def __init__(self, topic="/phase_offset_swarm/debug"):
        self.samples = []
        self.sub = rospy.Subscriber(topic, PhaseOffsetDebug, self.cb,
                                    queue_size=200)

    def cb(self, d):
        self.samples.append({
            "robot_id": d.robot_id,
            "phase": d.phase,
            "delta": d.delta,
            "delta_jump": d.delta_jump,
            "delta_r": d.delta_r_norm,
            "delta_rw": d.delta_rw_norm,
            "delta_error": d.delta_error_norm,
            "seq": d.continuation_sequence,
            "mode": d.mode,
            "r": (d.active_reference.x, d.active_reference.y,
                  d.active_reference.z),
        })

    def snapshot(self):
        return list(self.samples)


def check_part_a():
    corridor = os.path.join(
        WS, "src/uav_simulator/dynamic_map_generator/resource/"
            "phase_offset_open.pcd")
    launch = Proc(
        ["roslaunch", "bspline_race", "phase_offset_single_sim.launch",
         "enable_phase_offset:=true", "enable_swarm:=true",
         "enable_tube:=true", "test_delta:=0.3",
         "map_file:=" + corridor],
        "b9a_launch")
    try:
        if not wait_topic("/sim/odom"):
            return {"ok": False, "error": "no odom"}
        col = DebugCollector()
        publish_goal("/uav_0/goal", 8.0, 0.0)
        time.sleep(5.0)
        # Async path update: change the goal while the UAV is still flying.
        publish_goal("/uav_0/goal", 6.0, 3.0)
        time.sleep(15.0)
        samples = col.snapshot()

        seqs = sorted(set(s["seq"] for s in samples if s["seq"] > 0))

        max_jump = max((s["delta_jump"] for s in samples), default=0.0)
        max_r = max((s["delta_r"] for s in samples), default=0.0)
        max_rw = max((s["delta_rw"] for s in samples), default=0.0)
        deltas = [s["delta"] for s in samples]
        emergency = sum(1 for s in samples if s["mode"] == 4)
        nan_r = sum(1 for s in samples
                    if not all(math.isfinite(v) for v in s["r"]))
        # A delta reset would snap the value back to the 0.3 initial after it
        # had already decayed below 0.1 (recenter pulls it to 0).
        reset_detected = False
        seen_decayed = False
        for s in samples:
            if s["delta"] < 0.1:
                seen_decayed = True
            if seen_decayed and abs(s["delta"] - 0.3) < 1e-3:
                reset_detected = True

        ok = (len(seqs) >= 2 and
              max_jump <= 1e-6 and
              max_r < 1e-3 and
              max_rw < 1e-2 and
              not reset_detected and
              emergency <= max(1, len(samples) // 20) and
              nan_r == 0)
        return {
            "ok": ok,
            "samples": len(samples),
            "continuation_events": len(seqs),
            "max_delta_jump": max_jump,
            "max_delta_r": max_r,
            "max_delta_rw": max_rw,
            "delta_reset_detected": reset_detected,
            "emergency_samples": emergency,
            "nan_active_reference": nan_r,
        }
    finally:
        launch.stop()


def check_part_b():
    state_file = os.path.join(
        WS, "src/uav_simulator/so3_quadrotor_simulator/config/"
            "initial_states_cbf_2.yaml")
    launch = Proc(
        ["roslaunch", "bspline_race", "phase_offset_swarm_sim.launch",
         "num_agents:=2", "scenario:=open_hex",
         "initial_state_file:=" + state_file,
         "enable_phase_offset:=true", "enable_swarm:=true",
         "enable_tube:=true"],
        "b9b_launch")
    try:
        if not wait_topic("/uav_0/sim/odom") or \
           not wait_topic("/uav_1/sim/odom"):
            return {"ok": False, "error": "no dual odom"}
        col = DebugCollector()
        publish_goal("/uav_0/goal", 8.0, 1.0)
        publish_goal("/uav_1/goal", 8.0, -1.0)
        time.sleep(2.0)
        # Change ONLY uav_0's goal.
        publish_goal("/uav_0/goal", 5.0, 4.0)
        time.sleep(20.0)
        all_samples = col.snapshot()
        s0 = [s for s in all_samples if s["robot_id"] == 0]
        s1 = [s for s in all_samples if s["robot_id"] == 1]

        seq0 = sorted(set(s["seq"] for s in s0 if s["seq"] > 0))
        seq1 = sorted(set(s["seq"] for s in s1 if s["seq"] > 0))
        max_jump0 = max((s["delta_jump"] for s in s0), default=0.0)
        max_jump1 = max((s["delta_jump"] for s in s1), default=0.0)
        max_r1 = max((s["delta_r"] for s in s1), default=0.0)
        nan1 = sum(1 for s in s1
                   if not all(math.isfinite(v) for v in s["r"]))
        try:
            out = subprocess.check_output(
                ["pgrep", "-f", "multi_quadrotor_simulator_so3"],
                text=True, stderr=subprocess.DEVNULL)
            n_sim = len(out.strip().splitlines()) if out.strip() else 0
        except subprocess.CalledProcessError:
            n_sim = 0

        # uav_1 odom still alive after uav_0's change?
        try:
            rospy.wait_for_message("/uav_1/sim/odom", Odometry, timeout=3.0)
            odom_alive = True
        except Exception:
            odom_alive = False

        ok = (len(seq0) >= 2 and
              len(seq1) >= 1 and
              max_jump0 <= 1e-6 and
              max_jump1 <= 1e-6 and
              max_r1 < 1e-3 and
              nan1 == 0 and
              n_sim == 1 and
              odom_alive)
        return {
            "ok": ok,
            "uav0_events": len(seq0),
            "uav1_events": len(seq1),
            "uav0_max_delta_jump": max_jump0,
            "uav1_max_delta_jump": max_jump1,
            "uav1_max_delta_r": max_r1,
            "uav1_nan": nan1,
            "multi_sim_processes": n_sim,
            "uav1_odom_alive": odom_alive,
        }
    finally:
        launch.stop()


def main():
    if port_open():
        print("master already running; abort")
        return 2
    roscore = Proc(["roscore"], "b9_roscore")
    if not wait_master():
        roscore.stop()
        return 1
    rospy.init_node("batch9_test", anonymous=True)
    time.sleep(1.0)
    try:
        a = check_part_a()
        print("PART_A", json.dumps(a, sort_keys=True))
        b = check_part_b()
        print("PART_B", json.dumps(b, sort_keys=True))
        ok = a.get("ok") and b.get("ok")
        print("OVERALL_OK", ok)
        return 0 if ok else 1
    finally:
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


if __name__ == "__main__":
    sys.exit(main())
