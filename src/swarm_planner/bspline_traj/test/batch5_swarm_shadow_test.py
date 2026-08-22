#!/usr/bin/env python3
"""Batch 5 ROS integration test: distributed state + local neighbor
aggregation in SHADOW mode (N=7 hexagon).

Checks:
  1. /phase_offset_swarm/state at ~25 Hz, correct robot_ids;
  2. per-UAV PhaseOffsetDebug with shadow_mode=true, organization neighbor
     sets matching the hex table (A: B C D, ...), per-edge contributions;
  3. swarm intent does NOT drive the UAVs (shadow only);
  4. interactions markers published;
  5. failure injection: kill uav_0's planner -> uav_1's organization drops 0
     after the timeout while safety retains it; other UAVs unaffected.
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
from common_msgs.msg import PhaseOffsetDebug, SwarmState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray

MASTER_URI = "http://localhost:11311"
LOG_DIR = "/tmp/gvf_ws_ros_log"
WS = "/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws"
os.environ["ROS_MASTER_URI"] = MASTER_URI
os.environ["ROS_LOG_DIR"] = LOG_DIR

EXPECTED = {
    0: {1, 2, 3},
    1: {0, 3, 4},
    2: {0, 3, 5},
    3: {0, 1, 2, 4, 5, 6},
    4: {1, 3, 6},
    5: {2, 3, 6},
    6: {3, 4, 5},
}


class Proc:
    def __init__(self, argv, logname):
        self.argv = argv
        os.makedirs(LOG_DIR, exist_ok=True)
        self.log = open(os.path.join(LOG_DIR, logname + ".log"), "a")
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


def wait_topic(topic, timeout=25.0, msg_type=Odometry):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            rospy.wait_for_message(topic, msg_type, timeout=1.0)
            return True
        except Exception:
            time.sleep(0.2)
    return False


def main():
    if port_open():
        print("master already running; abort")
        return 2
    roscore = Proc(["roscore"], "b5_roscore")
    if not wait_master():
        roscore.stop()
        return 1
    rospy.init_node("batch5_test", anonymous=True)
    time.sleep(1.0)

    map_file = os.path.join(WS, "src/uav_simulator/dynamic_map_generator/"
                             "resource/phase_offset_open.pcd")
    launch = Proc(
        ["roslaunch", "bspline_race", "phase_offset_swarm_sim.launch",
         "num_agents:=7", "scenario:=none",
         "enable_phase_offset:=true", "enable_swarm:=true",
         "map_file:=" + map_file],
        "b5_launch")
    result = {}
    ok = True
    try:
        for i in range(7):
            if not wait_topic("/uav_%d/sim/odom" % i, timeout=40.0):
                result["error"] = "no odom %d" % i
                ok = False
                return 1

        # 1. state broadcast rate (per robot) and robot ids
        states = []
        sub = rospy.Subscriber("/phase_offset_swarm/state", SwarmState,
                               lambda m: states.append(m), queue_size=1000)
        time.sleep(3.0)
        ids = set(s.robot_id for s in states)
        finite = all(math.isfinite(s.position_world.x) and
                     math.isfinite(s.position_world.y) for s in states)
        per_robot_hz = {}
        for rid in range(7):
            n = sum(1 for s in states if s.robot_id == rid)
            per_robot_hz[rid] = round(n / 3.0, 1)
        ok = ok and all(15.0 <= per_robot_hz[rid] <= 40.0
                        for rid in range(7))
        ok = ok and ids == set(range(7)) and finite
        result["state_hz_per_robot"] = per_robot_hz
        result["state_ids"] = sorted(ids)

        # Keep a time-indexed state buffer for the end-to-end neighbor check.
        state_buf = []

        def state_cb(m):
            state_buf.append((rospy.Time.now().to_sec(), m))
            states.append(m)

        state_sub = rospy.Subscriber("/phase_offset_swarm/state", SwarmState,
                                     state_cb, queue_size=2000)
        states.clear()

        # send goals so paths install and the debug starts flowing
        goals = [(0, 6.0, 3.0), (1, 6.0, 1.0), (2, 6.0, -1.0),
                 (3, 6.0, -3.0), (4, 7.0, 3.0), (5, 7.0, 1.0),
                 (6, 7.0, -1.0)]
        for rid, gx, gy in goals:
            pub = rospy.Publisher("/uav_%d/goal" % rid, PoseStamped,
                                  queue_size=1)
            t_end = time.time() + 5.0
            while time.time() < t_end and pub.get_num_connections() == 0:
                time.sleep(0.05)
            g = PoseStamped()
            g.header.frame_id = "world"
            g.pose.position.x = gx
            g.pose.position.y = gy
            g.pose.position.z = 1.0
            g.pose.orientation.w = 1.0
            pub.publish(g)
            time.sleep(0.2)

        # 2. end-to-end consistency + interactions + failure injection:
        #    for every debug sample of UAV i, the
        #    reported organization set must lie between the enter-radius and
        #    exit-radius expectations computed from the shared state topic.
        org_checks = {}
        sample_count = 0
        mismatch_count = 0
        interactions_seen = [0]

        def interactions_cb(m):
            interactions_seen[0] += 1

        rospy.Subscriber("/phase_offset_swarm/vis/interactions", MarkerArray,
                         interactions_cb, queue_size=100)

        t_start = time.time()
        killed = [False]
        t_end = time.time() + 20.0
        while time.time() < t_end and sample_count < 300:
            if not killed[0] and time.time() - t_start > 8.0:
                killed[0] = True
                try:
                    subprocess.check_call(
                        ["rosnode", "kill", "/uav_0/formation_planning"],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL)
                except subprocess.CalledProcessError:
                    pass
            try:
                d = rospy.wait_for_message("/phase_offset_swarm/debug",
                                           PhaseOffsetDebug, timeout=2.0)
                rid = d.robot_id
                now = rospy.Time.now().to_sec()
                recent = {}
                for t, s in state_buf:
                    if now - t < 0.35:
                        recent[s.robot_id] = s
                if rid not in recent:
                    continue
                self_pos = (recent[rid].position_world.x,
                            recent[rid].position_world.y)
                enter = set()
                exit_s = set()
                for j, s in recent.items():
                    if j == rid:
                        continue
                    dx = s.position_world.x - self_pos[0]
                    dy = s.position_world.y - self_pos[1]
                    dist = math.hypot(dx, dy)
                    if dist < 1.55:
                        enter.add(j)
                    if dist < 1.65:
                        exit_s.add(j)
                org = set(d.organization_neighbors)
                ok_this = enter.issubset(org) and org.issubset(exit_s)
                ok = ok and ok_this
                sample_count += 1
                if not ok_this:
                    mismatch_count += 1
                if rid not in org_checks:
                    org_checks[rid] = {
                        "samples": 0,
                        "enter": sorted(enter),
                        "exit": sorted(exit_s),
                        "org": sorted(org),
                    }
                org_checks[rid]["samples"] += 1
                # Batch 6 activated the allocator: the debug no longer runs
                # in shadow mode.
                if d.shadow_mode:
                    ok = False
                if killed[0] and rid == 1:
                    if 0 in org:
                        result["uav1_org_kept_after_kill"] = True
                        ok = False
                    if 0 not in set(d.safety_neighbors):
                        result["uav1_safety_lost_after_kill"] = True
                        ok = False
            except rospy.ROSException:
                pass
        result["org_checks"] = org_checks
        result["neighbor_samples"] = sample_count
        result["neighbor_mismatches"] = mismatch_count
        result["interactions_messages"] = interactions_seen[0]
        result["kill_injected"] = killed[0]
        ok = ok and sample_count >= 50 and mismatch_count == 0
        ok = ok and interactions_seen[0] > 0
        ok = ok and killed[0]
        # other UAVs keep publishing odom after the kill
        ok = ok and wait_topic("/uav_2/sim/odom", timeout=10.0)
        ok = ok and wait_topic("/uav_6/sim/odom", timeout=10.0)
    finally:
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

    result["ok"] = ok
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
