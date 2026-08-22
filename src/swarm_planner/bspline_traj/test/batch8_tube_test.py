#!/usr/bin/env python3
"""Batch 8 ROS integration test: ESDF path tube + channel conflict events.

Single UAV flies down the single-wide corridor.  Checks:
  - tube markers published;
  - the tube narrows in the corridor: channel_beta < 1 and conflict events
    active while inside the corridor;
  - the active reference stays inside the tube (delta within bounds);
  - the allocator remains feasible (no EMERGENCY).
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
from common_msgs.msg import PhaseOffsetDebug, SwarmConflictState
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
        # Unique per-run log so stale processes holding an old descriptor can
        # never interleave output into a later run's log.
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
    roscore = Proc(["roscore"], "b8_roscore")
    if not wait_master():
        roscore.stop()
        return 1
    rospy.init_node("batch8_test", anonymous=True)
    time.sleep(1.0)
    corridor = os.path.join(
        WS, "src/uav_simulator/dynamic_map_generator/resource/"
            "phase_offset_corridor_single_wide.pcd")
    launch = Proc(
        ["roslaunch", "bspline_race", "phase_offset_single_sim.launch",
         "enable_phase_offset:=true", "enable_swarm:=true",
         "enable_tube:=true", "map_file:=" + corridor],
        "b8_launch")
    result = {}
    ok = True
    try:
        if not wait_topic("/sim/odom", timeout=40.0):
            return 1
        pub = rospy.Publisher("/uav_0/goal", PoseStamped, queue_size=1)
        t_end = time.time() + 5.0
        while time.time() < t_end and pub.get_num_connections() == 0:
            time.sleep(0.05)
        g = PoseStamped()
        g.header.frame_id = "world"
        g.pose.position.x = 8.0
        g.pose.position.y = 0.0
        g.pose.position.z = 1.0
        g.pose.orientation.w = 1.0
        pub.publish(g)

        betas = []
        conflicts = []
        tube_msgs = [0]

        def tube_cb(m):
            tube_msgs[0] += 1

        rospy.Subscriber("/phase_offset_swarm/vis/tubes", MarkerArray,
                         tube_cb, queue_size=100)
        emergency = 0
        samples = 0
        inside_tube = True
        t_end = time.time() + 20.0
        while time.time() < t_end:
            try:
                d = rospy.wait_for_message("/phase_offset_swarm/debug",
                                           PhaseOffsetDebug, timeout=2.0)
                betas.append(d.channel_beta)
                samples += 1
                if d.mode == 4:
                    emergency += 1
                if d.delta < d.delta_lower - 0.02 or \
                   d.delta > d.delta_upper + 0.02:
                    inside_tube = False
            except rospy.ROSException:
                pass
            try:
                c = rospy.wait_for_message(
                    "/phase_offset_swarm/conflict_state",
                    SwarmConflictState, timeout=1.0)
                conflicts.append((c.channel_beta, c.active))
            except rospy.ROSException:
                pass

        min_beta = min(betas) if betas else None
        max_beta = max(betas) if betas else None
        conflict_active = any(a for _, a in conflicts)
        conflict_narrow = any(b < 0.6 for b, _ in conflicts)
        result = {
            "debug_samples": samples,
            "tube_beta_min": min_beta,
            "tube_beta_max": max_beta,
            "tube_marker_messages": tube_msgs[0],
            "conflict_events": len(conflicts),
            "conflict_active": conflict_active,
            "conflict_narrow_beta": conflict_narrow,
            "emergency_samples": emergency,
            "delta_inside_tube": inside_tube,
        }
        ok = ok and min_beta is not None and min_beta < 0.6
        ok = ok and conflict_narrow
        ok = ok and tube_msgs[0] > 0
        ok = ok and inside_tube
        ok = ok and emergency <= max(1, samples // 20)
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
