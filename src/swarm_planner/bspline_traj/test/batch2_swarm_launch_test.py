#!/usr/bin/env python3
"""Batch 2 ROS integration test: unified swarm launch.

Checks for a given num_agents:
  1. exactly one multi simulator process / node;
  2. exactly one map_pub, nodelet manager, visualizer, RViz;
  3. one formation_planning + one local_sensing + one controller per UAV;
  4. /uav_i/sim/odom ~100 Hz, /uav_i/position_cmd present,
     /uav_i/particle0/path present, /uav_i/particle0/sdf_map/occupancy
     present, and NO global /particle0sdf_map or /particle0/path leftovers;
  5. per-UAV goals -> each UAV moves toward its own goal;
  6. killing the read-only visualizer does not stop planning/odom.

Run with a sourced ROS environment:
  python3 src/swarm_planner/bspline_traj/test/batch2_swarm_launch_test.py --n 3
"""

import argparse
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
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import PointCloud2

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


def hz_of(samples):
    if len(samples) < 2:
        return 0.0
    dt = samples[-1][0] - samples[0][0]
    return (len(samples) - 1) / dt if dt > 0 else 0.0


def node_list():
    out = subprocess.check_output(["rosnode", "list"], text=True)
    return set(out.split())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=3)
    ap.add_argument("--duration", type=float, default=30.0)
    args = ap.parse_args()
    n = args.n

    result = {"n": n}
    ok = True
    roscore = None
    launch = None
    try:
        if port_open():
            print("master already running; abort")
            return 2
        roscore = Proc(["roscore"], "roscore")
        if not wait_master():
            print("roscore failed")
            return 1
        rospy.init_node("batch2_test", anonymous=True)
        time.sleep(1.0)

        launch = Proc(
            ["roslaunch", "bspline_race",
             "phase_offset_swarm_sim.launch",
             "num_agents:=%d" % n, "scenario:=none"],
            "b2_launch_%d" % n)

        # 1. odom / local map / command topics
        hzs = {}
        topic_ok = {}
        for i in range(n):
            odom_ok = wait_topic("/uav_%d/sim/odom" % i, timeout=30.0)
            sdf_ok = wait_topic(
                "/uav_%d/particle0/sdf_map/occupancy" % i, timeout=20.0,
                msg_type=PointCloud2)
            topic_ok["uav_%d" % i] = {
                "odom": odom_ok, "sdf": sdf_ok,
            }
            ok = ok and odom_ok and sdf_ok
            if not odom_ok:
                hzs["uav_%d" % i] = 0.0
                continue
            rec = []
            sub = rospy.Subscriber("/uav_%d/sim/odom" % i, Odometry,
                                   lambda m, r=rec: r.append((time.time(), m)))
            time.sleep(3.0)
            hzs["uav_%d" % i] = hz_of(rec)
            ok = ok and 80.0 <= hzs["uav_%d" % i] <= 130.0

        # 2. no global leftovers
        topics = subprocess.check_output(
            ["rostopic", "list"], text=True).split()
        bad = [t for t in topics
               if t in ("/particle0sdf_map", "/particle0/path",
                        "/particle0/kinopath")]
        ok = ok and not bad
        result["leftover_topics"] = bad

        # 3. node counts
        nodes = node_list()
        multi_nodes = [x for x in nodes
                       if x == "/multi_quadrotor_simulator_so3"]
        planners = [x for x in nodes if x.endswith("/formation_planning")]
        sensors = [x for x in nodes if x.endswith("/map_generator")]
        ctrls = [x for x in nodes
                 if re.match(r"/so3_control_nodelet_\d+$", x)]
        vis = [x for x in nodes if x == "/phase_offset_swarm_visualizer"]
        rvizzes = [x for x in nodes if x == "/rviz"]
        map_pubs = [x for x in nodes if x == "/map_pub"]
        mgrs = [x for x in nodes if x == "/so3_nodelet_manager"]
        ps = subprocess.check_output(["ps", "-eo", "args"], text=True)
        multi_procs = [l for l in ps.splitlines()
                       if "multi_quadrotor_simulator_so3" in l
                       and "bash" not in l and "grep" not in l
                       and "batch2_swarm_launch_test" not in l]
        ok = ok and len(multi_nodes) == 1 and len(multi_procs) == 1
        ok = ok and len(planners) == n and len(sensors) == n
        ok = ok and len(ctrls) == n and len(vis) == 1 and len(rvizzes) == 1
        ok = ok and len(map_pubs) == 1 and len(mgrs) == 1
        result.update({
            "multi_nodes": len(multi_nodes),
            "multi_procs": len(multi_procs),
            "planners": len(planners),
            "local_sensing": len(sensors),
            "controllers": len(ctrls),
            "visualizers": len(vis),
            "rviz": len(rvizzes),
            "map_pub": len(map_pubs),
            "nodelet_managers": len(mgrs),
            "odom_hz": {k: round(v, 1) for k, v in hzs.items()},
        })

        # 4. per-UAV goals and independent motion
        goals = {}
        for i in range(n):
            g = PoseStamped()
            g.header.frame_id = "world"
            g.pose.position.x = 6.0 + i
            g.pose.position.y = 2.0 * (i - (n - 1) / 2.0)
            g.pose.position.z = 1.0
            g.pose.orientation.w = 1.0
            goals[i] = g
            pub = rospy.Publisher("/uav_%d/goal" % i, PoseStamped,
                                  queue_size=1)
            t_end = time.time() + 5.0
            while time.time() < t_end and pub.get_num_connections() == 0:
                time.sleep(0.05)
            time.sleep(0.2)
            pub.publish(g)

        # initial positions
        init_pos = {}
        for i in range(n):
            m = rospy.wait_for_message("/uav_%d/sim/odom" % i, Odometry,
                                       timeout=10.0)
            init_pos[i] = (m.pose.pose.position.x, m.pose.pose.position.y)

        # position_cmd and particle path appear only after the manager
        # leaves WAIT_TARGET (i.e. after the goal was received)
        cmd_ok = {}
        path_ok = {}
        for i in range(n):
            c = wait_topic("/uav_%d/position_cmd" % i, timeout=15.0,
                           msg_type=PositionCommand)
            p = wait_topic("/uav_%d/particle0/path" % i, timeout=15.0,
                           msg_type=Path)
            cmd_ok["uav_%d" % i] = c
            path_ok["uav_%d" % i] = p
            ok = ok and c and p
        result["topic_ok"] = topic_ok
        result["cmd_ok"] = cmd_ok
        result["path_ok"] = path_ok

        time.sleep(args.duration)

        motion = {}
        for i in range(n):
            m = rospy.wait_for_message("/uav_%d/sim/odom" % i, Odometry,
                                       timeout=10.0)
            pos = (m.pose.pose.position.x, m.pose.pose.position.y)
            fin = math.sqrt((pos[0] - goals[i].pose.position.x) ** 2 +
                            (pos[1] - goals[i].pose.position.y) ** 2)
            ini = math.sqrt((init_pos[i][0] - goals[i].pose.position.x) ** 2 +
                            (init_pos[i][1] - goals[i].pose.position.y) ** 2)
            moved = math.sqrt((pos[0] - init_pos[i][0]) ** 2 +
                              (pos[1] - init_pos[i][1]) ** 2)
            progress = 1.0 - fin / max(ini, 1e-6)
            finite = all(math.isfinite(v) for v in
                         (m.pose.pose.position.x, m.pose.pose.position.y,
                          m.pose.pose.position.z,
                          m.twist.twist.linear.x, m.twist.twist.linear.y,
                          m.twist.twist.linear.z))
            motion["uav_%d" % i] = {
                "moved_m": round(moved, 2),
                "progress": round(progress, 3),
                "finite": finite,
            }
            ok = ok and moved > 0.5 and progress > 0.3 and finite
        result["motion"] = motion

        # 5. kill the visualizer; everything else must continue
        viz_pids = []
        try:
            out = subprocess.check_output(
                ["pgrep", "-f", "phase_offset_swarm_visualizer"], text=True)
            viz_pids = out.split()
        except subprocess.CalledProcessError:
            pass
        try:
            subprocess.check_call(["rosnode", "kill",
                                   "/phase_offset_swarm_visualizer"],
                                  stdout=subprocess.DEVNULL,
                                  stderr=subprocess.DEVNULL)
        except subprocess.CalledProcessError:
            pass
        for pid in viz_pids:
            try:
                os.kill(int(pid), signal.SIGTERM)
            except OSError:
                pass
        time.sleep(2.0)
        for pid in viz_pids:
            try:
                os.kill(int(pid), signal.SIGKILL)
            except OSError:
                pass
        time.sleep(2.0)
        nodes_after = node_list()
        planners_after = [x for x in nodes_after
                          if x.endswith("/formation_planning")]
        ps_after = subprocess.check_output(["ps", "-eo", "args"], text=True)
        vis_procs = [l for l in ps_after.splitlines()
                     if "phase_offset_swarm_visualizer" in l
                     and "batch2_swarm_launch_test" not in l
                     and "ps -eo" not in l]
        odom_after = wait_topic("/uav_0/sim/odom", timeout=10.0)
        ok = ok and len(planners_after) == n and not vis_procs and odom_after
        result["after_visualizer_kill"] = {
            "planners": len(planners_after),
            "visualizer_processes": len(vis_procs),
            "uav0_odom_alive": odom_after,
        }
    finally:
        if launch is not None:
            launch.stop()
        if roscore is not None:
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
