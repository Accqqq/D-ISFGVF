#!/usr/bin/env python3
"""Batch 10 scenario runs: record rosbags, RViz screenshots, ablations.

Scenarios (each: launch -> fly -> record bag -> screenshot -> analyze):
  1. obstacle_split_merge N=3      open obstacle field with central box
  2. corridor_single_wide N=3      1.0 m narrow channel (line formation)
  3. circle N=3                    closed circle, per-UAV initial w
  4. figure8 N=3                   self-intersecting closed path

Ablations (open_hex N=3, 30 s each, live stats):
  mode 0 full | 1 phase-only | 2 offset-only | 3 no damping
  | 4 no weak cohesion | 5 direct-add unmatched baseline

Usage: python3 batch10_scenarios.py [--scenarios a,b,c,d] [--ablate]
"""

import json
import math
import os
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
BAG_DIR = "/tmp/gvf_ws_ros_log/bags"
WS = "/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws"
os.environ["ROS_MASTER_URI"] = MASTER_URI
os.environ["ROS_LOG_DIR"] = LOG_DIR
os.environ["DISPLAY"] = os.environ.get("DISPLAY", ":0")

_ros_initialized = False


def ensure_ros_init():
    global _ros_initialized
    if not _ros_initialized:
        rospy.init_node("b10_runner", anonymous=True)
        _ros_initialized = True


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
                m = __import__("re").search(r"pid=(\d+)", line)
                if m:
                    return int(m.group(1))
    except Exception:
        pass
    return None


def kill_leftover_ros():
    """Kill any leftover ROS test processes so the next launch is clean."""
    pats = ["rosmaster -core", "roslaunch ", "formation_planning",
            "multi_quadrotor_simulator_so3", "quadrotor_simulator_so3",
            "map_pub ", "phase_offset_swarm_visualizer",
            "swarm_scenario_publisher",
            "phase_offset_swarm.rviz", "rosbag record"]
    for pat in pats:
        try:
            out = subprocess.check_output(["pgrep", "-f", pat], text=True,
                                          stderr=subprocess.DEVNULL)
            for pid in out.strip().splitlines():
                try:
                    os.kill(int(pid), signal.SIGKILL)
                except (OSError, ValueError):
                    pass
        except subprocess.CalledProcessError:
            pass


def wait_master(timeout=25.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            rospy.get_master().getPid()
            return True
        except Exception:
            time.sleep(0.2)
    return False


def wait_topic(topic, timeout=60.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            rospy.wait_for_message(topic, Odometry, timeout=1.0)
            return True
        except Exception:
            time.sleep(0.2)
    return False


def publish_goal(topic, x, y, z=1.0):
    pub = rospy.Publisher(topic, PoseStamped, queue_size=1, latch=True)
    t_end = time.time() + 4.0
    while time.time() < t_end and pub.get_num_connections() == 0:
        time.sleep(0.05)
    g = PoseStamped()
    g.header.stamp = rospy.Time.now()
    g.header.frame_id = "world"
    g.pose.position.x = x
    g.pose.position.y = y
    g.pose.position.z = z
    g.pose.orientation.w = 1.0
    for _ in range(6):
        pub.publish(g)
        time.sleep(0.5)


def screenshot(path):
    try:
        subprocess.run(
            ["ffmpeg", "-y", "-loglevel", "error", "-f", "x11grab",
             "-video_size", "1280x800", "-i", ":0", "-frames:v", "1", path],
            timeout=15)
        return os.path.exists(path)
    except Exception:
        return False


def run_scenario(name, launch_args, run_s, goals=None, bag=None,
                 shot=None):
    """Launch, wait odom, publish goals, run, stop, return True."""
    if port_open():
        print("[%s] master busy; abort" % name)
        return False
    roscore = Proc(["roscore"], name + "_roscore")
    if not wait_master():
        roscore.stop()
        return False
    ensure_ros_init()
    time.sleep(1.0)
    rec = None
    if bag:
        os.makedirs(os.path.dirname(bag), exist_ok=True)
        rec = Proc(["rosbag", "record", "-O", bag, "--duration=%d" % (run_s + 10),
                    "/uav_0/sim/odom", "/uav_1/sim/odom", "/uav_2/sim/odom",
                    "/uav_0/sim/imu", "/uav_1/sim/imu", "/uav_2/sim/imu",
                    "/phase_offset_swarm/debug",
                    "/phase_offset_swarm/state",
                    "/phase_offset_swarm/conflict_state",
                    "/phase_offset_swarm/path_event",
                    "/phase_offset_swarm/vis/paths",
                    "/phase_offset_swarm/vis/tubes",
                    "/uav_0/goal", "/uav_1/goal", "/uav_2/goal"],
                   name + "_record")
    launch = Proc(["roslaunch", "bspline_race"] + launch_args, name + "_launch")
    try:
        odom_ok = wait_topic("/uav_0/sim/odom")
        if name != "corridor":
            odom_ok = odom_ok and wait_topic("/uav_1/sim/odom") and \
                wait_topic("/uav_2/sim/odom")
        if not odom_ok:
            print("[%s] no odom" % name)
            return False
        time.sleep(3.0)
        for g in (goals or []):
            publish_goal(g[0], g[1], g[2])
        time.sleep(run_s)
        if shot:
            ok_shot = screenshot(shot)
            print("[%s] screenshot %s" % (name, "ok" if ok_shot else "failed"))
        return True
    finally:
        launch.stop()
        if rec:
            rec.stop()
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
        kill_leftover_ros()


SCENARIOS = {
    "obstacle": {
        "launch": ["phase_offset_swarm_sim.launch", "num_agents:=3",
                   "scenario:=obstacle_split_merge", "enable_phase_offset:=true",
                   "enable_swarm:=true", "enable_tube:=true",
                   "enable_cbf_safety:=true",
                   "initial_state_file:=" + WS + "/src/uav_simulator/"
                   "so3_quadrotor_simulator/config/initial_states_split_merge_3.yaml",
                   "init0_x:=-4.0", "init0_y:=2.5",
                   "init1_x:=-4.0", "init1_y:=-2.5",
                   "init2_x:=-4.0", "init2_y:=0.0",
                   "map_file:=" + WS + "/src/uav_simulator/dynamic_map_generator/"
                   "resource/phase_offset_split_merge.pcd"],
        "run_s": 40,
        "goals": None,
    },
    "corridor": {
        "launch": ["phase_offset_swarm_sim.launch", "num_agents:=3",
                   "scenario:=none",
                   "initial_state_file:=" + WS + "/src/uav_simulator/"
                   "so3_quadrotor_simulator/config/initial_states_corridor_two_3.yaml",
                   "init0_x:=0.0", "init0_y:=0.0",
                   "init1_x:=2.0", "init1_y:=0.0",
                   "init2_x:=4.0", "init2_y:=0.0",
                   "enable_phase_offset:=true", "enable_swarm:=true",
                   "enable_tube:=true", "enable_cbf_safety:=true",
                   "alloc_u_w_slew_rate:=4.0",
                   "map_file:=" + WS + "/src/uav_simulator/dynamic_map_generator/"
                   "resource/phase_offset_corridor_two_wide.pcd"],
        "run_s": 40,
        "goals": [("/uav_0/goal", 13.0, 0.0),
                  ("/uav_1/goal", 15.5, 0.0),
                  ("/uav_2/goal", 18.0, 0.0)],
    },
    "circle": {
        "launch": ["phase_offset_swarm_sim.launch", "num_agents:=3",
                   "scenario:=none",
                   "initial_state_file:=" + WS + "/src/uav_simulator/"
                   "so3_quadrotor_simulator/config/initial_states_circle_3.yaml",
                   "init0_x:=4.0", "init0_y:=0.0",
                   "init1_x:=-2.0", "init1_y:=3.464",
                   "init2_x:=-2.0", "init2_y:=-3.464",
                   "enable_phase_offset:=true", "enable_swarm:=true",
                   "enable_tube:=true", "enable_cbf_safety:=true",
                   "closed_shape:=circle", "closed_radius:=4.0",
                   "map_file:=" + WS + "/src/uav_simulator/dynamic_map_generator/"
                   "resource/phase_offset_open.pcd"],
        "run_s": 45,
        "goals": None,
    },
    "figure8": {
        "launch": ["phase_offset_swarm_sim.launch", "num_agents:=3",
                   "scenario:=none",
                   "initial_state_file:=" + WS + "/src/uav_simulator/"
                   "so3_quadrotor_simulator/config/initial_states_figure8_3.yaml",
                   "init0_x:=1.732", "init0_y:=2.0",
                   "init1_x:=-1.732", "init1_y:=2.0",
                   "init2_x:=0.0", "init2_y:=-4.0",
                   "enable_phase_offset:=true", "enable_swarm:=true",
                   "enable_tube:=true", "enable_cbf_safety:=true",
                   "closed_shape:=figure8", "closed_radius:=4.0",
                   # 闭环 replan 会把 B 样条拼接出近奇异弯（κ≈3），
                   # 名义解析路径本身无奇异；本场景保持名义路径。
                   "plan_interval:=100.0",
                   "map_file:=" + WS + "/src/uav_simulator/dynamic_map_generator/"
                   "resource/phase_offset_open.pcd"],
        "run_s": 50,
        "goals": None,
    },
}


class LiveCollector:
    def __init__(self):
        self.debug = []
        self.odom = {}
        self.sub = rospy.Subscriber("/phase_offset_swarm/debug",
                                    PhaseOffsetDebug, self.cb_debug,
                                    queue_size=200)
        for i in range(3):
            rospy.Subscriber("/uav_%d/sim/odom" % i, Odometry,
                             lambda m, i=i: self.cb_odom(i, m),
                             queue_size=200)

    def cb_debug(self, d):
        self.debug.append((d.robot_id, d.delta, d.e_perp_norm,
                           d.matched_residual, d.mode,
                           d.min_neighbor_distance))

    def cb_odom(self, i, m):
        self.odom.setdefault(i, []).append(
            (m.pose.pose.position.x, m.pose.pose.position.y,
             m.pose.pose.position.z))

    def stats(self):
        min_dist = None
        if len(self.odom) >= 2:
            keys = list(self.odom.keys())
            n = min(len(self.odom[k]) for k in keys)
            for k in range(n):
                pts = [self.odom[ki][k] for ki in keys]
                for a in range(len(pts)):
                    for b in range(a + 1, len(pts)):
                        d = math.dist(pts[a], pts[b])
                        min_dist = d if min_dist is None else min(min_dist, d)
        e_perp = [x[2] for x in self.debug]
        resid = [x[3] for x in self.debug]
        delta = [x[1] for x in self.debug]
        return {
            "debug_samples": len(self.debug),
            "min_inter_uav_distance": round(min_dist, 4) if min_dist else None,
            "e_perp_mean": round(sum(e_perp) / len(e_perp), 4) if e_perp else None,
            "e_perp_max": round(max(e_perp), 4) if e_perp else None,
            "matched_residual_max": round(max(resid), 8) if resid else None,
            "delta_final": round(delta[-1], 4) if delta else None,
            "emergency": sum(1 for x in self.debug if x[4] == 4),
        }


def run_ablation(mode, duration=28):
    kill_leftover_ros()
    if port_open():
        print("[ablation %d] master busy; abort" % mode)
        return None
    roscore = Proc(["roscore"], "ablate%d_roscore" % mode)
    if not wait_master():
        roscore.stop()
        return None
    ensure_ros_init()
    time.sleep(1.0)
    launch = Proc(
        ["roslaunch", "bspline_race", "phase_offset_swarm_sim.launch",
         "num_agents:=3", "scenario:=open_hex",
         "enable_phase_offset:=true", "enable_swarm:=true",
         "enable_tube:=true", "enable_cbf_safety:=true",
         "map_file:=" + WS + "/src/uav_simulator/dynamic_map_generator/"
         "resource/phase_offset_open.pcd",
         "ablation_mode:=%d" % mode],
        "ablate%d_launch" % mode)
    try:
        if not wait_topic("/uav_0/sim/odom", timeout=60):
            return None
        time.sleep(2.0)
        col = LiveCollector()
        time.sleep(duration)
        return col.stats()
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
        kill_leftover_ros()


def main():
    ap = __import__("argparse").ArgumentParser()
    ap.add_argument("--scenarios", default="obstacle,corridor,circle,figure8")
    ap.add_argument("--ablate", action="store_true")
    ap.add_argument("--ablate-modes", default=None)
    args = ap.parse_args()
    os.makedirs(BAG_DIR, exist_ok=True)
    results = {}
    for name in args.scenarios.split(","):
        name = name.strip()
        if name not in SCENARIOS:
            continue
        cfg = SCENARIOS[name]
        bag = os.path.join(BAG_DIR, name + ".bag")
        shot = os.path.join(BAG_DIR, name + "_rviz.png")
        if os.path.exists(bag):
            print("[%s] bag exists, skip" % name)
            results[name] = "skipped"
            continue
        ok = run_scenario(name, cfg["launch"], cfg["run_s"],
                          cfg.get("goals"), bag, shot)
        results[name] = "ok" if ok else "failed"
        if ok:
            subprocess.run(
                [sys.executable, WS + "/src/swarm_planner/bspline_traj/scripts/"
                 "analyze_phase_offset_bag.py", bag, "--outdir", BAG_DIR],
                env=dict(os.environ))
    if args.ablate:
        ablate = {}
        modes = [int(m) for m in args.ablate_modes.split(",")] \
            if args.ablate_modes else [0, 1, 2, 3, 4, 5]
        for mode in modes:
            st = run_ablation(mode)
            ablate[mode] = st
            print("[ablation %d] %s" % (mode, json.dumps(st)))
        with open(os.path.join(BAG_DIR, "ablation_stats.json"), "w") as f:
            json.dump(ablate, f, indent=2, sort_keys=True)
    print(json.dumps(results, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
