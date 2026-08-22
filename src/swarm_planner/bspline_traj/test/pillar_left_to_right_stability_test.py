#!/usr/bin/env python3
"""P2P stabilization test on the original pillar.pcd (3 UAVs, left->right).

Stages:
  0: phase_offset=false swarm=false tube=false cbf=false (independent nav)
  1: phase_offset=true  swarm=false tube=false cbf=false
  2: phase_offset=true  swarm=true  tube=true  cbf=true

The speed is controlled ONLY through gvf_gain1/gvf_gain2 (0.8 / -0.88);
no max-velocity parameter is overridden.

Usage:
  python3 pillar_left_to_right_stability_test.py --stage 2 --runs 10 --tag p2p2
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

import numpy as np
import rospy
from common_msgs.msg import PhaseOffsetDebug, SwarmState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

MASTER_URI = "http://localhost:11311"
LOG_DIR = "/tmp/gvf_ws_ros_log"
OUT_DIR = os.path.join(LOG_DIR, "p2p")
WS = "/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws"
PILLAR = os.path.join(WS, "src/uav_simulator/dynamic_map_generator/"
                           "resource/pillar.pcd")
GOALS = {0: (5.0, -4.75), 1: (5.0, -3.50), 2: (5.0, -2.25)}

os.environ["ROS_MASTER_URI"] = MASTER_URI
os.environ["ROS_LOG_DIR"] = LOG_DIR

COLLISION_MARGIN = 0.45   # horizontal distance to pillar surface
ARRIVE_SEC = 45.0


class Proc:
    def __init__(self, argv, logname):
        os.makedirs(LOG_DIR, exist_ok=True)
        stamp = time.strftime("%Y%m%d-%H%M%S")
        self.log_path = os.path.join(LOG_DIR, logname + "_" + stamp + ".log")
        self.log = open(self.log_path, "a")
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


def kill_leftover_ros():
    pats = ["rosmaster -core", "roslaunch ", "formation_planning",
            "multi_quadrotor_simulator_so3", "quadrotor_simulator_so3",
            "map_pub ", "phase_offset_swarm_visualizer",
            "swarm_scenario_publisher", "phase_offset_swarm.rviz",
            "rosbag record"]
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


def wait_topic(topic, timeout=75.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            rospy.wait_for_message(topic, Odometry, timeout=1.0)
            return True
        except Exception:
            time.sleep(0.2)
    return False


def publish_goal_once(topic, x, y, z=1.0):
    pub = rospy.Publisher(topic, PoseStamped, queue_size=1, latch=True)
    t_end = time.time() + 8.0
    while time.time() < t_end and pub.get_num_connections() == 0:
        time.sleep(0.1)
    if pub.get_num_connections() == 0:
        return False
    g = PoseStamped()
    g.header.stamp = rospy.Time.now()
    g.header.frame_id = "world"
    g.pose.position.x = x
    g.pose.position.y = y
    g.pose.position.z = z
    g.pose.orientation.w = 1.0
    pub.publish(g)  # exactly once
    return True


def load_pillars():
    pts = []
    with open(PILLAR) as f:
        for i, line in enumerate(f):
            if i < 10:
                continue
            parts = line.split()
            if len(parts) < 3:
                continue
            z = float(parts[2])
            if 0.5 < z < 1.5:
                pts.append((float(parts[0]), float(parts[1])))
    return np.array(pts)


PILLAR_PTS = load_pillars()


def min_obstacle_distances(xy):
    """xy: (N,2) numpy; returns per-point min horizontal distance to pillars."""
    out = np.empty(len(xy))
    block = 1500
    for i in range(0, len(xy), block):
        d = np.hypot(xy[i:i + block, None, 0] - PILLAR_PTS[None, :, 0],
                     xy[i:i + block, None, 1] - PILLAR_PTS[None, :, 1])
        out[i:i + block] = d.min(axis=1)
    return out


def run_once(stage, run_idx, tag):
    kill_leftover_ros()
    if port_open():
        return {"ok": False, "error": "master busy"}
    roscore = Proc(["roscore"], tag + "_roscore")
    if not wait_master():
        roscore.stop()
        return {"ok": False, "error": "no master"}
    rospy.init_node("p2p_test_%s_%d" % (tag, run_idx), anonymous=True)
    time.sleep(1.0)

    phase_offset = stage >= 1
    swarm = stage >= 2
    tube = stage >= 2
    cbf = stage >= 2

    launch = Proc(
        ["roslaunch", "bspline_race",
         "phase_offset_pillar_left_to_right_3.launch",
         "enable_phase_offset:=%s" % ("true" if phase_offset else "false"),
         "enable_swarm:=%s" % ("true" if swarm else "false"),
         "enable_tube:=%s" % ("true" if tube else "false"),
         "enable_cbf_safety:=%s" % ("true" if cbf else "false"),
         "gvf_gain1:=0.8", "gvf_gain2:=-0.88"],
        tag + "_launch")
    result = {"stage": stage, "run": run_idx}
    try:
        for i in range(3):
            if not wait_topic("/uav_%d/sim/odom" % i):
                result.update({"ok": False, "error": "no odom %d" % i})
                return result
        time.sleep(2.0)

        # Publish each goal exactly once (wait for a subscriber first).
        goal_ok = True
        t_goal = {}
        for i in range(3):
            ok = publish_goal_once("/uav_%d/goal" % i,
                                   GOALS[i][0], GOALS[i][1])
            goal_ok = goal_ok and ok
            t_goal[i] = time.time()
        result["goal_published"] = goal_ok
        if not goal_ok:
            result.update({"ok": False,
                           "error": "goal topic no subscriber"})
            return result

        odom = {i: [] for i in range(3)}
        debug = {i: [] for i in range(3)}
        state_ts = []
        for i in range(3):
            rospy.Subscriber(
                "/uav_%d/sim/odom" % i, Odometry,
                lambda m, i=i: odom[i].append(
                    (m.header.stamp.to_sec(),
                     m.pose.pose.position.x, m.pose.pose.position.y,
                     m.pose.pose.position.z,
                     m.twist.twist.linear.x, m.twist.twist.linear.y,
                     m.twist.twist.linear.z)),
                queue_size=500)
        rospy.Subscriber("/phase_offset_swarm/debug", PhaseOffsetDebug,
                         lambda m: debug[m.robot_id].append(
                             (m.header.stamp.to_sec(), m.mode,
                              m.matched_residual, m.e_perp_norm, m.delta,
                              m.delta_jump, m.allocator_ms)),
                         queue_size=300)
        rospy.Subscriber("/phase_offset_swarm/state", SwarmState,
                         lambda m: state_ts.append(m.header.stamp.to_sec()),
                         queue_size=300)

        time.sleep(55.0)
        time.sleep(0.5)

        # --- evaluation ---
        ok = True
        reasons = []

        # Arrival: horizontal <= 0.25, z error <= 0.10, speed <= 0.15,
        # sustained >= 3 s, within 45 s after goal, drift <= 0.10 in 5 s.
        arrival = {}
        for i in range(3):
            pts = odom[i]
            t0 = t_goal[i]
            arrived_at = None
            if len(pts) < 20:
                ok = False
                reasons.append("uav%d no odom samples" % i)
                continue
            window = 3.0
            n_win = max(2, int(window / 0.01))
            for k in range(len(pts)):
                if pts[k][0] < t0:
                    continue
                if pts[k][0] - t0 > ARRIVE_SEC:
                    break
                err_h = math.hypot(pts[k][1] - GOALS[i][0],
                                   pts[k][2] - GOALS[i][1])
                err_z = abs(pts[k][3] - 1.0)
                spd = math.sqrt(pts[k][4] ** 2 + pts[k][5] ** 2 +
                                pts[k][6] ** 2)
                if err_h <= 0.25 and err_z <= 0.10 and spd <= 0.15:
                    # check the following 3 s
                    t_end = pts[k][0] + window
                    j = k
                    good = True
                    while j < len(pts) and pts[j][0] <= t_end:
                        e2 = math.hypot(pts[j][1] - GOALS[i][0],
                                        pts[j][2] - GOALS[i][1])
                        z2 = abs(pts[j][3] - 1.0)
                        s2 = math.sqrt(pts[j][4] ** 2 + pts[j][5] ** 2 +
                                       pts[j][6] ** 2)
                        if e2 > 0.25 or z2 > 0.10 or s2 > 0.15:
                            good = False
                            break
                        j += 1
                    if good:
                        arrived_at = pts[k][0] - t0
                        # drift over the next 5 s
                        drift = 1e9
                        t5 = pts[k][0] + 5.0
                        p0 = np.array([pts[k][1], pts[k][2], pts[k][3]])
                        for jj in range(k, len(pts)):
                            if pts[jj][0] >= t5:
                                drift = np.linalg.norm(
                                    p0 - [pts[jj][1], pts[jj][2],
                                          pts[jj][3]])
                                break
                        else:
                            if len(pts) > k + 1:
                                drift = np.linalg.norm(
                                    p0 - [pts[-1][1], pts[-1][2],
                                          pts[-1][3]])
                        arrival[i] = {"arrival_s": round(arrived_at, 2),
                                      "drift_5s": round(float(drift), 4)}
                        if arrived_at > ARRIVE_SEC:
                            ok = False
                            reasons.append("uav%d arrived late (%.1fs)" %
                                           (i, arrived_at))
                        if drift > 0.10:
                            ok = False
                            reasons.append("uav%d drift %.3f" % (i, drift))
                        break
            if arrived_at is None:
                ok = False
                reasons.append("uav%d never arrived" % i)
        result["arrival"] = arrival

        # Inter-UAV distance: nearest-timestamp alignment per pair.
        min_dist = 1e9
        close_samples = 0
        for a in range(3):
            for b in range(a + 1, 3):
                A, B = odom[a], odom[b]
                bi = 0
                for pa in A:
                    while bi + 1 < len(B) and B[bi + 1][0] < pa[0]:
                        bi += 1
                    for k in (bi - 1, bi, bi + 1):
                        if 0 <= k < len(B):
                            d = math.dist(pa[1:4], B[k][1:4])
                            min_dist = min(min_dist, d)
                            if d < 0.60:
                                close_samples += 1
        result["min_inter_uav_distance"] = round(min_dist, 4)
        result["close_samples_under_0.6m"] = close_samples
        if min_dist <= 0.80:
            ok = False
            reasons.append("min dist %.3f" % min_dist)

        # Static obstacle clearance.
        obs_min = 1e9
        obs_collision = 0
        for i in range(3):
            pts = odom[i]
            if len(pts) < 10:
                continue
            xy = np.array([[p[1], p[2]] for p in pts])
            d = min_obstacle_distances(xy)
            obs_min = min(obs_min, float(d.min()))
            obs_collision += int((d < COLLISION_MARGIN).sum())
        result["min_obstacle_distance"] = round(obs_min, 4)
        result["obstacle_collision_samples"] = obs_collision
        if obs_collision > 0:
            ok = False
            reasons.append("obstacle collision %d" % obs_collision)

        # Mode / allocator / residual stats (phase_offset stages).
        if phase_offset:
            emerg = 0
            rolling = 0
            terminal = 0
            safety = 0
            res_max = 0.0
            e_perp_all = []
            jump_max = 0.0
            alloc_ms_max = 0.0
            for i in range(3):
                for (_, mode, res, e_perp, _delta, jump, ams) in debug[i]:
                    if mode == 4:
                        emerg += 1
                    elif mode == 1:
                        rolling += 1
                    elif mode == 3:
                        terminal += 1
                    elif mode == 2:
                        safety += 1
                    res_max = max(res_max, res)
                    e_perp_all.append(e_perp)
                    jump_max = max(jump_max, jump)
                    alloc_ms_max = max(alloc_ms_max, ams)
            n_mode = emerg + rolling + terminal + safety
            result["mode_counts"] = {
                "emergency": emerg, "rolling": rolling,
                "terminal": terminal, "safety_priority": safety,
            }
            result["matched_residual_max"] = res_max
            result["e_perp_mean"] = round(float(np.mean(e_perp_all)), 4) \
                if e_perp_all else None
            result["e_perp_max"] = round(float(np.max(e_perp_all)), 4) \
                if e_perp_all else None
            result["delta_jump_max"] = jump_max
            result["allocator_ms_max"] = round(alloc_ms_max, 4)
            if emerg > 0:
                ok = False
                reasons.append("emergency %d" % emerg)
            if rolling == 0 and n_mode > 0:
                ok = False
                reasons.append("no rolling")
            if stage >= 1 and terminal == 0:
                ok = False
                reasons.append("no terminal mode")
            if n_mode > 0 and safety / n_mode > 0.05:
                ok = False
                reasons.append("safety ratio %.2f" % (safety / n_mode))
            if res_max > 1e-8:
                ok = False
                reasons.append("residual %.2e" % res_max)
            if result["e_perp_mean"] is not None and \
               result["e_perp_mean"] > 0.15:
                ok = False
                reasons.append("e_perp mean %.3f" % result["e_perp_mean"])
            if result["e_perp_max"] is not None and \
               result["e_perp_max"] > 0.50:
                ok = False
                reasons.append("e_perp max %.3f" % result["e_perp_max"])
            if jump_max > 1e-6:
                ok = False
                reasons.append("delta jump %.2e" % jump_max)

        # Communication rates.
        rates = {}
        for i in range(3):
            ts = [p[0] for p in odom[i]]
            rates[i] = round((len(ts) - 1) /
                             max(1e-6, ts[-1] - ts[0]), 1) \
                if len(ts) > 2 else 0.0
            if rates[i] < 90.0:
                ok = False
                reasons.append("odom rate uav%d %.1f" % (i, rates[i]))
        result["odom_rates_hz"] = rates
        if swarm:
            st_rate = round((len(state_ts) - 1) /
                            max(1e-6, state_ts[-1] - state_ts[0]), 1) \
                if len(state_ts) > 2 else 0.0
            result["swarm_state_rate_hz"] = st_rate
            if not (20.0 <= st_rate <= 30.0):
                ok = False
                reasons.append("state rate %.1f" % st_rate)

        # Goal receive count from the launch log.
        try:
            with open(launch.log_path) as f:
                text = f.read()
            total = len(re.findall(r"receive goal \(([0-9.\-]+), "
                                   r"([0-9.\-]+), ([0-9.\-]+)\)", text))
            result["goal_receive_lines_total"] = total
            if total != 3:
                ok = False
                reasons.append("goal receive lines %d" % total)
        except Exception:
            result["goal_receive_lines_total"] = None

        # Processes alive during the run.
        alive = True
        for i in range(3):
            try:
                subprocess.check_output(
                    ["pgrep", "-f",
                     "formation_planning.*uav_%d" % i],
                    stderr=subprocess.DEVNULL)
            except subprocess.CalledProcessError:
                alive = False
        result["processes_alive"] = alive
        if not alive:
            ok = False
            reasons.append("process died")

        result["reasons"] = reasons
        result["ok"] = ok
        return result
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
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", type=int, default=2)
    ap.add_argument("--runs", type=int, default=1)
    ap.add_argument("--tag", default="p2p")
    args = ap.parse_args()
    os.makedirs(OUT_DIR, exist_ok=True)

    runs = []
    for r in range(args.runs):
        res = run_once(args.stage, r, args.tag)
        path = os.path.join(OUT_DIR, "%s_run%02d.json" % (args.tag, r))
        with open(path, "w") as f:
            json.dump(res, f, indent=2, sort_keys=True)
        runs.append(res)
        print("[run %d] ok=%s %s" %
              (r, res.get("ok"), json.dumps(res, sort_keys=True)))

    summary = {
        "tag": args.tag,
        "stage": args.stage,
        "runs": args.runs,
        "passed": sum(1 for r in runs if r.get("ok")),
        "runs_detail": runs,
    }
    with open(os.path.join(OUT_DIR, "%s_summary.json" % args.tag),
              "w") as f:
        json.dump(summary, f, indent=2, sort_keys=True)
    print("SUMMARY %s/%s passed" % (summary["passed"], args.runs))
    return 0 if summary["passed"] == args.runs else 1


if __name__ == "__main__":
    sys.exit(main())
