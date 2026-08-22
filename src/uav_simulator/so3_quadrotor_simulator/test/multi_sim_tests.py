#!/usr/bin/env python3
"""ROS runtime verification for multi_quadrotor_simulator_so3.

Scenarios:
  smoke-single       A: original simulator.launch starts and publishes odom
  baseline-vs-single A+: baseline binary vs refactored single binary, same input
  multi-vs-single    B: multi simulator (num_agents=1) vs single simulator
  multi-n            C+D: N=3 / N=7, exactly one simulator process, all odoms
                          alive, and every UAV holds its YAML initial state
                          (finite state, |pos - init| < 2 cm, z > 0.95 m,
                          yaw == YAML yaw, unique child frame, no leading /)
  isolation          E: command on /uav_0 only must not affect other UAVs
  timeout            F: a clearly non-hover command is stopped -> after the
                        timeout the failsafe no longer executes the old
                        command; a new command exits the failsafe; other UAVs
                        keep integrating

Every scenario starts its own roscore on :11311 and kills everything it
started.  The script refuses to run against a master it did not start.

Run with a sourced ROS environment:
  source /opt/ros/noetic/setup.bash && source devel/setup.bash
  python3 src/uav_simulator/so3_quadrotor_simulator/test/multi_sim_tests.py <scenario> [args]
"""

import argparse
import bisect
import json
import math
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time

import yaml

import rospy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import AuxCommand, SO3Command
from sensor_msgs.msg import Imu

MASTER_URI = "http://localhost:11311"
LOG_DIR = "/tmp/gvf_ws_ros_log"
WS = "/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws"
PKG = "so3_quadrotor_simulator"
CONFIG_DIR = os.path.join(WS, "src/uav_simulator/so3_quadrotor_simulator/config")

# The test script manages its own roscore; make sure it (and all child
# processes) talk to that master and write logs to a writable location.
os.environ["ROS_MASTER_URI"] = MASTER_URI
os.environ["ROS_LOG_DIR"] = LOG_DIR


def env():
    return dict(os.environ)


class ManagedProc:
    def __init__(self, argv, logname):
        self.argv = argv
        os.makedirs(LOG_DIR, exist_ok=True)
        self.log = open(os.path.join(LOG_DIR, logname + ".log"), "a")
        self.proc = subprocess.Popen(
            argv, env=env(), stdout=self.log, stderr=subprocess.STDOUT,
            start_new_session=True)

    def alive(self):
        return self.proc.poll() is None

    def stop(self):
        if self.proc.poll() is None:
            # roscore spawns a child rosmaster that binds the port; kill the
            # whole process group so no orphan master survives.
            os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
            try:
                self.proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
                self.proc.wait(timeout=5)
        self.log.close()


class ProcGroup:
    def __init__(self):
        self.procs = []

    def add(self, argv, logname):
        p = ManagedProc(argv, logname)
        self.procs.append(p)
        return p

    def stop_all(self):
        for p in reversed(self.procs):
            p.stop()


def wait_master(timeout=25.0):
    """Poll for the master without requiring rospy.init_node()."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            rospy.get_master().getPid()
            return True
        except Exception:
            time.sleep(0.2)
    return False


def roscore_ok(group):
    """Our roscore process must be alive AND the master must respond."""
    return group.procs and group.procs[0].alive() and wait_master(timeout=10.0)


def stop_group(group):
    """Stop everything and wait until port 11311 is really free."""
    group.stop_all()
    t_end = time.time() + 10.0
    while time.time() < t_end:
        if not _port_open():
            return
        # An orphaned rosmaster (started by us) may survive its roscore
        # wrapper; kill whatever owns the port.
        pid = _port_owner_pid()
        if pid is not None:
            try:
                os.kill(pid, signal.SIGTERM)
            except OSError:
                pass
        time.sleep(0.2)


def _port_open():
    s = socket.socket()
    s.settimeout(0.3)
    try:
        s.connect(("127.0.0.1", 11311))
        s.close()
        return True
    except Exception:
        return False


def _port_owner_pid():
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


_rospy_inited = False


def ensure_rospy_node():
    """rospy.init_node() must run AFTER the roscore we manage is up."""
    global _rospy_inited
    if not _rospy_inited:
        rospy.init_node("multi_sim_tests", anonymous=True)
        _rospy_inited = True


def wait_topic(topic, timeout=25.0, msg_type=Odometry):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            rospy.wait_for_message(topic, msg_type, timeout=1.0)
            return True
        except Exception:
            time.sleep(0.2)
    return False


class OdomSampler:
    def __init__(self, topic):
        self.topic = topic
        self.data = []
        self.lock = threading.Lock()
        self.sub = rospy.Subscriber(topic, Odometry, self._cb, queue_size=5000)

    def _cb(self, msg):
        with self.lock:
            self.data.append((time.time(), msg))

    def snapshot(self):
        with self.lock:
            return list(self.data)


def _quat_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


def make_cmd(phase, mode, yaw=0.0):
    """Build a SO3Command.

    Horizontal acceleration in this controller comes from TILTING the body
    (a world-frame horizontal force component does nothing while the body is
    level, because getControl projects the force onto body z).  The tilt
    modes therefore command a pitched desired orientation and keep the
    vertical thrust at mg/cos(pitch).  The base orientation matches the
    agent's hover yaw so commands do not fight the initial attitude.
    """
    mg = 0.98 * 9.81
    cmd = SO3Command()
    cmd.header.stamp = rospy.Time.now()
    q_yaw = (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))
    if mode == "hover":
        q = q_yaw
        fx, fy, fz = 0.0, 0.0, mg
    elif mode == "tilt_x":
        pitch = 0.15
        q_pitch = (math.cos(pitch / 2.0), 0.0, math.sin(pitch / 2.0), 0.0)
        q = _quat_mult(q_yaw, q_pitch)
        fx, fy, fz = 0.0, 0.0, mg / math.cos(pitch)
    elif mode == "tilt_negx":
        pitch = -0.15
        q_pitch = (math.cos(pitch / 2.0), 0.0, math.sin(pitch / 2.0), 0.0)
        q = _quat_mult(q_yaw, q_pitch)
        fx, fy, fz = 0.0, 0.0, mg / math.cos(pitch)
    elif mode == "tilt_ramp":
        # Monotonic ramp to a constant pitch.  Used by the equivalence tests:
        # oscillating pitch commands put the open-loop SO3 plant close to a
        # parametric resonance, where sub-ms command-phase differences decide
        # which branch a run takes (a property of the plant, not of the
        # simulator implementation).
        pitch = 0.04 * min(1.0, phase / 3.0)
        q_pitch = (math.cos(pitch / 2.0), 0.0, math.sin(pitch / 2.0), 0.0)
        q = _quat_mult(q_yaw, q_pitch)
        fx, fy, fz = 0.0, 0.0, mg / math.cos(pitch)
    elif mode == "osc":
        # Gentle pitch oscillation only (no yaw modulation).  The
        # mg/cos(pitch) vertical compensation is only exact when the actual
        # pitch equals the commanded pitch; larger amplitudes make the drone
        # slowly descend and the descent rate becomes extremely sensitive to
        # sub-ms command-phase differences between two runs (a property of
        # the open-loop SO3 plant, not of the simulator implementation).
        pitch = 0.02 * math.sin(2.0 * math.pi * 0.2 * phase)
        q_pitch = (math.cos(pitch / 2.0), 0.0, math.sin(pitch / 2.0), 0.0)
        q = _quat_mult(q_yaw, q_pitch)
        fx, fy, fz = 0.0, 0.0, mg / math.cos(pitch)
    else:
        raise ValueError(mode)
    cmd.force.x = fx
    cmd.force.y = fy
    cmd.force.z = fz
    cmd.orientation.w = q[0]
    cmd.orientation.x = q[1]
    cmd.orientation.y = q[2]
    cmd.orientation.z = q[3]
    cmd.kR[0] = 3.0
    cmd.kR[1] = 3.0
    cmd.kR[2] = 3.0
    cmd.kOm[0] = 0.1
    cmd.kOm[1] = 0.1
    cmd.kOm[2] = 0.1
    cmd.aux.current_yaw = 0.0
    cmd.aux.kf_correction = 0.0
    cmd.aux.angle_corrections[0] = 0.0
    cmd.aux.angle_corrections[1] = 0.0
    cmd.aux.enable_motors = True
    cmd.aux.use_external_yaw = False
    return cmd


def publish_commands(topics, duration, rate, mode, t0_holder, stop_event,
                     phase_offset=0.0, yaw=0.0):
    """Publish commands phase-locked to t0_holder[0].

    The publisher first waits until every subscriber connection is
    established, then (if the holder is still None) records the command-phase
    epoch t0 = now.  All command phases are measured from that epoch, so two
    runs compared by phase see identical inputs regardless of startup or
    connection latencies.
    """
    pubs = [rospy.Publisher(t, SO3Command, queue_size=10) for t in topics]
    t_end = time.time() + 15.0
    while time.time() < t_end:
        if all(p.get_num_connections() > 0 for p in pubs):
            break
        time.sleep(0.05)
    if t0_holder[0] is None:
        t0_holder[0] = time.time()
    # Anchor the publish cadence to t0 (create the Rate AFTER the epoch is
    # fixed); otherwise the connection-wait duration shifts the 100 Hz grid
    # relative to the command values and the two runs see different streams.
    r = rospy.Rate(rate)
    while True:
        elapsed = time.time() - t0_holder[0]
        if elapsed >= phase_offset + duration or stop_event.is_set():
            break
        if elapsed >= phase_offset:
            for p in pubs:
                p.publish(make_cmd(elapsed, mode, yaw=yaw))
        r.sleep()


def run_plant(group, binary, node_name, odom_topic="/sim/odom",
              cmd_topic="/so3_cmd", odom_rate=100.0):
    # The node uses a private NodeHandle, so remaps must be private (~...).
    argv = [
        binary,
        "__name:=" + node_name,
        "~odom:=" + odom_topic,
        "~cmd:=" + cmd_topic,
        "~imu:=/sim/imu",
        "_simulator/init_state_x:=0.0",
        "_simulator/init_state_y:=0.0",
        "_simulator/init_state_z:=1.0",
        "_simulator/start_at_hover:=true",
        "_rate/simulation:=1000.0",
        "_rate/odom:=%.1f" % odom_rate,
    ]
    return group.add(argv, node_name)


def run_multi(group, n, extra=(), odom_rate=100.0):
    argv = [
        "roslaunch", PKG, "multi_simulator.launch",
        "num_agents:=" + str(n),
        "start_at_hover:=true",
        "simulation_rate:=1000.0",
        "odom_rate:=%.1f" % odom_rate,
    ] + list(extra)
    return group.add(argv, "multi_" + str(n))


def read_initial_states(n):
    path = os.path.join(CONFIG_DIR, "initial_states_%d.yaml" % n)
    return read_initial_states_file(path, n)


def read_initial_states_file(path, n):
    with open(path) as f:
        data = yaml.safe_load(f)
    states = {}
    for item in data["initial_states"]:
        states[item["robot_id"]] = item
    out = []
    for i in range(n):
        s = states.get(i)
        if s is None:
            raise RuntimeError("initial_states_%d.yaml missing robot %d" % (n, i))
        out.append(s)
    return path, out


def odom_vecs(rec):
    return (rec[1].pose.pose.position.x, rec[1].pose.pose.position.y,
            rec[1].pose.pose.position.z)


def odom_vels(rec):
    return (rec[1].twist.twist.linear.x, rec[1].twist.twist.linear.y,
            rec[1].twist.twist.linear.z)


def odom_quat(rec):
    q = rec[1].pose.pose.orientation
    return (q.x, q.y, q.z, q.w)


def quat_to_yaw(q):
    return math.atan2(2.0 * (q[3] * q[2] + q[0] * q[1]),
                      1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]))


def vec_diff(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def quat_angle(a, b):
    dot = abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3])
    dot = min(1.0, max(-1.0, dot))
    return 2.0 * math.acos(dot)


def _lerp(a, b, s):
    return tuple(x + (y - x) * s for x, y in zip(a, b))


def _slerp(qa, qb, s):
    dot = qa[0] * qb[0] + qa[1] * qb[1] + qa[2] * qb[2] + qa[3] * qb[3]
    if dot < 0.0:
        qb = tuple(-x for x in qb)
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:  # nearly parallel: lerp + normalize
        q = _lerp(qa, qb, s)
    else:
        theta = math.acos(dot)
        sa = math.sin((1.0 - s) * theta) / math.sin(theta)
        sb = math.sin(s * theta) / math.sin(theta)
        q = tuple(sa * x + sb * y for x, y in zip(qa, qb))
    n = math.sqrt(sum(x * x for x in q))
    return tuple(x / n for x in q)


def track_phase(records, t0):
    """Records -> sorted (phase, pos, vel, quat) list, phase = stamp - t0."""
    out = []
    for rec in records:
        phase = rec[1].header.stamp.to_sec() - t0
        out.append((phase, odom_vecs(rec), odom_vels(rec), odom_quat(rec)))
    out.sort(key=lambda x: x[0])
    return out


def compare_tracks(a_records, b_records, t0a, t0b,
                   tol_pos=2e-2, tol_vel=2e-2, tol_ang=1e-2,
                   max_gap=0.02):
    """Compare two tracks aligned by command phase (odom header stamps)."""
    A = [x for x in track_phase(a_records, t0a) if x[0] >= -0.05]
    B = [x for x in track_phase(b_records, t0b) if x[0] >= -0.05]
    if len(A) < 100 or len(B) < 100:
        return {"ok": False,
                "reason": "too few samples: A=%d B=%d" % (len(A), len(B))}
    phases_b = [x[0] for x in B]
    max_pos = 0.0
    max_vel = 0.0
    max_ang = 0.0
    n = 0
    worst_points = []
    worst_att = []
    for pa, posa, vela, qa in A:
        if pa < phases_b[0] or pa > phases_b[-1]:
            continue
        j = bisect.bisect_left(phases_b, pa)
        if j == 0:
            continue
        t0b, t1b = phases_b[j - 1], phases_b[j]
        if t1b - t0b <= 0.0:
            continue
        s = (pa - t0b) / (t1b - t0b)
        posb = _lerp(B[j - 1][1], B[j][1], s)
        velb = _lerp(B[j - 1][2], B[j][2], s)
        qb = _slerp(B[j - 1][3], B[j][3], s)
        max_pos = max(max_pos, vec_diff(posa, posb))
        worst_points.append((vec_diff(posa, posb), pa, posa, posb))
        max_vel = max(max_vel, vec_diff(vela, velb))
        ang = quat_angle(qa, qb)
        max_ang = max(max_ang, ang)
        worst_att.append((ang, pa, qa, qb))
        n += 1
    if n < 100:
        return {"ok": False, "reason": "too few matched samples: %d" % n}
    ok = max_pos <= tol_pos and max_vel <= tol_vel and max_ang <= tol_ang
    if os.environ.get("MST_DEBUG"):
        worst = sorted(worst_points, key=lambda x: -x[0])[:5]
        print("DEBUG worst pos errors:")
        for w in worst:
            print("  err=%.5f phase=%.3f posA=%s posB=%s"
                  % (w[0], w[1], w[2], w[3]))
        worsta = sorted(worst_att, key=lambda x: -x[0])[:5]
        print("DEBUG worst att errors:")
        for w in worsta:
            print("  err=%.5f phase=%.3f qA=%s qB=%s" % (w[0], w[1], w[2], w[3]))
    return {
        "ok": ok,
        "n_matched": n,
        "max_pos_err_m": max_pos,
        "max_vel_err_mps": max_vel,
        "max_att_err_rad": max_ang,
        "tol_pos_m": tol_pos,
        "tol_vel_mps": tol_vel,
        "tol_att_rad": tol_ang,
    }


def hz_of(samples):
    if len(samples) < 2:
        return 0.0
    dt = samples[-1][0] - samples[0][0]
    return (len(samples) - 1) / dt if dt > 0 else 0.0


def sample_window(samples, t0, ta, tb):
    return [s for s in samples if t0 + ta <= s[1].header.stamp.to_sec() <= t0 + tb]


def mean_accel_x(samples):
    """Mean x-acceleration (dv/dt over consecutive odom samples)."""
    if len(samples) < 3:
        return None
    acc = 0.0
    cnt = 0
    for i in range(len(samples) - 1):
        dt = samples[i + 1][1].header.stamp.to_sec() - \
             samples[i][1].header.stamp.to_sec()
        if dt <= 0:
            continue
        dv = samples[i + 1][1].twist.twist.linear.x - \
             samples[i][1].twist.twist.linear.x
        acc += dv / dt
        cnt += 1
    return acc / cnt if cnt > 0 else None


def scenario_smoke_single(args):
    """A: original simulator.launch compiles/launches and publishes odom."""
    group = ProcGroup()
    group.add(["roscore"], "roscore")
    if not roscore_ok(group):
        stop_group(group)
        return {"ok": False, "reason": "roscore did not come up"}
    ensure_rospy_node()
    time.sleep(1.0)
    group.add(["roslaunch", PKG, "simulator.launch"], "simulator_launch")
    time.sleep(15.0)

    odom_ok = wait_topic("/sim/odom", timeout=15.0)
    sampler = OdomSampler("/sim/odom")
    time.sleep(3.0)
    samples = sampler.snapshot()
    hz = hz_of(samples)

    nodes_out = subprocess.check_output(
        ["rosnode", "list"], env=env(), text=True)
    nodes = set(nodes_out.split())
    required = {"/quadrotor_simulator_so3", "/so3_control", "/map_pub",
                "/odom_visualization", "/rviz", "/rviz1"}
    missing = sorted(required - nodes)
    alive = any(
        p.alive() for p in group.procs if p.argv[0] == "roslaunch")

    result = {
        "ok": odom_ok and hz > 80.0 and not missing and alive,
        "odom_hz": round(hz, 1),
        "missing_nodes": missing,
        "roslaunch_alive": alive,
        "nodes": sorted(nodes),
    }
    stop_group(group)
    return result


def _single_equivalence(binary_a, name_a, binary_b, name_b):
    """A+: run two single-plant binaries simultaneously with one command
    stream (identical binaries must produce identical states)."""
    group = ProcGroup()
    group.add(["roscore"], "roscore")
    if not roscore_ok(group):
        stop_group(group)
        return {"ok": False, "reason": "roscore did not come up"}
    ensure_rospy_node()
    time.sleep(1.0)

    run_plant(group, binary_a, name_a, odom_topic="/sim/odom_a",
              cmd_topic="/so3_cmd_a", odom_rate=1000.0)
    run_plant(group, binary_b, name_b, odom_topic="/sim/odom_b",
              cmd_topic="/so3_cmd_b", odom_rate=1000.0)
    if not wait_topic("/sim/odom_a", timeout=20.0):
        stop_group(group)
        return {"ok": False, "reason": name_a + " no odom"}
    if not wait_topic("/sim/odom_b", timeout=20.0):
        stop_group(group)
        return {"ok": False, "reason": name_b + " no odom"}
    time.sleep(1.5)  # let both hover states settle
    result = _simultaneous_equivalence(
        (group, name_a, "/sim/odom_a", "/so3_cmd_a",
         name_b, "/sim/odom_b", "/so3_cmd_b"))
    stop_group(group)
    return result


def _simultaneous_equivalence(plants, duration=8.0):
    """Drive two plants with the same command stream at the same wall times.

    Sequential runs are NOT suitable for this comparison: the 100 Hz command
    staircase randomly excites a slow attitude mode of the open-loop SO3
    plant, so two sequential runs differ by up to ~6 cm even for the same
    binary.  With both plants driven by the same messages at the same wall
    times, the comparison reflects the implementations only.
    """
    (group, name_a, topic_a, cmd_a, name_b, topic_b, cmd_b) = plants
    sam_a = OdomSampler(topic_a)
    sam_b = OdomSampler(topic_b)
    holder = [None]
    stop_event = threading.Event()
    th = threading.Thread(target=publish_commands,
                          args=([cmd_a, cmd_b], duration, 100.0,
                                "tilt_ramp", holder, stop_event))
    th.start()
    th.join()
    t0 = holder[0]
    rec_a = sam_a.snapshot()
    rec_b = sam_b.snapshot()
    result = compare_tracks(rec_a, rec_b, t0, t0)
    result["n_" + name_a] = len(rec_a)
    result["n_" + name_b] = len(rec_b)
    return result


def scenario_baseline_vs_single(args):
    """A+: refactor must not change single-machine behavior."""
    baseline = "/tmp/quadrotor_simulator_so3_baseline"
    refactored = os.path.join(WS, "devel/lib/so3_quadrotor_simulator",
                              "quadrotor_simulator_so3")
    if not os.path.exists(baseline):
        return {"ok": False, "reason": "baseline binary missing: " + baseline}
    return _single_equivalence(baseline, "quadrotor_simulator_so3_baseline",
                               refactored, "quadrotor_simulator_so3")


def scenario_multi_vs_single(args):
    """B: num_agents=1 multi simulator == single simulator under the same
    simultaneous command stream."""
    single_bin = os.path.join(WS, "devel/lib/so3_quadrotor_simulator",
                              "quadrotor_simulator_so3")
    state_file, _ = read_initial_states(1)
    group = ProcGroup()
    group.add(["roscore"], "roscore")
    if not roscore_ok(group):
        stop_group(group)
        return {"ok": False, "reason": "roscore did not come up"}
    ensure_rospy_node()
    time.sleep(1.0)

    # Single simulator and multi simulator (num_agents=1), same initial state
    # (0,0,1), same rates, start_at_hover=true, long command timeout, driven
    # simultaneously by the same command stream.
    run_plant(group, single_bin, "quadrotor_simulator_so3",
              odom_topic="/sim/odom_a", cmd_topic="/so3_cmd_a",
              odom_rate=1000.0)
    multi = run_multi(group, 1, extra=["command_timeout:=5.0",
                                       "initial_state_file:=" + state_file],
                      odom_rate=1000.0)
    if not wait_topic("/sim/odom_a", timeout=20.0):
        stop_group(group)
        return {"ok": False, "reason": "single no odom"}
    if not wait_topic("/uav_0/sim/odom", timeout=20.0):
        stop_group(group)
        return {"ok": False, "reason": "multi no odom"}
    time.sleep(1.5)
    result = _simultaneous_equivalence(
        (group, "single", "/sim/odom_a", "/so3_cmd_a",
         "multi", "/uav_0/sim/odom", "/uav_0/so3_cmd"))
    stop_group(group)
    return result


def check_agent_state(records, expected, frame_ids, results, topic):
    """Finite state, initial-position/yaw hold, z>0.95, unique child frame."""
    if not records:
        results[topic] = "no samples"
        return False
    all_finite = all(
        math.isfinite(v)
        for rec in records
        for v in odom_vecs(rec) + odom_vels(rec) + odom_quat(rec))
    last = records[-1]
    pos = odom_vecs(last)
    q = odom_quat(last)
    exp_pos = (expected["x"], expected["y"], expected["z"])
    pos_err = vec_diff(pos, exp_pos)
    z = pos[2]
    yaw_err = abs(quat_to_yaw(q) - expected["yaw"])
    yaw_err = min(yaw_err, 2.0 * math.pi - yaw_err)
    child = last[1].child_frame_id
    expected_child = expected["name"] + "/base_link"
    ok = (all_finite and pos_err < 0.02 and z > 0.95 and
          yaw_err < 0.05 and child == expected_child)
    results[topic] = {
        "finite": all_finite,
        "pos_err_m": round(pos_err, 4),
        "z_m": round(z, 3),
        "yaw_err_rad": round(yaw_err, 4),
        "child_frame": child,
    }
    if child in frame_ids:
        results[topic]["child_frame_duplicate"] = True
        ok = False
    frame_ids.add(child)
    return ok


def scenario_multi_n(args):
    """C+D: N=3 / N=7, one process, all odoms live, UAVs hold their state."""
    n = args.n
    state_file, expected = read_initial_states(n)
    group = ProcGroup()
    group.add(["roscore"], "roscore")
    if not roscore_ok(group):
        stop_group(group)
        return {"ok": False, "reason": "roscore did not come up"}
    ensure_rospy_node()
    time.sleep(1.0)
    multi = run_multi(group, n, extra=["command_timeout:=5.0",
                                       "initial_state_file:=" + state_file])

    hzs = {}
    state_results = {}
    frame_ids = set()
    state_ok = True
    for i in range(n):
        topic = "/uav_%d/sim/odom" % i
        ok = wait_topic(topic, timeout=20.0)
        imu_ok = wait_topic("/uav_%d/sim/imu" % i, timeout=10.0,
                            msg_type=Imu)
        if not ok or not imu_ok:
            hzs[topic] = 0.0
            state_ok = False
            state_results[topic] = "topic/im u missing"
            continue
        sampler = OdomSampler(topic)
        time.sleep(3.0)
        records = sampler.snapshot()
        hzs[topic] = hz_of(records)
        state_ok = check_agent_state(records, expected[i], frame_ids,
                                     state_results, topic) and state_ok

    time.sleep(1.0)
    nodes_out = subprocess.check_output(
        ["rosnode", "list"], env=env(), text=True)
    node_count = sum(
        1 for line in nodes_out.splitlines() if line.strip() ==
        "/multi_quadrotor_simulator_so3")
    ps_out = subprocess.check_output(["ps", "-eo", "args"], text=True)
    proc_count = sum(
        1 for line in ps_out.splitlines()
        if "multi_quadrotor_simulator_so3" in line
        and "bash" not in line and "grep" not in line
        and "multi_sim_tests.py" not in line)

    alive = multi.alive()
    all_hz_ok = all(80.0 <= hzs[t] <= 130.0 for t in hzs)
    result = {
        "ok": state_ok and node_count == 1 and proc_count == 1 and
              all_hz_ok and alive,
        "n": n,
        "odom_hz": {t: round(h, 1) for t, h in hzs.items()},
        "state_checks": state_results,
        "multi_node_count": node_count,
        "multi_process_count": proc_count,
        "simulator_alive": alive,
    }
    stop_group(group)
    return result


def scenario_isolation(args):
    """E: command on /uav_0 only must not affect other UAVs."""
    state_file, _ = read_initial_states_file(
        os.path.join(CONFIG_DIR, "initial_states_dynamics.yaml"), 3)
    group = ProcGroup()
    group.add(["roscore"], "roscore")
    if not roscore_ok(group):
        stop_group(group)
        return {"ok": False, "reason": "roscore did not come up"}
    ensure_rospy_node()
    time.sleep(1.0)
    multi = run_multi(group, 3, extra=["command_timeout:=5.0",
                                       "initial_state_file:=" + state_file])
    for i in range(3):
        if not wait_topic("/uav_%d/sim/odom" % i, timeout=20.0):
            stop_group(group)
            return {"ok": False, "reason": "uav_%d no odom" % i}
    time.sleep(1.5)

    s1 = OdomSampler("/uav_1/sim/odom")
    s2 = OdomSampler("/uav_2/sim/odom")
    s0 = OdomSampler("/uav_0/sim/odom")
    time.sleep(1.0)
    base1 = s1.snapshot()
    base2 = s2.snapshot()
    base0 = s0.snapshot()

    t0_holder = [None]
    stop_event = threading.Event()
    th = threading.Thread(target=publish_commands,
                          args=(["/uav_0/so3_cmd"], 6.0, 100.0, "tilt_x",
                                t0_holder, stop_event))
    th.start()
    th.join()
    t0 = t0_holder[0]

    after1 = s1.snapshot()
    after2 = s2.snapshot()
    after0 = s0.snapshot()

    def max_disp(before, after):
        if not before or not after:
            return 0.0
        ref = odom_vecs(before[0])
        return max(vec_diff(odom_vecs(s), ref) for s in after)

    d0 = max_disp(base0, after0)
    d1 = max_disp(base1, after1)
    d2 = max_disp(base2, after2)
    ok = d0 > 0.1 and d1 < 1e-2 and d2 < 1e-2 and multi.alive()
    result = {
        "ok": ok,
        "uav0_max_disp_m": round(d0, 4),
        "uav1_max_disp_m": round(d1, 6),
        "uav2_max_disp_m": round(d2, 6),
        "simulator_alive": multi.alive(),
    }
    stop_group(group)
    return result


def scenario_timeout(args):
    """F: real timeout test with a non-hover command and recovery."""
    state_file, _ = read_initial_states_file(
        os.path.join(CONFIG_DIR, "initial_states_dynamics.yaml"), 3)
    group = ProcGroup()
    group.add(["roscore"], "roscore")
    if not roscore_ok(group):
        stop_group(group)
        return {"ok": False, "reason": "roscore did not come up"}
    ensure_rospy_node()
    time.sleep(1.0)
    multi = run_multi(group, 3, extra=["command_timeout:=1.0",
                                       "initial_state_file:=" + state_file])
    for i in range(3):
        if not wait_topic("/uav_%d/sim/odom" % i, timeout=20.0):
            stop_group(group)
            return {"ok": False, "reason": "uav_%d no odom" % i}
    time.sleep(1.5)

    s0 = OdomSampler("/uav_0/sim/odom")
    s1 = OdomSampler("/uav_1/sim/odom")
    s2 = OdomSampler("/uav_2/sim/odom")
    t0_holder = [None]
    stop_event = threading.Event()

    # uav_0: clearly non-hover tilted command (+x accel ~1.5 m/s^2) for 2.5 s,
    # then silence (timeout 1.0 s must stop executing the old command).
    th0 = threading.Thread(target=publish_commands,
                           args=(["/uav_0/so3_cmd"], 2.5, 100.0, "tilt_x",
                                 t0_holder, stop_event))
    # uav_1: oscillating command for the whole run.
    th1 = threading.Thread(target=publish_commands,
                           args=(["/uav_1/so3_cmd"], 12.0, 100.0, "osc",
                                 t0_holder, stop_event))
    th0.start()
    th1.start()
    th0.join()
    time.sleep(3.0)  # failsafe (timeout=1.0) engaged well before this

    # Recovery: a new, clearly different command must exit the failsafe.
    thr = threading.Thread(target=publish_commands,
                           args=(["/uav_0/so3_cmd"], 2.0, 100.0,
                                 "tilt_negx", t0_holder, stop_event),
                           kwargs={"phase_offset": 5.5})
    thr.start()
    thr.join()
    th1.join()
    time.sleep(1.0)
    t0 = t0_holder[0]

    all0 = s0.snapshot()
    all1 = s1.snapshot()
    all2 = s2.snapshot()

    a_cmd = mean_accel_x(sample_window(all0, t0, 0.5, 2.3))   # commanded
    a_fs = mean_accel_x(sample_window(all0, t0, 4.5, 5.3))    # failsafe
    a_rec = mean_accel_x(sample_window(all0, t0, 7.0, 8.5))   # recovery

    def window_max_disp(samples, ta, tb):
        win = sample_window(samples, t0, ta, tb)
        if len(win) < 2:
            return None
        ref = odom_vecs(win[0])
        return max(vec_diff(odom_vecs(s), ref) for s in win)

    d1_late = window_max_disp(all1, 5.0, 9.0)
    d1_early = window_max_disp(all1, 0.0, 2.3)
    d2_all = window_max_disp(all2, 0.0, 9.0)
    hz0 = hz_of(all0)
    hz1 = hz_of(all1)

    ok = (a_cmd is not None and a_fs is not None and a_rec is not None and
          a_cmd > 0.5 and a_fs < 0.3 and a_rec < -0.5 and
          d1_early is not None and d1_early > 0.1 and
          d1_late is not None and d1_late > 0.1 and
          d2_all is not None and d2_all < 1e-2 and
          hz0 > 80.0 and hz1 > 80.0 and multi.alive())
    result = {
        "ok": ok,
        "uav0_accel_while_commanded_mps2": (round(a_cmd, 3)
                                            if a_cmd is not None else None),
        "uav0_accel_after_timeout_mps2": (round(a_fs, 3)
                                          if a_fs is not None else None),
        "uav0_accel_after_recovery_mps2": (round(a_rec, 3)
                                           if a_rec is not None else None),
        "uav1_disp_while_commanded_m": (round(d1_early, 3)
                                        if d1_early is not None else None),
        "uav1_disp_after_uav0_timeout_m": (round(d1_late, 3)
                                           if d1_late is not None else None),
        "uav2_max_disp_m": (round(d2_all, 6)
                            if d2_all is not None else None),
        "uav0_odom_hz": round(hz0, 1),
        "uav1_odom_hz": round(hz1, 1),
        "simulator_alive": multi.alive(),
    }
    stop_group(group)
    return result


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="scenario")
    p = sub.add_parser("smoke-single")
    p = sub.add_parser("baseline-vs-single")
    p = sub.add_parser("multi-vs-single")
    p = sub.add_parser("multi-n")
    p.add_argument("--n", type=int, default=3)
    sub.add_parser("isolation")
    sub.add_parser("timeout")
    args = parser.parse_args()

    # Refuse to run against a master we did not start.
    try:
        rospy.get_master().getPid()
        print("A ROS master is already reachable at " + MASTER_URI +
              "; kill it first or change the port. Aborting.")
        return 2
    except Exception:
        pass

    if args.scenario == "smoke-single":
        result = scenario_smoke_single(args)
    elif args.scenario == "baseline-vs-single":
        result = scenario_baseline_vs_single(args)
    elif args.scenario == "multi-vs-single":
        result = scenario_multi_vs_single(args)
    elif args.scenario == "multi-n":
        result = scenario_multi_n(args)
    elif args.scenario == "isolation":
        result = scenario_isolation(args)
    elif args.scenario == "timeout":
        result = scenario_timeout(args)
    else:
        parser.print_help()
        return 2

    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
