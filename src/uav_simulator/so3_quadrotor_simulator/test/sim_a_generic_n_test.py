#!/usr/bin/python3
"""SIM-A generic-N physical simulator acceptance checks.

The test deliberately starts the same simulator executable for every case
and injects private parameters through the ROS parameter server.  This keeps
N, the state schema, and the optional compatibility parameter under test
without adding a per-agent launch/XML expansion.
"""

import copy
import json
import math
import os
import signal
import statistics
import subprocess
import sys
import threading
import time
import unittest

sys.dont_write_bytecode = True

import rospy
import rostest
import yaml
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

try:
    from quadrotor_msgs.msg import SO3Command
except ImportError:
    # A package-only catkin build can leave generated Python messages absent
    # from devel while the source package still provides them.
    workspace = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                             "../../../../"))
    message_source = os.path.join(
        workspace, "src/uav_simulator/Utils/quadrotor_msgs/src")
    sys.path.insert(0, message_source)
    for module_name in list(sys.modules):
        if (module_name == "quadrotor_msgs" or
                module_name.startswith("quadrotor_msgs.")):
            del sys.modules[module_name]
    from quadrotor_msgs.msg import SO3Command


PACKAGE = "so3_quadrotor_simulator"
EXECUTABLE = "multi_quadrotor_simulator_so3"
TEST_NAME = "sim_a_generic_n_test"
MASS = 0.98
GRAVITY = 9.81
HOVER_FORCE = MASS * GRAVITY
MISSING = object()


class MessageTrack:
    """Thread-safe message history used only for this test process."""

    def __init__(self, topic, message_type):
        self._lock = threading.Lock()
        self._records = []
        self._subscriber = rospy.Subscriber(
            topic, message_type, self._callback, queue_size=4000)

    def _callback(self, message):
        stamp = message.header.stamp.to_sec()
        with self._lock:
            self._records.append((stamp, time.monotonic(), message))

    def snapshot(self):
        with self._lock:
            return list(self._records)

    def close(self):
        self._subscriber.unregister()


def finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def finite_odom(message):
    values = [
        message.pose.pose.position.x,
        message.pose.pose.position.y,
        message.pose.pose.position.z,
        message.pose.pose.orientation.x,
        message.pose.pose.orientation.y,
        message.pose.pose.orientation.z,
        message.pose.pose.orientation.w,
        message.twist.twist.linear.x,
        message.twist.twist.linear.y,
        message.twist.twist.linear.z,
        message.twist.twist.angular.x,
        message.twist.twist.angular.y,
        message.twist.twist.angular.z,
    ]
    values.extend(message.pose.covariance)
    values.extend(message.twist.covariance)
    return all(finite(value) for value in values)


def finite_imu(message):
    values = [
        message.orientation.x,
        message.orientation.y,
        message.orientation.z,
        message.orientation.w,
        message.angular_velocity.x,
        message.angular_velocity.y,
        message.angular_velocity.z,
        message.linear_acceleration.x,
        message.linear_acceleration.y,
        message.linear_acceleration.z,
    ]
    values.extend(message.orientation_covariance)
    values.extend(message.angular_velocity_covariance)
    values.extend(message.linear_acceleration_covariance)
    return all(finite(value) for value in values)


def position(message):
    p = message.pose.pose.position
    return (p.x, p.y, p.z)


def quaternion(message):
    q = message.pose.pose.orientation
    value = (q.x, q.y, q.z, q.w)
    norm = math.sqrt(sum(component * component for component in value))
    if norm <= 0.0 or not finite(norm):
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(component / norm for component in value)


def yaw_from_quaternion(value):
    x, y, z, w = value
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def angular_distance(first, second):
    delta = abs(first - second) % (2.0 * math.pi)
    return min(delta, 2.0 * math.pi - delta)


def vector_distance(first, second):
    return math.sqrt(sum((a - b) * (a - b)
                         for a, b in zip(first, second)))


def header_rate(records):
    """Return rate from advancing ROS header timestamps only."""
    stamps = [record[0] for record in records]
    if len(stamps) < 2:
        return 0.0
    advancing = [(previous, current) for previous, current in zip(
        stamps, stamps[1:]) if finite(previous) and finite(current) and
        current > previous]
    if not advancing:
        return 0.0
    first = advancing[0][0]
    last = advancing[-1][1]
    if last <= first:
        return 0.0
    return float(len(advancing)) / (last - first)


def wait_for_count(track, count, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not rospy.is_shutdown():
        if len(track.snapshot()) >= count:
            return True
        time.sleep(0.02)
    return len(track.snapshot()) >= count


def wait_for_all_counts(tracks, count, timeout):
    """Wait for a set of streams against one shared wall-clock deadline."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not rospy.is_shutdown():
        if all(len(track.snapshot()) >= count for track in tracks):
            return True
        time.sleep(0.02)
    return all(len(track.snapshot()) >= count for track in tracks)


def wait_for_connections(publishers, timeout=15.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not rospy.is_shutdown():
        if all(pub.get_num_connections() > 0 for pub in publishers):
            return True
        time.sleep(0.05)
    return all(pub.get_num_connections() > 0 for pub in publishers)


def master_system_state():
    code, message, value = rospy.get_master().getSystemState()
    if code != 1:
        raise AssertionError("master getSystemState failed: %s" % message)
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise AssertionError("master getSystemState returned malformed value")
    return value[0], value[1], value[2]


def node_endpoint_sets(node_name):
    if not node_name.startswith("/"):
        node_name = "/" + node_name
    publications, subscriptions, services = master_system_state()
    published = set()
    subscribed = set()
    all_nodes = set()
    for topic, nodes in publications:
        all_nodes.update(nodes)
        if node_name in nodes:
            published.add(topic)
    for topic, nodes in subscriptions:
        all_nodes.update(nodes)
        if node_name in nodes:
            subscribed.add(topic)
    for _, nodes in services:
        all_nodes.update(nodes)
    return published, subscribed, all_nodes


def all_uav_topics():
    publications, subscriptions, _ = master_system_state()
    return {topic for topic, _ in publications + subscriptions
            if topic.startswith("/uav_")}


def expected_endpoints(n):
    published = set()
    subscribed = set()
    for index in range(n):
        prefix = "/uav_%d" % index
        published.update({prefix + "/sim/odom", prefix + "/sim/imu"})
        subscribed.update({prefix + "/so3_cmd",
                           prefix + "/force_disturbance",
                           prefix + "/moment_disturbance"})
    return published, subscribed


def assert_node_endpoints(testcase, node_name, n):
    published, subscribed, all_nodes = node_endpoint_sets(node_name)
    testcase.assertIn("/" + node_name.lstrip("/"), all_nodes)
    expected_published, expected_subscribed = expected_endpoints(n)
    testcase.assertEqual({topic for topic in published if topic.startswith("/uav_")},
                         expected_published)
    testcase.assertEqual({topic for topic in subscribed if topic.startswith("/uav_")},
                         expected_subscribed)
    testcase.assertEqual(len(expected_published | expected_subscribed), 5 * n)
    testcase.assertNotIn("/uav_%d/sim/odom" % n, published)
    testcase.assertNotIn("/uav_%d/sim/imu" % n, published)
    testcase.assertNotIn("/uav_%d/so3_cmd" % n, subscribed)


def load_fixture(filename):
    path = os.path.join(os.path.dirname(__file__), "../config", filename)
    with open(path, "r") as stream:
        data = yaml.safe_load(stream)
    return data["initial_states"]


def grid_states(n, metadata=False):
    """Deterministic 10-column grid used by the N=50 smoke."""
    output = []
    for index in range(n):
        state = {
            "robot_id": index,
            "x": float(2.0 * (index % 10)),
            "y": float(2.0 * (index // 10)),
            "z": 1.0,
            "yaw": 0.0,
        }
        if metadata:
            state["name"] = "uav_%d" % index
        output.append(state)
    return output


def copy_states(states):
    return copy.deepcopy(states)


def tilt_command(pitch):
    command = SO3Command()
    command.header.stamp = rospy.Time.now()
    half_pitch = pitch / 2.0
    command.force.x = 0.0
    command.force.y = 0.0
    command.force.z = HOVER_FORCE / math.cos(pitch)
    command.orientation.x = 0.0
    command.orientation.y = math.sin(half_pitch)
    command.orientation.z = 0.0
    command.orientation.w = math.cos(half_pitch)
    command.kR[0] = 3.0
    command.kR[1] = 3.0
    command.kR[2] = 3.0
    command.kOm[0] = 0.1
    command.kOm[1] = 0.1
    command.kOm[2] = 0.1
    command.aux.enable_motors = True
    return command


def hover_command():
    return tilt_command(0.0)


def delete_namespace(namespace):
    namespace = namespace.rstrip("/")
    for key in list(rospy.get_param_names()):
        if key == namespace or key.startswith(namespace + "/"):
            try:
                rospy.delete_param(key)
            except KeyError:
                pass


def set_private_params(node_name, states=MISSING, num_agents=MISSING,
                       simulation_rate=1000.0, odom_rate=100.0,
                       start_at_hover=True):
    namespace = "/" + node_name.lstrip("/")
    delete_namespace(namespace)
    if states is not MISSING:
        rospy.set_param(namespace + "/initial_states", states)
    if num_agents is not MISSING:
        rospy.set_param(namespace + "/num_agents", num_agents)
    rospy.set_param(namespace + "/frame_id", "world")
    rospy.set_param(namespace + "/rate/simulation", float(simulation_rate))
    rospy.set_param(namespace + "/rate/odom", float(odom_rate))
    rospy.set_param(namespace + "/start_at_hover", bool(start_at_hover))


class SimulatorProcess:
    def __init__(self, node_name, states=MISSING, num_agents=MISSING,
                 simulation_rate=1000.0, odom_rate=100.0):
        self.node_name = node_name
        self.namespace = "/" + node_name.lstrip("/")
        set_private_params(node_name, states, num_agents, simulation_rate,
                           odom_rate)
        command = ["rosrun", PACKAGE, EXECUTABLE,
                   "__name:=" + node_name.lstrip("/")]
        self.process = subprocess.Popen(
            command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True)

    def stop(self):
        process = self.process
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    process.wait(timeout=2.0)
        delete_namespace(self.namespace)

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback):
        self.stop()


def node_name_for(label):
    return "sim_a_%s_%d_%d" % (label, os.getpid(), int(time.time() * 1000))


def simulator_pids(node_name=None):
    result = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            executable_path = os.readlink(os.path.join("/proc", entry, "exe"))
        except (IOError, OSError):
            continue
        if os.path.basename(executable_path) != EXECUTABLE:
            continue
        path = os.path.join("/proc", entry, "cmdline")
        try:
            with open(path, "rb") as stream:
                command_line = stream.read().decode("utf-8", "ignore")
        except (IOError, OSError):
            continue
        arguments = command_line.replace("\x00", " ")
        if node_name is not None and ("__name:=" + node_name) not in arguments:
            continue
        result.append(int(entry))
    return result


def wait_for_endpoints(process, n, timeout=15.0):
    deadline = time.monotonic() + timeout
    expected_published, expected_subscribed = expected_endpoints(n)
    while time.monotonic() < deadline and not rospy.is_shutdown():
        if process.process.poll() is not None:
            return False
        published, subscribed, _ = node_endpoint_sets(process.node_name)
        if ({topic for topic in published if topic.startswith("/uav_")} ==
                expected_published and
                {topic for topic in subscribed if topic.startswith("/uav_")} ==
                expected_subscribed):
            return True
        time.sleep(0.05)
    return False


def process_sample(pid):
    """Read CPU ticks and RSS from /proc; return None if unavailable."""
    try:
        with open("/proc/%d/stat" % pid, "r") as stream:
            fields = stream.read().split()
        # utime/stime are fields 14/15 in procfs' one-based layout.
        cpu_ticks = int(fields[13]) + int(fields[14])
        with open("/proc/%d/status" % pid, "r") as stream:
            rss_kb = None
            for line in stream:
                if line.startswith("VmRSS:"):
                    rss_kb = int(line.split()[1])
                    break
        return {"cpu_ticks": cpu_ticks, "rss_kb": rss_kb}
    except (IOError, OSError, ValueError, IndexError):
        return None


def cpu_rss_report(pid, first, second, elapsed):
    if pid is None or first is None or second is None or elapsed <= 0.0:
        return {"available": False,
                "reason": "standard /proc sample unavailable"}
    hz = float(os.sysconf(os.sysconf_names["SC_CLK_TCK"]))
    cpu_percent = ((second["cpu_ticks"] - first["cpu_ticks"]) / hz /
                   elapsed * 100.0)
    return {"available": True,
            "cpu_percent": cpu_percent,
            "rss_kb": second["rss_kb"]}


class SimAGenericNTest(unittest.TestCase):
    def setUp(self):
        self.created_processes = []

    def tearDown(self):
        for process in self.created_processes:
            process.stop()
        self.created_processes = []

    def start(self, label, states=MISSING, num_agents=MISSING,
              simulation_rate=1000.0, odom_rate=100.0):
        process = SimulatorProcess(node_name_for(label), states, num_agents,
                                    simulation_rate, odom_rate)
        self.created_processes.append(process)
        return process

    def stop(self, process):
        process.stop()
        if process in self.created_processes:
            self.created_processes.remove(process)

    def make_tracks(self, n):
        odom = {index: MessageTrack("/uav_%d/sim/odom" % index, Odometry)
                for index in range(n)}
        imu = {index: MessageTrack("/uav_%d/sim/imu" % index, Imu)
               for index in range(n)}
        return odom, imu

    def close_tracks(self, odom, imu):
        for track in list(odom.values()) + list(imu.values()):
            track.close()

    def wait_ready(self, process, n, odom, imu, sample_count=250,
                   timeout=15.0):
        readiness_deadline = time.monotonic() + timeout
        remaining = max(0.0, readiness_deadline - time.monotonic())
        self.assertTrue(wait_for_endpoints(process, n, remaining),
                        "simulator endpoints did not become ready")
        all_tracks = list(odom.values()) + list(imu.values())
        remaining = max(0.0, readiness_deadline - time.monotonic())
        self.assertTrue(wait_for_all_counts(all_tracks, sample_count, remaining),
                        "one or more streams did not reach %d samples" %
                        sample_count)

    def assert_hover(self, states, odom, imu, minimum_hz=80.0,
                     maximum_hz=130.0):
        metrics = {}
        for state in states:
            index = int(state["robot_id"])
            records = odom[index].snapshot()
            imu_records = imu[index].snapshot()
            self.assertTrue(records)
            self.assertTrue(imu_records)
            self.assertTrue(all(finite_odom(record[2]) for record in records))
            self.assertTrue(all(finite_imu(record[2]) for record in imu_records))
            rate = header_rate(records)
            self.assertGreaterEqual(rate, minimum_hz)
            self.assertLessEqual(rate, maximum_hz)
            latest = records[-1][2]
            self.assertEqual(latest.child_frame_id,
                             "uav_%d/base_link" % index)
            self.assertEqual(latest.header.frame_id, "world")
            self.assertLess(vector_distance(position(latest),
                                             (float(state["x"]),
                                              float(state["y"]),
                                              float(state["z"]))), 0.020)
            self.assertLess(angular_distance(
                yaw_from_quaternion(quaternion(latest)), float(state["yaw"])),
                0.050)
            metrics[index] = {
                "odom_hz": rate,
                "odom_samples": len(records),
                "imu_samples": len(imu_records),
                "position": position(latest),
                "yaw": yaw_from_quaternion(quaternion(latest)),
            }
        return metrics

    def run_hover_case(self, label, states, num_agents, sample_count=250,
                       hover_seconds=3.0):
        process = self.start(label, states, num_agents)
        odom, imu = self.make_tracks(len(states))
        try:
            hover_start = time.monotonic()
            self.wait_ready(process, len(states), odom, imu, sample_count)
            self.assertEqual(len(simulator_pids(process.node_name)), 1)
            remaining = hover_seconds - (time.monotonic() - hover_start)
            if remaining > 0.0:
                time.sleep(remaining)
            metrics = self.assert_hover(states, odom, imu)
            assert_node_endpoints(self, process.node_name, len(states))
            self.assertEqual(all_uav_topics(),
                             expected_endpoints(len(states))[0] |
                             expected_endpoints(len(states))[1])
            return metrics
        finally:
            self.close_tracks(odom, imu)
            self.stop(process)

    def run_n8_isolation(self, label, states, num_agents=8):
        process = self.start(label, states, num_agents)
        odom, imu = self.make_tracks(8)
        command_publisher = rospy.Publisher("/uav_0/so3_cmd", SO3Command,
                                            queue_size=100)
        try:
            hover_start = time.monotonic()
            self.wait_ready(process, 8, odom, imu, 250, 15.0)
            self.assertEqual(len(simulator_pids(process.node_name)), 1)
            remaining = 3.0 - (time.monotonic() - hover_start)
            if remaining > 0.0:
                time.sleep(remaining)
            self.assertGreaterEqual(time.monotonic() - hover_start, 3.0)
            hover_metrics = self.assert_hover(states, odom, imu)

            before = {index: odom[index].snapshot() for index in range(8)}
            before_imu = {index: imu[index].snapshot() for index in range(8)}
            before_positions = {index: position(before[index][-1][2])
                                for index in range(8)}
            command = tilt_command(0.15)
            self.assertTrue(wait_for_connections([command_publisher]))
            start_wall = time.monotonic()
            rate = rospy.Rate(100.0)
            while (time.monotonic() - start_wall) < 4.0:
                command.header.stamp = rospy.Time.now()
                command_publisher.publish(command)
                rate.sleep()

            after = {index: odom[index].snapshot() for index in range(8)}
            displacements = [
                vector_distance(before_positions[index],
                                position(after[index][-1][2]))
                for index in range(8)]
            self.assertGreater(displacements[0], 0.10)
            for index in range(1, 8):
                self.assertLess(displacements[index], 0.020)
            command_liveness = {}
            for index in range(8):
                new_records = after[index][len(before[index]):]
                after_imu = imu[index].snapshot()
                new_imu = after_imu[len(before_imu[index]):]
                self.assertGreater(len(new_records), 0)
                self.assertGreater(len(new_imu), 0)
                self.assertGreater(new_records[-1][0], before[index][-1][0])
                self.assertGreater(after_imu[-1][0], before_imu[index][-1][0])
                self.assertTrue(all(finite_odom(record[2])
                                    for record in new_records))
                self.assertTrue(all(finite_imu(record[2]) for record in new_imu))
                phase_rate = header_rate(new_records)
                self.assertGreaterEqual(phase_rate, 80.0)
                command_liveness[index] = {
                    "samples": len(new_records),
                    "odom_hz": phase_rate,
                }
            assert_node_endpoints(self, process.node_name, 8)
            test_published, _, _ = node_endpoint_sets(rospy.get_name())
            self.assertEqual({topic for topic in test_published
                              if topic.startswith("/uav_")},
                             {"/uav_0/so3_cmd"})
            return {
                "hover": hover_metrics,
                "displacements": displacements,
                "command_liveness": command_liveness,
                "positions": [hover_metrics[index]["position"]
                              for index in range(8)],
                "yaws": [hover_metrics[index]["yaw"] for index in range(8)],
                "endpoint_published": sorted(
                    expected_endpoints(8)[0]),
                "endpoint_subscribed": sorted(
                    expected_endpoints(8)[1]),
                "command_pass": (displacements[0] > 0.10 and
                                  all(value < 0.020
                                      for value in displacements[1:])),
            }
        finally:
            command_publisher.unregister()
            self.close_tracks(odom, imu)
            self.stop(process)

    def run_n50(self, states):
        process = self.start("n50", states, 50)
        odom, imu = self.make_tracks(50)
        try:
            readiness_start = time.monotonic()
            readiness_deadline = readiness_start + 30.0
            remaining = max(0.0, readiness_deadline - time.monotonic())
            self.assertTrue(wait_for_endpoints(process, 50, remaining))
            all_tracks = list(odom.values()) + list(imu.values())
            remaining = max(0.0, readiness_deadline - time.monotonic())
            self.assertTrue(wait_for_all_counts(all_tracks, 20, remaining),
                            "one or more N=50 streams did not become live")
            readiness_wall_time = time.monotonic() - readiness_start
            self.assertIsNone(process.process.poll())
            self.assertEqual(len(simulator_pids(process.node_name)), 1)
            assert_node_endpoints(self, process.node_name, 50)
            expected_published, expected_subscribed = expected_endpoints(50)
            self.assertEqual(len(expected_published | expected_subscribed), 250)
            self.assertFalse(any(topic.startswith("/uav_50")
                                 for topic in all_uav_topics()))
            self.assertEqual(
                {int(topic.split("/uav_")[1].split("/")[0])
                 for topic in expected_published}, set(range(50)))
            for state in states:
                index = int(state["robot_id"])
                latest = odom[index].snapshot()[-1][2]
                self.assertEqual(latest.child_frame_id,
                                 "uav_%d/base_link" % index)
                self.assertLess(vector_distance(position(latest),
                                                 (state["x"], state["y"],
                                                  state["z"])), 0.020)

            before_odom = {index: odom[index].snapshot() for index in range(50)}
            before_imu = {index: imu[index].snapshot() for index in range(50)}
            simulator_pid = next(iter(simulator_pids(process.node_name)), None)
            first_proc = process_sample(simulator_pid)
            window_start = time.monotonic()
            time.sleep(5.0)
            elapsed = time.monotonic() - window_start
            second_proc = process_sample(simulator_pid)
            rates = []
            for index in range(50):
                records = odom[index].snapshot()
                imu_records = imu[index].snapshot()
                new_records = records[len(before_odom[index]):]
                new_imu = imu_records[len(before_imu[index]):]
                self.assertGreater(len(new_records), 0)
                self.assertGreater(len(new_imu), 0)
                self.assertGreater(records[-1][0], before_odom[index][-1][0])
                self.assertGreater(imu_records[-1][0], before_imu[index][-1][0])
                self.assertTrue(all(finite_odom(record[2])
                                    for record in new_records))
                self.assertTrue(all(finite_imu(record[2])
                                    for record in new_imu))
                rate = header_rate(new_records)
                self.assertGreaterEqual(rate, 80.0)
                self.assertLessEqual(rate, 130.0)
                rates.append(rate)
            self.assertIsNone(process.process.poll())
            self.assertTrue(wait_for_endpoints(process, 50, 1.0))
            rate_summary = {
                "min_hz": min(rates),
                "median_hz": statistics.median(rates),
                "max_hz": max(rates),
            }
            performance = cpu_rss_report(simulator_pid, first_proc,
                                         second_proc, elapsed)
            return {
                "n": 50,
                "simulation_rate_hz": 1000.0,
                "odom_rate_hz": 100.0,
                "readiness_wall_time_s": readiness_wall_time,
                "window_s": elapsed,
                "odom_rate": rate_summary,
                "cpu_rss": performance,
                "simulator_pid": simulator_pid,
                "process_alive": process.process.poll() is None,
                "publication_count": len(expected_published),
                "subscription_count": len(expected_subscribed),
                "endpoint_count": 250,
            }
        finally:
            self.close_tracks(odom, imu)
            self.stop(process)

    def run_invalid(self, label, states=MISSING, num_agents=MISSING):
        process = self.start("invalid_" + label, states, num_agents)
        try:
            deadline = time.monotonic() + 8.0
            while (time.monotonic() < deadline and
                   process.process.poll() is None):
                time.sleep(0.05)
            self.assertIsNotNone(process.process.poll(),
                                 "invalid case %s did not exit" % label)
            self.assertNotEqual(process.process.returncode, 0,
                                "invalid case %s exited successfully" % label)
            self.assertFalse(any(topic.startswith("/uav_")
                                 for topic in all_uav_topics()),
                             "invalid case %s created a plant endpoint" % label)
            self.assertEqual(node_endpoint_sets(process.node_name)[0], set())
            self.assertEqual(node_endpoint_sets(process.node_name)[1], set())
        finally:
            self.stop(process)

    def test_sim_a_generic_n(self):
        states7 = load_fixture("initial_states_7.yaml")
        states8 = load_fixture("initial_states_8.yaml")

        n7_metrics = self.run_hover_case("n7", states7, 7)
        plain = self.run_n8_isolation("n8_plain", states8, 8)
        metadata_states = copy_states(states8)
        for state in metadata_states:
            state["name"] = "uav_%d" % state["robot_id"]
        metadata = self.run_n8_isolation("n8_metadata", metadata_states, 8)

        self.assertEqual(plain["endpoint_published"],
                         metadata["endpoint_published"])
        self.assertEqual(plain["endpoint_subscribed"],
                         metadata["endpoint_subscribed"])
        for index in range(8):
            self.assertLess(vector_distance(plain["positions"][index],
                                             metadata["positions"][index]), 0.020)
            self.assertLess(angular_distance(plain["yaws"][index],
                                             metadata["yaws"][index]), 0.050)
        self.assertEqual(plain["command_pass"], metadata["command_pass"])

        # Absence of num_agents is a positive compatibility case.
        absent_metrics = self.run_hover_case("absent_num_agents", states8,
                                             MISSING, sample_count=250,
                                             hover_seconds=3.0)

        states50 = grid_states(50)
        n50_metrics = self.run_n50(states50)

        valid_states = grid_states(8)
        invalid_cases = [
            ("missing_initial_states", MISSING, MISSING),
            ("non_array_initial_states", {"robot_id": 0}, 1),
            ("empty_initial_states", [], MISSING),
            ("num_agents_string", valid_states, "8"),
            ("num_agents_double", valid_states, 8.0),
            ("num_agents_bool", valid_states, True),
            ("num_agents_mismatch", valid_states, 7),
        ]
        duplicate = grid_states(8)
        duplicate[7]["robot_id"] = 6
        invalid_cases.append(("duplicate_id", duplicate, 8))
        negative = grid_states(8)
        negative[0]["robot_id"] = -1
        invalid_cases.append(("negative_id", negative, 8))
        out_of_range = grid_states(8)
        out_of_range[7]["robot_id"] = 8
        invalid_cases.append(("out_of_range_id", out_of_range, 8))
        non_contiguous = grid_states(4)
        non_contiguous[2]["robot_id"] = 3
        non_contiguous[3]["robot_id"] = 4
        invalid_cases.append(("non_contiguous_id", non_contiguous, 4))
        missing_field = grid_states(8)
        del missing_field[0]["yaw"]
        invalid_cases.append(("missing_required_field", missing_field, 8))
        wrong_id_type = grid_states(8)
        wrong_id_type[0]["robot_id"] = "0"
        invalid_cases.append(("robot_id_string", wrong_id_type, 8))
        nonnumeric_x = grid_states(8)
        nonnumeric_x[0]["x"] = "zero"
        invalid_cases.append(("nonnumeric_x", nonnumeric_x, 8))
        for field in ("y", "z", "yaw"):
            nonnumeric = grid_states(8)
            nonnumeric[0][field] = "not-a-number"
            invalid_cases.append(("nonnumeric_" + field, nonnumeric, 8))
        nan_y = grid_states(8)
        nan_y[0]["y"] = float("nan")
        invalid_cases.append(("nonfinite_y", nan_y, 8))
        inf_x = grid_states(8)
        inf_x[0]["x"] = float("inf")
        invalid_cases.append(("nonfinite_x", inf_x, 8))
        for field in ("z", "yaw"):
            nonfinite = grid_states(8)
            nonfinite[0][field] = float("nan")
            invalid_cases.append(("nonfinite_" + field, nonfinite, 8))
        zero_z = grid_states(8)
        zero_z[0]["z"] = 0.0
        invalid_cases.append(("zero_z", zero_z, 8))
        negative_z = grid_states(8)
        negative_z[0]["z"] = -1.0
        invalid_cases.append(("negative_z", negative_z, 8))
        for label, states, num_agents in invalid_cases:
            self.run_invalid(label, states, num_agents)

        metrics = {
            "scenario": "sim_a_generic_n",
            "n7": n7_metrics,
            "n8_plain": plain,
            "n8_metadata": metadata,
            "n8_absent_num_agents": absent_metrics,
            "n50": n50_metrics,
            "invalid_cases": [label for label, _, _ in invalid_cases],
        }
        line = "SIM_A_GENERIC_N_METRICS " + json.dumps(metrics, sort_keys=True)
        print(line, flush=True)
        rospy.loginfo(line)


if __name__ == "__main__":
    rospy.init_node(TEST_NAME, anonymous=True)
    rostest.rosrun(PACKAGE, TEST_NAME, SimAGenericNTest)
