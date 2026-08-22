#!/usr/bin/python3
"""Focused B0 runtime checks for the isolated multi-plant executable."""

import bisect
import json
import math
import os
import sys
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

try:
    from quadrotor_msgs.msg import SO3Command
except ImportError:
    # A package-only catkin build can leave the generated Python message
    # files absent from devel while the source package still provides them.
    workspace = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "../../../../"))
    message_source = os.path.join(
        workspace, "src/uav_simulator/Utils/quadrotor_msgs/src")
    sys.path.insert(0, message_source)
    for module_name in list(sys.modules):
        if (module_name == "quadrotor_msgs" or
                module_name.startswith("quadrotor_msgs.")):
            del sys.modules[module_name]
    from quadrotor_msgs.msg import SO3Command


PACKAGE = "so3_quadrotor_simulator"
TEST_NAME = "b0_multi_simulator_test"
MASS = 0.98
GRAVITY = 9.81
HOVER_FORCE = MASS * GRAVITY


class MessageTrack:
    def __init__(self, topic, message_type):
        self._lock = threading.Lock()
        self._records = []
        self._subscriber = rospy.Subscriber(
            topic, message_type, self._callback, queue_size=2000)

    def _callback(self, message):
        stamp = message.header.stamp.to_sec()
        with self._lock:
            self._records.append((stamp, time.monotonic(), message))

    def snapshot(self):
        with self._lock:
            return list(self._records)


def wait_for_count(track, count, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not rospy.is_shutdown():
        if len(track.snapshot()) >= count:
            return True
        time.sleep(0.02)
    return len(track.snapshot()) >= count


def wait_for_connections(publishers, timeout=15.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not rospy.is_shutdown():
        if all(pub.get_num_connections() > 0 for pub in publishers):
            return True
        time.sleep(0.05)
    return all(pub.get_num_connections() > 0 for pub in publishers)


def finite(value):
    return math.isfinite(float(value))


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


def velocity(message):
    v = message.twist.twist.linear
    return (v.x, v.y, v.z)


def quaternion(message):
    q = message.pose.pose.orientation
    return normalize_quaternion((q.x, q.y, q.z, q.w))


def normalize_quaternion(value):
    norm = math.sqrt(sum(component * component for component in value))
    if norm <= 0.0 or not finite(norm):
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(component / norm for component in value)


def vector_distance(first, second):
    return math.sqrt(sum((a - b) * (a - b)
                         for a, b in zip(first, second)))


def quaternion_distance(first, second):
    dot = abs(sum(a * b for a, b in zip(first, second)))
    dot = min(1.0, max(-1.0, dot))
    return 2.0 * math.acos(dot)


def yaw_from_quaternion(value):
    x, y, z, w = value
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def angular_distance(first, second):
    delta = abs(first - second) % (2.0 * math.pi)
    return min(delta, 2.0 * math.pi - delta)


def interpolation(first, second, fraction):
    return tuple(a + fraction * (b - a) for a, b in zip(first, second))


def slerp(first, second, fraction):
    dot = sum(a * b for a, b in zip(first, second))
    if dot < 0.0:
        second = tuple(-component for component in second)
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        return normalize_quaternion(interpolation(first, second, fraction))
    theta = math.acos(dot)
    sine = math.sin(theta)
    left = math.sin((1.0 - fraction) * theta) / sine
    right = math.sin(fraction * theta) / sine
    return normalize_quaternion(tuple(left * a + right * b
                                      for a, b in zip(first, second)))


def command_for_elapsed(elapsed, pitch_amplitude=0.04):
    """Create a bounded, smooth, yaw-neutral positive-thrust command."""
    pitch = pitch_amplitude * math.sin(2.0 * math.pi * 0.18 * elapsed)
    half_pitch = pitch / 2.0
    command = SO3Command()
    command.header.stamp = rospy.Time.now()
    command.force.z = HOVER_FORCE / math.cos(pitch)
    command.orientation.w = math.cos(half_pitch)
    command.orientation.y = math.sin(half_pitch)
    command.kR[0] = 3.0
    command.kR[1] = 3.0
    command.kR[2] = 3.0
    command.kOm[0] = 0.1
    command.kOm[1] = 0.1
    command.kOm[2] = 0.1
    command.aux.enable_motors = True
    return command


def tilt_command(pitch):
    command = SO3Command()
    command.header.stamp = rospy.Time.now()
    half_pitch = pitch / 2.0
    command.force.z = HOVER_FORCE / math.cos(pitch)
    command.orientation.w = math.cos(half_pitch)
    command.orientation.y = math.sin(half_pitch)
    command.kR[0] = 3.0
    command.kR[1] = 3.0
    command.kR[2] = 3.0
    command.kOm[0] = 0.1
    command.kOm[1] = 0.1
    command.kOm[2] = 0.1
    command.aux.enable_motors = True
    return command


def hover_command():
    command = SO3Command()
    command.header.stamp = rospy.Time.now()
    command.force.z = HOVER_FORCE
    command.orientation.w = 1.0
    command.kR[0] = 3.0
    command.kR[1] = 3.0
    command.kR[2] = 3.0
    command.kOm[0] = 0.1
    command.kOm[1] = 0.1
    command.kOm[2] = 0.1
    command.aux.enable_motors = True
    return command


def publish_waveform(publishers, duration, pitch_amplitude=0.04,
                     publish_rate=100.0):
    if not wait_for_connections(publishers):
        raise AssertionError("plant command subscribers did not connect")
    start_wall = time.monotonic()
    start_ros = rospy.Time.now().to_sec()
    rate = rospy.Rate(publish_rate)
    while (time.monotonic() - start_wall) < duration:
        elapsed = time.monotonic() - start_wall
        command = command_for_elapsed(elapsed, pitch_amplitude)
        command.header.stamp = rospy.Time.from_sec(start_ros + elapsed)
        for publisher in publishers:
            publisher.publish(command)
        rate.sleep()
    return start_ros


def publish_fixed(publisher, command, duration, publish_rate=100.0):
    if not wait_for_connections([publisher]):
        raise AssertionError("plant command subscriber did not connect")
    deadline = time.monotonic() + duration
    rate = rospy.Rate(publish_rate)
    while time.monotonic() < deadline and not rospy.is_shutdown():
        command.header.stamp = rospy.Time.now()
        publisher.publish(command)
        rate.sleep()


def publish_disturbance(publisher, x_force, duration, publish_rate=100.0):
    if not wait_for_connections([publisher]):
        raise AssertionError("plant disturbance subscriber did not connect")
    message = Vector3()
    message.x = x_force
    deadline = time.monotonic() + duration
    rate = rospy.Rate(publish_rate)
    while time.monotonic() < deadline and not rospy.is_shutdown():
        publisher.publish(message)
        rate.sleep()
    message.x = 0.0
    for _ in range(20):
        publisher.publish(message)
        rate.sleep()


def emit_metrics(payload):
    line = "B0_METRICS " + json.dumps(payload, sort_keys=True)
    print(line, flush=True)
    rospy.loginfo(line)


def track_phase(records, origin):
    output = []
    for stamp, _, message in records:
        phase = stamp - origin
        if phase >= 0.0:
            output.append((phase, position(message), velocity(message),
                           quaternion(message)))
    output.sort(key=lambda item: item[0])
    return output


def compare_tracks(first, second, origin, minimum_samples=300):
    first_track = track_phase(first, origin)
    second_track = track_phase(second, origin)
    if len(first_track) < minimum_samples or len(second_track) < minimum_samples:
        return {"ok": False, "reason": "insufficient odometry samples",
                "first_samples": len(first_track),
                "second_samples": len(second_track)}
    second_times = [item[0] for item in second_track]
    max_position_error = 0.0
    max_velocity_error = 0.0
    max_attitude_error = 0.0
    matched = 0
    for phase, first_position, first_velocity, first_quaternion in first_track:
        index = bisect.bisect_left(second_times, phase)
        if index == 0 or index >= len(second_track):
            continue
        left = second_track[index - 1]
        right = second_track[index]
        span = right[0] - left[0]
        if span <= 0.0:
            continue
        fraction = (phase - left[0]) / span
        second_position = interpolation(left[1], right[1], fraction)
        second_velocity = interpolation(left[2], right[2], fraction)
        second_quaternion = slerp(left[3], right[3], fraction)
        max_position_error = max(
            max_position_error,
            vector_distance(first_position, second_position))
        max_velocity_error = max(
            max_velocity_error,
            vector_distance(first_velocity, second_velocity))
        max_attitude_error = max(
            max_attitude_error,
            quaternion_distance(first_quaternion, second_quaternion))
        matched += 1
    return {
        "ok": (matched >= minimum_samples and
               max_position_error <= 0.020 and
               max_velocity_error <= 0.020 and
               max_attitude_error <= 0.010),
        "matched_samples": matched,
        "max_position_error_m": max_position_error,
        "max_velocity_error_mps": max_velocity_error,
        "max_attitude_error_rad": max_attitude_error,
        "first_samples": len(first_track),
        "second_samples": len(second_track),
    }


def header_rate(records):
    if len(records) < 2:
        return 0.0
    first = records[0][0]
    last = records[-1][0]
    return (len(records) - 1) / (last - first) if last > first else 0.0


def stage_liveness(tracks, before, after, stage_name):
    """Require every odom stream to advance throughout a test phase."""
    evidence = {}
    for index, track in tracks.items():
        before_records = before[index]
        after_records = after[index]
        growth = len(after_records) - len(before_records)
        last_delta = (after_records[-1][0] - before_records[-1][0]
                      if before_records and after_records else 0.0)
        new_records = after_records[len(before_records):]
        finite_tail = all(finite_odom(item[2]) for item in new_records)
        phase_rate = header_rate(new_records)
        evidence[index] = {
            "before_samples": len(before_records),
            "after_samples": len(after_records),
            "growth_samples": growth,
            "last_stamp_delta_s": last_delta,
            "phase_odom_hz": phase_rate,
            "finite_new_samples": finite_tail,
        }
        if (growth <= 0 or last_delta <= 0.0 or phase_rate < 80.0 or
                not finite_tail):
            raise AssertionError("%s odom stream %d did not remain live: %s" %
                                 (stage_name, index, evidence[index]))
    return evidence


def node_endpoint_sets(node_name):
    publications, subscriptions, services = master_system_state()
    published = set()
    subscribed = set()
    for topic, nodes in publications:
        if node_name in nodes:
            published.add(topic)
    for topic, nodes in subscriptions:
        if node_name in nodes:
            subscribed.add(topic)
    all_nodes = set()
    for _, nodes in publications + subscriptions + services:
        all_nodes.update(nodes)
    return published, subscribed, all_nodes


def master_system_state():
    code, message, value = rospy.get_master().getSystemState()
    if code != 1:
        raise AssertionError("master getSystemState failed: %s" % message)
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise AssertionError("master getSystemState returned malformed value")
    publications, subscriptions, services = value
    return publications, subscriptions, services


def check_no_extra_plant_topics(expected_topics):
    publications, subscriptions, services = master_system_state()
    topics = {topic for topic, _ in publications + subscriptions}
    plant_topics = {topic for topic in topics if topic.startswith("/uav_")}
    if plant_topics != expected_topics:
        raise AssertionError("unexpected plant topics: %s" % sorted(plant_topics))
    if any(topic.startswith("/uav_3") for topic in topics):
        raise AssertionError("unexpected fourth-agent topic")
    infrastructure = {"/rosout", "/rosout_agg", "/clock"}
    unexpected = topics - expected_topics - infrastructure
    if unexpected:
        raise AssertionError("unexpected non-plant topics: %s" %
                             sorted(unexpected))
    forbidden_prefixes = ("/position", "/agent", "/map", "/planner",
                          "/swarm", "/control", "/goal")
    forbidden = sorted(topic for topic in topics
                       if topic.startswith(forbidden_prefixes))
    if forbidden:
        raise AssertionError("unexpected global coordination topics: %s" %
                             forbidden)


class B0MultiSimulatorTest(unittest.TestCase):
    def setUp(self):
        self.scenario_name = rospy.get_param("~scenario", "equivalence")

    def test_b0(self):
        if self.scenario_name == "equivalence":
            self.run_equivalence()
        elif self.scenario_name == "three_agent":
            self.run_three_agent()
        else:
            self.fail("unknown B0 scenario: %s" % self.scenario_name)

    def run_equivalence(self):
        original_odom = MessageTrack("/single/sim/odom", Odometry)
        multi_odom = MessageTrack("/uav_0/sim/odom", Odometry)
        original_imu = MessageTrack("/single/sim/imu", Imu)
        multi_imu = MessageTrack("/uav_0/sim/imu", Imu)
        original_command = rospy.Publisher(
            "/single/so3_cmd", SO3Command, queue_size=20)
        multi_command = rospy.Publisher(
            "/uav_0/so3_cmd", SO3Command, queue_size=20)
        self.assertTrue(wait_for_count(original_odom, 20, 15.0))
        self.assertTrue(wait_for_count(multi_odom, 20, 15.0))
        self.assertTrue(wait_for_count(original_imu, 20, 15.0))
        self.assertTrue(wait_for_count(multi_imu, 20, 15.0))
        origin = publish_waveform([original_command, multi_command], 6.5)
        time.sleep(0.4)
        result = compare_tracks(original_odom.snapshot(),
                                multi_odom.snapshot(), origin)
        self.assertTrue(result["ok"], "equivalence metrics: %s" % result)
        self.assertGreaterEqual(header_rate(original_odom.snapshot()), 80.0)
        self.assertGreaterEqual(header_rate(multi_odom.snapshot()), 80.0)
        for record in original_odom.snapshot() + multi_odom.snapshot():
            self.assertTrue(finite_odom(record[2]))
        for record in original_imu.snapshot() + multi_imu.snapshot():
            self.assertTrue(finite_imu(record[2]))

        original_published, _, _ = node_endpoint_sets(
            "/quadrotor_simulator_so3")
        _, original_subscribed, _ = node_endpoint_sets(
            "/quadrotor_simulator_so3")
        multi_published, multi_subscribed, _ = node_endpoint_sets(
            "/multi_quadrotor_simulator_so3")
        self.assertEqual(original_published - {"/rosout"},
                         {"/single/sim/odom", "/single/sim/imu"})
        self.assertEqual(multi_published - {"/rosout"},
                         {"/uav_0/sim/odom", "/uav_0/sim/imu"})
        self.assertEqual(original_subscribed,
                         {"/single/so3_cmd",
                          "/quadrotor_simulator_so3/force_disturbance",
                          "/quadrotor_simulator_so3/moment_disturbance"})
        self.assertEqual(multi_subscribed,
                         {"/uav_0/so3_cmd", "/uav_0/force_disturbance",
                          "/uav_0/moment_disturbance"})
        publications, subscriptions, _ = master_system_state()
        graph_topics = {topic for topic, _ in publications + subscriptions}
        graph_topics -= {"/rosout", "/rosout_agg", "/clock"}
        self.assertEqual(graph_topics, {
            "/single/sim/odom", "/single/sim/imu", "/single/so3_cmd",
            "/uav_0/sim/odom", "/uav_0/sim/imu", "/uav_0/so3_cmd",
            "/uav_0/force_disturbance", "/uav_0/moment_disturbance",
            "/quadrotor_simulator_so3/force_disturbance",
            "/quadrotor_simulator_so3/moment_disturbance",
        })
        emit_metrics({
            "scenario": "equivalence",
            "matched_samples": result["matched_samples"],
            "max_position_error_m": result["max_position_error_m"],
            "max_velocity_error_mps": result["max_velocity_error_mps"],
            "max_attitude_error_rad": result["max_attitude_error_rad"],
            "original_odom_hz": header_rate(original_odom.snapshot()),
            "multi_odom_hz": header_rate(multi_odom.snapshot()),
        })

    def run_three_agent(self):
        expected = {
            0: (0.0, -2.0, 1.0, 0.0),
            1: (0.0, 0.0, 1.0, 0.0),
            2: (0.0, 2.0, 1.0, 0.0),
        }
        odom = {}
        imu = {}
        for index in range(3):
            odom[index] = MessageTrack("/uav_%d/sim/odom" % index,
                                        Odometry)
            imu[index] = MessageTrack("/uav_%d/sim/imu" % index, Imu)
        commands = [rospy.Publisher("/uav_%d/so3_cmd" % index,
                                    SO3Command, queue_size=20)
                    for index in range(3)]
        force = [rospy.Publisher("/uav_%d/force_disturbance" % index,
                                 Vector3, queue_size=20)
                 for index in range(3)]
        hover_start = time.monotonic()
        for index in range(3):
            self.assertTrue(wait_for_count(odom[index], 250, 15.0))
            self.assertTrue(wait_for_count(imu[index], 250, 15.0))
        remaining_hover = 3.0 - (time.monotonic() - hover_start)
        if remaining_hover > 0.0:
            time.sleep(remaining_hover)
        hover_duration = time.monotonic() - hover_start
        self.assertGreaterEqual(hover_duration, 3.0)

        hover_metrics = {}
        for index in range(3):
            records = odom[index].snapshot()
            self.assertTrue(all(finite_odom(item[2]) for item in records))
            imu_records = imu[index].snapshot()
            self.assertTrue(all(finite_imu(item[2]) for item in imu_records))
            self.assertGreaterEqual(header_rate(records), 80.0)
            self.assertLessEqual(header_rate(records), 130.0)
            latest = records[-1][2]
            latest_imu = imu_records[-1][2]
            expected_position = expected[index][:3]
            position_error = vector_distance(position(latest),
                                             expected_position)
            yaw_error = angular_distance(yaw_from_quaternion(
                quaternion(latest)), expected[index][3])
            self.assertLess(position_error, 0.020)
            self.assertGreater(position(latest)[2], 0.95)
            self.assertLess(yaw_error, 0.050)
            self.assertEqual(latest.child_frame_id,
                             "uav_%d/base_link" % index)
            self.assertEqual(latest.header.frame_id, "world")
            self.assertEqual(latest_imu.header.frame_id, "world")
            hover_metrics[index] = {
                "hover_duration_s": hover_duration,
                "odom_hz": header_rate(records),
                "position_error_m": position_error,
                "yaw_error_rad": yaw_error,
                "z_m": position(latest)[2],
                "child_frame_id": latest.child_frame_id,
                "odom_frame_id": latest.header.frame_id,
                "imu_frame_id": latest_imu.header.frame_id,
                "odom_samples": len(records),
                "imu_samples": len(imu_records),
            }

        expected_topics = set()
        for index in range(3):
            expected_topics.update({
                "/uav_%d/so3_cmd" % index,
                "/uav_%d/force_disturbance" % index,
                "/uav_%d/moment_disturbance" % index,
                "/uav_%d/sim/odom" % index,
                "/uav_%d/sim/imu" % index,
            })
        check_no_extra_plant_topics(expected_topics)
        published, subscribed, all_nodes = node_endpoint_sets(
            "/multi_quadrotor_simulator_so3")
        self.assertEqual(
            {topic for topic in published if topic.startswith("/uav_")},
            {"/uav_%d/sim/odom" % index for index in range(3)} |
            {"/uav_%d/sim/imu" % index for index in range(3)})
        self.assertEqual(
            {topic for topic in subscribed if topic.startswith("/uav_")},
            {topic for topic in expected_topics
             if not topic.endswith("/sim/odom") and
             not topic.endswith("/sim/imu")})
        self.assertEqual(sum(node.endswith("multi_quadrotor_simulator_so3")
                             for node in all_nodes), 1)

        command_before = {index: odom[index].snapshot() for index in range(3)}
        reference = [position(command_before[index][-1][2])
                     for index in range(3)]
        publish_fixed(commands[0], tilt_command(0.15), 4.0)
        command_after = {index: odom[index].snapshot() for index in range(3)}
        command_liveness = stage_liveness(
            odom, command_before, command_after, "command")
        moved = [position(command_after[index][-1][2])
                 for index in range(3)]
        command_displacements = [vector_distance(moved[index], reference[index])
                                 for index in range(3)]
        self.assertGreater(command_displacements[0], 0.10)
        self.assertLess(command_displacements[1], 0.020)
        self.assertLess(command_displacements[2], 0.020)
        publish_fixed(commands[0], hover_command(), 0.7)

        disturbance_before = {
            index: odom[index].snapshot() for index in range(3)}
        disturbance_reference = [position(disturbance_before[index][-1][2])
                                 for index in (1, 2)]
        publish_disturbance(force[1], 0.20, 1.5)
        disturbance_after = {
            index: odom[index].snapshot() for index in range(3)}
        disturbance_liveness = stage_liveness(
            odom, disturbance_before, disturbance_after, "disturbance")
        disturbed = [position(disturbance_after[index][-1][2])
                     for index in (1, 2)]
        disturbance_displacements = [
            vector_distance(disturbed[index], disturbance_reference[index])
            for index in (0, 1)]
        self.assertGreater(disturbance_displacements[0], 0.020)
        self.assertLess(disturbance_displacements[1], 0.020)
        for index in range(3):
            self.assertGreaterEqual(len(odom[index].snapshot()), 300)
            self.assertTrue(all(finite_odom(item[2])
                                for item in odom[index].snapshot()))
        emit_metrics({
            "scenario": "three_agent",
            "hover": hover_metrics,
            "command_liveness": command_liveness,
            "command_displacement_m": command_displacements,
            "disturbance_liveness": disturbance_liveness,
            "disturbance_displacement_m": disturbance_displacements,
        })


if __name__ == "__main__":
    rospy.init_node(TEST_NAME, anonymous=True)
    rospy.loginfo("B0_TEST_PYTHON executable=%s version=%s",
                  sys.executable, sys.version.split()[0])
    rostest.rosrun(PACKAGE, TEST_NAME, B0MultiSimulatorTest)
