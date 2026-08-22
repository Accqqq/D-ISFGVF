#!/usr/bin/python3
"""B1 independent-navigation acceptance checks.

The launch fixtures provide the only stage selection.  This test records the
existing planner, local-sensing, SO3, and B0 plant interfaces without adding a
controller, a scenario publisher, or a process supervisor.
"""

import collections
import json
import math
import os
import platform
import re
import sys
import threading
import time
import unittest

sys.dont_write_bytecode = True

import rosgraph
import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from nodelet.srv import NodeletList
from sensor_msgs.msg import Imu, PointCloud2

try:
    from quadrotor_msgs.msg import PositionCommand, SO3Command
except ImportError:
    # A package-only build can leave generated Python files absent from devel.
    # The checked-in source-generated package is a read-only fallback.
    workspace = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../"))
    message_source = os.path.join(
        workspace, "src/uav_simulator/Utils/quadrotor_msgs/src")
    sys.path.insert(0, message_source)
    for module_name in list(sys.modules):
        if (module_name == "quadrotor_msgs" or
                module_name.startswith("quadrotor_msgs.")):
            del sys.modules[module_name]
    from quadrotor_msgs.msg import PositionCommand, SO3Command


WORKSPACE = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../"))
PILLAR_PATH = os.path.join(
    WORKSPACE, "src/uav_simulator/dynamic_map_generator/resource/pillar.pcd")
MAX_ARRIVAL_SECONDS = 45.0
STABLE_SECONDS = 3.0
DRIFT_SECONDS = 5.0
SCHEDULING_MARGIN_SECONDS = 1.0
CELL_SIZE = 0.1
LEGACY_GOAL_Z_OFFSET_M = 1.0

SCENARIOS = {
    "single": {
        "ids": (0,),
        "starts": {0: (-5.0, -3.50, 1.0)},
        "published_goals": {0: (5.0, -3.50, 1.0)},
    },
    "three": {
        "ids": (0, 1, 2),
        "starts": {
            0: (-5.0, -4.75, 1.0),
            1: (-5.0, -3.50, 1.0),
            2: (-5.0, -2.25, 1.0),
        },
        "published_goals": {
            0: (5.0, -4.75, 1.0),
            1: (5.0, -3.50, 1.0),
            2: (5.0, -2.25, 1.0),
        },
    },
}


def finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def effective_goal(published_goal):
    return (
        published_goal[0],
        published_goal[1],
        published_goal[2] + LEGACY_GOAL_Z_OFFSET_M,
    )


def finite_odom(message):
    p = message.pose.pose.position
    q = message.pose.pose.orientation
    v = message.twist.twist.linear
    w = message.twist.twist.angular
    values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w,
              v.x, v.y, v.z, w.x, w.y, w.z]
    values.extend(message.pose.covariance)
    values.extend(message.twist.covariance)
    return all(finite(value) for value in values)


def finite_position_command(message):
    values = [message.position.x, message.position.y, message.position.z,
              message.velocity.x, message.velocity.y, message.velocity.z,
              message.acceleration.x, message.acceleration.y,
              message.acceleration.z, message.jerk.x, message.jerk.y,
              message.jerk.z, message.yaw, message.yaw_dot]
    values.extend(message.kx)
    values.extend(message.kv)
    return all(finite(value) for value in values)


def finite_so3_command(message):
    values = [message.force.x, message.force.y, message.force.z,
              message.orientation.x, message.orientation.y,
              message.orientation.z, message.orientation.w]
    values.extend(message.kR)
    values.extend(message.kOm)
    values.extend([message.aux.current_yaw, message.aux.kf_correction])
    values.extend(message.aux.angle_corrections)
    return all(finite(value) for value in values)


def stamp_seconds(message):
    value = message.header.stamp.to_sec()
    return value if finite(value) else None


class PcdSpatialHash:
    """ASCII PCD XY hash with exact nearby-cell distance evaluation."""

    def __init__(self, path, cell_size=CELL_SIZE):
        self.cell_size = float(cell_size)
        self.cells = {}
        self.min_cell = [0, 0]
        self.max_cell = [0, 0]
        self.point_count = 0
        self._load(path)

    def _cell(self, x, y):
        return (int(math.floor(x / self.cell_size)),
                int(math.floor(y / self.cell_size)))

    def _load(self, path):
        data = False
        first = True
        with open(path, "r") as stream:
            for line in stream:
                text = line.strip()
                if not text:
                    continue
                if not data:
                    if text.upper().startswith("DATA "):
                        if text.lower() != "data ascii":
                            raise ValueError("B1 requires ASCII PCD input")
                        data = True
                    continue
                fields = text.split()
                if len(fields) < 3:
                    continue
                x, y, z = (float(fields[0]), float(fields[1]),
                           float(fields[2]))
                if not all(finite(value) for value in (x, y, z)):
                    continue
                if z < 0.5 or z > 1.5:
                    continue
                cell = self._cell(x, y)
                self.cells.setdefault(cell, []).append((x, y))
                if first:
                    self.min_cell = [cell[0], cell[1]]
                    self.max_cell = [cell[0], cell[1]]
                    first = False
                else:
                    self.min_cell[0] = min(self.min_cell[0], cell[0])
                    self.min_cell[1] = min(self.min_cell[1], cell[1])
                    self.max_cell[0] = max(self.max_cell[0], cell[0])
                    self.max_cell[1] = max(self.max_cell[1], cell[1])
                self.point_count += 1
        if not data or not self.cells:
            raise ValueError("B1 PCD has no flight-height points")

    def min_distance(self, x, y):
        """Return the exact distance to the nearest retained raw PCD point."""
        if not (finite(x) and finite(y)):
            return float("inf")
        cx, cy = self._cell(x, y)
        max_radius = max(abs(cx - self.min_cell[0]),
                         abs(cx - self.max_cell[0]),
                         abs(cy - self.min_cell[1]),
                         abs(cy - self.max_cell[1])) + 2
        best = float("inf")
        for radius in range(max_radius + 1):
            for ix in range(cx - radius, cx + radius + 1):
                for iy in sorted(set((cy - radius, cy + radius))):
                    for px, py in self.cells.get((ix, iy), ()):
                        distance = math.hypot(x - px, y - py)
                        if distance < best:
                            best = distance
            for iy in range(cy - radius + 1, cy + radius):
                for ix in (cx - radius, cx + radius):
                    for px, py in self.cells.get((ix, iy), ()):
                        distance = math.hypot(x - px, y - py)
                        if distance < best:
                            best = distance
            if finite(best):
                right = max(0.0, (cx + radius + 1) * self.cell_size - x)
                left = max(0.0, x - (cx - radius) * self.cell_size)
                top = max(0.0, (cy + radius + 1) * self.cell_size - y)
                bottom = max(0.0, y - (cy - radius) * self.cell_size)
                if min(right, left, top, bottom) > best:
                    break
        return best


class AgentTrace:
    """Bounded scalar/tuple traces; raw PointCloud2 messages are not retained."""

    def __init__(self, robot_id, clearance_hash):
        self.robot_id = robot_id
        self.clearance_hash = clearance_hash
        self.lock = threading.Lock()
        self.odom = collections.deque(maxlen=12000)
        self.map_stamps = collections.deque(maxlen=2000)
        self.position_stamps = collections.deque(maxlen=4000)
        self.so3_stamps = collections.deque(maxlen=4000)
        self.path_stamps = collections.deque(maxlen=1000)
        self.path_clearances = collections.deque(maxlen=1000)
        self.path_pose_counts = collections.deque(maxlen=1000)
        self.path_frames = collections.deque(maxlen=1000)
        self.odom_finite = True
        self.map_finite = True
        self.position_finite = True
        self.so3_finite = True
        self.path_finite = True
        self.odom_frame = ""
        self.child_frame = ""
        self.map_frame = ""
        self.path_frame = ""
        self.odom_sub = rospy.Subscriber(
            "/uav_%d/sim/odom" % robot_id, Odometry, self.odom_callback,
            queue_size=300)
        self.map_sub = rospy.Subscriber(
            "/uav_%d/sim/local_map" % robot_id, PointCloud2,
            self.map_callback, queue_size=30)
        self.position_sub = rospy.Subscriber(
            "/uav_%d/position_cmd" % robot_id, PositionCommand,
            self.position_callback, queue_size=100)
        self.so3_sub = rospy.Subscriber(
            "/uav_%d/so3_cmd" % robot_id, SO3Command,
            self.so3_callback, queue_size=100)
        self.path_sub = rospy.Subscriber(
            "/uav_%d/particle0/path" % robot_id, Path,
            self.path_callback, queue_size=20)

    def odom_callback(self, message):
        stamp = stamp_seconds(message)
        finite_data = finite_odom(message) and stamp is not None
        p = message.pose.pose.position
        v = message.twist.twist.linear
        clearance = (self.clearance_hash.min_distance(p.x, p.y)
                     if finite_data else float("inf"))
        record = (time.monotonic(), stamp if stamp is not None else 0.0,
                  p.x, p.y, p.z,
                  v.x, v.y, v.z, clearance)
        with self.lock:
            self.odom_finite = self.odom_finite and finite_data
            self.odom_frame = message.header.frame_id
            self.child_frame = message.child_frame_id
            self.odom.append(record)

    def map_callback(self, message):
        value = stamp_seconds(message)
        finite_data = value is not None
        with self.lock:
            self.map_finite = self.map_finite and finite_data
            self.map_frame = message.header.frame_id
            self.map_stamps.append((time.monotonic(),
                                    value if value is not None else 0.0))

    def position_callback(self, message):
        stamp = stamp_seconds(message)
        finite_data = finite_position_command(message)
        finite_data = finite_data and stamp is not None
        with self.lock:
            self.position_finite = self.position_finite and finite_data
            self.position_stamps.append(
                (time.monotonic(), stamp if stamp is not None else 0.0))

    def so3_callback(self, message):
        stamp = stamp_seconds(message)
        finite_data = finite_so3_command(message)
        finite_data = finite_data and stamp is not None
        with self.lock:
            self.so3_finite = self.so3_finite and finite_data
            self.so3_stamps.append(
                (time.monotonic(), stamp if stamp is not None else 0.0))

    def path_callback(self, message):
        stamp = stamp_seconds(message)
        finite_data = True
        finite_data = finite_data and stamp is not None
        minimum = float("inf")
        for pose in message.poses:
            p = pose.pose.position
            values = (p.x, p.y, p.z, pose.pose.orientation.x,
                      pose.pose.orientation.y, pose.pose.orientation.z,
                      pose.pose.orientation.w)
            if not all(finite(value) for value in values):
                finite_data = False
                continue
            minimum = min(minimum, self.clearance_hash.min_distance(p.x, p.y))
        with self.lock:
            self.path_finite = self.path_finite and finite_data
            self.path_frame = message.header.frame_id
            self.path_stamps.append(
                (time.monotonic(), stamp if stamp is not None else 0.0))
            self.path_clearances.append(minimum)
            self.path_pose_counts.append(len(message.poses))
            self.path_frames.append(message.header.frame_id)

    def snapshot(self):
        with self.lock:
            return {
                "odom": list(self.odom),
                "map_stamps": list(self.map_stamps),
                "position_stamps": list(self.position_stamps),
                "so3_stamps": list(self.so3_stamps),
                "path_stamps": list(self.path_stamps),
                "path_clearances": list(self.path_clearances),
                "path_pose_counts": list(self.path_pose_counts),
                "path_frames": list(self.path_frames),
                "odom_finite": self.odom_finite,
                "map_finite": self.map_finite,
                "position_finite": self.position_finite,
                "so3_finite": self.so3_finite,
                "path_finite": self.path_finite,
                "odom_frame": self.odom_frame,
                "child_frame": self.child_frame,
                "map_frame": self.map_frame,
                "path_frame": self.path_frame,
            }


def wait_until(predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not rospy.is_shutdown():
        if predicate():
            return True
        time.sleep(0.05)
    return bool(predicate())


def rate_from_stamps(stamps):
    if len(stamps) < 2:
        return 0.0
    start = stamps[0][1]
    end = stamps[-1][1]
    if end <= start:
        return 0.0
    return float(len(stamps) - 1) / (end - start)


def finite_sample_rate(stamps):
    """Require finite header stamps and positive timestamp progress."""
    if len(stamps) < 2:
        return False
    if not all(finite(item[1]) for item in stamps):
        return False
    return stamps[-1][1] > stamps[0][1]


def condition(record, goal):
    _, _, x, y, z, vx, vy, vz, _ = record
    horizontal = math.hypot(x - goal[0], y - goal[1])
    height = abs(z - goal[2])
    speed = math.sqrt(vx * vx + vy * vy + vz * vz)
    return horizontal <= 0.25 and height <= 0.10 and speed <= 0.15


def stable_arrival(records, goal, goal_wall):
    """Find a frozen arrival, using receiver monotonic times only.

    A candidate starts after the receiver-side goal anchor.  Its stable
    window ends at the first received sample at least three seconds later;
    that endpoint is the stable reference.  The post-stable drift window
    likewise ends at the first sample at least five seconds after that
    reference, and includes both endpoint samples.
    """
    for index, record in enumerate(records):
        start_wall = record[0]
        if start_wall < goal_wall:
            continue
        if start_wall - goal_wall > MAX_ARRIVAL_SECONDS:
            break
        if not condition(record, goal):
            continue

        stable_end = start_wall + STABLE_SECONDS
        stable_index = None
        for candidate_index in range(index, len(records)):
            if records[candidate_index][0] >= stable_end:
                stable_index = candidate_index
                break
        if stable_index is None:
            continue
        stable_records = records[index:stable_index + 1]
        if not all(condition(item, goal) for item in stable_records):
            continue

        # This is the first complete stable window.  Lock its endpoint as
        # the reference: later candidate windows may not wash out a failed
        # post-stable drift check.
        reference = records[stable_index]
        drift_end = reference[0] + DRIFT_SECONDS
        drift_index = None
        for candidate_index in range(stable_index, len(records)):
            if records[candidate_index][0] >= drift_end:
                drift_index = candidate_index
                break
        if drift_index is None:
            return None
        drift_records = records[stable_index:drift_index + 1]
        maximum_drift = 0.0
        for item in drift_records:
            maximum_drift = max(
                maximum_drift,
                math.sqrt((item[2] - reference[2]) ** 2 +
                          (item[3] - reference[3]) ** 2 +
                          (item[4] - reference[4]) ** 2))
        if maximum_drift <= 0.10:
            return {
                "arrival_s": start_wall - goal_wall,
                "final_xy_error_m": math.hypot(
                    reference[2] - goal[0], reference[3] - goal[1]),
                "final_z_error_m": abs(reference[4] - goal[2]),
                "final_speed_mps": math.sqrt(reference[5] ** 2 +
                                               reference[6] ** 2 +
                                               reference[7] ** 2),
                "drift_m": maximum_drift,
            }
        return None
    return None


def pair_statistics(first, second):
    if not first or not second:
        return float("inf"), 0
    minimum = float("inf")
    below = 0
    other_index = 0
    for record in first:
        stamp = record[1]
        while (other_index + 1 < len(second) and
               abs(second[other_index + 1][1] - stamp) <=
               abs(second[other_index][1] - stamp)):
            other_index += 1
        other = second[other_index]
        distance = math.sqrt(
            (record[2] - other[2]) ** 2 +
            (record[3] - other[3]) ** 2 +
            (record[4] - other[4]) ** 2)
        minimum = min(minimum, distance)
        if distance < 0.60:
            below += 1
    return minimum, below


def nonempty_paths_after(snapshot, wall_time):
    return [
        (stamp, clearance, frame)
        for (received, stamp), count, clearance, frame in zip(
            snapshot["path_stamps"], snapshot["path_pose_counts"],
            snapshot["path_clearances"], snapshot["path_frames"])
        if received >= wall_time and count > 0
    ]


def invert_endpoint_state(entries):
    """Invert ROS master `(endpoint, [node, ...])` state to node ownership."""
    inverted = {}
    for endpoint, nodes in entries:
        endpoint = str(endpoint)
        for node in nodes:
            inverted.setdefault(str(node), set()).add(endpoint)
    return inverted


def decode_system_state(state):
    """Validate the direct ROS master system-state three-section result."""
    section_names = ("publishers", "subscribers", "services")
    if not isinstance(state, (list, tuple)) or len(state) != 3:
        raise RuntimeError("malformed getSystemState outer structure")
    for section_index, section in enumerate(state):
        if not isinstance(section, (list, tuple)):
            raise RuntimeError("malformed getSystemState %s section" %
                               section_names[section_index])
        for entry_index, entry in enumerate(section):
            if not isinstance(entry, (list, tuple)) or len(entry) != 2:
                raise RuntimeError(
                    "malformed getSystemState %s entry %d" %
                    (section_names[section_index], entry_index))
            if not isinstance(entry[1], (list, tuple)):
                raise RuntimeError(
                    "malformed getSystemState %s nodes %d" %
                    (section_names[section_index], entry_index))
    return state[0], state[1], state[2]


def get_system_state():
    master = rosgraph.Master(rospy.get_name())
    publishers, subscribers, services = decode_system_state(
        master.getSystemState())
    return (invert_endpoint_state(publishers),
            invert_endpoint_state(subscribers),
            invert_endpoint_state(services))


def extract_agent_ids(value):
    return set(int(match.group(1)) for match in re.finditer(
        r"(?:^|/)uav_(\d+)(?:/|$)", value))


def nodelet_audit(agent_ids):
    expected = set("/so3_control_nodelet_%d" % robot_id
                   for robot_id in agent_ids)
    loaded = set()
    service_error = ""
    try:
        rospy.wait_for_service("/so3_nodelet_manager/list", timeout=5.0)
        response = rospy.ServiceProxy(
            "/so3_nodelet_manager/list", NodeletList)()
        loaded = set("/" + str(name).lstrip("/")
                     for name in response.nodelets)
    except Exception as error:
        service_error = str(error)
    return {
        "nodelet_list": sorted(loaded),
        "nodelet_expected": sorted(expected),
        "nodelet_mismatch": sorted(loaded ^ expected),
        "nodelet_service_error": service_error,
    }


def graph_audit(agent_ids):
    publishers, subscribers, services = get_system_state()
    all_topics = set()
    for topic_set in list(publishers.values()) + list(subscribers.values()):
        all_topics.update(topic_set)

    # The manager owns the process-level registration for SO3 nodelets.  The
    # logical nodelet names are audited through NodeletList below; they are not
    # inferred from the master caller identity.
    expected_nodes = {
        "/map_pub", "/multi_quadrotor_simulator_so3", "/so3_nodelet_manager",
    }
    for robot_id in agent_ids:
        expected_nodes.update({
            "/uav_%d/formation_planning" % robot_id,
            "/uav_%d/map_generator" % robot_id,
        })
    node_names = set(publishers) | set(subscribers) | set(services)
    missing_nodes = sorted(expected_nodes - node_names)

    forbidden_global_endpoints = {
        "/sim/odom", "/sim/imu", "/sim/local_map",
        "/move_base_simple/goal", "/position_cmd", "/so3_cmd",
        "/motors", "/corrections", "/force_disturbance",
        "/moment_disturbance", "/gvf_force", "/path_vis", "/goal_vis",
        "/optimization_path", "/manual_obstacle", "/manual_boundary",
        "/manual_map/occupancy", "/manual_map/add_obstacle_center",
        "/manual_map/add_boundary_point", "/clicked_point", "/dynamic/obj",
    }
    forbidden_global_endpoints = set(
        endpoint.lower() for endpoint in forbidden_global_endpoints)
    forbidden_topics = [
        topic for topic in all_topics
        if topic.lower() in forbidden_global_endpoints or
        topic.lower().startswith("/particle0/") or
        topic.lower().startswith("/particle0sdf_map/") or
        topic.lower().startswith("/particle0/gvf/")
    ]
    forbidden_semantic_fragments = (
        "phase_offset_swarm", "swarm_intent", "swarm", "allocator", "cbf", "tube",
        "scenario", "visualizer", "rviz",
    )
    forbidden_semantic_topics = sorted(
        topic for topic in all_topics
        if any(fragment in topic.lower()
               for fragment in forbidden_semantic_fragments))
    forbidden_node_fragments = forbidden_semantic_fragments
    forbidden_nodes = sorted(
        node for node in node_names
        if any(fragment in node.lower() for fragment in forbidden_node_fragments))

    expected_topics = {"/mock_map"}
    for robot_id in agent_ids:
        prefix = "/uav_%d" % robot_id
        # Core operational topics.
        expected_topics.update({
            prefix + "/sim/odom", prefix + "/sim/imu",
            prefix + "/sim/local_map", prefix + "/goal",
            prefix + "/position_cmd", prefix + "/so3_cmd",
            prefix + "/motors", prefix + "/corrections",
            prefix + "/force_disturbance", prefix + "/moment_disturbance",
        })
        # Constructor-guaranteed planner, SDF, and GVF diagnostics.
        expected_topics.update({
            prefix + "/gvf_force", prefix + "/path_vis",
            prefix + "/goal_vis", prefix + "/optimization_path",
            prefix + "/particle0/path", prefix + "/particle0/kinopath",
            prefix + "/particle0/circle_reference",
            prefix + "/particle0sdf_map/occupancy",
            prefix + "/particle0sdf_map/occupancy_inflate",
            prefix + "/particle0sdf_map/esdf",
            prefix + "/particle0sdf_map/update_range",
            prefix + "/particle0sdf_map/unknown",
            prefix + "/particle0sdf_map/depth_cloud",
            prefix + "/particle0/gvf/occupancy",
            prefix + "/particle0/gvf/occupancy_inflate",
            prefix + "/particle0/gvf/esdf",
            prefix + "/particle0/gvf/update_range",
            prefix + "/particle0/gvf/traj_vis",
            prefix + "/particle0/gvf/vector_field",
            prefix + "/manual_map/occupancy",
        })
    missing_topics = sorted(topic for topic in expected_topics
                            if topic not in all_topics)

    configured_ids = set(agent_ids)
    unexpected_agent_topics = sorted(
        topic for topic in all_topics
        if any(robot_id not in configured_ids
               for robot_id in extract_agent_ids(topic)))
    unexpected_agent_nodes = sorted(
        node for node in node_names
        if any(robot_id not in configured_ids
               for robot_id in extract_agent_ids(node)))
    fourth_agent_topics = sorted(topic for topic in all_topics
                                 if topic.startswith("/uav_3/"))

    cross_agent_edges = []
    for node, topics in subscribers.items():
        owner_match = re.search(r"(?:^|/)uav_(\d+)(?:/|$)", node)
        if owner_match is None:
            owner_match = re.search(r"so3_control_nodelet_(\d+)$", node)
        if owner_match is None:
            continue
        owner = int(owner_match.group(1))
        for topic in topics:
            match = re.match(r"/uav_(\d+)/", topic)
            if match is not None and int(match.group(1)) != owner:
                cross_agent_edges.append((node, topic))

    infrastructure_topics = {
        "/rosout", "/rosout_agg", "/clock", "/tf", "/tf_static",
        "/diagnostics",
    }
    shared_subscribers = collections.defaultdict(set)
    for node, topics in subscribers.items():
        owner_match = re.match(
            r"^/uav_(\d+)/(formation_planning|map_generator)$", node)
        if owner_match is None:
            continue
        owner = int(owner_match.group(1))
        for topic in topics:
            lower_topic = topic.lower()
            if lower_topic == "/mock_map" or lower_topic in infrastructure_topics:
                continue
            # Namespaced algorithm inputs are audited by cross_agent_edges;
            # only globally shared algorithm inputs belong in this audit.
            if re.match(r"^/uav_\d+/", topic):
                continue
            shared_subscribers[topic].add(owner)
    shared_algorithm_edges = [
        {"topic": topic, "owners": sorted(owners)}
        for topic, owners in sorted(shared_subscribers.items())
        if len(owners) > 1
    ]

    manager_topics = set(publishers.get("/so3_nodelet_manager", set()))
    manager_topics.update(subscribers.get("/so3_nodelet_manager", set()))
    so3_suffixes = (
        "/sim/odom", "/sim/imu", "/position_cmd", "/motors",
        "/corrections", "/so3_cmd",
    )
    so3_manager_operational_topics = sorted(
        topic for topic in manager_topics
        if any(topic.lower().endswith(suffix) for suffix in so3_suffixes))
    so3_manager_expected_topics = sorted(
        "%s/%s" % (prefix, suffix.lstrip("/"))
        for robot_id in agent_ids
        for prefix in ("/uav_%d" % robot_id,)
        for suffix in so3_suffixes)
    so3_manager_endpoint_mismatch = {
        "missing": sorted(set(so3_manager_expected_topics) -
                           set(so3_manager_operational_topics)),
        "unexpected": sorted(set(so3_manager_operational_topics) -
                              set(so3_manager_expected_topics)),
    }

    nodelet_metrics = nodelet_audit(agent_ids)
    return {
        "missing_nodes": missing_nodes,
        "missing_topics": missing_topics,
        "forbidden_topics": sorted(set(forbidden_topics)),
        "forbidden_semantic_topics": forbidden_semantic_topics,
        "forbidden_nodes": forbidden_nodes,
        "unexpected_agent_topics": unexpected_agent_topics,
        "unexpected_agent_nodes": unexpected_agent_nodes,
        "fourth_agent_topics": fourth_agent_topics,
        "cross_agent_edges": cross_agent_edges,
        "shared_algorithm_edges": shared_algorithm_edges,
        "nodelet_list": nodelet_metrics["nodelet_list"],
        "nodelet_expected": nodelet_metrics["nodelet_expected"],
        "nodelet_mismatch": nodelet_metrics["nodelet_mismatch"],
        "nodelet_service_error": nodelet_metrics["nodelet_service_error"],
        "so3_manager_topics": sorted(manager_topics),
        "so3_manager_operational_topics": so3_manager_operational_topics,
        "so3_manager_expected_topics": so3_manager_expected_topics,
        "so3_manager_endpoint_mismatch": so3_manager_endpoint_mismatch,
        "all_topics": sorted(all_topics),
        "node_count": len(node_names),
    }


def parameter_audit(agent_ids):
    required = {}
    failures = []
    for robot_id in agent_ids:
        prefix = "/uav_%d" % robot_id
        expected = {
            prefix + "/formation_planning/gvf/odom_topic":
                prefix + "/sim/odom",
            prefix + "/formation_planning/gvf/cloud_topic":
                prefix + "/sim/local_map",
            prefix + "/formation_planning/gvf/cmd_topic":
                prefix + "/position_cmd",
            prefix + "/formation_planning/phase_offset/mode": "disabled",
            prefix + "/formation_planning/gvf/gvf_gain1": 0.8,
            prefix + "/formation_planning/gvf/gvf_gain2": -0.88,
            prefix + "/formation_planning/sdf_map/enable_manual_map": False,
            prefix + "/formation_planning/sdf_map/manual_map_auto_load": False,
            prefix + "/formation_planning/sdf_map/manual_map_auto_save": False,
            prefix + "/map_generator/odom_topic": prefix + "/sim/odom",
            prefix + "/map_generator/global_map_topic": "/mock_map",
            prefix + "/map_generator/local_map_topic": prefix + "/sim/local_map",
            prefix + "/map_generator/output_frame": "world",
        }
        for key, expected_value in expected.items():
            try:
                actual = rospy.get_param(key)
            except KeyError:
                failures.append(key + " missing")
                continue
            required[key] = actual
            if isinstance(expected_value, float):
                if not finite(actual) or abs(float(actual) - expected_value) > 1e-9:
                    failures.append("%s=%r expected %r" %
                                    (key, actual, expected_value))
            elif actual != expected_value:
                failures.append("%s=%r expected %r" %
                                (key, actual, expected_value))
    return {"values": required, "failures": failures}


def sanitize(value):
    if isinstance(value, float):
        return value if finite(value) else None
    if isinstance(value, dict):
        return dict((key, sanitize(item)) for key, item in value.items())
    if isinstance(value, (list, tuple)):
        return [sanitize(item) for item in value]
    return value


class B1IndependentNavigationTest(unittest.TestCase):
    def setUp(self):
        self.metrics = {
            "scenario": rospy.get_param("~scenario", "single"),
            "python": {
                "executable": sys.executable,
                "version": platform.python_version(),
            },
            "goal_publish_count": {},
            "path": {},
            "arrival": {},
            "rates": {},
            "sample_counts": {},
            "graph": {},
            "parameters": {},
            "finite_data": True,
        }

    def test_independent_navigation(self):
        scenario = self.metrics["scenario"]
        if scenario not in SCENARIOS:
            self.metrics["error"] = "unknown scenario: %s" % scenario
            print("B1_METRICS " + json.dumps(
                sanitize(self.metrics), sort_keys=True, allow_nan=False))
            self.fail("unknown scenario: %s" % scenario)
        data = SCENARIOS[scenario]
        agent_ids = data["ids"]
        self.metrics["legacy_goal_z_offset_m"] = LEGACY_GOAL_Z_OFFSET_M
        self.metrics["published_goals"] = dict(
            (str(robot_id), list(data["published_goals"][robot_id]))
            for robot_id in agent_ids)
        self.metrics["effective_goals"] = dict(
            (str(robot_id), list(effective_goal(
                data["published_goals"][robot_id])))
            for robot_id in agent_ids)
        try:
            clearance_hash = PcdSpatialHash(PILLAR_PATH)
        except Exception as error:
            self.metrics["error"] = "PCD parse failed: %s" % error
            print("B1_METRICS " + json.dumps(
                sanitize(self.metrics), sort_keys=True, allow_nan=False))
            raise
        self.metrics["pcd_points_in_height_band"] = clearance_hash.point_count
        traces = dict((robot_id, AgentTrace(robot_id, clearance_hash))
                      for robot_id in agent_ids)
        goal_publishers = []
        goal_times = {}
        try:
            ready = wait_until(
                lambda: all(
                    traces[robot_id].snapshot()["odom"] and
                    traces[robot_id].snapshot()["map_stamps"]
                    for robot_id in agent_ids), 25.0)
            self.assertTrue(ready, "finite odom/local-map readiness timed out")
            self.metrics["initial_position_error_m"] = {}
            for robot_id in agent_ids:
                first = traces[robot_id].snapshot()["odom"][0]
                expected = data["starts"][robot_id]
                initial_error = math.sqrt(
                    (first[2] - expected[0]) ** 2 +
                    (first[3] - expected[1]) ** 2 +
                    (first[4] - expected[2]) ** 2)
                self.metrics["initial_position_error_m"][str(robot_id)] = initial_error
                self.assertLess(initial_error, 0.02)

            # Create every goal publisher before the single shared connection
            # wait, then publish exactly one goal per agent.
            goal_publishers_by_id = {}
            for robot_id in agent_ids:
                goal_topic = "/uav_%d/goal" % robot_id
                publisher = rospy.Publisher(
                    goal_topic, PoseStamped, queue_size=1, latch=True)
                goal_publishers.append(publisher)
                goal_publishers_by_id[robot_id] = publisher
            connected = wait_until(
                lambda: all(publisher.get_num_connections() > 0
                            for publisher in goal_publishers), 15.0)
            self.assertTrue(connected, "not all goal subscribers connected")
            for robot_id in agent_ids:
                publisher = goal_publishers_by_id[robot_id]
                goal = PoseStamped()
                goal.header.stamp = rospy.Time.now()
                goal.header.frame_id = "world"
                goal.pose.position.x = data["published_goals"][robot_id][0]
                goal.pose.position.y = data["published_goals"][robot_id][1]
                goal.pose.position.z = data["published_goals"][robot_id][2]
                goal.pose.orientation.w = 1.0
                goal_times[robot_id] = time.monotonic()
                publisher.publish(goal)
                self.metrics["goal_publish_count"][str(robot_id)] = 1

            paths_ready = wait_until(
                lambda: all(
                    nonempty_paths_after(
                        traces[robot_id].snapshot(), goal_times[robot_id])
                    for robot_id in agent_ids), 25.0)
            self.assertTrue(paths_ready, "no non-empty installed path")

            stable_deadline = max(goal_times.values()) + \
                MAX_ARRIVAL_SECONDS + STABLE_SECONDS + DRIFT_SECONDS + \
                SCHEDULING_MARGIN_SECONDS
            arrivals = {}
            while time.monotonic() < stable_deadline and not rospy.is_shutdown():
                for robot_id in agent_ids:
                    if robot_id in arrivals:
                        continue
                    records = traces[robot_id].snapshot()["odom"]
                    result = stable_arrival(
                        records,
                        effective_goal(data["published_goals"][robot_id]),
                        goal_times[robot_id])
                    if result is not None:
                        arrivals[robot_id] = result
                if len(arrivals) == len(agent_ids):
                    break
                time.sleep(0.2)
            self.metrics["arrival"] = arrivals
            self.assertEqual(set(arrivals), set(agent_ids),
                             "not all agents reached stable arrival")

            for robot_id in agent_ids:
                snapshot = traces[robot_id].snapshot()
                odom = snapshot["odom"]
                paths_after_goal = nonempty_paths_after(
                    snapshot, goal_times[robot_id])
                path_clearances = [value for _, value, _ in paths_after_goal]
                path_frames = [frame for _, _, frame in paths_after_goal]
                bad_path_frames = sorted(set(
                    frame for frame in path_frames if frame not in ("", "world")))
                after_goal = lambda samples: [
                    item for item in samples
                    if item[0] >= goal_times[robot_id]]
                odom_after = after_goal(odom)
                map_after = after_goal(snapshot["map_stamps"])
                position_after = after_goal(snapshot["position_stamps"])
                so3_after = after_goal(snapshot["so3_stamps"])
                timestamp_progress = {
                    "odom": finite_sample_rate(odom_after),
                    "local_map": finite_sample_rate(map_after),
                    "position_command": finite_sample_rate(position_after),
                    "so3_command": finite_sample_rate(so3_after),
                }
                self.metrics["path"][str(robot_id)] = {
                    "count": len(paths_after_goal),
                    "clearances_m": path_clearances,
                    "minimum_clearance_m": min(path_clearances)
                    if path_clearances else None,
                    "frame_ids": path_frames,
                    "bad_frame_ids": bad_path_frames,
                    "frame_id": snapshot["path_frame"],
                }
                self.metrics["rates"][str(robot_id)] = {
                    "odom_hz": rate_from_stamps(odom_after),
                    "local_map_hz": rate_from_stamps(map_after),
                    "position_command_hz": rate_from_stamps(position_after),
                    "so3_command_hz": rate_from_stamps(so3_after),
                }
                self.metrics.setdefault("timestamp_progress", {})[
                    str(robot_id)] = timestamp_progress
                self.metrics["sample_counts"][str(robot_id)] = {
                    "odom": len(odom),
                    "odom_after_goal": len(odom_after),
                    "local_map": len(snapshot["map_stamps"]),
                    "local_map_after_goal": len(map_after),
                    "position_command": len(snapshot["position_stamps"]),
                    "position_command_after_goal": len(position_after),
                    "so3_command": len(snapshot["so3_stamps"]),
                    "so3_command_after_goal": len(so3_after),
                }
                self.assertTrue(snapshot["odom_finite"] and
                                snapshot["map_finite"] and
                                snapshot["position_finite"] and
                                snapshot["so3_finite"] and
                                snapshot["path_finite"])
                self.assertTrue(all(timestamp_progress.values()))
                self.assertGreaterEqual(self.metrics["rates"][str(robot_id)]["odom_hz"], 80.0)
                self.assertLessEqual(self.metrics["rates"][str(robot_id)]["odom_hz"], 130.0)
                self.assertEqual(snapshot["odom_frame"], "world")
                self.assertEqual(snapshot["map_frame"], "world")
                self.assertFalse(bad_path_frames)
                self.assertGreater(self.metrics["path"][str(robot_id)]["count"], 0)
                self.assertTrue(path_clearances and
                                all(finite(value) for value in path_clearances))
                self.assertGreater(
                    self.metrics["path"][str(robot_id)]["minimum_clearance_m"], 0.45)
                physical_clearance = min(
                    (record[8] for record in odom if finite(record[8])),
                    default=float("inf"))
                self.metrics.setdefault("physical_obstacle_clearance_m", {})[
                    str(robot_id)] = physical_clearance
                self.assertGreater(physical_clearance, 0.45)

            if scenario == "three":
                pair_distances = {}
                for left_index, left_id in enumerate(agent_ids):
                    for right_id in agent_ids[left_index + 1:]:
                        pair = "%d-%d" % (left_id, right_id)
                        left = traces[left_id].snapshot()["odom"]
                        right = traces[right_id].snapshot()["odom"]
                        pair_distances[pair], pair_below = pair_statistics(
                            left, right)
                        self.metrics.setdefault("pair_samples_by_pair", {})[
                            pair] = pair_below
                self.metrics["minimum_pair_distance_m"] = min(
                    pair_distances.values())
                self.metrics["pair_distances_m"] = pair_distances
                self.metrics["pair_samples_below_0.60m"] = sum(
                    self.metrics["pair_samples_by_pair"].values())
                self.assertGreater(self.metrics["minimum_pair_distance_m"], 0.80)
                self.assertEqual(self.metrics["pair_samples_below_0.60m"], 0)

            self.metrics["parameters"] = parameter_audit(agent_ids)
            self.assertFalse(self.metrics["parameters"]["failures"])
            self.metrics["graph"] = graph_audit(agent_ids)
            graph = self.metrics["graph"]
            self.assertFalse(graph["missing_nodes"])
            self.assertFalse(graph["missing_topics"])
            self.assertFalse(graph["forbidden_topics"])
            self.assertFalse(graph["forbidden_semantic_topics"])
            self.assertFalse(graph["forbidden_nodes"])
            self.assertFalse(graph["unexpected_agent_topics"])
            self.assertFalse(graph["unexpected_agent_nodes"])
            self.assertFalse(graph["fourth_agent_topics"])
            self.assertFalse(graph["cross_agent_edges"])
            self.assertFalse(graph["shared_algorithm_edges"])
            self.assertFalse(graph["nodelet_service_error"])
            self.assertFalse(graph["nodelet_mismatch"])
            self.assertFalse(graph["so3_manager_endpoint_mismatch"]["missing"])
            self.assertFalse(graph["so3_manager_endpoint_mismatch"]["unexpected"])
        finally:
            self.metrics["finite_data"] = all(
                trace.snapshot()["odom_finite"] and
                trace.snapshot()["map_finite"] and
                trace.snapshot()["position_finite"] and
                trace.snapshot()["so3_finite"] and
                trace.snapshot()["path_finite"]
                for trace in traces.values()) if traces else True
            print("B1_METRICS " + json.dumps(
                sanitize(self.metrics), sort_keys=True, allow_nan=False))
            sys.stdout.flush()
            for publisher in goal_publishers:
                publisher.unregister()


if __name__ == "__main__":
    rospy.init_node("b1_independent_navigation_test", anonymous=True)
    rostest.rosrun(
        "bspline_race", "b1_independent_navigation_test",
        B1IndependentNavigationTest)
