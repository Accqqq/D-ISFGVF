#!/usr/bin/python3
"""Read-only B1E single-UAV equivalence diagnostic.

The launch file fixes the variant.  This process only observes the plant,
planner, map, and controller, publishes one goal, and emits one metrics line.
It deliberately does not use unittest/rostest assertions: strict B1 failure is
diagnostic data, while infrastructure failure is the process exit condition.
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
import xml.etree.ElementTree as ElementTree

sys.dont_write_bytecode = True

import rosgraph
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from nodelet.srv import NodeletList
from sensor_msgs.msg import Imu, PointCloud2

try:
    from quadrotor_msgs.msg import PositionCommand, SO3Command
except ImportError:
    # Keep the checked-in generated package as the safe package-only fallback.
    workspace = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../"))
    source_messages = os.path.join(workspace, "src/uav_simulator/Utils/quadrotor_msgs/src")
    sys.path.insert(0, source_messages)
    for module_name in list(sys.modules):
        if module_name == "quadrotor_msgs" or module_name.startswith("quadrotor_msgs."):
            del sys.modules[module_name]
    from quadrotor_msgs.msg import PositionCommand, SO3Command


WORKSPACE = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../"))
PILLAR_PATH = os.path.join(
    WORKSPACE, "src/uav_simulator/dynamic_map_generator/resource/pillar.pcd")

READINESS_SECONDS = 25.0
GOAL_CONNECTION_SECONDS = 15.0
OBSERVATION_SECONDS = 60.0
STABLE_SECONDS = 3.0
DRIFT_SECONDS = 5.0
ARRIVAL_SECONDS = 45.0
CELL_SIZE = 0.1

START = (-5.0, -3.50, 1.0)
GOAL = (5.0, -3.50, 1.0)

GLOBAL_LAYOUT = {
    "odom": "/sim/odom",
    "imu": "/sim/imu",
    "local_map": "/sim/local_map",
    "goal": "/move_base_simple/goal",
    "position_cmd": "/position_cmd",
    "so3_cmd": "/so3_cmd",
    "path": "/particle0/path",
}
NAMESPACED_LAYOUT = {
    "odom": "/uav_0/sim/odom",
    "imu": "/uav_0/sim/imu",
    "local_map": "/uav_0/sim/local_map",
    "goal": "/uav_0/goal",
    "position_cmd": "/uav_0/position_cmd",
    "so3_cmd": "/uav_0/so3_cmd",
    "path": "/uav_0/particle0/path",
}


def finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def stamp_seconds(message):
    try:
        value = message.header.stamp.to_sec()
    except (AttributeError, TypeError, ValueError):
        return None
    return value if finite(value) else None


def distance3(left, right):
    return math.sqrt(sum((float(a) - float(b)) ** 2
                         for a, b in zip(left, right)))


def xy_error(position, goal=GOAL):
    return math.hypot(position[0] - goal[0], position[1] - goal[1])


def z_error(position, goal=GOAL):
    return abs(position[2] - goal[2])


def speed_of(record):
    return math.sqrt(record[5] ** 2 + record[6] ** 2 + record[7] ** 2)


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


class PcdSpatialHash:
    """Bounded XY hash used for raw (uninflated) path/physical clearance."""

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
                            raise ValueError("B1E requires ASCII PCD input")
                        data = True
                    continue
                fields = text.split()
                if len(fields) < 3:
                    continue
                x, y, z = (float(fields[0]), float(fields[1]), float(fields[2]))
                if not all(finite(value) for value in (x, y, z)):
                    continue
                # Match the protected B1 clearance band.
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
            raise ValueError("PCD has no flight-height points")

    def min_distance(self, x, y):
        if not (finite(x) and finite(y)):
            return float("inf")
        cx, cy = self._cell(x, y)
        radius_limit = max(abs(cx - self.min_cell[0]),
                           abs(cx - self.max_cell[0]),
                           abs(cy - self.min_cell[1]),
                           abs(cy - self.max_cell[1])) + 2
        best = float("inf")
        for radius in range(radius_limit + 1):
            for ix in range(cx - radius, cx + radius + 1):
                for iy in sorted(set((cy - radius, cy + radius))):
                    for px, py in self.cells.get((ix, iy), ()):
                        best = min(best, math.hypot(x - px, y - py))
            for iy in range(cy - radius + 1, cy + radius):
                for ix in (cx - radius, cx + radius):
                    for px, py in self.cells.get((ix, iy), ()):
                        best = min(best, math.hypot(x - px, y - py))
            if finite(best):
                right = max(0.0, (cx + radius + 1) * self.cell_size - x)
                left = max(0.0, x - (cx - radius) * self.cell_size)
                top = max(0.0, (cy + radius + 1) * self.cell_size - y)
                bottom = max(0.0, y - (cy - radius) * self.cell_size)
                if min(right, left, top, bottom) > best:
                    break
        return best


class AgentTrace:
    """Thread-safe bounded scalar traces; no raw point clouds are retained."""

    def __init__(self, topics, clearance_hash):
        self.topics = topics
        self.clearance_hash = clearance_hash
        self.lock = threading.Lock()
        self.odom = collections.deque(maxlen=16000)
        self.map_stamps = collections.deque(maxlen=4000)
        self.position = collections.deque(maxlen=8000)
        self.so3 = collections.deque(maxlen=8000)
        self.paths = collections.deque(maxlen=2000)
        self.odom_finite = True
        self.map_finite = True
        self.position_finite = True
        self.so3_finite = True
        self.path_finite = True
        self.odom_frame = ""
        self.child_frame = ""
        self.map_frame = ""
        self.path_frames = collections.deque(maxlen=2000)

        self.odom_sub = rospy.Subscriber(
            topics["odom"], Odometry, self.odom_callback, queue_size=300)
        self.map_sub = rospy.Subscriber(
            topics["local_map"], PointCloud2, self.map_callback, queue_size=30)
        self.position_sub = rospy.Subscriber(
            topics["position_cmd"], PositionCommand,
            self.position_callback, queue_size=100)
        self.so3_sub = rospy.Subscriber(
            topics["so3_cmd"], SO3Command, self.so3_callback, queue_size=100)
        self.path_sub = rospy.Subscriber(
            topics["path"], Path, self.path_callback, queue_size=20)

    def odom_callback(self, message):
        receive = time.monotonic()
        stamp = stamp_seconds(message)
        good = finite_odom(message) and stamp is not None
        p = message.pose.pose.position
        v = message.twist.twist.linear
        clearance = (self.clearance_hash.min_distance(p.x, p.y)
                     if good else float("inf"))
        record = (receive, stamp if stamp is not None else 0.0,
                  p.x, p.y, p.z, v.x, v.y, v.z, clearance)
        with self.lock:
            self.odom_finite = self.odom_finite and good
            self.odom_frame = str(message.header.frame_id)
            self.child_frame = str(message.child_frame_id)
            self.odom.append(record)

    def map_callback(self, message):
        receive = time.monotonic()
        stamp = stamp_seconds(message)
        good = stamp is not None
        with self.lock:
            self.map_finite = self.map_finite and good
            self.map_frame = str(message.header.frame_id)
            self.map_stamps.append((receive, stamp if stamp is not None else 0.0))

    def position_callback(self, message):
        receive = time.monotonic()
        stamp = stamp_seconds(message)
        good = finite_position_command(message) and stamp is not None
        p, v, a = message.position, message.velocity, message.acceleration
        record = (receive, stamp if stamp is not None else 0.0,
                  (p.x, p.y, p.z), (v.x, v.y, v.z), (a.x, a.y, a.z),
                  message.yaw, message.yaw_dot)
        with self.lock:
            self.position_finite = self.position_finite and good
            self.position.append(record)

    def so3_callback(self, message):
        receive = time.monotonic()
        stamp = stamp_seconds(message)
        good = finite_so3_command(message) and stamp is not None
        with self.lock:
            self.so3_finite = self.so3_finite and good
            self.so3.append((receive, stamp if stamp is not None else 0.0))

    def path_callback(self, message):
        receive = time.monotonic()
        stamp = stamp_seconds(message)
        good = stamp is not None
        endpoint = None
        minimum_clearance = float("inf")
        for pose in message.poses:
            p = pose.pose.position
            values = (p.x, p.y, p.z, pose.pose.orientation.x,
                      pose.pose.orientation.y, pose.pose.orientation.z,
                      pose.pose.orientation.w)
            if not all(finite(value) for value in values):
                good = False
                continue
            endpoint = (p.x, p.y, p.z)
            minimum_clearance = min(minimum_clearance,
                                    self.clearance_hash.min_distance(p.x, p.y))
        with self.lock:
            self.path_finite = self.path_finite and good
            frame = str(message.header.frame_id)
            self.path_frames.append(frame)
            self.paths.append((receive, stamp if stamp is not None else 0.0,
                               len(message.poses), frame, endpoint,
                               minimum_clearance))

    def snapshot(self):
        with self.lock:
            return {
                "odom": list(self.odom),
                "map_stamps": list(self.map_stamps),
                "position": list(self.position),
                "so3": list(self.so3),
                "paths": list(self.paths),
                "odom_finite": self.odom_finite,
                "map_finite": self.map_finite,
                "position_finite": self.position_finite,
                "so3_finite": self.so3_finite,
                "path_finite": self.path_finite,
                "odom_frame": self.odom_frame,
                "child_frame": self.child_frame,
                "map_frame": self.map_frame,
                "path_frames": list(self.path_frames),
            }


def wait_until(predicate, timeout, errors=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not rospy.is_shutdown():
        try:
            if predicate():
                return True
        except Exception as error:
            if errors is not None:
                errors.append("wait_until predicate exception: %s" % error)
            return False
        time.sleep(0.05)
    try:
        return bool(predicate())
    except Exception as error:
        if errors is not None:
            errors.append("wait_until final predicate exception: %s" % error)
        return False


def latest_odom_record(snapshot):
    """Return the latest locked odometry record, never the first record."""
    records = snapshot.get("odom", []) if isinstance(snapshot, dict) else []
    return records[-1] if records else None


def readiness_snapshot_ready(snapshot, start=START):
    """Evaluate all frozen readiness predicates from one locked snapshot."""
    if not isinstance(snapshot, dict):
        return False
    if not snapshot.get("odom") or not snapshot.get("map_stamps"):
        return False
    if not snapshot.get("odom_finite") or not snapshot.get("map_finite"):
        return False
    latest = latest_odom_record(snapshot)
    return latest is not None and distance3(latest[2:5], start) < 0.02


def strict_condition(record, goal=GOAL):
    position = record[2:5]
    return (xy_error(position, goal) <= 0.25 and
            z_error(position, goal) <= 0.10 and
            speed_of(record) <= 0.15)


def state_metrics(record, goal=GOAL):
    position = record[2:5]
    return {
        "receive_time_s": record[0],
        "header_stamp_s": record[1],
        "position_m": list(position),
        "xy_error_m": xy_error(position, goal),
        "z_error_m": z_error(position, goal),
        "3d_error_m": distance3(position, goal),
        "speed_mps": speed_of(record),
    }


def initial_physical_state(record, goal_publish_anchor=None, goal=GOAL):
    """Summarize the latest physical state immediately before goal publish."""
    if record is None:
        return None
    result = state_metrics(record, goal)
    result["velocity_mps"] = [record[5], record[6], record[7]]
    result["distance_to_start_m"] = distance3(record[2:5], START)
    if goal_publish_anchor is not None:
        result["relative_receive_s"] = goal_publish_anchor - record[0]
    return result


def first_entry(records, anchor, goal=GOAL):
    for record in records:
        if record[0] >= anchor and strict_condition(record, goal):
            result = state_metrics(record, goal)
            result["relative_receive_s"] = record[0] - anchor
            return result
    return None


def longest_dwell(records, anchor, goal=GOAL):
    best = None
    start = None
    previous = None
    for record in records:
        if record[0] < anchor:
            continue
        if strict_condition(record, goal):
            if start is None:
                start = record
            previous = record
            continue
        if start is not None and previous is not None:
            dwell = previous[0] - start[0]
            if best is None or dwell > best["duration_s"]:
                best = {"start_s": start[0] - anchor,
                        "end_s": previous[0] - anchor,
                        "duration_s": dwell}
        start = None
        previous = None
    if start is not None and previous is not None:
        dwell = previous[0] - start[0]
        if best is None or dwell > best["duration_s"]:
            best = {"start_s": start[0] - anchor,
                    "end_s": previous[0] - anchor,
                    "duration_s": dwell}
    return best


def stable_analysis(records, anchor, goal=GOAL):
    """Lock the first complete 3 s window and never search after drift fail."""
    result = {
        "first_window": None,
        "post_stable_drift": None,
        "strict_pass": False,
        "failure_components": [],
    }
    for index, record in enumerate(records):
        if record[0] < anchor:
            continue
        if record[0] - anchor > ARRIVAL_SECONDS:
            break
        if not strict_condition(record, goal):
            continue
        end_time = record[0] + STABLE_SECONDS
        endpoint_index = None
        for candidate in range(index, len(records)):
            if records[candidate][0] >= end_time:
                endpoint_index = candidate
                break
        if endpoint_index is None:
            continue
        window = records[index:endpoint_index + 1]
        if not all(strict_condition(item, goal) for item in window):
            continue

        reference = records[endpoint_index]
        result["first_window"] = {
            "start_s": record[0] - anchor,
            "end_s": reference[0] - anchor,
            "duration_s": reference[0] - record[0],
            "reference": state_metrics(reference, goal),
        }
        drift_end = reference[0] + DRIFT_SECONDS
        drift_index = None
        for candidate in range(endpoint_index, len(records)):
            if records[candidate][0] >= drift_end:
                drift_index = candidate
                break
        if drift_index is None:
            result["failure_components"].append("post_stable_drift_observation_incomplete")
            return result
        maximum = 0.0
        for item in records[endpoint_index:drift_index + 1]:
            maximum = max(maximum, distance3(item[2:5], reference[2:5]))
        result["post_stable_drift"] = {
            "reference_s": reference[0] - anchor,
            "end_s": records[drift_index][0] - anchor,
            "maximum_displacement_m": maximum,
            "pass": maximum <= 0.10,
        }
        if maximum > 0.10:
            result["failure_components"].append("post_stable_drift")
            return result
        result["strict_pass"] = True
        return result
    result["failure_components"].append("first_stable_window")
    return result


def minimum_goal_metrics(records, anchor, goal=GOAL):
    after = [record for record in records if record[0] >= anchor]
    result = {
        "minimum_xy": None,
        "minimum_z": None,
        "minimum_3d": None,
        "minimum_speed_after_first_xy_entry": None,
    }
    if not after:
        return result
    for record in after:
        state = state_metrics(record, goal)
        for key in ("minimum_xy", "minimum_z", "minimum_3d"):
            value_key = {"minimum_xy": "xy_error_m",
                         "minimum_z": "z_error_m",
                         "minimum_3d": "3d_error_m"}[key]
            if result[key] is None or state[value_key] < result[key][value_key]:
                result[key] = state
    first_xy = None
    for index, record in enumerate(after):
        if xy_error(record[2:5], goal) <= 0.25:
            first_xy = index
            break
    if first_xy is not None:
        speed_record = min(after[first_xy:], key=speed_of)
        result["minimum_speed_after_first_xy_entry"] = state_metrics(
            speed_record, goal)
    return result


def threshold_observations(records, anchor, goal=GOAL):
    """Report joint threshold evidence without combining separate minima."""
    after = [record for record in records if record[0] >= anchor]
    ever_xy = any(xy_error(record[2:5], goal) <= 0.25 for record in after)
    ever_z = any(z_error(record[2:5], goal) <= 0.10 for record in after)
    ever_speed = any(speed_of(record) <= 0.15 for record in after)
    first_xy_index = next((index for index, record in enumerate(after)
                           if xy_error(record[2:5], goal) <= 0.25), None)
    after_first_xy = (after[first_xy_index:] if first_xy_index is not None
                      else [])
    ever_speed_after_first_xy = any(
        speed_of(record) <= 0.15 for record in after_first_xy)
    ever_xy_speed = any(
        xy_error(record[2:5], goal) <= 0.25 and
        speed_of(record) <= 0.15 for record in after)
    ever_xy_z = any(
        xy_error(record[2:5], goal) <= 0.25 and
        z_error(record[2:5], goal) <= 0.10 for record in after)
    ever_full = any(strict_condition(record, goal) for record in after)
    return {
        "ever_xy_ok": ever_xy,
        "ever_z_ok": ever_z,
        "ever_speed_ok": ever_speed,
        "ever_speed_ok_after_first_xy_entry": ever_speed_after_first_xy,
        "ever_xy_and_speed_ok": ever_xy_speed,
        "ever_xy_and_z_ok": ever_xy_z,
        "ever_full_strict_set": ever_full,
    }


def final_window_summary(records, anchor, goal=GOAL):
    window = [record for record in records
              if anchor + 55.0 <= record[0] <= anchor + 60.0]
    if not window:
        return {"count": 0}
    positions = [record[2:5] for record in window]
    speeds = [speed_of(record) for record in window]
    xy_values = [xy_error(position, goal) for position in positions]
    z_values = [z_error(position, goal) for position in positions]
    first = positions[0]
    return {
        "count": len(window),
        "first_position_m": list(first),
        "last_position_m": list(positions[-1]),
        "mean_position_m": [sum(position[i] for position in positions) /
                            float(len(positions)) for i in range(3)],
        "min_xy_error_m": min(xy_values),
        "max_xy_error_m": max(xy_values),
        "min_z_error_m": min(z_values),
        "max_z_error_m": max(z_values),
        "min_speed_mps": min(speeds),
        "max_speed_mps": max(speeds),
        "mean_speed_mps": sum(speeds) / float(len(speeds)),
        "maximum_displacement_m": max(distance3(position, first)
                                       for position in positions),
    }


def terminal_position_summary(position_records, odom_records, anchor, goal=GOAL,
                              observation_duration=None):
    after = [item for item in position_records if item[0] >= anchor]
    if not after:
        end_relative = (OBSERVATION_SECONDS if observation_duration is None
                        else max(0.0, float(observation_duration)))
        return {
            "count": 0,
            "last_command": None,
            "silence_to_observation_end_s": end_relative,
            "physical_state_closest_to_last_command": None,
            "final_physical_state": None,
            "physical_distance_closest_to_last_command_m": None,
            "final_physical_distance_from_last_command_m": None,
        }
    last = after[-1]
    relative = last[0] - anchor
    end_relative = (OBSERVATION_SECONDS if observation_duration is None
                    else max(0.0, float(observation_duration)))
    after_odom = [record for record in odom_records if record[0] >= anchor]
    closest = min(after_odom, key=lambda record: abs(record[0] - last[0]),
                  default=None)
    final_state = after_odom[-1] if after_odom else None
    last_position, last_velocity, last_acceleration = last[2], last[3], last[4]
    command_error = {
        "xy_error_m": xy_error(last_position, goal),
        "z_error_m": z_error(last_position, goal),
        "3d_error_m": distance3(last_position, goal),
    }
    result = {
        "count": len(after),
        "first_relative_receive_s": after[0][0] - anchor,
        "last_relative_receive_s": relative,
        "last_command": {
            "relative_receive_s": relative,
            "header_stamp_s": last[1],
            "position_m": list(last_position),
            "velocity_mps": list(last_velocity),
            "acceleration_mps2": list(last_acceleration),
            "yaw_rad": last[5],
            "yaw_rate_rps": last[6],
            "goal_error": command_error,
        },
        "silence_to_observation_end_s": max(0.0, end_relative - relative),
        "physical_state_closest_to_last_command": None,
        "final_physical_state": None,
        "physical_distance_closest_to_last_command_m": None,
        "final_physical_distance_from_last_command_m": None,
    }
    if closest is not None:
        result["physical_state_closest_to_last_command"] = state_metrics(
            closest, goal)
        result["physical_state_closest_to_last_command"]["relative_receive_s"] = (
            closest[0] - anchor)
        result["physical_distance_closest_to_last_command_m"] = distance3(
            closest[2:5], last_position)
    if final_state is not None:
        result["final_physical_state"] = state_metrics(final_state, goal)
        result["final_physical_state"]["relative_receive_s"] = (
            final_state[0] - anchor)
        result["final_physical_distance_from_last_command_m"] = distance3(
            final_state[2:5], last_position)
    return result


def path_summary(path_records, anchor, goal=GOAL):
    after = [item for item in path_records if item[0] >= anchor]
    nonempty = [item for item in after if item[2] > 0 and item[4] is not None]
    frames = sorted(set(item[3] for item in after))
    result = {
        "count": len(nonempty),
        "all_count": len(after),
        "frames": frames,
        "nonempty_frames": sorted(set(item[3] for item in nonempty)),
        "minimum_raw_pcd_clearance_m": None,
        "last_endpoint": None,
    }
    if nonempty:
        result["minimum_raw_pcd_clearance_m"] = min(item[5] for item in nonempty)
        last = nonempty[-1]
        endpoint = last[4]
        result["last_endpoint"] = {
            "position_m": list(endpoint),
            "header_stamp_s": last[1],
            "goal_xy_error_m": xy_error(endpoint, goal),
            "goal_z_error_m": z_error(endpoint, goal),
            "goal_3d_error_m": distance3(endpoint, goal),
        }
    return result


def stream_summary(samples):
    if not samples:
        return {"count": 0, "finite": False, "timestamp_progress": False,
                "rate_hz": 0.0}
    stamps = [item[1] for item in samples]
    finite_stamps = all(finite(stamp) for stamp in stamps)
    progress = finite_stamps and len(stamps) >= 2 and stamps[-1] > stamps[0]
    rate = ((len(stamps) - 1) / (stamps[-1] - stamps[0])
            if progress else 0.0)
    return {"count": len(samples), "finite": finite_stamps,
            "timestamp_progress": progress, "rate_hz": rate,
            "first_header_stamp_s": stamps[0],
            "last_header_stamp_s": stamps[-1]}


def invert_endpoint_state(entries):
    """Invert ROS master `(endpoint, [node, ...])` into node ownership."""
    inverted = {}
    for endpoint, nodes in entries:
        endpoint = str(endpoint)
        for node in nodes:
            inverted.setdefault(str(node), set()).add(endpoint)
    return inverted


def decode_system_state_sections(system_state):
    """Validate and return the direct three-section Master.getSystemState value."""
    if not isinstance(system_state, (list, tuple)) or len(system_state) != 3:
        raise ValueError("ROS master system state must contain three sections")
    if not all(isinstance(section, (list, tuple)) for section in system_state):
        raise ValueError("ROS master system state sections must be list-like")
    return tuple(system_state)


def get_system_state():
    master = rosgraph.Master(rospy.get_name())
    publishers, subscribers, services = decode_system_state_sections(
        master.getSystemState())
    return (invert_endpoint_state(publishers),
            invert_endpoint_state(subscribers),
            invert_endpoint_state(services))


def normalize_names(values):
    return sorted("/" + str(value).lstrip("/") for value in values)


def extract_agent_ids(value):
    return set(int(match.group(1)) for match in re.finditer(
        r"(?:^|/)uav_(\d+)(?:/|$)", str(value)))


def invert_node_endpoints(node_endpoints):
    inverted = collections.defaultdict(set)
    for node, endpoints in node_endpoints.items():
        for endpoint in endpoints:
            inverted[endpoint].add(node)
    return inverted


def endpoint_ownership_audit(publishers, subscribers):
    """Build real topic edges from the inverted ROS master endpoint state."""
    publisher_topics = invert_node_endpoints(publishers)
    subscriber_topics = invert_node_endpoints(subscribers)
    all_endpoints = set(publisher_topics) | set(subscriber_topics)
    unexpected = []
    for topic in sorted(all_endpoints):
        topic_ids = extract_agent_ids(topic)
        for role, nodes in (("publisher", publisher_topics.get(topic, set())),
                            ("subscriber", subscriber_topics.get(topic, set()))):
            for node in sorted(nodes):
                ids = extract_agent_ids(node) | topic_ids
                for agent_id in sorted(ids):
                    if agent_id != 0:
                        unexpected.append({
                            "role": role,
                            "node": node,
                            "topic": topic,
                            "agent_id": agent_id,
                        })

    cross_agent = []
    for topic in sorted(all_endpoints):
        publishers_for_topic = sorted(publisher_topics.get(topic, set()))
        subscribers_for_topic = sorted(subscriber_topics.get(topic, set()))
        for publisher in publishers_for_topic:
            publisher_ids = extract_agent_ids(publisher) | extract_agent_ids(topic)
            for subscriber in subscribers_for_topic:
                subscriber_ids = extract_agent_ids(subscriber) | extract_agent_ids(topic)
                if publisher_ids and subscriber_ids and publisher_ids != subscriber_ids:
                    cross_agent.append({
                        "topic": topic,
                        "publisher": publisher,
                        "subscriber": subscriber,
                        "publisher_agent_ids": sorted(publisher_ids),
                        "subscriber_agent_ids": sorted(subscriber_ids),
                    })

    shared_algorithm = []
    algorithm_nodes = {
        "/uav_0/formation_planning", "/uav_0/map_generator",
    }
    infrastructure_topics = {"/mock_map"}
    for topic in sorted(all_endpoints):
        if topic in infrastructure_topics:
            continue
        publishers_for_topic = sorted(publisher_topics.get(topic, set()))
        subscribers_for_topic = sorted(subscriber_topics.get(topic, set()))
        for algorithm_node in sorted(algorithm_nodes):
            if algorithm_node not in publishers_for_topic and algorithm_node not in subscribers_for_topic:
                continue
            counterparts = ((set(publishers_for_topic) | set(subscribers_for_topic)) -
                            {algorithm_node})
            for counterpart in sorted(counterparts):
                counterpart_ids = extract_agent_ids(counterpart) | extract_agent_ids(topic)
                if counterpart_ids and counterpart_ids != {0}:
                    shared_algorithm.append({
                        "topic": topic,
                        "algorithm_node": algorithm_node,
                        "counterpart": counterpart,
                        "counterpart_agent_ids": sorted(counterpart_ids),
                    })
    return {
        "unexpected_agent_endpoints": unexpected,
        "cross_agent_edges": cross_agent,
        "shared_algorithm_edges": shared_algorithm,
        "publisher_topics": {topic: sorted(nodes)
                              for topic, nodes in publisher_topics.items()},
        "subscriber_topics": {topic: sorted(nodes)
                              for topic, nodes in subscriber_topics.items()},
    }


def graph_audit(config):
    publishers, subscribers, services = get_system_state()
    all_topics = set()
    for endpoint_set in list(publishers.values()) + list(subscribers.values()):
        all_topics.update(endpoint_set)
    node_names = set(publishers) | set(subscribers) | set(services)
    expected_nodes = set(config["expected_nodes"])
    missing_nodes = sorted(expected_nodes - node_names)

    legacy_global_topics = {
        "/sim/odom", "/sim/imu", "/sim/local_map",
        "/move_base_simple/goal", "/position_cmd", "/so3_cmd",
        "/motors", "/corrections", "/force_disturbance",
        "/moment_disturbance", "/gvf_force", "/path_vis", "/goal_vis",
        "/optimization_path", "/particle0/path", "/particle0/kinopath",
        "/particle0/circle_reference", "/particle0sdf_map/occupancy",
        "/particle0sdf_map/occupancy_inflate", "/particle0sdf_map/esdf",
        "/particle0sdf_map/update_range", "/particle0sdf_map/map_boundary",
        "/particle0sdf_map/unknown", "/particle0sdf_map/depth_cloud",
        "/particle0/gvf/occupancy", "/particle0/gvf/occupancy_inflate",
        "/particle0/gvf/esdf", "/particle0/gvf/update_range",
        "/particle0/gvf/traj_vis", "/particle0/gvf/vector_field",
        "/manual_obstacle", "/manual_boundary",
        "/manual_map", "/manual_map/add_obstacle_center", "/manual_map/add_boundary_point",
        "/manual_map/occupancy", "/clicked_point", "/dynamic/obj",
    }
    if config["layout"] == "global":
        expected_topics = {
            "/mock_map", "/sim/odom", "/sim/imu", "/sim/local_map",
            "/move_base_simple/goal", "/position_cmd", "/so3_cmd",
            "/force_disturbance", "/moment_disturbance", "/motors",
            "/corrections", "/particle0/path",
        }
        forbidden_operational = set(NAMESPACED_LAYOUT.values())
    else:
        prefix = "/uav_0"
        expected_topics = {
            "/mock_map", prefix + "/sim/odom", prefix + "/sim/imu",
            prefix + "/sim/local_map", prefix + "/goal",
            prefix + "/position_cmd", prefix + "/so3_cmd",
            prefix + "/force_disturbance", prefix + "/moment_disturbance",
            prefix + "/motors", prefix + "/corrections",
            prefix + "/particle0/path",
        }
        forbidden_operational = legacy_global_topics
    missing_topics = sorted(expected_topics - all_topics)
    forbidden_topics = sorted(
        topic for topic in all_topics
        if topic in forbidden_operational or
        (config["layout"] == "global" and topic.startswith("/uav_")) or
        (config["layout"] == "namespaced" and
         (topic.startswith("/particle0/") or
          topic.startswith("/particle0sdf_map/") or
          topic.startswith("/particle0/gvf/") or
          topic == "/manual_map" or topic.startswith("/manual_map/"))))
    semantic_fragments = (
        "phase_offset_swarm", "swarm_intent", "swarm", "allocator",
        "cbf", "tube", "scenario", "visualizer", "rviz")
    forbidden_semantic_topics = sorted(
        topic for topic in all_topics
        if any(fragment in topic.lower() for fragment in semantic_fragments))
    forbidden_nodes = sorted(
        node for node in node_names
        if any(fragment in node.lower() for fragment in semantic_fragments))

    nodelet = {
        "required": config["so3_topology"] == "manager",
        "nodelet_list": [],
        "nodelet_expected": config["expected_nodelets"],
        "nodelet_mismatch": [],
        "nodelet_service_error": "",
    }
    if nodelet["required"]:
        try:
            rospy.wait_for_service("/so3_nodelet_manager/list", timeout=5.0)
            response = rospy.ServiceProxy("/so3_nodelet_manager/list", NodeletList)()
            nodelet["nodelet_list"] = normalize_names(response.nodelets)
            nodelet["nodelet_mismatch"] = sorted(
                set(nodelet["nodelet_list"]) ^ set(nodelet["nodelet_expected"]))
        except Exception as error:
            nodelet["nodelet_service_error"] = str(error)
    else:
        nodelet["unexpected_manager_node"] = "/so3_nodelet_manager" in node_names

    unexpected_plant_nodes = []
    if config["plant"] == "multi" and "/quadrotor_simulator_so3" in node_names:
        unexpected_plant_nodes.append("/quadrotor_simulator_so3")
    if config["plant"] == "original" and "/multi_quadrotor_simulator_so3" in node_names:
        unexpected_plant_nodes.append("/multi_quadrotor_simulator_so3")
    unexpected_so3_topology_nodes = []
    if config["so3_topology"] == "manager" and "/so3_control" in node_names:
        unexpected_so3_topology_nodes.append("/so3_control")
    if config["so3_topology"] == "standalone":
        if "/so3_nodelet_manager" in node_names:
            unexpected_so3_topology_nodes.append("/so3_nodelet_manager")
        unexpected_so3_topology_nodes.extend(
            sorted(node for node in node_names
                   if re.match(r"^/so3_control_nodelet_\d+$", node)))
    unexpected_global_nodes = []
    if config["layout"] == "namespaced":
        for node in ("/formation_planning", "/map_generator"):
            if node in node_names:
                unexpected_global_nodes.append(node)
    unexpected_nonzero_agent_nodes = sorted(
        node for node in node_names
        if any(agent_id != 0 for agent_id in extract_agent_ids(node)))
    unexpected_agent_nodes = sorted(
        node for node in node_names
        if config["layout"] == "global" and "/uav_" in node)

    manager_topics = set(publishers.get("/so3_nodelet_manager", set()))
    manager_topics.update(subscribers.get("/so3_nodelet_manager", set()))
    operational_suffixes = ("/sim/odom", "/sim/imu", "/position_cmd",
                            "/motors", "/corrections", "/so3_cmd")
    manager_operational = sorted(
        topic for topic in manager_topics
        if any(topic.endswith(suffix) for suffix in operational_suffixes))
    if config["so3_topology"] == "manager":
        expected_manager_operational = sorted(
            config["topics"][key] for key in
            ("odom", "imu", "position_cmd", "so3_cmd"))
        expected_manager_operational += [
            config["topics"]["motors"], config["topics"]["corrections"]]
    else:
        expected_manager_operational = []
    manager_endpoint_mismatch = {
        "missing": sorted(set(expected_manager_operational) -
                           set(manager_operational)),
        "unexpected": sorted(set(manager_operational) -
                              set(expected_manager_operational)),
    }

    # Endpoint ownership is derived by inversion; manager caller identity is
    # intentionally not treated as logical nodelet ownership.
    ownership = endpoint_ownership_audit(publishers, subscribers)
    return {
        "expected_nodes": sorted(expected_nodes),
        "missing_nodes": missing_nodes,
        "all_topics": sorted(all_topics),
        "expected_topics": sorted(expected_topics),
        "missing_topics": missing_topics,
        "forbidden_topics": forbidden_topics,
        "forbidden_semantic_topics": forbidden_semantic_topics,
        "forbidden_nodes": forbidden_nodes,
        "unexpected_plant_nodes": sorted(unexpected_plant_nodes),
        "unexpected_so3_topology_nodes": sorted(unexpected_so3_topology_nodes),
        "unexpected_global_nodes": sorted(unexpected_global_nodes),
        "unexpected_nonzero_agent_nodes": unexpected_nonzero_agent_nodes,
        "unexpected_agent_nodes": unexpected_agent_nodes,
        "edge_audit_basis": {
            "agent_id_pattern": r"(?:^|/)uav_(\d+)(?:/|$)",
            "critical_algorithm_nodes": [
                "/uav_0/formation_planning", "/uav_0/map_generator"],
            "infrastructure_topics_excluded_from_shared_edges": ["/mock_map"],
            "manager_caller_identity_used_for_logical_nodelet_ownership": False,
        },
        "unexpected_agent_endpoints": ownership["unexpected_agent_endpoints"],
        "cross_agent_edges": ownership["cross_agent_edges"],
        "shared_algorithm_edges": ownership["shared_algorithm_edges"],
        "publisher_topics": ownership["publisher_topics"],
        "subscriber_topics": ownership["subscriber_topics"],
        "node_count": len(node_names),
        "nodelet": nodelet,
        "so3_manager_topics": sorted(manager_topics),
        "so3_manager_operational_topics": manager_operational,
        "so3_manager_expected_topics": expected_manager_operational,
        "so3_manager_endpoint_mismatch": manager_endpoint_mismatch,
        "publishers_by_node": {node: sorted(topics)
                                for node, topics in publishers.items()},
        "subscribers_by_node": {node: sorted(topics)
                                 for node, topics in subscribers.items()},
    }


def compare_param(values, failures, key, expected, allow_absent=False,
                  effective_default=None):
    try:
        actual = rospy.get_param(key)
    except KeyError:
        values[key] = {
            "present": False,
            "effective_value": (effective_default if allow_absent else None),
        }
        if not allow_absent:
            failures.append("%s missing" % key)
        return
    values[key] = ({"present": True, "value": actual,
                    "effective_value": actual}
                   if allow_absent else actual)
    if isinstance(expected, float):
        if not finite(actual) or abs(float(actual) - expected) > 1e-9:
            failures.append("%s=%r expected %r" % (key, actual, expected))
    elif actual != expected:
        failures.append("%s=%r expected %r" % (key, actual, expected))


def parameter_audit(config):
    values = {}
    failures = []
    if config["layout"] == "global":
        planner = "/formation_planning"
        map_node = "/map_generator"
    else:
        planner = "/uav_0/formation_planning"
        map_node = "/uav_0/map_generator"

    if config["plant"] == "original":
        plant = "/quadrotor_simulator_so3"
        compare_param(values, failures, plant + "/rate/simulation", 1000.0)
        compare_param(values, failures, plant + "/rate/odom", 100.0)
        compare_param(values, failures, plant + "/simulator/frame_id",
                      config["plant_frame"])
        compare_param(values, failures, plant + "/simulator/start_at_hover",
                      config["hover"])
        compare_param(values, failures, plant + "/simulator/init_state_x", -5.0)
        compare_param(values, failures, plant + "/simulator/init_state_y", -3.50)
        compare_param(values, failures, plant + "/simulator/init_state_z", 1.0)
    else:
        plant = "/multi_quadrotor_simulator_so3"
        compare_param(values, failures, plant + "/rate/simulation", 1000.0)
        compare_param(values, failures, plant + "/rate/odom", 100.0)
        compare_param(values, failures, plant + "/frame_id", config["plant_frame"])
        compare_param(values, failures, plant + "/start_at_hover", config["hover"])
        compare_param(values, failures, plant + "/num_agents", 1)

    so3 = "/so3_control" if config["so3_topology"] == "standalone" \
        else "/so3_control_nodelet_0"
    compare_param(values, failures, so3 + "/mass", 0.98)
    compare_param(values, failures, so3 + "/use_angle_corrections", False)
    compare_param(values, failures, so3 + "/use_external_yaw",
                  config["use_external_yaw"])
    compare_param(values, failures, so3 + "/gains/rot/z", 1.0)
    compare_param(values, failures, so3 + "/gains/ang/z", 0.1)

    compare_param(values, failures, planner + "/gvf/odom_topic",
                  config["topics"]["odom"])
    compare_param(values, failures, planner + "/gvf/cloud_topic",
                  config["topics"]["local_map"])
    compare_param(values, failures, planner + "/gvf/cmd_topic",
                  config["topics"]["position_cmd"])
    compare_param(values, failures, planner + "/phase_offset/mode", "disabled")
    compare_param(values, failures, planner + "/gvf/gvf_gain1", config["gain1"])
    compare_param(values, failures, planner + "/gvf/gvf_gain2", config["gain2"])
    compare_param(values, failures, planner + "/sdf_map/frame_id", "world")
    compare_param(values, failures, planner + "/sdf_map/enable_manual_map",
                  config["manual_enable"])
    compare_param(values, failures, planner + "/sdf_map/manual_map_auto_load",
                  config["manual_load"])
    compare_param(values, failures, planner + "/sdf_map/manual_map_auto_save",
                  config["manual_save"])
    defaults = {
        "odom_topic": "/sim/odom",
        "global_map_topic": "/mock_map",
        "local_map_topic": "/sim/local_map",
        "output_frame": "world",
    }
    defaults_are_effective = (config["layout"] == "global" and
                              config["variant_id"] in ("E0", "E1"))
    compare_param(values, failures, map_node + "/odom_topic",
                  config["topics"]["odom"],
                  allow_absent=defaults_are_effective,
                  effective_default=defaults["odom_topic"])
    compare_param(values, failures, map_node + "/global_map_topic", "/mock_map",
                  allow_absent=defaults_are_effective,
                  effective_default=defaults["global_map_topic"])
    compare_param(values, failures, map_node + "/local_map_topic",
                  config["topics"]["local_map"],
                  allow_absent=defaults_are_effective,
                  effective_default=defaults["local_map_topic"])
    compare_param(values, failures, map_node + "/output_frame", "world",
                  allow_absent=defaults_are_effective,
                  effective_default=defaults["output_frame"])
    if config["variant_id"] == "E0":
        compare_param(
            values, failures, planner + "/sdf_map/manual_map_file",
            os.path.join(WORKSPACE,
                         "src/swarm_planner/bspline_traj/config/manual_obstacles_test_gvf.txt"))
    return {
        "values": values,
        "failures": failures,
        "effective_map_generator_contract": (
            defaults if defaults_are_effective else "explicit launch parameters"),
    }


def sanitize(value):
    if isinstance(value, float):
        return value if finite(value) else None
    if isinstance(value, (str, int, bool)) or value is None:
        return value
    if isinstance(value, dict):
        return {str(key): sanitize(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [sanitize(item) for item in value]
    return str(value)


def parse_gtest_xml_path(argv=None):
    """Accept only the standard absolute --gtest_output=xml:<path> argument."""
    arguments = sys.argv[1:] if argv is None else list(argv)
    paths = []
    malformed = False
    prefix = "--gtest_output="
    for argument in arguments:
        if not str(argument).startswith(prefix):
            continue
        value = str(argument)[len(prefix):]
        if not value.startswith("xml:"):
            malformed = True
            continue
        path = value[len("xml:"):]
        if not path or not os.path.isabs(path):
            malformed = True
            continue
        paths.append(path)
    if malformed or len(paths) != 1:
        return None
    return paths[0]


def bounded_failure_message(failures):
    """Make a bounded, secret-safe message from infrastructure labels only."""
    fragments = []
    for failure in failures:
        text = re.sub(r"/[A-Za-z0-9_.~+@%=-]+(?:/[A-Za-z0-9_.~+@%=-]+)*",
                      "<path>", str(failure))
        text = re.sub(r"https?://[^ ]+", "<endpoint>", text)
        text = "".join(character if character.isprintable() else " "
                        for character in text)
        fragments.append(text[:160])
    message = "; ".join(fragments)[:512]
    return message or "B1E infrastructure validation failed"


def junit_xml_bytes(run_valid, failures):
    """Serialize the minimal UTF-8 JUnit artifact without metrics or environment."""
    root = ElementTree.Element(
        "testsuite", {
            "name": "b1e_single_equivalence_diagnostic",
            "tests": "1",
            "errors": "0",
            "failures": "0" if run_valid else "1",
        })
    testcase = ElementTree.SubElement(
        root, "testcase", {"classname": "b1e", "name": "diagnostic"})
    if not run_valid:
        failure = ElementTree.SubElement(
            testcase, "failure", {"type": "B1EInfrastructureFailure"})
        failure.text = bounded_failure_message(failures)
    return ElementTree.tostring(root, encoding="utf-8", xml_declaration=True)


def write_junit_result(path, run_valid, failures):
    """Write exactly one minimal JUnit result to the requested absolute path."""
    if not path or not os.path.isabs(path):
        raise ValueError("missing absolute gtest XML result path")
    payload = junit_xml_bytes(run_valid, failures)
    with open(path, "wb") as stream:
        stream.write(payload)


def finalize_metrics(metrics, config=None):
    failures = list(dict.fromkeys(metrics.get("infrastructure_failures", [])))
    metrics["infrastructure_failures"] = failures
    metrics["run_valid"] = not failures
    stable = metrics.get("stable_arrival", {})
    metrics["strict_b1_pass"] = bool(stable.get("strict_pass", False))
    metrics["strict_failure_components"] = list(
        stable.get("failure_components", []))
    if config is not None:
        metrics["protected_variant_facts"] = {
            "variant_id": config["variant_id"],
            "layout": config["layout"],
            "plant": config["plant"],
            "so3_topology": config["so3_topology"],
            "plant_frame": config["plant_frame"],
            "start_at_hover": config["hover"],
            "use_external_yaw": config["use_external_yaw"],
            "gvf_gain1": config["gain1"],
            "gvf_gain2": config["gain2"],
            "manual_flags": {
                "enable": config["manual_enable"],
                "auto_load": config["manual_load"],
                "auto_save": config["manual_save"],
            },
        }


def synthetic_records(points):
    # points are (receive, x, y, z, vx, vy, vz); header stamp follows receive.
    return [(point[0], point[0], point[1], point[2], point[3],
             point[4], point[5], point[6], float("inf")) for point in points]


def synthetic_checks():
    checks = {}
    goal = (0.0, 0.0, 1.0)
    direct_system_state = [
        [["/topic_a", ["/node_pub"]]],
        [["/topic_a", ["/node_sub"]]],
        [["/service_a", ["/node_srv"]]],
    ]
    decoded_system_state = decode_system_state_sections(direct_system_state)
    checks["ros_master_system_state_contract"] = (
        len(decoded_system_state) == 3 and
        decoded_system_state[0][0][0] == "/topic_a" and
        decoded_system_state[1][0][1] == ["/node_sub"] and
        decoded_system_state[2][0][0] == "/service_a")

    early_far = synthetic_records(
        [(0.0, -4.90, -3.50, 1.0, 0.0, 0.0, 0.0)])[0]
    later_near = synthetic_records(
        [(1.0, -5.00, -3.50, 1.0, 0.0, 0.0, 0.0)])[0]
    readiness_snapshot = {
        "odom": [early_far, later_near],
        "map_stamps": [(0.5, 0.5)],
        "odom_finite": True,
        "map_finite": True,
    }
    selected = latest_odom_record(readiness_snapshot)
    checks["latest_pre_goal_state"] = (
        selected == later_near and
        readiness_snapshot_ready(readiness_snapshot))

    valid_xml_path = parse_gtest_xml_path(
        ["--gtest_output=xml:/tmp/b1e-result.xml"])
    checks["junit_argument_parsing"] = (
        valid_xml_path == "/tmp/b1e-result.xml" and
        parse_gtest_xml_path(["--gtest_output=xml:relative.xml"]) is None and
        parse_gtest_xml_path(["--gtest_output=json:/tmp/b1e-result.json"]) is None)
    pass_root = ElementTree.fromstring(junit_xml_bytes(True, []))
    fail_root = ElementTree.fromstring(
        junit_xml_bytes(False, ["graph audit failed", "/secret/path leaked"]))
    pass_failure = pass_root.find("testcase/failure")
    fail_failure = fail_root.find("testcase/failure")
    checks["junit_pass_xml"] = (
        pass_root.tag == "testsuite" and pass_root.get("tests") == "1" and
        pass_root.get("failures") == "0" and
        len(pass_root.findall("testcase")) == 1 and pass_failure is None and
        b"B1E_METRICS" not in junit_xml_bytes(True, []))
    checks["junit_pass_strict_false"] = (
        ElementTree.fromstring(
            junit_xml_bytes(True, ["strict_b1_pass=false"])).get("failures") == "0")
    checks["junit_fail_xml"] = (
        fail_root.tag == "testsuite" and fail_root.get("tests") == "1" and
        fail_root.get("failures") == "1" and
        len(fail_root.findall("testcase")) == 1 and
        fail_failure is not None and fail_failure.text and
        "/secret/path" not in fail_failure.text and
        b"B1E_METRICS" not in junit_xml_bytes(False, ["graph audit failed"]))

    stable_points = [(float(index), 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
                     for index in range(10)]
    strict = stable_analysis(synthetic_records(stable_points), 0.0, goal)
    checks["strict_pass"] = strict["strict_pass"]

    xy_z_fail = synthetic_records([(0.0, 0.1, 0.1, 1.3, 0.0, 0.0, 0.0)])
    checks["xy_pass_z_fail"] = (xy_error(xy_z_fail[0][2:5], goal) <= 0.25 and
                                 z_error(xy_z_fail[0][2:5], goal) > 0.10 and
                                 not strict_condition(xy_z_fail[0], goal))
    xy_z_observations = threshold_observations(xy_z_fail, 0.0, goal)
    checks["xy_z_joint_component_semantics"] = (
        xy_z_observations["ever_xy_ok"] and
        not xy_z_observations["ever_z_ok"] and
        not xy_z_observations["ever_xy_and_z_ok"] and
        not xy_z_observations["ever_full_strict_set"])
    speed_fail = synthetic_records([(0.0, 0.1, 0.1, 1.0, 0.2, 0.0, 0.0)])
    checks["xy_z_pass_speed_fail"] = (
        xy_error(speed_fail[0][2:5], goal) <= 0.25 and
        z_error(speed_fail[0][2:5], goal) <= 0.10 and
        speed_of(speed_fail[0]) > 0.15 and
        not strict_condition(speed_fail[0], goal))
    speed_observations = threshold_observations(speed_fail, 0.0, goal)
    checks["speed_joint_component_semantics"] = (
        speed_observations["ever_xy_ok"] and
        speed_observations["ever_z_ok"] and
        speed_observations["ever_xy_and_z_ok"] and
        not speed_observations["ever_speed_ok_after_first_xy_entry"] and
        not speed_observations["ever_full_strict_set"])

    # First complete 3 s window (t=0..3), first 5 s drift window
    # (reference t=3 through t=8) fails, then a later complete 3 s + 5 s
    # success window exists (t=9..14).  stable_analysis must remain frozen at
    # the first drift failure.
    drift_points = [(float(index), 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
                    for index in range(5)]
    drift_points += [(5.0, 0.3, 0.0, 1.0, 0.0, 0.0, 0.0),
                     (6.0, 0.3, 0.0, 1.0, 0.0, 0.0, 0.0),
                     (7.0, 0.3, 0.0, 1.0, 0.0, 0.0, 0.0),
                     (8.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)]
    drift_points += [(float(index), 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
                     for index in range(9, 19)]
    drift = stable_analysis(synthetic_records(drift_points), 0.0, goal)
    late_records = synthetic_records([point for point in drift_points
                                      if point[0] >= 9.0])
    late_success_analysis = stable_analysis(late_records, 9.0, goal)
    later_success = [item for item in drift_points if item[0] >= 9.0]
    checks["stable_pass_drift_fail"] = (
        drift["first_window"] is not None and
        drift["post_stable_drift"] is not None and
        not drift["strict_pass"] and
        "post_stable_drift" in drift["failure_components"])
    checks["first_drift_not_washed_out"] = (
        len(later_success) >= 10 and
        later_success[-1][0] - later_success[0][0] >= 9.0 and
        late_success_analysis["strict_pass"] and
        drift["first_window"] is not None and not drift["strict_pass"])

    final = final_window_summary(synthetic_records(
        [(55.0, 1.0, 2.0, 1.0, 0.1, 0.0, 0.0),
         (57.0, 1.2, 2.0, 1.0, 0.2, 0.0, 0.0),
         (60.0, 1.1, 2.0, 1.0, 0.1, 0.0, 0.0)]), 0.0,
                                 (1.0, 2.0, 1.0))
    checks["final_window_summary"] = final["count"] == 3

    commands = [(1.0, 1.0, (1.0, 2.0, 1.0), (0.1, 0.0, 0.0),
                 (0.0, 0.0, 0.0), 0.0, 0.0),
                (4.0, 4.0, (2.0, 2.0, 1.0), (0.0, 0.0, 0.0),
                 (0.0, 0.0, 0.0), 0.0, 0.0)]
    odom = synthetic_records([(4.1, 2.1, 2.0, 1.0, 0.0, 0.0, 0.0)])
    terminal = terminal_position_summary(commands, odom, 0.0,
                                         (5.0, 2.0, 1.0))
    checks["terminal_position_command_summary"] = (
        terminal["count"] == 2 and terminal["last_command"] is not None and
        terminal["physical_state_closest_to_last_command"] is not None)

    inversion = invert_endpoint_state([
        ("/topic_a", ["/node_a", "/node_b"]),
        ("/topic_b", ["/node_a"]),
    ])
    checks["ros_master_endpoint_inversion"] = (
        inversion["/node_a"] == {"/topic_a", "/topic_b"} and
        inversion["/node_b"] == {"/topic_a"})
    return {"cases": checks, "all_pass": all(checks.values())}


def read_config():
    variant = str(rospy.get_param("~variant_id", "unknown"))
    layout_name = str(rospy.get_param("~layout", "global"))
    topics = dict(GLOBAL_LAYOUT if layout_name == "global" else NAMESPACED_LAYOUT)
    if layout_name not in ("global", "namespaced"):
        layout_name = "unknown"
    topics["motors"] = "/motors" if layout_name == "global" else "/uav_0/motors"
    topics["corrections"] = ("/corrections" if layout_name == "global"
                              else "/uav_0/corrections")
    expected_nodes_text = str(rospy.get_param("~expected_nodes", ""))
    expected_nodelets_text = str(rospy.get_param("~expected_nodelets", ""))
    return {
        "variant_id": variant,
        "layout": layout_name,
        "topics": topics,
        "plant": str(rospy.get_param("~expected_plant", "")),
        "so3_topology": str(rospy.get_param("~expected_so3_topology", "")),
        "plant_frame": str(rospy.get_param("~expected_plant_frame", "")),
        "hover": bool(rospy.get_param("~expected_start_at_hover", False)),
        "use_external_yaw": bool(rospy.get_param("~expected_use_external_yaw", True)),
        "gain1": float(rospy.get_param("~expected_gain1", 2.0)),
        "gain2": float(rospy.get_param("~expected_gain2", -2.2)),
        "manual_enable": bool(rospy.get_param("~expected_manual_enable", False)),
        "manual_load": bool(rospy.get_param("~expected_manual_auto_load", False)),
        "manual_save": bool(rospy.get_param("~expected_manual_auto_save", False)),
        "expected_nodes": [item for item in expected_nodes_text.split("|") if item],
        "expected_nodelets": [item for item in expected_nodelets_text.split("|") if item],
    }


def record_infra(metrics, failures, text):
    failures.append(str(text))
    metrics.setdefault("infrastructure_failures", [])
    metrics["infrastructure_failures"] = failures


def run_diagnostic(config, metrics):
    failures = list(metrics.get("infrastructure_failures", []))
    trace = None
    goal_publisher = None
    anchor = None
    clearance_hash = None
    try:
        if config["layout"] not in ("global", "namespaced"):
            record_infra(metrics, failures, "unknown diagnostic layout")
        if config["plant"] not in ("original", "multi"):
            record_infra(metrics, failures, "unknown diagnostic plant")
        if config["so3_topology"] not in ("standalone", "manager"):
            record_infra(metrics, failures, "unknown SO3 topology")
        clearance_hash = PcdSpatialHash(PILLAR_PATH)
        metrics["pcd_points_in_height_band"] = clearance_hash.point_count
        trace = AgentTrace(config["topics"], clearance_hash)

        ready = wait_until(
            lambda: readiness_snapshot_ready(trace.snapshot()),
            READINESS_SECONDS, errors=failures)
        metrics["readiness"] = {
            "finite_odom_local_map_and_start_state": ready,
        }
        if not ready:
            record_infra(metrics, failures,
                         "finite odom/local-map/start-state readiness timed out")
            metrics["goal_publish_count"] = 0
            return 1

        goal_publisher = rospy.Publisher(
            config["topics"]["goal"], PoseStamped, queue_size=1, latch=True)
        connected = wait_until(lambda: goal_publisher.get_num_connections() > 0,
                               GOAL_CONNECTION_SECONDS, errors=failures)
        metrics["goal_subscriber_connected"] = connected
        if not connected:
            record_infra(metrics, failures, "goal subscriber connection timed out")

        # This is the latest locked state immediately before the one goal
        # publication; never substitute the first retained odometry sample.
        pre_goal_snapshot = trace.snapshot()
        if not readiness_snapshot_ready(pre_goal_snapshot):
            record_infra(metrics, failures,
                         "pre-goal odom/local-map/start-state revalidation failed")
            metrics["goal_publish_count"] = 0
            return 1
        initial = latest_odom_record(pre_goal_snapshot)
        if initial is None:
            metrics["initial_physical_state"] = None
            metrics["initial_position_error_m"] = None
            metrics["initial_state_relative_to_goal_publish_s"] = None
            record_infra(metrics, failures, "no pre-goal odometry sample")
            metrics["goal_publish_count"] = 0
            return 1
        initial_error = distance3(initial[2:5], START)
        metrics["initial_position_error_m"] = initial_error
        metrics["initial_physical_state"] = initial_physical_state(initial)
        metrics["readiness"]["latest_start_error_m"] = initial_error
        metrics["readiness"]["latest_start_state"] = metrics[
            "initial_physical_state"]
        if initial_error >= 0.02:
            record_infra(metrics, failures, "initial position error >= 0.02 m")
            metrics["initial_state_relative_to_goal_publish_s"] = None
            metrics["goal_publish_count"] = 0
            return 1

        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "world"
        goal.pose.position.x = GOAL[0]
        goal.pose.position.y = GOAL[1]
        goal.pose.position.z = GOAL[2]
        goal.pose.orientation.w = 1.0
        anchor = time.monotonic()
        metrics["initial_state_relative_to_goal_publish_s"] = (
            anchor - initial[0])
        metrics["initial_physical_state"] = initial_physical_state(
            initial, goal_publish_anchor=anchor)
        goal_publisher.publish(goal)
        metrics["goal_publish_count"] = 1
        metrics["goal_anchor_receive_monotonic_s"] = anchor
        metrics["expected_observation_duration_s"] = OBSERVATION_SECONDS

        # Never stop early on strict success/failure; collect the complete 60 s.
        deadline = anchor + OBSERVATION_SECONDS
        try:
            while time.monotonic() < deadline and not rospy.is_shutdown():
                time.sleep(0.05)
        except Exception as error:
            metrics["observation_duration_s"] = max(0.0, time.monotonic() - anchor)
            record_infra(metrics, failures,
                         "observation loop exception: %s" % error)
            raise
        observed_duration = max(0.0, time.monotonic() - anchor)
        metrics["observation_duration_s"] = observed_duration
        if observed_duration < OBSERVATION_SECONDS:
            record_infra(metrics, failures,
                         "observation ended before the frozen 60 s window")

        snapshot = trace.snapshot()
        records = snapshot["odom"]
        metrics["finite_data"] = bool(
            snapshot["odom_finite"] and snapshot["map_finite"] and
            snapshot["position_finite"] and snapshot["so3_finite"] and
            snapshot["path_finite"])
        if not metrics["finite_data"]:
            record_infra(metrics, failures, "non-finite observed stream data")

        post_odom = [item for item in snapshot["odom"] if item[0] >= anchor]
        post_map = [item for item in snapshot["map_stamps"] if item[0] >= anchor]
        post_position = [item for item in snapshot["position"] if item[0] >= anchor]
        post_so3 = [item for item in snapshot["so3"] if item[0] >= anchor]
        metrics["streams"] = {
            "odom": stream_summary(post_odom),
            "local_map": stream_summary(post_map),
            "position_command": stream_summary(post_position),
            "so3_command": stream_summary(post_so3),
        }
        metrics["post_goal_sample_counts"] = {
            "odom": len(post_odom),
            "local_map": len(post_map),
            "position_command": len(post_position),
            "so3_command": len(post_so3),
        }
        required_progress = {
            name: summary["timestamp_progress"]
            for name, summary in metrics["streams"].items()
        }
        metrics["post_goal_timestamp_progress"] = required_progress
        for name, progress in required_progress.items():
            if not progress:
                record_infra(metrics, failures,
                             "%s lacks positive post-goal timestamp progress" % name)
        odom_rate = metrics["streams"]["odom"]["rate_hz"]
        if not 80.0 <= odom_rate <= 130.0:
            record_infra(metrics, failures,
                         "post-goal odom rate outside 80-130 Hz")
        metrics["frames"] = {
            "odom": snapshot["odom_frame"],
            "odom_child": snapshot["child_frame"],
            "local_map": snapshot["map_frame"],
            "path": sorted(set(snapshot["path_frames"])),
        }
        metrics["minimum_goal_metrics"] = minimum_goal_metrics(records, anchor)
        metrics["strict_component_observations"] = threshold_observations(
            records, anchor)
        metrics["first_instantaneous_strict_set_entry"] = first_entry(records, anchor)
        metrics["longest_continuous_strict_set_dwell"] = longest_dwell(records, anchor)
        metrics["stable_arrival"] = stable_analysis(records, anchor)
        if not metrics["stable_arrival"].get("strict_pass", False):
            observations = metrics["strict_component_observations"]
            if not observations["ever_xy_ok"]:
                metrics["stable_arrival"]["failure_components"].append(
                    "xy_error_never_ok")
            if not observations["ever_z_ok"]:
                metrics["stable_arrival"]["failure_components"].append(
                    "z_error_never_ok")
            if (observations["ever_xy_ok"] and observations["ever_z_ok"] and
                    not observations["ever_xy_and_z_ok"]):
                metrics["stable_arrival"]["failure_components"].append(
                    "xy_z_never_jointly_ok")
            if (observations["ever_xy_and_z_ok"] and
                    not observations["ever_full_strict_set"]):
                metrics["stable_arrival"]["failure_components"].append(
                    "speed_never_jointly_ok_with_xy_and_z")
            metrics["stable_arrival"]["failure_components"] = list(dict.fromkeys(
                metrics["stable_arrival"]["failure_components"]))
        metrics["final_five_second_state"] = final_window_summary(records, anchor)
        metrics["position_command_terminal"] = terminal_position_summary(
            snapshot["position"], records, anchor,
            observation_duration=observed_duration)
        metrics["so3_summary"] = dict(metrics["streams"]["so3_command"])
        metrics["path"] = path_summary(snapshot["paths"], anchor)
        physical_records = [record for record in records if record[0] >= anchor]
        metrics["physical_clearance"] = {
            "minimum_raw_pcd_clearance_m": (
                min((record[8] for record in physical_records
                     if finite(record[8])), default=None)),
            "sample_count": len(physical_records),
        }
        if metrics["path"]["count"] == 0:
            record_infra(metrics, failures, "no non-empty installed path")

        if snapshot["odom_frame"] != config["plant_frame"]:
            record_infra(metrics, failures,
                         "odom frame does not match expected plant frame")
        if snapshot["map_frame"] != "world":
            record_infra(metrics, failures, "local map frame is not world")
        bad_path_frames = [frame for frame in metrics["path"]["frames"]
                           if frame not in ("", "world")]
        metrics["path"]["bad_frame_ids"] = bad_path_frames
        if bad_path_frames:
            record_infra(metrics, failures, "path frame is not world")

        try:
            metrics["parameters"] = parameter_audit(config)
            if metrics["parameters"]["failures"]:
                record_infra(metrics, failures, "parameter audit failed")
        except Exception as error:
            metrics["parameters"] = {"values": {}, "failures": [str(error)]}
            record_infra(metrics, failures, "parameter audit exception")
        try:
            metrics["graph"] = graph_audit(config)
            graph = metrics["graph"]
            graph_failures = (graph["missing_nodes"] + graph["missing_topics"] +
                              graph["forbidden_topics"] +
                              graph["forbidden_semantic_topics"] +
                              graph["forbidden_nodes"] +
                              graph["unexpected_plant_nodes"] +
                              graph["unexpected_so3_topology_nodes"] +
                              graph["unexpected_global_nodes"] +
                              graph["unexpected_nonzero_agent_nodes"] +
                              graph["unexpected_agent_nodes"] +
                              graph["unexpected_agent_endpoints"] +
                              graph["cross_agent_edges"] +
                              graph["shared_algorithm_edges"] +
                              graph["nodelet"]["nodelet_mismatch"] +
                              ([graph["nodelet"]["nodelet_service_error"]]
                               if graph["nodelet"]["nodelet_service_error"] else []) +
                              graph["so3_manager_endpoint_mismatch"]["missing"] +
                              graph["so3_manager_endpoint_mismatch"]["unexpected"])
            if graph.get("nodelet", {}).get("unexpected_manager_node", False):
                graph_failures.append("unexpected nodelet manager in standalone variant")
            if graph_failures:
                record_infra(metrics, failures, "graph audit failed")
        except Exception as error:
            metrics["graph"] = {"error": str(error)}
            record_infra(metrics, failures, "graph audit exception")
    except Exception as error:
        record_infra(metrics, failures, "diagnostic exception: %s" % error)
    finally:
        if goal_publisher is not None:
            try:
                goal_publisher.unregister()
            except Exception:
                pass
        metrics["infrastructure_failures"] = failures
        finalize_metrics(metrics, config)
    return 0 if metrics["run_valid"] else 1


def main():
    junit_path = parse_gtest_xml_path()
    config = None
    try:
        synthetic = synthetic_checks()
    except Exception as error:
        synthetic = {"cases": {}, "all_pass": False,
                     "error": "synthetic checks exception: %s" % error}
    metrics = {
        "variant_id": None,
        "run_valid": False,
        "infrastructure_failures": [],
        "strict_b1_pass": False,
        "strict_failure_components": [],
        "observation_duration_s": None,
        "expected_observation_duration_s": OBSERVATION_SECONDS,
        "initial_physical_state": None,
        "initial_position_error_m": None,
        "initial_state_relative_to_goal_publish_s": None,
        "goal_publish_count": 0,
        "python": {"executable": sys.executable,
                    "version": platform.python_version()},
        "start": list(START),
        "goal": list(GOAL),
        "synthetic_checks": synthetic,
    }
    if junit_path is None:
        metrics["infrastructure_failures"].append(
            "missing standard gtest XML result path")
    result = 1
    try:
        rospy.init_node("b1e_single_equivalence_diagnostic", anonymous=True)
        config = read_config()
        metrics["variant_id"] = config["variant_id"]
        metrics["configuration"] = {
            "layout": config["layout"],
            "topics": config["topics"],
            "expected_nodes": config["expected_nodes"],
            "expected_nodelets": config["expected_nodelets"],
        }
        if not synthetic["all_pass"]:
            metrics["infrastructure_failures"].append("synthetic checks failed")
        result = run_diagnostic(config, metrics)
    except Exception as error:
        record_infra(metrics, metrics["infrastructure_failures"],
                     "startup exception: %s" % error)
        result = 1
    finally:
        # Prepare the final validity state before creating the JUnit artifact.
        finalize_metrics(metrics, config)
        if junit_path is not None:
            try:
                write_junit_result(
                    junit_path, metrics["run_valid"],
                    metrics["infrastructure_failures"])
                metrics["junit_result"] = {"created": True}
            except Exception as error:
                record_infra(metrics, metrics["infrastructure_failures"],
                             "JUnit result creation failed: %s" % error)
                metrics["junit_result"] = {"created": False}
                result = 1
        else:
            metrics["junit_result"] = {"created": False}
            result = 1
        # XML creation can add an infrastructure failure; reconcile exit and
        # metrics semantics before the one and only B1E_METRICS line.
        finalize_metrics(metrics, config)
        result = 0 if metrics["run_valid"] else 1
        try:
            payload = json.dumps(sanitize(metrics), sort_keys=True,
                                 allow_nan=False)
        except Exception as error:
            record_infra(metrics, metrics["infrastructure_failures"],
                         "metrics serialization failed")
            finalize_metrics(metrics, config)
            if junit_path is not None:
                try:
                    write_junit_result(
                        junit_path, False, metrics["infrastructure_failures"])
                except Exception:
                    record_infra(metrics, metrics["infrastructure_failures"],
                                 "JUnit failure rewrite failed")
                finalize_metrics(metrics, config)
            try:
                payload = json.dumps(sanitize(metrics), sort_keys=True,
                                     allow_nan=False)
            except Exception:
                payload = json.dumps({
                    "variant_id": metrics.get("variant_id"),
                    "run_valid": False,
                    "strict_b1_pass": False,
                    "infrastructure_failures": [
                        "metrics serialization failed"],
                }, sort_keys=True, allow_nan=False)
            result = 1
        print("B1E_METRICS " + payload)
        sys.stdout.flush()
    return result


if __name__ == "__main__":
    sys.exit(main())
