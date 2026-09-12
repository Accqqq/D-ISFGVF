#!/usr/bin/env python3
"""Static and opt-in runtime checks for the SIM-B bringup boundary.

The static checks run without a ROS master and cover the frozen schema,
launch topology, install metadata, compatibility no-op classification, and
generic-N generation for N=1, 3, 8, and 50.  Set ``SIM_B_RUN_RUNTIME=1`` to
enable the longer ROS process/liveness matrix on a task-owned ROS master.
"""

from __future__ import print_function

import json
import math
import os
import signal
import subprocess
import tempfile
import threading
import time
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
WORKSPACE = os.path.abspath(os.path.join(PACKAGE_DIR, "../../../.."))
LAUNCH_DIR = os.path.join(PACKAGE_DIR, "launch")
SCENARIO_PATH = os.path.join(PACKAGE_DIR, "config/scenarios/sim_b_open_3.yaml")
CONFLICT_SCENARIO_PATH = os.path.join(
    PACKAGE_DIR, "config/scenarios/sim_b_conflict_3.yaml")
FORMATION_SCENARIO_PATH = os.path.join(
    PACKAGE_DIR, "config/scenarios/sim_b_formation_3.yaml")
ORCHESTRATOR_PATH = os.path.join(PACKAGE_DIR, "scripts/swarm_orchestrator.py")
PUBLISHER_PATH = os.path.join(PACKAGE_DIR, "scripts/scenario_goal_publisher.py")
AGENT_LAUNCH = os.path.join(LAUNCH_DIR, "phase_offset_agent.launch")
WORLD_LAUNCH = os.path.join(LAUNCH_DIR, "phase_offset_world.launch")
SWARM_LAUNCH = os.path.join(LAUNCH_DIR, "phase_offset_swarm.launch")
VISUALIZER_SOURCE = os.path.join(PACKAGE_DIR, "src/swarm_visualizer.cpp")
FORMATION_SOURCE = os.path.join(
    WORKSPACE, "src/swarm_planner/bspline_traj/src/formation_planning.cpp")

import sys
sys.path.insert(0, os.path.join(PACKAGE_DIR, "scripts"))
from swarm_orchestrator import (  # noqa: E402
    ScenarioValidationError,
    _validate_options,
    build_child_launch,
    canonical_scenario,
    translate_initial_states,
    validate_scenario,
    write_scenario,
)


def _launch_arg_defaults(path):
    """Return the roslaunch ``<arg>`` defaults of one launch document."""
    defaults = {}
    for element in ET.parse(path).getroot().findall("arg"):
        name = element.get("name")
        if name is not None and name not in defaults:
            defaults[name] = element.get("default", "")
    return defaults


def _finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _finite_odom(message):
    p = message.pose.pose.position
    q = message.pose.pose.orientation
    v = message.twist.twist.linear
    w = message.twist.twist.angular
    values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w,
              v.x, v.y, v.z, w.x, w.y, w.z]
    values.extend(message.pose.covariance)
    values.extend(message.twist.covariance)
    return all(_finite(value) for value in values)


def _finite_position(message):
    values = [message.position.x, message.position.y, message.position.z,
              message.velocity.x, message.velocity.y, message.velocity.z,
              message.acceleration.x, message.acceleration.y,
              message.acceleration.z, message.jerk.x, message.jerk.y,
              message.jerk.z, message.yaw, message.yaw_dot]
    values.extend(message.kx)
    values.extend(message.kv)
    return all(_finite(value) for value in values)


def _so3_invalid_fields(message):
    """Return labeled SO3 fields that are non-finite, in wire order."""
    fields = [
        ("force.x", message.force.x),
        ("force.y", message.force.y),
        ("force.z", message.force.z),
        ("orientation.x", message.orientation.x),
        ("orientation.y", message.orientation.y),
        ("orientation.z", message.orientation.z),
        ("orientation.w", message.orientation.w),
    ]
    fields.extend(("kR[%d]" % index, value)
                  for index, value in enumerate(message.kR))
    fields.extend(("kOm[%d]" % index, value)
                  for index, value in enumerate(message.kOm))
    fields.extend([
        ("aux.current_yaw", message.aux.current_yaw),
        ("aux.kf_correction", message.aux.kf_correction),
    ])
    fields.extend(("aux.angle_corrections[%d]" % index, value)
                  for index, value in enumerate(message.aux.angle_corrections))
    return [name for name, value in fields if not _finite(value)]


def _finite_so3(message):
    return not _so3_invalid_fields(message)


def _stamp(message):
    value = message.header.stamp.to_sec()
    return value if _finite(value) else None


def _header_rate(records):
    stamps = [item[1] for item in records if item[1] is not None]
    if len(stamps) < 2 or stamps[-1] <= stamps[0]:
        return 0.0
    return float(len(stamps) - 1) / (stamps[-1] - stamps[0])


def _system_state(rospy):
    import rosgraph
    master = rosgraph.Master(rospy.get_name())
    # rosgraph.Master unwraps the XML-RPC response in Noetic and returns the
    # three state lists directly: (publishers, subscribers, services).
    return master.getSystemState()


def _all_nodes(rospy):
    state = _system_state(rospy)
    names = set()
    for entries in state:
        for _, owners in entries:
            names.update(str(owner) for owner in owners)
    return names


def _topic_owners(rospy):
    publications, subscriptions, _ = _system_state(rospy)
    pubs = {}
    subs = {}
    for topic, owners in publications:
        pubs[str(topic)] = set(str(owner) for owner in owners)
    for topic, owners in subscriptions:
        subs[str(topic)] = set(str(owner) for owner in owners)
    return pubs, subs


def _proc_metric(pid):
    try:
        with open("/proc/%d/stat" % pid, "r") as stream:
            fields = stream.read().split()
        ticks = int(fields[13]) + int(fields[14])
        rss_kb = None
        with open("/proc/%d/status" % pid, "r") as stream:
            for line in stream:
                if line.startswith("VmRSS:"):
                    rss_kb = int(line.split()[1])
                    break
        return {"cpu_ticks": ticks, "rss_kb": rss_kb}
    except (IOError, OSError, ValueError, IndexError):
        return None


def _proc_cmdline(pid):
    """Return one process command line, or an empty string if it vanished."""
    try:
        with open("/proc/%d/cmdline" % pid, "rb") as stream:
            raw = stream.read().replace(b"\x00", b" ")
        return raw.decode("utf-8", "replace").strip()
    except (IOError, OSError, ValueError):
        return ""


def _proc_ppid(pid):
    """Read a process parent PID from procfs without depending on ``ps``."""
    try:
        with open("/proc/%d/stat" % pid, "r") as stream:
            line = stream.read()
        # ``comm`` is enclosed in parentheses and may contain spaces.  The
        # fields after the final ')' begin with state, then ppid.
        tail = line.rsplit(")", 1)[1].split()
        return int(tail[1])
    except (IOError, OSError, ValueError, IndexError):
        return None


def _owned_process_tree(roots):
    """Return roots and all currently reachable descendants by PPID."""
    root_set = {int(pid) for pid in roots if pid is not None}
    if not root_set:
        return set()
    children = {}
    try:
        entries = os.listdir("/proc")
    except (IOError, OSError):
        entries = []
    for entry in entries:
        if not entry.isdigit():
            continue
        pid = int(entry)
        ppid = _proc_ppid(pid)
        if ppid is not None:
            children.setdefault(ppid, set()).add(pid)
    owned = set(root_set)
    pending = list(root_set)
    while pending:
        parent = pending.pop()
        for child in children.get(parent, ()):
            if child not in owned:
                owned.add(child)
                pending.append(child)
    return owned


def _find_processes(pids, command_token):
    """Find task-owned PIDs whose command line contains ``command_token``."""
    matches = []
    for pid in sorted(pids):
        command = _proc_cmdline(pid)
        if command_token in command:
            matches.append(pid)
    return matches


class _OwnedLaunch(object):
    """A task-owned roslaunch process group for runtime acceptance."""

    def __init__(self, command, log_path):
        self._log = open(log_path, "w")
        self.process = subprocess.Popen(command, stdout=self._log,
                                        stderr=subprocess.STDOUT,
                                        start_new_session=True)

    def alive(self):
        return self.process.poll() is None

    def stop(self):
        if self.process.poll() is not None:
            self._log.close()
            return True
        for sig, timeout in ((signal.SIGINT, 5.0),
                             (signal.SIGTERM, 2.0),
                             (signal.SIGKILL, 2.0)):
            try:
                os.killpg(self.process.pid, sig)
            except (OSError, ProcessLookupError):
                pass
            try:
                self.process.wait(timeout=timeout)
                self._log.close()
                return True
            except subprocess.TimeoutExpired:
                continue
        self._log.close()
        return self.process.poll() is not None


class _Trace(object):
    """Per-UAV wire trace used by the goal/command/survival gate."""

    def __init__(self, rospy, robot_id):
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        from quadrotor_msgs.msg import PositionCommand, SO3Command

        self.robot_id = robot_id
        self.lock = threading.Lock()
        self.goals = []
        self.odom = []
        self.local_map = []
        self.position = []
        self.so3 = []
        self.finite = {"odom": True, "map": True,
                       "position": True, "so3": True}
        self.frames = {"odom": "", "child": "", "map": "", "position": ""}
        self.first_invalid_so3 = None
        prefix = "/uav_%d" % robot_id
        self.subscribers = [
            rospy.Subscriber(prefix + "/goal", PoseStamped, self._goal,
                             queue_size=5),
            rospy.Subscriber(prefix + "/sim/odom", Odometry, self._odom,
                             queue_size=500),
            rospy.Subscriber(prefix + "/sim/local_map", PointCloud2,
                             self._map, queue_size=100),
            rospy.Subscriber(prefix + "/position_cmd", PositionCommand,
                             self._position, queue_size=200),
            rospy.Subscriber(prefix + "/so3_cmd", SO3Command, self._so3,
                             queue_size=200),
        ]

    def _goal(self, message):
        with self.lock:
            self.goals.append((time.monotonic(), _stamp(message), message))

    def _odom(self, message):
        p = message.pose.pose.position
        with self.lock:
            self.odom.append((time.monotonic(), _stamp(message), p.x, p.y, p.z))
            self.finite["odom"] = self.finite["odom"] and _finite_odom(message)
            self.frames["odom"] = message.header.frame_id
            self.frames["child"] = message.child_frame_id

    def _map(self, message):
        with self.lock:
            self.local_map.append((time.monotonic(), _stamp(message)))
            self.finite["map"] = self.finite["map"] and _stamp(message) is not None
            self.frames["map"] = message.header.frame_id

    def _position(self, message):
        with self.lock:
            self.position.append((time.monotonic(), _stamp(message), message))
            self.finite["position"] = self.finite["position"] and _finite_position(message)
            self.frames["position"] = message.header.frame_id

    def _so3(self, message):
        now = time.monotonic()
        invalid_fields = _so3_invalid_fields(message)
        with self.lock:
            self.so3.append((now, _stamp(message), message))
            self.finite["so3"] = self.finite["so3"] and not invalid_fields
            if invalid_fields and self.first_invalid_so3 is None:
                self.first_invalid_so3 = {
                    "monotonic": now,
                    "stamp": _stamp(message),
                    "fields": invalid_fields,
                }

    def snapshot(self):
        with self.lock:
            return {"goals": list(self.goals), "odom": list(self.odom),
                    "local_map": list(self.local_map),
                    "position": list(self.position), "so3": list(self.so3),
                    "finite": dict(self.finite), "frames": dict(self.frames),
                    "first_invalid_so3": self.first_invalid_so3}

    def close(self):
        for subscriber in self.subscribers:
            subscriber.unregister()


class SimBBRingupTest(unittest.TestCase):
    def test_static_contract(self):
        with open(SCENARIO_PATH, "r") as stream:
            document = yaml.safe_load(stream)
        self.assertEqual(document, {"scenario": canonical_scenario(
            3, "sim_b_open_3")})
        self.assertEqual(validate_scenario(SCENARIO_PATH), document["scenario"])

        for path in (AGENT_LAUNCH, WORLD_LAUNCH, SWARM_LAUNCH):
            ET.parse(path)

        swarm_text = open(SWARM_LAUNCH, "r").read()
        self.assertIn('name="goal_publish_timeout" default="25.0"',
                      swarm_text)
        self.assertIn(
            'name="goal_publish_timeout" value="$(arg goal_publish_timeout)"',
            swarm_text)

        with open(ORCHESTRATOR_PATH, "r") as stream:
            orchestrator_text = stream.read()
        self.assertIn("scenario_goal_publisher.py", orchestrator_text)
        self.assertIn("/uav_%d/formation_planning", orchestrator_text)
        self.assertIn("MAX_CONCURRENT_GOAL_PUBLISHERS", orchestrator_text)
        self.assertIn("MAX_CONCURRENT_GOAL_PUBLISHERS = 16", orchestrator_text)
        self.assertIn("def _invoke_goal_publishers", orchestrator_text)
        self.assertIn("def _control_graph_ready", orchestrator_text)
        self.assertIn("self._control_graph_ready", orchestrator_text)
        self.assertIn("goal_publish_timeout\", 25.0", orchestrator_text)
        for topic_suffix in ("/sim/odom", "/position_cmd", "/so3_cmd"):
            self.assertIn(topic_suffix, orchestrator_text)
        self.assertIn("not_started", orchestrator_text)
        self.assertNotIn("_invoke_goal_publisher(agent", orchestrator_text)
        self.assertNotIn("rospy.Subscriber", orchestrator_text)
        self.assertNotIn("PositionCommand", orchestrator_text)
        self.assertNotIn("SO3Command", orchestrator_text)

        with open(PUBLISHER_PATH, "r") as stream:
            publisher_text = stream.read()
        self.assertIn("PoseStamped", publisher_text)
        self.assertIn("anonymous=True", publisher_text)
        self.assertIn("disable_rosout=True", publisher_text)
        self.assertNotIn("rospy.loginfo", publisher_text)
        self.assertIn("frame_id = \"world\"", publisher_text)
        self.assertIn("position.z = goal[\"z\"]", publisher_text)
        self.assertIn("getSystemState()", publisher_text)
        self.assertIn("/uav_%d/formation_planning", publisher_text)
        self.assertIn("connections >= len(owners)", publisher_text)

        with open(VISUALIZER_SOURCE, "r") as stream:
            visualizer_text = stream.read()
        self.assertIn("/sim_b_world/scenario/agents", visualizer_text)
        self.assertIn("/uav_\" + std::to_string(robot_id) + \"/sim/odom", visualizer_text)
        self.assertNotIn("num_agents_", visualizer_text)

        with open(FORMATION_SOURCE, "r") as stream:
            formation_text = stream.read()
        self.assertIn("ros::AsyncSpinner spinner(8)", formation_text)
        self.assertNotIn("spinner_threads", formation_text)

        with open(os.path.join(PACKAGE_DIR, "CMakeLists.txt"), "r") as stream:
            cmake_text = stream.read()
        self.assertIn("add_executable(swarm_visualizer src/swarm_visualizer.cpp)",
                      cmake_text)
        self.assertIn("catkin_install_python", cmake_text)
        for dependency in ("roscpp", "geometry_msgs", "nav_msgs",
                           "visualization_msgs"):
            self.assertIn(dependency, cmake_text)

        with open(os.path.join(PACKAGE_DIR, "package.xml"), "r") as stream:
            package_text = stream.read()
        for dependency in ("roscpp", "rospy", "roslaunch", "rosgraph",
                           "nodelet", "geometry_msgs", "nav_msgs",
                           "sensor_msgs", "visualization_msgs", "quadrotor_msgs",
                           "bspline_race", "so3_control",
                           "so3_quadrotor_simulator", "map_generator",
                           "python3-yaml"):
            self.assertIn("<exec_depend>%s</exec_depend>" % dependency,
                          package_text)

        agent_text = open(AGENT_LAUNCH, "r").read()
        self.assertEqual(agent_text.count("test_gvf.launch"), 1)
        self.assertEqual(agent_text.count("SO3ControlNodelet"), 1)
        self.assertIn('name="spinner_threads"', agent_text)
        self.assertIn('name="use_velocity_feedforward"', agent_text)
        self.assertNotIn('name="spinner_threads" value="$(arg spinner_threads)"',
                         agent_text)
        self.assertNotIn('name="use_velocity_feedforward" value="$(arg use_velocity_feedforward)"',
                         agent_text)
        self.assertIn('name="phase_offset/mode" value="active"', agent_text)
        self.assertIn('name="gvf/circle_test/enable" value="false"', agent_text)
        self.assertIn('name="gvf/circle_test/auto_start" value="false"', agent_text)
        self.assertIn('name="sdf_map/manual_map_auto_load" value="false"',
                      agent_text)
        self.assertIn('name="sdf_map/manual_map_auto_save" value="false"',
                      agent_text)

        world_text = open(WORLD_LAUNCH, "r").read()
        self.assertEqual(world_text.count('type="map_pub"'), 1)
        self.assertEqual(world_text.count('type="swarm_visualizer"'), 1)
        self.assertNotIn("multi_simulator.launch", world_text)
        self.assertNotIn("SO3ControlNodelet", world_text)

        swarm_text = open(SWARM_LAUNCH, "r").read()
        self.assertEqual(swarm_text.count("swarm_orchestrator.py"), 1)
        self.assertNotIn("test_gvf.launch", swarm_text)

        for count in (1, 3, 8, 50):
            scenario = canonical_scenario(count)
            child = build_child_launch(scenario, "/tmp/sim_b_initial.yaml")
            self.assertEqual(child.count("multi_simulator.launch"), 1)
            self.assertEqual(child.count("phase_offset_agent.launch"), count)
            self.assertEqual(child.count('name="so3_nodelet_manager"'), 1)
            self.assertNotIn("spinner_threads", child)
            self.assertNotIn("use_velocity_feedforward", child)
            if count < 50:
                self.assertNotIn("uav_50", child)
            for robot_id in range(count):
                self.assertIn("uav_%d" % robot_id, child)

    def test_swarm_defaults_are_sph_ready(self):
        """Shipped swarm defaults must satisfy the frozen SPH contract.

        The orchestrator already rejects ``sph`` unless the run is MANUAL with
        ``observe_only=false``, neighbour transport, the SPH provider and all
        six normal preview values.  Parsing the launch defaults and feeding
        them straight into that validator proves the defaults are accepted
        without duplicating the contract here.
        """
        defaults = _launch_arg_defaults(SWARM_LAUNCH)
        options = {
            "scenario_file": CONFLICT_SCENARIO_PATH,
            # 0 keeps the explicit scenario file (no generated formation); the
            # validator only needs it to be a non-negative integer.
            "agent_count": 0,
            "formation_spacing": float(defaults["formation_spacing"]),
            "formation_shape": defaults["formation_shape"],
            "formation_axis": defaults["formation_axis"],
            "formation_goal_translation_x": float(
                defaults["formation_goal_translation_x"]),
            "formation_goal_translation_y": float(
                defaults["formation_goal_translation_y"]),
            "sph_reference_spacing": float(defaults["sph_reference_spacing"]),
            "sph_h": float(defaults["sph_h"]),
            "local_update_range_x": float(defaults["local_update_range_x"]),
            "local_update_range_y": float(defaults["local_update_range_y"]),
            "local_update_range_z": float(defaults["local_update_range_z"]),
            "simulation_rate": 1000.0,
            "odom_rate": 100.0,
            "start_at_hover": True,
            "init_x": float(defaults["init_x"]),
            "init_y": float(defaults["init_y"]),
            "init_z": float(defaults["init_z"]),
            "frame_id": "world",
            "map_resolution": 0.10,
            "map_size_x": 20.0,
            "map_size_y": 30.0,
            "map_size_z": 2.5,
            "nodelet_manager": "/so3_nodelet_manager",
            "goal_connection_timeout": 30.0,
            "goal_publish_timeout": 25.0,
        }
        preview_keys = (
            "phase_offset_normal_preview_horizon_w",
            "phase_offset_normal_preview_sample_spacing_w",
            "phase_offset_normal_preview_lower_nu",
            "phase_offset_normal_preview_upper_nu",
            "phase_offset_normal_preview_b_tight",
            "phase_offset_normal_preview_b_open",
        )
        for key in ("phase_offset_mode", "phase_offset_coordination_backend",
                    "agent_state_topic", "neighbor_config_file",
                    "sph_beta_topic_pattern", "sph_g_coord_topic_pattern"):
            options[key] = defaults[key]
        for key in preview_keys:
            self.assertNotEqual(defaults[key], "")
            options[key] = defaults[key]
        for key in ("phase_offset_manual_observe_only",
                    "phase_offset_tube_cloud_obstacle_set_complete",
                    "enable_neighbor_transport", "enable_sph_provider",
                    "all_visible_neighbors"):
            options[key] = defaults[key] == "true"
        for key in ("neighbor_enter_radius", "neighbor_exit_radius",
                    "beta_fresh_timeout", "snapshot_fresh_timeout",
                    "future_timestamp_tolerance",
                    "phase_offset_g_coord_fresh_timeout",
                    "phase_offset_future_timestamp_tolerance"):
            options[key] = float(defaults[key])

        _validate_options(options)
        self.assertEqual(options["phase_offset_mode"], "manual")
        self.assertEqual(options["phase_offset_coordination_backend"], "sph")
        self.assertTrue(options["enable_neighbor_transport"])
        self.assertTrue(options["enable_sph_provider"])
        self.assertFalse(options["phase_offset_manual_observe_only"])
        self.assertTrue(
            options["phase_offset_tube_cloud_obstacle_set_complete"])
        self.assertEqual(
            (options["init_x"], options["init_y"], options["init_z"]),
            (0.0, 20.0, 1.0))

        agent_defaults = _launch_arg_defaults(AGENT_LAUNCH)
        self.assertEqual(agent_defaults["phase_offset_mode"], "manual")
        self.assertEqual(agent_defaults["phase_offset_coordination_backend"],
                         "sph")
        self.assertEqual(
            agent_defaults["phase_offset_tube_cloud_obstacle_set_complete"],
            "true")
        for key in preview_keys:
            self.assertNotEqual(agent_defaults[key], "")

        conflict = validate_scenario(CONFLICT_SCENARIO_PATH)
        self.assertEqual(conflict["name"], "sim_b_conflict_3")
        lanes = sorted(agent["initial"]["x"] for agent in conflict["agents"])
        self.assertEqual(len(lanes), 3)
        for left, right in zip(lanes, lanes[1:]):
            self.assertAlmostEqual(right - left, 1.2, places=9)

    def test_formation_scenario_matches_sph_goal_style(self):
        """Preserve the SPH formation geometry and translated goals.

        SPH-planning computes ``goal_i = init_i + clicked_goal + init_bias``,
        so the goal formation is the start formation shifted by one common
        vector: every UAV keeps its 1 m relative offset.  The launch-level
        ``init_x/init_y/init_z`` translation is checked below separately.
        """
        formation = validate_scenario(FORMATION_SCENARIO_PATH)
        self.assertEqual(formation["name"], "sim_b_formation_3")
        starts = [(agent["initial"]["x"], agent["initial"]["y"])
                  for agent in formation["agents"]]
        goals = [(agent["goal_message"]["x"], agent["goal_message"]["y"])
                 for agent in formation["agents"]]
        self.assertEqual(len(starts), 3)
        self.assertTrue(all(abs(y - 9.0) < 1e-9 for _, y in starts))
        self.assertTrue(all(abs(x) <= 1.0 + 1e-9 for x, _ in starts))
        for left, right in zip(sorted(starts), sorted(starts)[1:]):
            self.assertAlmostEqual(right[0] - left[0], 1.0, places=9)

        # One common translation vector for the whole formation.
        translations = {(round(gx - sx, 9), round(gy - sy, 9))
                        for (sx, sy), (gx, gy) in zip(starts, goals)}
        self.assertEqual(len(translations), 1)
        self.assertLess(translations.pop()[1], 0.0)  # the group moves south

        effective = translate_initial_states(formation, 0.0, 20.0, 1.0)
        effective_starts = [
            (agent["initial"]["x"], agent["initial"]["y"],
             agent["initial"]["z"])
            for agent in effective["agents"]]
        self.assertEqual(effective_starts,
                         [(-1.0, 20.0, 1.0),
                          (0.0, 20.0, 1.0),
                          (1.0, 20.0, 1.0)])
        self.assertEqual(
            [agent["goal_message"] for agent in effective["agents"]],
            [agent["goal_message"] for agent in formation["agents"]])

    def test_launch_initial_pose_translation_does_not_change_map(self):
        scenario = canonical_scenario(3)
        original_offsets = [
            (agent["initial"]["x"] - scenario["agents"][1]["initial"]["x"],
             agent["initial"]["y"] - scenario["agents"][1]["initial"]["y"])
            for agent in scenario["agents"]]
        translated = translate_initial_states(scenario, 4.0, 20.0, 1.5)
        translated_offsets = [
            (agent["initial"]["x"] - translated["agents"][1]["initial"]["x"],
             agent["initial"]["y"] - translated["agents"][1]["initial"]["y"])
            for agent in translated["agents"]]
        self.assertEqual(translated_offsets, original_offsets)
        self.assertAlmostEqual(
            sum(agent["initial"]["x"] for agent in translated["agents"]) / 3.0,
            4.0)
        self.assertAlmostEqual(
            sum(agent["initial"]["y"] for agent in translated["agents"]) / 3.0,
            20.0)
        self.assertAlmostEqual(
            sum(agent["initial"]["z"] for agent in translated["agents"]) / 3.0,
            1.5)
        child = build_child_launch(
            translated, "/tmp/translated_initial_states.yaml",
            map_size_x=20.0, map_size_y=30.0, map_size_z=2.5)
        self.assertIn('<arg name="map_size_x" value="20.0"/>', child)
        self.assertIn('<arg name="map_size_y" value="30.0"/>', child)
        self.assertIn('<arg name="map_size_z" value="2.5"/>', child)

    def test_invalid_schema_rejected(self):
        with tempfile.TemporaryDirectory(prefix="sim_b_schema_") as directory:
            valid = canonical_scenario(1)
            cases = []
            cases.append(("missing_goal", {"scenario": {
                "name": valid["name"], "frame_id": "world",
                "agents": [{"robot_id": 0, "initial": valid["agents"][0]["initial"]}]
            }}))
            wrong_z = canonical_scenario(1)
            wrong_z["agents"][0]["goal_message"]["z"] = 1.0
            cases.append(("goal_z", {"scenario": wrong_z}))
            wrong_frame = canonical_scenario(1)
            wrong_frame["frame_id"] = "map"
            cases.append(("frame", {"scenario": wrong_frame}))
            duplicate = canonical_scenario(2)
            duplicate["agents"][1]["robot_id"] = 0
            cases.append(("duplicate", {"scenario": duplicate}))
            extra = canonical_scenario(1)
            extra["unexpected"] = True
            cases.append(("extra", {"scenario": extra}))
            for name, document in cases:
                path = os.path.join(directory, name + ".yaml")
                with open(path, "w") as stream:
                    yaml.safe_dump(document, stream, sort_keys=False)
                with self.assertRaises(ScenarioValidationError, msg=name):
                    validate_scenario(path)

    @staticmethod
    def _expected_nodes(agent_ids):
        names = {
            "/map_pub", "/phase_offset_swarm_visualizer",
            "/phase_offset_swarm_orchestrator",
            "/multi_quadrotor_simulator_so3", "/so3_nodelet_manager",
        }
        for robot_id in agent_ids:
            names.update({
                "/so3_control_nodelet_%d" % robot_id,
                "/uav_%d/formation_planning" % robot_id,
                "/uav_%d/map_generator" % robot_id,
            })
        return names

    @staticmethod
    def _nodelet_names(rospy):
        try:
            from nodelet.srv import NodeletList
            rospy.wait_for_service("/so3_nodelet_manager/list", timeout=1.0)
            response = rospy.ServiceProxy("/so3_nodelet_manager/list",
                                          NodeletList)()
            return set("/" + str(name).lstrip("/")
                       for name in response.nodelets)
        except Exception:
            return set()

    @staticmethod
    def _wait_for_trace_subscriptions(rospy, agent_ids, timeout=10.0):
        """Wait until this test process is registered on every goal topic.

        The goal publisher is intentionally one-shot.  Waiting only for the
        protected planner subscriber is not sufficient for this acceptance
        test because the test's own subscriber registration is asynchronous;
        without this barrier it can miss the latched message before the short
        publisher process exits.
        """
        node_name = str(rospy.get_name())
        topics = ["/uav_%d/goal" % robot_id for robot_id in agent_ids]
        deadline = time.monotonic() + float(timeout)
        missing = list(topics)
        while time.monotonic() < deadline and not rospy.is_shutdown():
            try:
                _, subscriptions, _ = _system_state(rospy)
                owners = {
                    str(topic): set(str(owner) for owner in owner_names)
                    for topic, owner_names in subscriptions
                }
                missing = [topic for topic in topics
                           if node_name not in owners.get(topic, set())]
                if not missing:
                    return True
            except Exception:
                missing = list(topics)
            time.sleep(0.05)
        print("SIM_B_TRACE_REGISTRATION_TIMEOUT " + json.dumps({
            "node": node_name,
            "missing_topics": missing,
            "topics": topics,
        }, sort_keys=True))
        return False

    @staticmethod
    def _readiness_diagnostic(rospy, world, swarm, agent_ids, traces):
        """Return compact evidence for a failed runtime readiness gate."""
        try:
            live_nodes = sorted(_all_nodes(rospy))
        except Exception as exc:
            live_nodes = ["<master query failed: %s>" % exc]
        expected = SimBBRingupTest._expected_nodes(agent_ids)
        try:
            nodelets = sorted(SimBBRingupTest._nodelet_names(rospy))
        except Exception as exc:
            nodelets = ["<nodelet query failed: %s>" % exc]
        trace_reports = []
        for robot_id, trace in zip(agent_ids, traces):
            snapshot = trace.snapshot()
            trace_reports.append({
                "robot_id": robot_id,
                "goals": len(snapshot["goals"]),
                "odom": len(snapshot["odom"]),
                "local_map": len(snapshot["local_map"]),
                "position": len(snapshot["position"]),
                "so3": len(snapshot["so3"]),
                "finite": snapshot["finite"],
                "frames": snapshot["frames"],
                "first_invalid_so3": snapshot["first_invalid_so3"],
            })
        return {
            "world_alive": bool(world is not None and world.alive()),
            "swarm_alive": bool(swarm is not None and swarm.alive()),
            "missing_nodes": sorted(expected - set(live_nodes)),
            "unexpected_nodes": sorted(set(live_nodes) - expected),
            "expected_nodes": sorted(expected),
            "live_nodes": live_nodes,
            "expected_nodelets": sorted(
                "/so3_control_nodelet_%d" % i for i in agent_ids),
            "nodelets": nodelets,
            "traces": trace_reports,
        }

    @staticmethod
    def _total_metric(roots):
        """Aggregate CPU/RSS over the complete task-owned process trees."""
        pids = _owned_process_tree(roots)
        metrics = [_proc_metric(pid) for pid in pids]
        metrics = [item for item in metrics if item is not None]
        if not metrics:
            return None
        return {
            "cpu_ticks": sum(item["cpu_ticks"] for item in metrics),
            "rss_kb": sum(item["rss_kb"] or 0 for item in metrics),
            "process_count": len(metrics),
        }

    @unittest.skipUnless(os.environ.get("SIM_B_RUN_RUNTIME") == "1",
                         "set SIM_B_RUN_RUNTIME=1 for ROS runtime matrix")
    def test_runtime_matrix(self):
        """Exercise the complete goal/command/liveness matrix for all N."""
        import rospy

        counts = (1, 3, 8, 50)
        requested_counts = os.environ.get("SIM_B_RUNTIME_COUNTS", "").strip()
        if requested_counts:
            try:
                counts = tuple(int(item.strip())
                               for item in requested_counts.split(",")
                               if item.strip())
            except ValueError:
                self.fail("SIM_B_RUNTIME_COUNTS must be comma-separated integers")
            if not counts or any(item <= 0 for item in counts):
                self.fail("SIM_B_RUNTIME_COUNTS must contain positive counts")

        try:
            rospy.init_node("sim_b_bringup_runtime_test", anonymous=False)
        except rospy.exceptions.ROSException:
            # rostest may have initialized the process already.
            pass

        runtime_metrics = {}
        with tempfile.TemporaryDirectory(prefix="sim_b_runtime_") as directory:
            for count in counts:
                scenario = canonical_scenario(count,
                                               "sim_b_open_%d" % count)
                scenario_path = os.path.join(directory,
                                              "scenario_%d.yaml" % count)
                write_scenario(scenario_path, scenario)
                world_log = os.path.join(directory, "world_%d.log" % count)
                swarm_log = os.path.join(directory, "swarm_%d.log" % count)
                world = _OwnedLaunch(
                    ["roslaunch", WORLD_LAUNCH,
                     "scenario_file:=" + scenario_path,
                     "enable_rviz:=false"], world_log)
                swarm = None
                traces = []
                try:
                    agent_ids = list(range(count))
                    world_deadline = time.monotonic() + 30.0
                    while time.monotonic() < world_deadline:
                        self.assertTrue(world.alive(),
                                        "WORLD exited for N=%d" % count)
                        if {"/map_pub", "/phase_offset_swarm_visualizer"}.issubset(
                                _all_nodes(rospy)):
                            break
                        time.sleep(0.1)
                    else:
                        self.fail("WORLD readiness timed out for N=%d" % count)

                    traces = [_Trace(rospy, robot_id) for robot_id in agent_ids]
                    self.assertTrue(
                        self._wait_for_trace_subscriptions(rospy, agent_ids),
                        "test subscriber registration timed out for N=%d" % count)
                    start_wall = time.monotonic()
                    swarm = _OwnedLaunch(
                        ["roslaunch", SWARM_LAUNCH,
                         "scenario_file:=" + scenario_path], swarm_log)

                    def complete_ready():
                        if not world.alive() or not swarm.alive():
                            return False
                        if not self._expected_nodes(agent_ids).issubset(
                                _all_nodes(rospy)):
                            return False
                        if self._nodelet_names(rospy) != set(
                                "/so3_control_nodelet_%d" % i
                                for i in agent_ids):
                            return False
                        for trace in traces:
                            snapshot = trace.snapshot()
                            if (len(snapshot["goals"]) < 1 or
                                    len(snapshot["odom"]) < 20 or
                                    len(snapshot["local_map"]) < 5 or
                                    len(snapshot["position"]) < 1 or
                                    len(snapshot["so3"]) < 1):
                                return False
                            if not all(snapshot["finite"].values()):
                                return False
                        return True

                    deadline = time.monotonic() + 30.0
                    while time.monotonic() < deadline and not rospy.is_shutdown():
                        if complete_ready():
                            break
                        time.sleep(0.1)
                    else:
                        print("SIM_B_READINESS_DIAGNOSTIC " + json.dumps(
                            self._readiness_diagnostic(
                                rospy, world, swarm, agent_ids, traces),
                            sort_keys=True))
                        self.fail("goal/command readiness timed out for N=%d" %
                                  count)
                    ready_wall = time.monotonic()

                    for robot_id, trace in zip(agent_ids, traces):
                        snapshot = trace.snapshot()
                        self.assertEqual(len(snapshot["goals"]), 1,
                                         "goal must be published once for uav_%d" %
                                         robot_id)
                        goal_message = snapshot["goals"][0][2]
                        expected_goal = scenario["agents"][robot_id]["goal_message"]
                        self.assertEqual(goal_message.header.frame_id, "world")
                        self.assertAlmostEqual(goal_message.pose.position.x,
                                               expected_goal["x"])
                        self.assertAlmostEqual(goal_message.pose.position.y,
                                               expected_goal["y"])
                        self.assertEqual(goal_message.pose.position.z, 0.0)
                        self.assertEqual(goal_message.pose.orientation.x, 0.0)
                        self.assertEqual(goal_message.pose.orientation.y, 0.0)
                        self.assertEqual(goal_message.pose.orientation.z, 0.0)
                        self.assertEqual(goal_message.pose.orientation.w, 1.0)

                        first = snapshot["odom"][0]
                        initial = scenario["agents"][robot_id]["initial"]
                        initial_error = math.sqrt(
                            (first[2] - initial["x"]) ** 2 +
                            (first[3] - initial["y"]) ** 2 +
                            (first[4] - initial["z"]) ** 2)
                        self.assertLess(initial_error, 0.05)
                        self.assertEqual(snapshot["frames"]["odom"], "world")
                        self.assertEqual(snapshot["frames"]["child"],
                                         "uav_%d/base_link" % robot_id)
                        self.assertEqual(snapshot["frames"]["map"], "world")
                        self.assertEqual(snapshot["frames"]["position"], "world")

                        goal_wall = snapshot["goals"][0][0]
                        active_position = [item for item in snapshot["position"]
                                           if item[0] >= goal_wall]
                        active_so3 = [item for item in snapshot["so3"]
                                      if item[0] >= goal_wall]
                        self.assertTrue(active_position,
                                        "no active PositionCommand for uav_%d" %
                                        robot_id)
                        self.assertTrue(active_so3,
                                        "no active SO3Command for uav_%d" %
                                        robot_id)

                    # Five-second survival gate: command streams may stop
                    # after legitimate completion, but all plant/planner/
                    # controller/sensing/world infrastructure must survive.
                    survival_deadline = time.monotonic() + 5.0
                    before = {id(trace): trace.snapshot() for trace in traces}
                    survival_samples = {}
                    while (time.monotonic() < survival_deadline and
                           not rospy.is_shutdown()):
                        self.assertTrue(world.alive())
                        self.assertTrue(swarm.alive())
                        self.assertTrue(self._expected_nodes(agent_ids).issubset(
                            _all_nodes(rospy)))
                        self.assertEqual(
                            self._nodelet_names(rospy),
                            set("/so3_control_nodelet_%d" % i
                                for i in agent_ids))
                        time.sleep(0.2)
                    for trace in traces:
                        after = trace.snapshot()
                        prior = before[id(trace)]
                        samples = {
                            key: list(after[key][len(prior[key]):])
                            for key in ("odom", "local_map", "position", "so3")
                        }
                        survival_samples[trace.robot_id] = samples
                        self.assertGreater(len(samples["odom"]), 0,
                                           "no survival odom samples for uav_%d" %
                                           trace.robot_id)
                        self.assertGreater(len(samples["local_map"]), 0,
                                           "no survival local-map samples for uav_%d" %
                                           trace.robot_id)
                        self.assertTrue(after["finite"]["odom"])
                        self.assertTrue(after["finite"]["map"])

                    # Rate checks use only the five-second survival window.
                    # This excludes startup catch-up bursts while retaining
                    # the original acceptance bounds and command-stream gate.
                    for robot_id in agent_ids:
                        samples = survival_samples[robot_id]
                        odom_rate = _header_rate(samples["odom"])
                        map_rate = _header_rate(samples["local_map"])
                        position_rate = _header_rate(samples["position"])
                        so3_rate = _header_rate(samples["so3"])
                        self.assertGreaterEqual(
                            odom_rate, 80.0,
                            "uav_%d odom rate %.3f < 80.0" %
                            (robot_id, odom_rate))
                        self.assertLessEqual(
                            odom_rate, 130.0,
                            "uav_%d odom rate %.3f > 130.0" %
                            (robot_id, odom_rate))
                        self.assertGreaterEqual(
                            map_rate, 5.0,
                            "uav_%d local-map rate %.3f < 5.0" %
                            (robot_id, map_rate))
                        self.assertLessEqual(
                            map_rate, 15.0,
                            "uav_%d local-map rate %.3f > 15.0" %
                            (robot_id, map_rate))
                        if len(samples["position"]) >= 3:
                            self.assertGreaterEqual(
                                position_rate, 20.0,
                                "uav_%d PositionCommand rate %.3f < 20.0" %
                                (robot_id, position_rate))
                            self.assertLessEqual(
                                position_rate, 70.0,
                                "uav_%d PositionCommand rate %.3f > 70.0" %
                                (robot_id, position_rate))
                        if len(samples["so3"]) >= 3:
                            self.assertGreaterEqual(
                                so3_rate, 80.0,
                                "uav_%d SO3Command rate %.3f < 80.0" %
                                (robot_id, so3_rate))
                            self.assertLessEqual(
                                so3_rate, 130.0,
                                "uav_%d SO3Command rate %.3f > 130.0" %
                                (robot_id, so3_rate))

                    publications, subscriptions = _topic_owners(rospy)
                    topics = set(publications) | set(subscriptions)
                    for robot_id in agent_ids:
                        prefix = "/uav_%d" % robot_id
                        for suffix in ("/sim/odom", "/sim/imu",
                                       "/sim/local_map", "/goal",
                                       "/position_cmd", "/so3_cmd",
                                       "/force_disturbance",
                                       "/moment_disturbance", "/motors",
                                       "/corrections"):
                            self.assertIn(prefix + suffix, topics)
                    self.assertNotIn("/uav_%d/sim/odom" % count, topics)
                    self.assertNotIn("/uav_%d/goal" % count, topics)

                    live_nodes = _all_nodes(rospy)
                    nodelets = self._nodelet_names(rospy)
                    odom_rates = [_header_rate(survival_samples[robot_id]["odom"])
                                  for robot_id in agent_ids]
                    task_roots = [world.process.pid, swarm.process.pid]
                    task_pids = _owned_process_tree(task_roots)
                    simulator_pids = _find_processes(
                        task_pids, "multi_quadrotor_simulator_so3")
                    self.assertEqual(
                        len(simulator_pids), 1,
                        "expected one task-owned multi_quadrotor_simulator_so3 "
                        "process for N=%d, found %s" %
                        (count, simulator_pids))
                    simulator_pid = simulator_pids[0]
                    simulator_metric = _proc_metric(simulator_pid)
                    total_metric = self._total_metric(task_roots)
                    runtime_metrics[str(count)] = {
                        "startup_time": ready_wall - start_wall,
                        "observation_window": time.monotonic() - ready_wall,
                        "odom_rate_min": min(odom_rates),
                        "odom_rate_median": sorted(odom_rates)[len(odom_rates) // 2],
                        "odom_rate_max": max(odom_rates),
                        "simulator_pid": simulator_pid,
                        "simulator_process_count": len(simulator_pids),
                        "simulator_cpu": (simulator_metric or {}).get(
                            "cpu_ticks"),
                        "simulator_rss": (simulator_metric or {}).get(
                            "rss_kb"),
                        "total_cpu": (total_metric or {}).get("cpu_ticks"),
                        "total_memory": (total_metric or {}).get("rss_kb"),
                        "total_process_count": (total_metric or {}).get(
                            "process_count"),
                        "live_nodes": len(live_nodes),
                        "plants": count,
                        "initialized_agent_stacks": count,
                        "manager_count": int("/so3_nodelet_manager" in live_nodes),
                        "nodelet_count": len(nodelets),
                        "survival": True,
                    }
                finally:
                    for trace in traces:
                        trace.close()
                    if swarm is not None:
                        swarm.stop()
                    world.stop()

        print("SIM_B_BRINGUP_METRICS " + json.dumps({
            "cases": runtime_metrics,
            "N50_COORDINATED_NAVIGATION_CLAIM": False,
        }, sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    try:
        import rostest
        rostest.rosrun("phase_offset_sim_bringup", "sim_b_bringup",
                       SimBBRingupTest)
    except ImportError:
        unittest.main()
