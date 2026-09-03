#!/usr/bin/env python3
"""SIM-B generic-N process/configuration supervisor.

This node owns no navigation or control logic.  It validates the frozen
scenario schema, creates task-owned simulator/agent launch material, starts a
single child roslaunch, waits for the per-agent goal subscribers to appear,
and invokes :mod:`scenario_goal_publisher` exactly once for each agent.
"""

from __future__ import print_function

import argparse
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from xml.sax.saxutils import escape as xml_escape

import yaml


SCENARIO_KEYS = frozenset(("name", "frame_id", "agents"))
AGENT_KEYS = frozenset(("robot_id", "initial", "goal_message"))
INITIAL_KEYS = frozenset(("x", "y", "z", "yaw"))
GOAL_KEYS = frozenset(("x", "y", "z"))
WORLD_FRAME = "world"
MANAGER_NAME = "/so3_nodelet_manager"
GOAL_TOPIC_TEMPLATE = "/uav_{robot_id}/goal"
SPH_BETA_TOPIC_PATTERN = "/uav_{robot_id}/phase_offset/sph_beta"
SPH_G_COORD_TOPIC_PATTERN = "/uav_{robot_id}/phase_offset/g_coord"
# Keep the publisher fan-out bounded so a large batch does not overwhelm the
# ROS master with simultaneous publisherUpdate callbacks.  The shared
# deadline below gives all N=50 batches one common completion window.
MAX_CONCURRENT_GOAL_PUBLISHERS = 16


class ScenarioValidationError(ValueError):
    """Raised when a scenario violates the frozen SIM-B schema."""


def _is_finite_number(value):
    """Accept YAML numeric scalars, but never booleans or strings."""
    return (isinstance(value, (int, float)) and
            not isinstance(value, bool) and
            math.isfinite(float(value)))


def _require_mapping(value, label):
    if not isinstance(value, dict):
        raise ScenarioValidationError("%s must be a mapping" % label)
    return value


def _reject_extra_keys(mapping, allowed, label):
    extras = set(mapping) - set(allowed)
    if extras:
        raise ScenarioValidationError(
            "%s has unsupported field(s): %s" %
            (label, ", ".join(sorted(str(key) for key in extras))))


def _numeric(mapping, key, label):
    if key not in mapping:
        raise ScenarioValidationError("%s.%s is required" % (label, key))
    value = mapping[key]
    if not _is_finite_number(value):
        raise ScenarioValidationError(
            "%s.%s must be a finite numeric scalar" % (label, key))
    return float(value)


def validate_scenario(path, require_goals=True):
    """Load and normalize one exact SIM-B scenario document.

    The returned mapping contains only ``name``, ``frame_id`` and sorted
    contiguous agent records.  Acceptance callers leave ``require_goals`` at
    its default; the optional switch is useful for schema-only diagnostics.
    """
    if not path or not os.path.isfile(path):
        raise ScenarioValidationError("scenario file not found: %s" % path)
    try:
        with open(path, "r") as stream:
            document = yaml.safe_load(stream)
    except (IOError, OSError, yaml.YAMLError) as exc:
        raise ScenarioValidationError("cannot read scenario %s: %s" %
                                      (path, exc))

    root = _require_mapping(document, "scenario document")
    if set(root) != {"scenario"}:
        extras = set(root) - {"scenario"}
        missing = {"scenario"} - set(root)
        details = []
        if missing:
            details.append("missing scenario")
        if extras:
            details.append("unsupported root field(s): %s" %
                           ", ".join(sorted(str(key) for key in extras)))
        raise ScenarioValidationError("; ".join(details))

    scenario = _require_mapping(root["scenario"], "scenario")
    _reject_extra_keys(scenario, SCENARIO_KEYS, "scenario")
    for key in ("name", "frame_id", "agents"):
        if key not in scenario:
            raise ScenarioValidationError("scenario.%s is required" % key)
    if not isinstance(scenario["name"], str) or not scenario["name"]:
        raise ScenarioValidationError("scenario.name must be a non-empty string")
    if scenario["frame_id"] != WORLD_FRAME:
        raise ScenarioValidationError("scenario.frame_id must be world")
    if not isinstance(scenario["agents"], list) or not scenario["agents"]:
        raise ScenarioValidationError("scenario.agents must be a non-empty list")

    normalized_agents = []
    seen_ids = set()
    for index, raw_agent in enumerate(scenario["agents"]):
        label = "scenario.agents[%d]" % index
        agent = _require_mapping(raw_agent, label)
        _reject_extra_keys(agent, AGENT_KEYS, label)

        if "robot_id" not in agent:
            raise ScenarioValidationError("%s.robot_id is required" % label)
        robot_id = agent["robot_id"]
        if not isinstance(robot_id, int) or isinstance(robot_id, bool):
            raise ScenarioValidationError("%s.robot_id must be an integer" % label)
        if robot_id < 0 or robot_id in seen_ids:
            raise ScenarioValidationError(
                "%s.robot_id must be unique and non-negative" % label)
        seen_ids.add(robot_id)

        initial = _require_mapping(agent.get("initial"), "%s.initial" % label)
        _reject_extra_keys(initial, INITIAL_KEYS, "%s.initial" % label)
        initial_values = {
            "x": _numeric(initial, "x", "%s.initial" % label),
            "y": _numeric(initial, "y", "%s.initial" % label),
            "z": _numeric(initial, "z", "%s.initial" % label),
            "yaw": _numeric(initial, "yaw", "%s.initial" % label),
        }
        if initial_values["z"] <= 0.0:
            raise ScenarioValidationError(
                "%s.initial.z must be greater than zero" % label)

        if "goal_message" not in agent or agent["goal_message"] is None:
            if require_goals:
                raise ScenarioValidationError("%s.goal_message is required" % label)
            goal_values = None
        else:
            goal = _require_mapping(agent["goal_message"],
                                    "%s.goal_message" % label)
            _reject_extra_keys(goal, GOAL_KEYS, "%s.goal_message" % label)
            goal_values = {
                "x": _numeric(goal, "x", "%s.goal_message" % label),
                "y": _numeric(goal, "y", "%s.goal_message" % label),
                "z": _numeric(goal, "z", "%s.goal_message" % label),
            }
            # The planner intentionally adds +1.0 to the received z.  SIM-B
            # therefore accepts only the frozen wire value 0.0.
            if goal_values["z"] != 0.0:
                raise ScenarioValidationError(
                    "%s.goal_message.z must be exactly 0.0" % label)

        normalized_agents.append({
            "robot_id": robot_id,
            "initial": initial_values,
            "goal_message": goal_values,
        })

    normalized_agents.sort(key=lambda item: item["robot_id"])
    actual_ids = [item["robot_id"] for item in normalized_agents]
    if actual_ids != list(range(len(normalized_agents))):
        raise ScenarioValidationError(
            "robot_id values must be contiguous and exactly 0..N-1")

    return {
        "name": scenario["name"],
        "frame_id": WORLD_FRAME,
        "agents": normalized_agents,
    }


def canonical_agent(robot_id):
    """Return the frozen acceptance record for one robot id."""
    column = robot_id % 10
    row = robot_id // 10
    x = -9.0 + 2.0 * column
    y = -10.0 + 2.0 * row
    return {
        "robot_id": robot_id,
        "initial": {"x": x, "y": y, "z": 1.0, "yaw": 0.0},
        "goal_message": {"x": x, "y": y + 12.0, "z": 0.0},
    }


def canonical_scenario(agent_count, name=None):
    """Build the deterministic N-prefix acceptance scenario."""
    if (not isinstance(agent_count, int) or isinstance(agent_count, bool) or
            agent_count <= 0):
        raise ValueError("agent_count must be a positive integer")
    return {
        "name": name or "sim_b_open_%d" % agent_count,
        "frame_id": WORLD_FRAME,
        "agents": [canonical_agent(index) for index in range(agent_count)],
    }


def write_scenario(path, scenario):
    """Write a task-owned scenario with deterministic key ordering."""
    with open(path, "w") as stream:
        yaml.safe_dump({"scenario": scenario}, stream,
                       default_flow_style=False, sort_keys=False)


def write_initial_states(path, scenario):
    """Write the accepted multi-simulator initial-state schema."""
    states = []
    for agent in scenario["agents"]:
        initial = agent["initial"]
        states.append({
            "robot_id": agent["robot_id"],
            "x": initial["x"],
            "y": initial["y"],
            "z": initial["z"],
            "yaw": initial["yaw"],
        })
    with open(path, "w") as stream:
        yaml.safe_dump({"initial_states": states}, stream,
                       default_flow_style=False, sort_keys=False)


def _xml_value(value):
    return xml_escape(str(value), {"\"": "&quot;"})


def resolve_topic_pattern(pattern, robot_id, label):
    """Resolve one generic-N topic pattern with exactly one robot placeholder."""
    if not isinstance(pattern, str) or not pattern:
        raise ScenarioValidationError("%s must be a non-empty string" % label)
    if pattern.count("{robot_id}") != 1:
        raise ScenarioValidationError(
            "%s must contain exactly one {robot_id} replacement" % label)
    try:
        resolved = pattern.format(robot_id=robot_id)
    except (KeyError, IndexError, ValueError) as exc:
        raise ScenarioValidationError("invalid %s: %s" % (label, exc))
    if "{robot_id}" in resolved or not resolved:
        raise ScenarioValidationError("invalid %s" % label)
    return resolved


def build_child_launch(scenario, initial_state_file, frame_id=WORLD_FRAME,
                       simulation_rate=1000.0, odom_rate=100.0,
                       start_at_hover=True, map_resolution=0.20,
                       map_size_x=20.0, map_size_y=30.0, map_size_z=2.5,
                       nodelet_manager=MANAGER_NAME,
                       enable_neighbor_transport=False,
                       agent_state_topic="/phase_offset/agent_state",
                       neighbor_config_file=(
                           "$(find phase_offset_swarm)/config/"
                           "neighbor_transport_v1.yaml"),
                       neighbor_enter_radius=2.45,
                       neighbor_exit_radius=2.50,
                       all_visible_neighbors=False,
                       enable_sph_provider=False,
                       sph_beta_topic_pattern=SPH_BETA_TOPIC_PATTERN,
                       sph_g_coord_topic_pattern=SPH_G_COORD_TOPIC_PATTERN,
                       beta_fresh_timeout=0.10,
                       snapshot_fresh_timeout=0.10,
                       future_timestamp_tolerance=0.02,
                       phase_offset_mode="active",
                       phase_offset_coordination_backend="disabled",
                       phase_offset_manual_observe_only=True,
                       phase_offset_manual_tube_source="none",
                       phase_offset_tube_cloud_obstacle_set_complete=False,
                       phase_offset_tube_preincluded_map_uncertainty=0.0,
                       phase_offset_normal_preview_horizon_w="",
                       phase_offset_normal_preview_sample_spacing_w="",
                       phase_offset_normal_preview_lower_nu="",
                       phase_offset_normal_preview_upper_nu="",
                       phase_offset_normal_preview_b_tight="",
                       phase_offset_normal_preview_b_open="",
                       phase_offset_g_coord_fresh_timeout=0.10,
                       phase_offset_future_timestamp_tolerance=0.02):
    """Generate one simulator/manager/N-agent launch document."""
    if all_visible_neighbors:
        neighbor_enter_radius = 100.0
        neighbor_exit_radius = 101.0
    for key, value in (("beta_fresh_timeout", beta_fresh_timeout),
                       ("snapshot_fresh_timeout", snapshot_fresh_timeout),
                       ("future_timestamp_tolerance",
                        future_timestamp_tolerance),
                       ("phase_offset_tube_preincluded_map_uncertainty",
                        phase_offset_tube_preincluded_map_uncertainty)):
        if not math.isfinite(float(value)) or float(value) < 0.0:
            raise ScenarioValidationError(
                "%s must be finite and non-negative" % key)
    lines = [
        "<launch>",
        "  <include file=\"$(find so3_quadrotor_simulator)/launch/multi_simulator.launch\">",
        "    <arg name=\"num_agents\" value=\"%s\"/>" %
        len(scenario["agents"]),
        "    <arg name=\"initial_state_file\" value=\"%s\"/>" %
        _xml_value(initial_state_file),
        "    <arg name=\"frame_id\" value=\"%s\"/>" %
        _xml_value(frame_id),
        "    <arg name=\"simulation_rate\" value=\"%s\"/>" %
        _xml_value(simulation_rate),
        "    <arg name=\"odom_rate\" value=\"%s\"/>" %
        _xml_value(odom_rate),
        "    <arg name=\"start_at_hover\" value=\"%s\"/>" %
        ("true" if start_at_hover else "false"),
        "  </include>",
        "  <node pkg=\"nodelet\" type=\"nodelet\" name=\"so3_nodelet_manager\"",
        "        args=\"manager\" output=\"screen\"/>",
    ]
    for agent in scenario["agents"]:
        robot_id = agent["robot_id"]
        sph_beta_topic = resolve_topic_pattern(
            sph_beta_topic_pattern, robot_id, "sph_beta_topic_pattern")
        sph_g_coord_topic = resolve_topic_pattern(
            sph_g_coord_topic_pattern, robot_id, "sph_g_coord_topic_pattern")
        initial = agent["initial"]
        lines.extend([
            "  <include file=\"$(find phase_offset_sim_bringup)/launch/phase_offset_agent.launch\">",
            "    <arg name=\"robot_id\" value=\"%d\"/>" % robot_id,
            "    <arg name=\"uav_ns\" value=\"uav_%d\"/>" % robot_id,
            "    <arg name=\"nodelet_manager\" value=\"%s\"/>" %
            _xml_value(nodelet_manager),
            "    <arg name=\"init_x\" value=\"%s\"/>" %
            _xml_value(initial["x"]),
            "    <arg name=\"init_y\" value=\"%s\"/>" %
            _xml_value(initial["y"]),
            "    <arg name=\"init_z\" value=\"%s\"/>" %
            _xml_value(initial["z"]),
            "    <arg name=\"frame_id\" value=\"%s\"/>" %
            _xml_value(frame_id),
            "    <arg name=\"map_resolution\" value=\"%s\"/>" %
            _xml_value(map_resolution),
            "    <arg name=\"map_size_x\" value=\"%s\"/>" %
            _xml_value(map_size_x),
            "    <arg name=\"map_size_y\" value=\"%s\"/>" %
            _xml_value(map_size_y),
            "    <arg name=\"map_size_z\" value=\"%s\"/>" %
            _xml_value(map_size_z),
            "    <arg name=\"phase_offset_mode\" value=\"%s\"/>" %
            _xml_value(phase_offset_mode),
            "    <arg name=\"phase_offset_coordination_backend\" value=\"%s\"/>" %
            _xml_value(phase_offset_coordination_backend),
            "    <arg name=\"phase_offset_manual_observe_only\" value=\"%s\"/>" %
            ("true" if phase_offset_manual_observe_only else "false"),
            "    <arg name=\"phase_offset_manual_tube_source\" value=\"%s\"/>" %
            _xml_value(phase_offset_manual_tube_source),
            "    <arg name=\"phase_offset_tube_cloud_obstacle_set_complete\" value=\"%s\"/>" %
            ("true" if phase_offset_tube_cloud_obstacle_set_complete else "false"),
            "    <arg name=\"phase_offset_tube_preincluded_map_uncertainty\" value=\"%s\"/>" %
            _xml_value(phase_offset_tube_preincluded_map_uncertainty),
            "    <arg name=\"phase_offset_normal_preview_horizon_w\" value=\"%s\"/>" %
            _xml_value(phase_offset_normal_preview_horizon_w),
            "    <arg name=\"phase_offset_normal_preview_sample_spacing_w\" value=\"%s\"/>" %
            _xml_value(phase_offset_normal_preview_sample_spacing_w),
            "    <arg name=\"phase_offset_normal_preview_lower_nu\" value=\"%s\"/>" %
            _xml_value(phase_offset_normal_preview_lower_nu),
            "    <arg name=\"phase_offset_normal_preview_upper_nu\" value=\"%s\"/>" %
            _xml_value(phase_offset_normal_preview_upper_nu),
            "    <arg name=\"phase_offset_normal_preview_b_tight\" value=\"%s\"/>" %
            _xml_value(phase_offset_normal_preview_b_tight),
            "    <arg name=\"phase_offset_normal_preview_b_open\" value=\"%s\"/>" %
            _xml_value(phase_offset_normal_preview_b_open),
            "    <arg name=\"phase_offset_g_coord_fresh_timeout\" value=\"%s\"/>" %
            _xml_value(phase_offset_g_coord_fresh_timeout),
            "    <arg name=\"phase_offset_future_timestamp_tolerance\" value=\"%s\"/>" %
            _xml_value(phase_offset_future_timestamp_tolerance),
            "    <arg name=\"enable_neighbor_transport\" value=\"%s\"/>" %
            ("true" if enable_neighbor_transport else "false"),
            "    <arg name=\"agent_count\" value=\"%d\"/>" %
            len(scenario["agents"]),
            "    <arg name=\"agent_state_topic\" value=\"%s\"/>" %
            _xml_value(agent_state_topic),
            "    <arg name=\"neighbor_config_file\" value=\"%s\"/>" %
            _xml_value(neighbor_config_file),
            "    <arg name=\"neighbor_enter_radius\" value=\"%s\"/>" %
            _xml_value(neighbor_enter_radius),
            "    <arg name=\"neighbor_exit_radius\" value=\"%s\"/>" %
            _xml_value(neighbor_exit_radius),
            "    <arg name=\"all_visible_neighbors\" value=\"%s\"/>" %
            ("true" if all_visible_neighbors else "false"),
            "    <arg name=\"enable_sph_provider\" value=\"%s\"/>" %
            ("true" if enable_sph_provider else "false"),
            "    <arg name=\"sph_beta_topic\" value=\"%s\"/>" %
            _xml_value(sph_beta_topic),
            "    <arg name=\"sph_g_coord_topic\" value=\"%s\"/>" %
            _xml_value(sph_g_coord_topic),
            "    <arg name=\"beta_fresh_timeout\" value=\"%s\"/>" %
            _xml_value(beta_fresh_timeout),
            "    <arg name=\"snapshot_fresh_timeout\" value=\"%s\"/>" %
            _xml_value(snapshot_fresh_timeout),
            "    <arg name=\"future_timestamp_tolerance\" value=\"%s\"/>" %
            _xml_value(future_timestamp_tolerance),
            "  </include>",
        ])
    lines.append("</launch>")
    return "\n".join(lines) + "\n"


def write_runtime_material(scenario, directory, **launch_options):
    """Write task-owned initial states and child launch files."""
    initial_state_file = os.path.join(directory, "initial_states.yaml")
    child_launch_file = os.path.join(directory, "sim_b_child.launch")
    write_initial_states(initial_state_file, scenario)
    with open(child_launch_file, "w") as stream:
        stream.write(build_child_launch(scenario, initial_state_file,
                                        **launch_options))
    return initial_state_file, child_launch_file


def manifest_for(scenario, scenario_path, initial_state_file,
                 child_launch_file, nodelet_manager=MANAGER_NAME,
                 enable_neighbor_transport=False,
                 agent_state_topic="/phase_offset/agent_state",
                 enable_sph_provider=False,
                 sph_beta_topic_pattern=SPH_BETA_TOPIC_PATTERN,
                 sph_g_coord_topic_pattern=SPH_G_COORD_TOPIC_PATTERN):
    """Return deterministic effective orchestration metadata."""
    ids = [agent["robot_id"] for agent in scenario["agents"]]
    topics = {}
    for robot_id in ids:
        prefix = "/uav_%d" % robot_id
        topics[str(robot_id)] = {
            "odom": prefix + "/sim/odom",
            "imu": prefix + "/sim/imu",
            "local_map": prefix + "/sim/local_map",
            "goal": GOAL_TOPIC_TEMPLATE.format(robot_id=robot_id),
            "position_cmd": prefix + "/position_cmd",
            "so3_cmd": prefix + "/so3_cmd",
            "force_disturbance": prefix + "/force_disturbance",
            "moment_disturbance": prefix + "/moment_disturbance",
            "motors": prefix + "/motors",
            "corrections": prefix + "/corrections",
            "sph_beta": resolve_topic_pattern(
                sph_beta_topic_pattern, robot_id, "sph_beta_topic_pattern"),
            "sph_g_coord": resolve_topic_pattern(
                sph_g_coord_topic_pattern, robot_id,
                "sph_g_coord_topic_pattern"),
        }
    return {
        "scenario_file": os.path.abspath(scenario_path),
        "agent_count": len(ids),
        "robot_ids": ids,
        "nodelet_manager": nodelet_manager,
        "enable_neighbor_transport": bool(enable_neighbor_transport),
        "enable_sph_provider": bool(enable_sph_provider),
        "sph_beta_topic_pattern": sph_beta_topic_pattern,
        "sph_g_coord_topic_pattern": sph_g_coord_topic_pattern,
        "agent_state_topic": agent_state_topic,
        "topics": topics,
        "initial_state_file": os.path.abspath(initial_state_file),
        "child_launch_file": os.path.abspath(child_launch_file),
    }


def _as_bool(value, default=False):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return default


class SwarmSupervisor(object):
    """Own exactly one child roslaunch process and goal invocation per UAV."""

    def __init__(self, scenario, scenario_path, options):
        self.scenario = scenario
        self.scenario_path = scenario_path
        self.options = options
        self.child = None
        self.temp_dir = None
        self._cleanup_done = False
        self._failed = False
        self._rospy = None

    def _start_child(self, child_launch_file):
        try:
            self.child = subprocess.Popen(
                ["roslaunch", child_launch_file], start_new_session=True)
        except (OSError, IOError) as exc:
            raise RuntimeError("unable to start child roslaunch: %s" % exc)
        if self.child.poll() is not None:
            raise RuntimeError("child roslaunch exited immediately (%d)" %
                               self.child.returncode)

    def _system_state(self):
        import rosgraph
        master = rosgraph.Master(self._rospy.get_name())
        # rosgraph.Master unwraps the XML-RPC response in Noetic and returns
        # the three state lists directly: (publishers, subscribers, services).
        return master.getSystemState()

    @staticmethod
    def _indexed_topics(entries):
        """Normalize one ROS master system-state section by topic."""
        return {str(topic): set(str(owner) for owner in owners)
                for topic, owners in entries}

    def _goal_subscribers_ready(self, subscriptions=None):
        """Require each protected planner's goal subscriber to be registered.

        The acceptance trace also subscribes to each goal topic.  Treating
        any subscriber as readiness would let the one-shot publisher fire
        before the planner is connected, delivering the goal only to the
        trace and leaving the planner in WAIT_TARGET forever.
        """
        if subscriptions is None:
            _, subscriptions, _ = self._system_state()
        subscribers = self._indexed_topics(subscriptions)
        return all(
            "/uav_%d/formation_planning" % agent["robot_id"] in
            subscribers.get(GOAL_TOPIC_TEMPLATE.format(
                robot_id=agent["robot_id"]), set())
            for agent in self.scenario["agents"])

    def _control_graph_ready(self, publications=None, subscriptions=None):
        """Require the manager's complete per-agent control graph.

        This is deliberately a ROS master graph check rather than a data
        subscriber.  SO3ControlNodelet initializes its state from odometry;
        publishing a goal before the manager has registered all three legs of
        the graph can let the first position command race that initialization.
        The stable window in :meth:`_wait_for_goal_subscribers` then keeps the
        graph registered long enough for the initial odometry callbacks to
        run, without changing the control node itself.
        """
        if publications is None or subscriptions is None:
            publications, subscriptions, _ = self._system_state()
        publishers = self._indexed_topics(publications)
        subscribers = self._indexed_topics(subscriptions)
        manager = str(self.options["nodelet_manager"])
        for agent in self.scenario["agents"]:
            robot_id = agent["robot_id"]
            odom_topic = "/uav_%d/sim/odom" % robot_id
            position_topic = "/uav_%d/position_cmd" % robot_id
            so3_topic = "/uav_%d/so3_cmd" % robot_id
            if manager not in subscribers.get(odom_topic, set()):
                return False
            if manager not in subscribers.get(position_topic, set()):
                return False
            if manager not in publishers.get(so3_topic, set()):
                return False
        return True

    def _wait_for_goal_subscribers(self):
        deadline = time.monotonic() + self.options["goal_connection_timeout"]
        ready_since = None
        # Master registration and the planner's XML-RPC/TCPROS endpoint are
        # separate asynchronous events.  Require a short stable interval so
        # a one-shot publisher cannot race the planner immediately after its
        # subscription first appears in getSystemState().
        stable_seconds = 0.5
        while time.monotonic() < deadline and not self._rospy.is_shutdown():
            if self.child is not None and self.child.poll() is not None:
                raise RuntimeError("child roslaunch exited before goal readiness")
            publications, subscriptions, _ = self._system_state()
            if (self._goal_subscribers_ready(subscriptions) and
                    self._control_graph_ready(publications, subscriptions)):
                if ready_since is None:
                    ready_since = time.monotonic()
                elif time.monotonic() - ready_since >= stable_seconds:
                    return
            else:
                ready_since = None
            time.sleep(0.05)
        raise RuntimeError("goal subscriber readiness exceeded %.1fs" %
                           self.options["goal_connection_timeout"])

    @staticmethod
    def _stop_goal_publishers(processes):
        """Terminate any still-running goal publishers and reap them."""
        live = [process for process in processes.values()
                if process.poll() is None]
        for process in live:
            try:
                process.terminate()
            except OSError:
                pass
        for process in live:
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                try:
                    process.kill()
                except OSError:
                    pass
                try:
                    process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    pass

    def _invoke_goal_publishers(self):
        """Run one package-owned publisher per agent under one deadline.

        A bounded batch keeps the N=50 process footprint controlled while
        avoiding the serial startup cost that previously consumed the runtime
        readiness window.  The deadline starts before the first batch and is
        shared by every batch; each robot is started at most once.
        """
        publisher = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 "scenario_goal_publisher.py")
        agents = list(self.scenario["agents"])
        deadline = (time.monotonic() +
                    float(self.options["goal_publish_timeout"]))
        next_index = 0
        while next_index < len(agents):
            active = {}
            started_ids = []
            try:
                while (len(active) < MAX_CONCURRENT_GOAL_PUBLISHERS and
                       next_index < len(agents)):
                    robot_id = agents[next_index]["robot_id"]
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        not_started = [
                            item["robot_id"] for item in agents[next_index:]]
                        raise RuntimeError(
                            "goal publishers timed out for uav(s): "
                            "failed=[] pending=%s not_started=%s" %
                            (started_ids, not_started))
                    command = [sys.executable, publisher,
                               "--scenario-file", self.scenario_path,
                               "--robot-id", str(robot_id),
                               "--wait-timeout", "%.3f" % remaining]
                    try:
                        active[robot_id] = subprocess.Popen(command)
                    except (OSError, IOError) as exc:
                        not_started = [
                            item["robot_id"] for item in agents[next_index:]]
                        raise RuntimeError(
                            "goal publisher start failed for uav(s): "
                            "started=%s not_started=%s (%s)" %
                            (started_ids, not_started, exc))
                    started_ids.append(robot_id)
                    next_index += 1

                pending = set(active)
                while pending:
                    failed = []
                    for robot_id in sorted(pending):
                        result = active[robot_id].poll()
                        if result is None:
                            continue
                        pending.remove(robot_id)
                        if result != 0:
                            failed.append((robot_id, result))
                    if failed:
                        not_started = [
                            item["robot_id"] for item in agents[next_index:]]
                        self._stop_goal_publishers(active)
                        raise RuntimeError(
                            "goal publishers failed for uav(s): "
                            "failed=%s pending=%s not_started=%s" %
                            (failed, sorted(pending), not_started))
                    if not pending:
                        break
                    if time.monotonic() >= deadline:
                        not_started = [
                            item["robot_id"] for item in agents[next_index:]]
                        self._stop_goal_publishers(active)
                        raise RuntimeError(
                            "goal publishers timed out for uav(s): "
                            "failed=[] pending=%s not_started=%s" %
                            (sorted(pending), not_started))
                    time.sleep(0.02)
            except Exception:
                self._stop_goal_publishers(active)
                raise

    def run(self):
        import rospy
        self._rospy = rospy
        self.temp_dir = tempfile.mkdtemp(prefix="sim_b_bringup_")
        initial_state_file, child_launch_file = write_runtime_material(
            self.scenario, self.temp_dir,
            frame_id=self.options["frame_id"],
            simulation_rate=self.options["simulation_rate"],
            odom_rate=self.options["odom_rate"],
            start_at_hover=self.options["start_at_hover"],
            map_resolution=self.options["map_resolution"],
            map_size_x=self.options["map_size_x"],
            map_size_y=self.options["map_size_y"],
            map_size_z=self.options["map_size_z"],
            nodelet_manager=self.options["nodelet_manager"],
            enable_neighbor_transport=self.options[
                "enable_neighbor_transport"],
            agent_state_topic=self.options["agent_state_topic"],
            neighbor_config_file=self.options["neighbor_config_file"],
            neighbor_enter_radius=self.options["neighbor_enter_radius"],
            neighbor_exit_radius=self.options["neighbor_exit_radius"],
            all_visible_neighbors=self.options["all_visible_neighbors"],
            enable_sph_provider=self.options["enable_sph_provider"],
            sph_beta_topic_pattern=self.options["sph_beta_topic_pattern"],
            sph_g_coord_topic_pattern=self.options[
                "sph_g_coord_topic_pattern"],
            beta_fresh_timeout=self.options["beta_fresh_timeout"],
            snapshot_fresh_timeout=self.options["snapshot_fresh_timeout"],
            future_timestamp_tolerance=self.options[
                "future_timestamp_tolerance"],
            phase_offset_mode=self.options["phase_offset_mode"],
            phase_offset_coordination_backend=self.options[
                "phase_offset_coordination_backend"],
            phase_offset_manual_observe_only=self.options[
                "phase_offset_manual_observe_only"],
            phase_offset_manual_tube_source=self.options[
                "phase_offset_manual_tube_source"],
            phase_offset_tube_cloud_obstacle_set_complete=self.options[
                "phase_offset_tube_cloud_obstacle_set_complete"],
            phase_offset_tube_preincluded_map_uncertainty=self.options[
                "phase_offset_tube_preincluded_map_uncertainty"],
            phase_offset_normal_preview_horizon_w=self.options[
                "phase_offset_normal_preview_horizon_w"],
            phase_offset_normal_preview_sample_spacing_w=self.options[
                "phase_offset_normal_preview_sample_spacing_w"],
            phase_offset_normal_preview_lower_nu=self.options[
                "phase_offset_normal_preview_lower_nu"],
            phase_offset_normal_preview_upper_nu=self.options[
                "phase_offset_normal_preview_upper_nu"],
            phase_offset_normal_preview_b_tight=self.options[
                "phase_offset_normal_preview_b_tight"],
            phase_offset_normal_preview_b_open=self.options[
                "phase_offset_normal_preview_b_open"],
            phase_offset_g_coord_fresh_timeout=self.options[
                "phase_offset_g_coord_fresh_timeout"],
            phase_offset_future_timestamp_tolerance=self.options[
                "phase_offset_future_timestamp_tolerance"])
        manifest = manifest_for(self.scenario, self.scenario_path,
                                initial_state_file, child_launch_file,
                                self.options["nodelet_manager"],
                                self.options["enable_neighbor_transport"],
                                self.options["agent_state_topic"],
                                self.options["enable_sph_provider"],
                                self.options["sph_beta_topic_pattern"],
                                self.options["sph_g_coord_topic_pattern"])
        rospy.loginfo("SIM_B_MANIFEST %s", json.dumps(manifest, sort_keys=True))

        self._start_child(child_launch_file)
        self._wait_for_goal_subscribers()
        self._invoke_goal_publishers()

        while not rospy.is_shutdown():
            if self.child.poll() is not None:
                self._failed = True
                rospy.logerr("child roslaunch exited with code %d",
                             self.child.returncode)
                return 1
            time.sleep(0.2)
        return 0

    def _terminate_child(self):
        if self.child is None or self.child.poll() is not None:
            return True
        try:
            os.killpg(self.child.pid, signal.SIGINT)
        except (OSError, ProcessLookupError):
            pass
        try:
            self.child.wait(timeout=5.0)
            return True
        except subprocess.TimeoutExpired:
            pass
        for sig, timeout in ((signal.SIGTERM, 2.0), (signal.SIGKILL, 2.0)):
            try:
                os.killpg(self.child.pid, sig)
            except (OSError, ProcessLookupError):
                pass
            try:
                self.child.wait(timeout=timeout)
                return True
            except subprocess.TimeoutExpired:
                continue
        return self.child.poll() is not None

    def cleanup(self):
        if self._cleanup_done:
            return not self._failed
        self._cleanup_done = True
        if not self._terminate_child():
            self._failed = True
        if self.temp_dir:
            try:
                shutil.rmtree(self.temp_dir)
            except (OSError, IOError):
                self._failed = True
        return not self._failed


def _ros_options():
    import rospy
    return {
        "scenario_file": rospy.get_param("~scenario_file", ""),
        "simulation_rate": float(rospy.get_param("~simulation_rate", 1000.0)),
        "odom_rate": float(rospy.get_param("~odom_rate", 100.0)),
        "start_at_hover": _as_bool(rospy.get_param("~start_at_hover", True), True),
        "frame_id": rospy.get_param("~frame_id", WORLD_FRAME),
        "map_resolution": float(rospy.get_param("~map_resolution", 0.20)),
        "map_size_x": float(rospy.get_param("~map_size_x", 20.0)),
        "map_size_y": float(rospy.get_param("~map_size_y", 30.0)),
        "map_size_z": float(rospy.get_param("~map_size_z", 2.5)),
        "nodelet_manager": rospy.get_param("~nodelet_manager", MANAGER_NAME),
        "goal_connection_timeout": float(
            rospy.get_param("~goal_connection_timeout", 30.0)),
        "goal_publish_timeout": float(
            rospy.get_param("~goal_publish_timeout", 25.0)),
        "phase_offset_mode": rospy.get_param(
            "~phase_offset_mode", "active"),
        "phase_offset_coordination_backend": rospy.get_param(
            "~phase_offset_coordination_backend", "disabled"),
        "phase_offset_manual_observe_only": _as_bool(
            rospy.get_param("~phase_offset_manual_observe_only", True), True),
        "phase_offset_manual_tube_source": rospy.get_param(
            "~phase_offset_manual_tube_source", "none"),
        "phase_offset_tube_cloud_obstacle_set_complete": _as_bool(
            rospy.get_param("~phase_offset_tube_cloud_obstacle_set_complete",
                            False), False),
        "phase_offset_tube_preincluded_map_uncertainty": float(
            rospy.get_param("~phase_offset_tube_preincluded_map_uncertainty",
                            0.0)),
        "phase_offset_normal_preview_horizon_w": rospy.get_param(
            "~phase_offset_normal_preview_horizon_w", ""),
        "phase_offset_normal_preview_sample_spacing_w": rospy.get_param(
            "~phase_offset_normal_preview_sample_spacing_w", ""),
        "phase_offset_normal_preview_lower_nu": rospy.get_param(
            "~phase_offset_normal_preview_lower_nu", ""),
        "phase_offset_normal_preview_upper_nu": rospy.get_param(
            "~phase_offset_normal_preview_upper_nu", ""),
        "phase_offset_normal_preview_b_tight": rospy.get_param(
            "~phase_offset_normal_preview_b_tight", ""),
        "phase_offset_normal_preview_b_open": rospy.get_param(
            "~phase_offset_normal_preview_b_open", ""),
        "phase_offset_g_coord_fresh_timeout": float(
            rospy.get_param("~phase_offset_g_coord_fresh_timeout", 0.10)),
        "phase_offset_future_timestamp_tolerance": float(
            rospy.get_param("~phase_offset_future_timestamp_tolerance", 0.02)),
        "enable_neighbor_transport": _as_bool(
            rospy.get_param("~enable_neighbor_transport", False), False),
        "agent_state_topic": rospy.get_param(
            "~agent_state_topic", "/phase_offset/agent_state"),
        "neighbor_config_file": rospy.get_param(
            "~neighbor_config_file", "$(find phase_offset_swarm)/config/"
            "neighbor_transport_v1.yaml"),
        "neighbor_enter_radius": float(
            rospy.get_param("~neighbor_enter_radius", 2.45)),
        "neighbor_exit_radius": float(
            rospy.get_param("~neighbor_exit_radius", 2.50)),
        "all_visible_neighbors": _as_bool(
            rospy.get_param("~all_visible_neighbors", False), False),
        "enable_sph_provider": _as_bool(
            rospy.get_param("~enable_sph_provider", False), False),
        "sph_beta_topic_pattern": rospy.get_param(
            "~sph_beta_topic_pattern", SPH_BETA_TOPIC_PATTERN),
        "sph_g_coord_topic_pattern": rospy.get_param(
            "~sph_g_coord_topic_pattern", SPH_G_COORD_TOPIC_PATTERN),
        "beta_fresh_timeout": float(
            rospy.get_param("~beta_fresh_timeout", 0.10)),
        "snapshot_fresh_timeout": float(
            rospy.get_param("~snapshot_fresh_timeout", 0.10)),
        "future_timestamp_tolerance": float(
            rospy.get_param("~future_timestamp_tolerance", 0.02)),
    }


def _validate_options(options):
    backend = options["phase_offset_coordination_backend"]
    if not isinstance(backend, str) or backend not in ("disabled", "d1b", "sph"):
        raise ScenarioValidationError(
            "phase_offset_coordination_backend must be one of disabled|d1b|sph")
    enable_neighbor_transport = bool(options["enable_neighbor_transport"])
    enable_sph_provider = bool(options["enable_sph_provider"])
    if backend == "sph":
        if not (enable_neighbor_transport and enable_sph_provider):
            raise ScenarioValidationError(
                "sph coordination requires enable_neighbor_transport and "
                "enable_sph_provider")
        if options["phase_offset_mode"] != "manual":
            raise ScenarioValidationError(
                "sph coordination requires phase_offset_mode=manual")
        if options["phase_offset_manual_observe_only"]:
            raise ScenarioValidationError(
                "sph coordination requires phase_offset_manual_observe_only=false")
        if options["phase_offset_manual_tube_source"] not in ("fixed", "esdf"):
            raise ScenarioValidationError(
                "sph coordination requires phase_offset_manual_tube_source to be fixed or esdf")
    elif enable_neighbor_transport or enable_sph_provider:
        raise ScenarioValidationError(
            "non-sph coordination requires enable_neighbor_transport=false "
            "and enable_sph_provider=false")
    preview_keys = (
        "phase_offset_normal_preview_horizon_w",
        "phase_offset_normal_preview_sample_spacing_w",
        "phase_offset_normal_preview_lower_nu",
        "phase_offset_normal_preview_upper_nu",
        "phase_offset_normal_preview_b_tight",
        "phase_offset_normal_preview_b_open",
    )
    if backend == "sph":
        missing_preview = [key for key in preview_keys
                           if options[key] == ""]
        if missing_preview:
            raise ScenarioValidationError(
                "sph coordination requires all normal preview values to be "
                "non-empty")
    for key in preview_keys:
        value = options[key]
        if value == "":
            continue
        try:
            finite = math.isfinite(float(value))
        except (TypeError, ValueError):
            finite = False
        if not finite:
            raise ScenarioValidationError(
                "%s must be finite when non-empty" % key)
    if options["frame_id"] != WORLD_FRAME:
        raise ScenarioValidationError("frame_id must be world")
    for key in ("simulation_rate", "odom_rate", "map_resolution",
                "map_size_x", "map_size_y", "map_size_z",
                "goal_connection_timeout", "goal_publish_timeout"):
        value = options[key]
        if not math.isfinite(value) or value <= 0.0:
            raise ScenarioValidationError("%s must be finite and positive" % key)
    if options["odom_rate"] > options["simulation_rate"]:
        raise ScenarioValidationError("odom_rate cannot exceed simulation_rate")
    if options["nodelet_manager"] != MANAGER_NAME:
        raise ScenarioValidationError("nodelet_manager is frozen to %s" %
                                      MANAGER_NAME)
    if (not isinstance(options["agent_state_topic"], str) or
            not options["agent_state_topic"]):
        raise ScenarioValidationError("agent_state_topic must be non-empty")
    if (not isinstance(options["neighbor_config_file"], str) or
            not options["neighbor_config_file"]):
        raise ScenarioValidationError("neighbor_config_file must be non-empty")
    for key in ("neighbor_enter_radius", "neighbor_exit_radius"):
        value = options[key]
        if not math.isfinite(value) or value <= 0.0:
            raise ScenarioValidationError("%s must be finite and positive" % key)
    if options["neighbor_enter_radius"] >= options["neighbor_exit_radius"]:
        raise ScenarioValidationError(
            "neighbor_enter_radius must be less than neighbor_exit_radius")
    for key in ("beta_fresh_timeout", "snapshot_fresh_timeout",
                "future_timestamp_tolerance",
                "phase_offset_g_coord_fresh_timeout",
                "phase_offset_future_timestamp_tolerance",
                "phase_offset_tube_preincluded_map_uncertainty"):
        value = options[key]
        if not math.isfinite(value) or value < 0.0:
            raise ScenarioValidationError(
                "%s must be finite and non-negative" % key)
    resolve_topic_pattern(options["sph_beta_topic_pattern"], 0,
                          "sph_beta_topic_pattern")
    resolve_topic_pattern(options["sph_g_coord_topic_pattern"], 0,
                          "sph_g_coord_topic_pattern")


def _validate_only(argv):
    if "--validate-only" not in argv:
        return None
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument("--allow-missing-goals", action="store_true")
    parser.add_argument("scenario_file")
    args = parser.parse_args(argv)
    try:
        scenario = validate_scenario(
            args.scenario_file,
            require_goals=not args.allow_missing_goals)
    except ScenarioValidationError as exc:
        print("SIM_B_SCENARIO_INVALID: %s" % exc, file=sys.stderr)
        return 2
    print("SIM_B_SCENARIO_VALID %s" % json.dumps(scenario, sort_keys=True))
    return 0


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    validate_result = _validate_only(argv)
    if validate_result is not None:
        return validate_result

    import rospy
    rospy.init_node("phase_offset_swarm_orchestrator", anonymous=False)
    supervisor = None
    result = 1
    try:
        options = _ros_options()
        _validate_options(options)
        scenario = validate_scenario(options["scenario_file"], require_goals=True)
        supervisor = SwarmSupervisor(scenario, options["scenario_file"], options)
        rospy.on_shutdown(supervisor.cleanup)
        result = supervisor.run()
    except (ScenarioValidationError, RuntimeError, ValueError) as exc:
        rospy.logerr("[SIM_B] %s", exc)
        if not rospy.is_shutdown():
            rospy.signal_shutdown(str(exc))
        result = 2
    finally:
        if supervisor is not None and not supervisor.cleanup():
            result = 3
    return result


if __name__ == "__main__":
    sys.exit(main())
