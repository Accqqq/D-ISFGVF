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
# The paper orders the SPH distances 0 < d_rep < d_star (= the formation
# spacing).  The validated pair for a 1.5 m formation is d_rep = 1.4 m, so the
# same ratio scales d_rep down with a tighter formation instead of letting the
# ordering break and abort the neighbour runtime.
SPH_REPULSION_RATIO = 1.4 / 1.5
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


def circular_formation_offsets(agent_count, spacing, shape="disk", axis="y"):
    """Return `agent_count` formation offsets centred on their own centroid.

    `ring` : every agent on one circle, evenly spaced by `spacing`.
    `disk` : concentric rings every `spacing` (1, then floor(2*pi*r/spacing),
             ...).  A partially filled outermost ring is still spread evenly
             over the full circle, so the outline stays round instead of
             showing a gap.
    `line` : a single column along the flight `axis`, `spacing` apart.  This is
             the only formation that fits a 1 m narrow channel; the units fly
             one behind the other.
    """
    if agent_count <= 0:
        return []
    if not math.isfinite(spacing) or spacing <= 0.0:
        raise ValueError("spacing must be finite and positive")
    if shape == "ring":
        if agent_count == 1:
            return [(0.0, 0.0)]
        radius = spacing / (2.0 * math.sin(math.pi / float(agent_count)))
        return [(radius * math.cos(2.0 * math.pi * index / agent_count),
                 radius * math.sin(2.0 * math.pi * index / agent_count))
                for index in range(agent_count)]
    if shape != "disk":
        if shape == "line":
            if axis not in ("x", "y"):
                raise ValueError("axis must be 'x' or 'y'")
            total = (agent_count - 1) * spacing
            offsets = []
            for index in range(agent_count):
                along = index * spacing - 0.5 * total
                offsets.append((along, 0.0) if axis == "x"
                               else (0.0, along))
            return offsets
        raise ValueError("shape must be 'disk', 'ring' or 'line'")
    rings = []          # (ring index, capacity, count)
    remaining = agent_count - 1
    ring = 1
    while remaining > 0:
        radius = ring * spacing
        capacity = max(1, int(2.0 * math.pi * radius / spacing))
        count = min(capacity, remaining)
        rings.append((ring, capacity, count))
        remaining -= count
        ring += 1
    # A disk whose outermost ring holds only a vehicle or two looks like a
    # circle with a stray outlier (N=20 used to give 0 / 1.2 / 2.4 / 3.6 m with
    # a single UAV on the 3.6 m ring).  Fold such a thin ring into the one
    # below it and widen that ring so the neighbour spacing is preserved.
    if len(rings) >= 2:
        _, last_capacity, last_count = rings[-1]
        if last_count < max(2, int(0.4 * last_capacity)):
            rings.pop()
            prev_ring, prev_capacity, prev_count = rings.pop()
            rings.append((prev_ring, prev_capacity, prev_count + last_count))
    offsets = [(0.0, 0.0)]
    for ring, capacity, count in rings:
        if count <= 0:
            continue
        radius = ring * spacing
        if count > capacity:
            # Merged ring: widen it just enough to keep `spacing` between
            # neighbours along the circle.
            radius = max(radius, count * spacing / (2.0 * math.pi))
        for index in range(count):
            angle = 2.0 * math.pi * index / float(count)
            offsets.append((radius * math.cos(angle), radius * math.sin(angle)))
    return offsets[:agent_count]


def cubic_kernel_weight(q):
    """Normalized cubic compact-support kernel Wbar(q) of the paper.

    Mirrors the C++ shape in ``phase_offset_swarm/src/sph_intent.cpp`` exactly
    (the constant normalization is absorbed into the coordination gains).
    """
    if q < 0.0:
        return 0.0
    if q <= 1.0:
        return 1.0 - 1.5 * q * q + 0.75 * q * q * q
    if q < 2.0:
        tail = 2.0 - q
        return 0.25 * tail * tail * tail
    return 0.0


def nominal_reference_densities(initial_positions, h, cutoff):
    """Paper reference density rho_0 for every agent [1 + sum Wbar(d*/h)].

    ``initial_positions`` must be the nominal formation; the paper's
    ``N_i^*``/``d*_ij`` are exactly the neighbour set and distances of that
    configuration.  ``cutoff`` must be the radius the runtime neighbour
    manager actually reports (its enter radius): a neighbour outside it is one
    the provider never counts in rho_i, so including it here would bake a
    constant bias into eta_i.
    """
    if not math.isfinite(h) or h <= 0.0:
        raise ValueError("h must be finite and positive")
    if not math.isfinite(cutoff) or cutoff <= 0.0:
        raise ValueError("cutoff must be finite and positive")
    support = min(2.0 * h, cutoff)
    densities = []
    for index, origin in enumerate(initial_positions):
        total = 1.0
        for other_index, other in enumerate(initial_positions):
            if index == other_index:
                continue
            distance = math.hypot(origin[0] - other[0], origin[1] - other[1])
            if distance < support:
                total += cubic_kernel_weight(distance / h)
        densities.append(total)
    return densities


def sph_disk_scenario(agent_count, spacing=1.5, goal_translation_y=-38.0,
                      goal_reference_y=20.0, name=None, goal_base_y=None,
                      shape="disk", axis="y", goal_translation_x=0.0,
                      goal_reference_x=0.0):
    """Build the SPH-style circular formation scenario for N agents.

    related_work/SPH-planning seeds its particles on a 0.5 m grid
    (src/swarm_planner/water_swarm/src/sph_zhang_3d.cpp::initParticles) and lets
    the density interaction settle them into a round cluster at the kernel's
    rest density.  This generator reproduces that *settled* shape directly with
    `circular_formation_offsets()`: concentric rings `spacing` apart, each ring
    evenly filled, so the formation is a clean circle with uniform separation
    instead of a tight rectangular grid (0.5 m neighbours were far too close).

    The block is centred on its own centroid -- the launch then places it at
    init_x/init_y/init_z -- and the goals keep the identical shape, flown
    `goal_translation_y` south of `goal_reference_y` (the launch translates only
    the initial poses, never the goals).  The defaults give a 3.7 m radius disk
    with 1.5 m neighbour spacing, flying 38 m south so every goal cell stays
    clear of the pillar inflation.  Pass goal_base_y to pin the southern-most
    goal row directly instead.
    """
    if (not isinstance(agent_count, int) or isinstance(agent_count, bool) or
            agent_count <= 0):
        raise ValueError("agent_count must be a positive integer")
    if axis not in ("x", "y"):
        raise ValueError("axis must be 'x' or 'y'")
    raw = circular_formation_offsets(agent_count, spacing, shape, axis)
    mean_x = sum(item[0] for item in raw) / float(agent_count)
    mean_y = sum(item[1] for item in raw) / float(agent_count)
    min_y_raw = min(item[1] for item in raw)
    if not math.isfinite(goal_reference_y):
        raise ValueError("goal_reference_y must be finite")
    if goal_base_y is not None and not math.isfinite(goal_base_y):
        raise ValueError("goal_base_y must be finite when provided")
    if not math.isfinite(goal_translation_x) or not math.isfinite(goal_reference_x):
        raise ValueError("goal translation/reference on x must be finite")
    agents = []
    for index, (x_raw, y_raw) in enumerate(raw):
        x = x_raw - mean_x
        y = y_raw - mean_y
        goal_y = (float(goal_base_y) + (y_raw - min_y_raw)) \
            if goal_base_y is not None \
            else (y + float(goal_reference_y) + float(goal_translation_y)
                  if axis == "y" else y)
        goal_x = (x + float(goal_reference_x) + float(goal_translation_x)) \
            if axis == "x" else x
        agents.append({
            "robot_id": index,
            "initial": {"x": x, "y": y, "z": 1.0, "yaw": 0.0},
            "goal_message": {"x": goal_x, "y": goal_y, "z": 0.0},
        })
    return {
        "name": name or "sim_b_sph_disk_%d" % agent_count,
        "frame_id": WORLD_FRAME,
        "agents": agents,
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


def translate_initial_states(scenario, center_x, center_y, center_z):
    """Translate all physical initial poses to one launch-selected centre.

    Scenario files describe the formation geometry (relative offsets).  The
    launch arguments select where that geometry is spawned in the simulator.
    Goals are deliberately left untouched: they are planner targets, not
    simulator initial-state parameters.
    """
    agents = scenario["agents"]
    mean_x = sum(float(agent["initial"]["x"]) for agent in agents) / len(agents)
    mean_y = sum(float(agent["initial"]["y"]) for agent in agents) / len(agents)
    mean_z = sum(float(agent["initial"]["z"]) for agent in agents) / len(agents)
    dx = float(center_x) - mean_x
    dy = float(center_y) - mean_y
    dz = float(center_z) - mean_z
    translated = {
        "name": scenario["name"],
        "frame_id": scenario["frame_id"],
        "agents": [],
    }
    for agent in agents:
        initial = agent["initial"]
        translated["agents"].append({
            "robot_id": agent["robot_id"],
            "initial": {
                "x": float(initial["x"]) + dx,
                "y": float(initial["y"]) + dy,
                "z": float(initial["z"]) + dz,
                "yaw": float(initial["yaw"]),
            },
            "goal_message": dict(agent["goal_message"]),
        })
    return translated


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
                       map_size_x=20.0, map_size_y=50.0, map_size_z=2.5,
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
                       phase_offset_normal_preview_horizon_w="",
                       phase_offset_normal_preview_sample_spacing_w="",
                       phase_offset_normal_preview_lower_nu="",
                       phase_offset_normal_preview_upper_nu="",
                       phase_offset_normal_preview_b_tight="",
                       phase_offset_normal_preview_b_open="",
                       phase_offset_g_coord_fresh_timeout=0.10,
                       phase_offset_future_timestamp_tolerance=0.02,
                       sph_reference_spacing=1.5,
                       sph_d_rep=1.4,
                       rviz_follow_agent=0,
                       sph_h=2.0,
                       local_update_range_x=4.0,
                       local_update_range_y=4.0,
                       local_update_range_z=2.5):
    """Generate one simulator/manager/N-agent launch document."""
    # One shared anchor for the distributed shared-click rule: the negative of
    # the formation centroid in world coordinates.  Every agent applies the same
    # rule ``goal_i = initial_i + bias + clicked``, so with
    # ``bias = -centroid`` the operator's click becomes the formation centre::
    #
    #     goal_i = clicked + (initial_i - centroid)
    #
    # Each UAV keeps its own formation offset around the clicked point, and the
    # constant is shared by every agent, so the rule stays leaderless and
    # symmetric.
    agent_count = len(scenario["agents"])
    centroid = [0.0, 0.0, 0.0]
    if agent_count:
        for axis, key in enumerate(("x", "y", "z")):
            centroid[axis] = sum(
                float(agent["initial"][key]) for agent in scenario["agents"]
            ) / float(agent_count)
    click_bias = (-centroid[0], -centroid[1], -centroid[2])
    if all_visible_neighbors:
        neighbor_enter_radius = 100.0
        neighbor_exit_radius = 101.0
    for key, value in (("beta_fresh_timeout", beta_fresh_timeout),
                       ("snapshot_fresh_timeout", snapshot_fresh_timeout),
                       ("future_timestamp_tolerance",
                        future_timestamp_tolerance)):
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
    # Paper reference density per agent: rho_0 = 1 + sum Wbar(d*_ij/h) over the
    # nominal formation.  This is the one quantity the provider cannot derive
    # from live neighbour states (doing so would make eta identically zero), so
    # the side that owns the formation geometry computes it once and hands it
    # down per agent.
    reference_densities = nominal_reference_densities(
        [(float(agent["initial"]["x"]), float(agent["initial"]["y"]))
         for agent in scenario["agents"]],
        float(sph_h), float(neighbor_enter_radius))
    for index, agent in enumerate(scenario["agents"]):
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
            "    <arg name=\"goal_bias_x\" value=\"%s\"/>" %
            _xml_value(click_bias[0]),
            "    <arg name=\"goal_bias_y\" value=\"%s\"/>" %
            _xml_value(click_bias[1]),
            "    <arg name=\"goal_bias_z\" value=\"%s\"/>" %
            _xml_value(click_bias[2]),
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
            "    <arg name=\"local_update_range_x\" value=\"%.3f\"/>" %
            float(local_update_range_x),
            "    <arg name=\"local_update_range_y\" value=\"%.3f\"/>" %
            float(local_update_range_y),
            "    <arg name=\"local_update_range_z\" value=\"%.3f\"/>" %
            float(local_update_range_z),
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
            "    <arg name=\"sph_h\" value=\"%s\"/>" %
            _xml_value(sph_h),
            "    <arg name=\"sph_reference_density\" value=\"%.6f\"/>" %
            reference_densities[index],
            "    <arg name=\"sph_reference_spacing\" value=\"%.3f\"/>" %
            float(sph_reference_spacing),
            "    <arg name=\"sph_d_rep\" value=\"%.3f\"/>" %
            float(sph_d_rep),
            "    <arg name=\"rviz_follow_agent\" value=\"%d\"/>" %
            int(rviz_follow_agent),
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


def _as_int(value, default=0):
    if isinstance(value, bool):
        return default
    try:
        return int(value)
    except (TypeError, ValueError):
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
            local_update_range_x=self.options["local_update_range_x"],
            local_update_range_y=self.options["local_update_range_y"],
            local_update_range_z=self.options["local_update_range_z"],
            nodelet_manager=self.options["nodelet_manager"],
            enable_neighbor_transport=self.options[
                "enable_neighbor_transport"],
            agent_state_topic=self.options["agent_state_topic"],
            neighbor_config_file=self.options["neighbor_config_file"],
            neighbor_enter_radius=self.options["neighbor_enter_radius"],
            neighbor_exit_radius=self.options["neighbor_exit_radius"],
            all_visible_neighbors=self.options["all_visible_neighbors"],
            enable_sph_provider=self.options["enable_sph_provider"],
            sph_reference_spacing=self.options["sph_reference_spacing"],
            sph_d_rep=self.options["sph_d_rep"],
            rviz_follow_agent=self.options["rviz_follow_agent"],
            sph_h=self.options["sph_h"],
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
        if self.options["publish_goals"]:
            self._invoke_goal_publishers()
        else:
            rospy.loginfo(
                "SIM_B goal publication disabled (publish_goals=false); "
                "publish /uav_<id>/goal manually")

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
        # 0 means "use scenario_file".  A positive count lets the launch own the
        # swarm size: the orchestrator then generates the SPH-planning native
        # formation itself, so one launch argument is enough to fly N agents.
        "agent_count": _as_int(rospy.get_param("~agent_count", 0), 0),
        "formation_spacing": float(
            rospy.get_param("~formation_spacing", 0.5)),
        "formation_shape": rospy.get_param("~formation_shape", "disk"),
        "formation_axis": rospy.get_param("~formation_axis", "y"),
        "formation_goal_translation_y": float(
            rospy.get_param("~formation_goal_translation_y", -34.9)),
        "formation_goal_translation_x": float(
            rospy.get_param("~formation_goal_translation_x", 0.0)),
        "formation_goal_base_y": rospy.get_param("~formation_goal_base_y", ""),
        "simulation_rate": float(rospy.get_param("~simulation_rate", 1000.0)),
        "odom_rate": float(rospy.get_param("~odom_rate", 100.0)),
        "start_at_hover": _as_bool(rospy.get_param("~start_at_hover", True), True),
        "init_x": float(rospy.get_param("~init_x", 0.0)),
        "init_y": float(rospy.get_param("~init_y", 20.0)),
        "init_z": float(rospy.get_param("~init_z", 1.0)),
        # When false the orchestrator still brings the world up and waits for
        # the planner's goal endpoint to register, but never publishes the
        # scenario goals: the operator flies the swarm by publishing goals
        # manually (for example with RViz's "2D Nav Goal" tool).
        "publish_goals": _as_bool(rospy.get_param("~publish_goals", True), True),
        "frame_id": rospy.get_param("~frame_id", WORLD_FRAME),
        "map_resolution": float(rospy.get_param("~map_resolution", 0.10)),
        "map_size_x": float(rospy.get_param("~map_size_x", 20.0)),
        "map_size_y": float(rospy.get_param("~map_size_y", 50.0)),
        "map_size_z": float(rospy.get_param("~map_size_z", 2.5)),
        # Local sensing / planner map half-range [m].  The narrow-gate scenarios
        # need this large enough that the single opening is visible from the
        # whole approach; SPH-planning itself runs 7.0.
        "local_update_range_x": float(
            rospy.get_param("~local_update_range_x", 4.0)),
        "local_update_range_y": float(
            rospy.get_param("~local_update_range_y", 4.0)),
        "local_update_range_z": float(
            rospy.get_param("~local_update_range_z", 2.5)),
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
        # Nominal SPH spacing d_star [m].  Zero (the launch default) means
        # "follow formation_spacing", so the SPH reference density always
        # matches the formation the operator actually spawned.
        "sph_reference_spacing": float(
            rospy.get_param("~sph_reference_spacing", 0.0)),
        # d_rep: activation distance of the bounded short-range repulsion [m].
        # Zero (the launch default) means "scale with the formation spacing".
        "sph_d_rep": float(rospy.get_param("~sph_d_rep", 0.0)),
        # Which agent the follow-camera RViz window centres on (that agent
        # broadcasts world->base).
        "rviz_follow_agent": _as_int(
            rospy.get_param("~rviz_follow_agent", 0), 0),
        # Smoothing length used both for the coordination kernel and for the
        # per-agent reference density handed to each provider instance.
        "sph_h": float(rospy.get_param("~sph_h", 2.0)),
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
    if not isinstance(options["agent_count"], int) or options["agent_count"] < 0:
        raise ScenarioValidationError("agent_count must be a non-negative integer")
    if not math.isfinite(options["formation_spacing"]) or \
            options["formation_spacing"] <= 0.0:
        raise ScenarioValidationError("formation_spacing must be finite and positive")
    # Zero means "follow formation_spacing"; anything else must be a real
    # spacing.  The paper ordering d_rep < d_star < 2*h is enforced by the
    # C++ provider, which reports the effective values on startup.
    if not math.isfinite(options["sph_reference_spacing"]) or \
            options["sph_reference_spacing"] < 0.0:
        raise ScenarioValidationError(
            "sph_reference_spacing must be finite and non-negative")
    if not math.isfinite(options["sph_h"]) or options["sph_h"] <= 0.0:
        raise ScenarioValidationError("sph_h must be finite and positive")
    if options["formation_shape"] not in ("disk", "ring", "line"):
        raise ScenarioValidationError("formation_shape must be disk|ring|line")
    if options["formation_axis"] not in ("x", "y"):
        raise ScenarioValidationError("formation_axis must be x|y")
    if not math.isfinite(options["formation_goal_translation_y"]):
        raise ScenarioValidationError("formation_goal_translation_y must be finite")
    if not math.isfinite(options["formation_goal_translation_x"]):
        raise ScenarioValidationError("formation_goal_translation_x must be finite")
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
        # `phase_offset_manual_tube_source` is retained as a compatibility
        # launch argument for recorded scenarios, but MANUAL production now
        # always builds the Section bundle.  It must not select a legacy
        # fixed/ESDF backend or reject an otherwise valid SPH scenario.
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
    for key in ("init_x", "init_y", "init_z"):
        value = options[key]
        if not math.isfinite(value):
            raise ScenarioValidationError(
                "%s must be finite" % key)
    if options["init_z"] <= 0.0:
        raise ScenarioValidationError("init_z must be greater than zero")
    for key in ("simulation_rate", "odom_rate", "map_resolution",
                "map_size_x", "map_size_y", "map_size_z",
                "local_update_range_x", "local_update_range_y",
                "local_update_range_z",
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
                "phase_offset_future_timestamp_tolerance"):
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
        if options["sph_reference_spacing"] <= 0.0:
            # Generated formations carry their own spacing; an explicit
            # scenario file keeps the phase_offset_agent.launch default.
            options["sph_reference_spacing"] = (
                options["formation_spacing"]
                if options["agent_count"] > 0 else 1.5)
        if options["sph_d_rep"] <= 0.0:
            # Follow the formation spacing the operator actually spawned, so
            # the paper ordering 0 < d_rep < d_star always holds.  The ratio
            # reproduces the validated 1.5 m pair (d_rep = 1.4 m).
            options["sph_d_rep"] = (
                SPH_REPULSION_RATIO * options["sph_reference_spacing"])
        if not 0.0 < options["sph_d_rep"] < options["sph_reference_spacing"]:
            raise ValueError(
                "sph_d_rep %.3f must satisfy 0 < sph_d_rep < sph_reference_spacing (%.3f)"
                % (options["sph_d_rep"], options["sph_reference_spacing"]))
        scenario_path = options["scenario_file"]
        use_scenario_file = isinstance(scenario_path, str) and \
            scenario_path.strip() and \
            scenario_path.strip().lower() not in ("none", "null")
        # A positive agent_count owns the swarm size: the launch argument is the
        # single place the operator sets N.  agent_count=0 falls back to the
        # explicit scenario file.
        if options["agent_count"] > 0:
            goal_base_raw = options["formation_goal_base_y"]
            goal_base_y = None
            if isinstance(goal_base_raw, str):
                goal_base_y = float(goal_base_raw) if goal_base_raw.strip() else None
            elif goal_base_raw not in (None, ""):
                goal_base_y = float(goal_base_raw)
            scenario = sph_disk_scenario(
                options["agent_count"],
                spacing=options["formation_spacing"],
                goal_translation_y=options["formation_goal_translation_y"],
                goal_reference_y=options["init_y"],
                goal_base_y=goal_base_y,
                shape=options["formation_shape"],
                axis=options["formation_axis"],
                goal_translation_x=options["formation_goal_translation_x"],
                goal_reference_x=options["init_x"])
            scenario_directory = tempfile.mkdtemp(prefix="sim_b_scenario_")
            scenario_path = os.path.join(scenario_directory,
                                         "sph_disk_scenario.yaml")
            write_scenario(scenario_path, scenario)
            options["scenario_file"] = scenario_path
            rospy.loginfo(
                "[SIM_B] generated the SPH-style disk formation for %d agents "
                "(spacing %.2f m, goal rows %.2f..%.2f) at %s",
                options["agent_count"], options["formation_spacing"],
                min(agent["goal_message"]["y"] for agent in scenario["agents"]),
                max(agent["goal_message"]["y"] for agent in scenario["agents"]),
                scenario_path)
        elif not use_scenario_file:
            raise ScenarioValidationError(
                "either a positive agent_count or a scenario_file is required")
        scenario = validate_scenario(options["scenario_file"], require_goals=True)
        scenario = translate_initial_states(
            scenario, options["init_x"], options["init_y"], options["init_z"])
        # The read-only world visualizer consumes the same agent list.  When the
        # world launch loaded a scenario file the parameter already exists; for a
        # generated scenario this is what makes bodies/labels/trails appear
        # without a second file-based command.
        try:
            rospy.set_param("/sim_b_world/scenario/agents",
                            [dict(agent) for agent in scenario["agents"]])
        except Exception as exc:  # pragma: no cover - parameter server only
            rospy.logwarn("[SIM_B] could not publish the scenario agents: %s", exc)
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
