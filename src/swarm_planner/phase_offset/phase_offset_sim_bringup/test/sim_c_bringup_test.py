#!/usr/bin/env python3
"""Static SIM-C bringup and control-plane isolation checks."""

from __future__ import print_function

import os
import json
import math
import signal
import subprocess
import tempfile
import time
import threading
import unittest
import xml.etree.ElementTree as ET

PACKAGE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
WORKSPACE = os.path.abspath(os.path.join(PACKAGE_DIR, "../../../.."))
LAUNCH_DIR = os.path.join(PACKAGE_DIR, "launch")
SWARM_PACKAGE = os.path.join(WORKSPACE,
                             "src/swarm_planner/phase_offset/phase_offset_swarm")
SCRIPT_DIR = os.path.join(PACKAGE_DIR, "scripts")
N50_ACCEPTANCE_DIR = "/tmp/sim_c_n50_acceptance"
SIM_B_CPU_REFERENCE_PERCENT = 431.832486680
SIM_B_RSS_REFERENCE_KIB = 14095467.666667
import sys
sys.path.insert(0, SCRIPT_DIR)
from swarm_orchestrator import canonical_scenario, build_child_launch  # noqa: E402


def _proc_cmdline(pid):
    try:
        with open("/proc/%d/cmdline" % pid, "rb") as stream:
            return stream.read().replace(b"\x00", b" ").decode(
                "utf-8", "replace").strip()
    except (IOError, OSError, ValueError):
        return ""


def _proc_ppid(pid):
    try:
        with open("/proc/%d/stat" % pid, "r") as stream:
            tail = stream.read().rsplit(")", 1)[1].split()
        return int(tail[1])
    except (IOError, OSError, ValueError, IndexError):
        return None


def _proc_metric(pid):
    try:
        with open("/proc/%d/stat" % pid, "r") as stream:
            fields = stream.read().split()
        ticks = int(fields[13]) + int(fields[14])
        rss = 0
        with open("/proc/%d/status" % pid, "r") as stream:
            for line in stream:
                if line.startswith("VmRSS:"):
                    rss = int(line.split()[1])
                    break
        return ticks, rss
    except (IOError, OSError, ValueError, IndexError):
        return None


def _owned_process_tree(roots):
    root_set = {int(pid) for pid in roots if pid is not None}
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
    """Return task-owned processes whose command line contains a token."""
    matches = []
    for pid in sorted(pids):
        if command_token in _proc_cmdline(pid):
            matches.append(pid)
    return matches


def _percentile(values, percentile):
    """Return a linearly interpolated percentile, or None for no samples."""
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * float(percentile)
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return ordered[lower]
    fraction = rank - lower
    return ordered[lower] + fraction * (ordered[upper] - ordered[lower])


class _AgentStateTrace(object):
    """Minimal external trace for the frozen N=50 AgentState gate."""

    def __init__(self, rospy):
        from phase_offset_msgs.msg import AgentState

        self._lock = threading.Lock()
        self._recording = False
        self._window_start = None
        self._window_end = None
        self._records = []
        self._subscriber = rospy.Subscriber(
            "/phase_offset/agent_state", AgentState, self._callback,
            queue_size=2000)

    @staticmethod
    def _finite_message(message):
        stamp = message.header.stamp.to_sec()
        values = [stamp, message.position_world.x,
                  message.position_world.y, message.position_world.z,
                  message.velocity_world.x, message.velocity_world.y,
                  message.velocity_world.z]
        return all(math.isfinite(float(value)) for value in values)

    def _callback(self, message):
        arrival = time.monotonic()
        with self._lock:
            if not self._recording:
                return
            if (self._window_start is not None and
                    arrival < self._window_start):
                return
            if (self._window_end is not None and arrival >= self._window_end):
                return
            self._records.append((
                arrival,
                int(message.robot_id),
                float(message.header.stamp.to_sec()),
                str(message.header.frame_id),
                float(message.position_world.x),
                float(message.position_world.y),
                float(message.position_world.z),
                float(message.velocity_world.x),
                float(message.velocity_world.y),
                float(message.velocity_world.z),
                self._finite_message(message),
            ))

    def start(self):
        with self._lock:
            self._records = []
            self._window_start = time.monotonic()
            self._window_end = None
            self._recording = True
            return self._window_start

    def stop(self, cutoff=None):
        with self._lock:
            self._window_end = (time.monotonic() if cutoff is None else
                                float(cutoff))
            self._recording = False
            return self._window_end

    def snapshot(self):
        with self._lock:
            return list(self._records)

    def close(self):
        self._subscriber.unregister()


def _process_tree_sample(roots, previous, previous_time, now):
    """Sample task-owned tree and derive one-CPU-normalized CPU percent."""
    current = {}
    for pid in _owned_process_tree(roots):
        metric = _proc_metric(pid)
        if metric is not None:
            current[int(pid)] = metric

    rss_kb = sum(metric[1] or 0 for metric in current.values())
    cpu_percent = None
    if previous is not None and previous_time is not None and now > previous_time:
        ticks = 0
        for pid, metric in current.items():
            prior = previous.get(pid)
            if prior is not None:
                ticks += max(0, metric[0] - prior[0])
        hz = float(os.sysconf(os.sysconf_names["SC_CLK_TCK"]))
        cpu_percent = ticks / hz / (now - previous_time) * 100.0
    return {
        "monotonic": now,
        "cpu_percent": cpu_percent,
        "rss_kb": rss_kb,
        "process_count": len(current),
    }, current


class SimCBringupTest(unittest.TestCase):
    def test_static_contract_and_generic_n_generation(self):
        agent_launch = os.path.join(PACKAGE_DIR, "launch/phase_offset_agent.launch")
        swarm_launch = os.path.join(PACKAGE_DIR, "launch/phase_offset_swarm.launch")
        runtime_launch = os.path.join(
            SWARM_PACKAGE, "launch/agent_state_neighbor_runtime.launch")
        ET.parse(agent_launch)
        ET.parse(swarm_launch)
        ET.parse(runtime_launch)
        agent_text = open(agent_launch, "r").read()
        swarm_text = open(swarm_launch, "r").read()
        self.assertIn('name="enable_neighbor_transport" default="false"',
                      agent_text)
        self.assertIn('type="agent_state_neighbor_runtime_node"', agent_text)
        self.assertIn('name="sim_c_neighbor_runtime"', agent_text)
        self.assertIn('name="agent_count"', agent_text)
        self.assertIn('name="agent_state_topic"', agent_text)
        self.assertIn('name="neighbor_enter_radius"', agent_text)
        self.assertIn('name="neighbor_exit_radius"', agent_text)
        self.assertIn('name="all_visible_neighbors"', agent_text)
        runtime_launch_text = open(runtime_launch, "r").read()
        self.assertIn('type="agent_state_neighbor_runtime_node"',
                      runtime_launch_text)
        self.assertIn('name="sim_c_neighbor_runtime"', runtime_launch_text)
        self.assertIn('name="enable_neighbor_transport" default="false"',
                      swarm_text)
        self.assertIn('name="enable_neighbor_transport" value="$(arg enable_neighbor_transport)"',
                      swarm_text)

        config = os.path.join(SWARM_PACKAGE, "config/neighbor_transport_v1.yaml")
        config_text = open(config, "r").read()
        for line in ("world_frame: world", "publish_rate_hz: 20.0",
                     "own_odom_timeout: 0.20", "fresh_timeout: 0.30",
                     "lost_timeout: 0.60", "retention_timeout: 1.50",
                     "prediction_horizon_max: 0.30",
                     "future_timestamp_tolerance: 0.02",
                     "source_too_old_timeout: 0.60", "max_neighbors: 0"):
            self.assertIn(line, config_text)

        for count in (1, 3, 8, 50):
            child = build_child_launch(
                canonical_scenario(count), "/tmp/sim_c_initial.yaml",
                enable_neighbor_transport=True,
                agent_state_topic="/phase_offset/agent_state",
                all_visible_neighbors=True)
            self.assertEqual(count, child.count('agent_count" value="%d"' % count))
            self.assertEqual(count, child.count(
                'enable_neighbor_transport" value="true"'))
            self.assertEqual(count, child.count(
                'neighbor_enter_radius" value="100.0"'))
            self.assertEqual(count, child.count(
                'neighbor_exit_radius" value="101.0"'))
            self.assertEqual(count, child.count(
                'all_visible_neighbors" value="true"'))
            self.assertEqual(1, child.count("multi_simulator.launch"))
            self.assertEqual(count, child.count("phase_offset_agent.launch"))
            if count < 50:
                self.assertNotIn("uav_50", child)
            else:
                self.assertIn("uav_49", child)
                self.assertNotIn("uav_50", child)

        cmake_text = open(os.path.join(SWARM_PACKAGE, "CMakeLists.txt"),
                          "r").read()
        self.assertIn("add_executable(agent_state_neighbor_runtime_node", cmake_text)
        self.assertIn("std_srvs", cmake_text)
        package_text = open(os.path.join(SWARM_PACKAGE, "package.xml"),
                            "r").read()
        self.assertIn("<exec_depend>std_srvs</exec_depend>", package_text)

        runtime_text = open(os.path.join(
            SWARM_PACKAGE, "src/agent_state_neighbor_runtime_node.cpp"),
            "r").read()
        self.assertIn("ros::SteadyTime", runtime_text)
        self.assertIn("AgentState", runtime_text)
        self.assertIn("std_srvs/Trigger", runtime_text)
        for forbidden in ("g_des", "position_cmd", "so3_cmd",
                          "setPhaseOffsetGDes", "clearPhaseOffsetGDes",
                          "capturePhaseOffsetGDes"):
            self.assertNotIn(forbidden, runtime_text)

    @unittest.skipUnless(os.environ.get("SIM_C_RUN_RUNTIME") == "1",
                         "set SIM_C_RUN_RUNTIME=1 for SIM-C runtime matrix")
    def test_runtime_matrix(self):
        import rospy
        import rosgraph
        from std_srvs.srv import Trigger

        try:
            rospy.init_node("sim_c_bringup_runtime", anonymous=False)
        except rospy.exceptions.ROSException:
            pass

        counts = (1, 3, 8, 50)
        requested = os.environ.get("SIM_C_RUNTIME_COUNTS", "").strip()
        if requested:
            counts = tuple(int(item.strip()) for item in requested.split(",")
                           if item.strip())
        metrics = {}
        with tempfile.TemporaryDirectory(prefix="sim_c_runtime_") as directory:
            for count in counts:
                scenario = canonical_scenario(count, "sim_c_open_%d" % count)
                scenario_path = os.path.join(directory,
                                              "scenario_%d.yaml" % count)
                import yaml
                with open(scenario_path, "w") as stream:
                    yaml.safe_dump({"scenario": scenario}, stream,
                                   default_flow_style=False, sort_keys=False)
                world_log = os.path.join(directory, "world_%d.log" % count)
                swarm_log = os.path.join(directory, "swarm_%d.log" % count)
                world = subprocess.Popen(
                    ["roslaunch", os.path.join(LAUNCH_DIR,
                     "phase_offset_world.launch"),
                     "scenario_file:=" + scenario_path,
                     "enable_rviz:=false"], stdout=open(world_log, "w"),
                    stderr=subprocess.STDOUT, start_new_session=True)
                swarm = None
                try:
                    master = rosgraph.Master(rospy.get_name())

                    def system_state():
                        return master.getSystemState()

                    def node_names():
                        state = system_state()
                        names = set()
                        for section in state:
                            for _, owners in section:
                                names.update(str(owner) for owner in owners)
                        return names

                    def wait_for(predicate, timeout, description):
                        deadline = time.monotonic() + timeout
                        while time.monotonic() < deadline and not rospy.is_shutdown():
                            if predicate():
                                return
                            time.sleep(0.1)
                        try:
                            live = sorted(node_names())
                        except Exception as error:
                            live = ["<master query failed: %s>" % error]
                        try:
                            with open(world_log, "r") as stream:
                                world_tail = stream.read()[-4000:]
                        except (IOError, OSError):
                            world_tail = ""
                        print("SIM_C_READINESS_DIAGNOSTIC " + json.dumps({
                            "description": description,
                            "count": count,
                            "world_poll": world.poll(),
                            "swarm_poll": None if swarm is None else swarm.poll(),
                            "live_nodes": live,
                            "world_log_tail": world_tail,
                        }, sort_keys=True))
                        self.fail("SIM-C %s timed out for N=%d" %
                                  (description, count))

                    wait_for(lambda: {"/map_pub",
                                      "/phase_offset_swarm_visualizer"}.issubset(
                                          node_names()), 30.0, "WORLD readiness")
                    swarm = subprocess.Popen(
                        ["roslaunch", os.path.join(LAUNCH_DIR,
                         "phase_offset_swarm.launch"),
                         "scenario_file:=" + scenario_path,
                         "enable_neighbor_transport:=true",
                         "all_visible_neighbors:=true"],
                        stdout=open(swarm_log, "w"),
                        stderr=subprocess.STDOUT, start_new_session=True)

                    runtime_names = {
                        "/uav_%d/sim_c_neighbor_runtime" % index
                        for index in range(count)}
                    expected_nodes = runtime_names | {
                        "/multi_quadrotor_simulator_so3",
                        "/so3_nodelet_manager",
                        "/phase_offset_swarm_orchestrator",
                    }
                    for index in range(count):
                        expected_nodes.update({
                            "/so3_control_nodelet_%d" % index,
                            "/uav_%d/formation_planning" % index,
                        })
                    wait_for(lambda: expected_nodes.issubset(node_names()),
                             90.0, "runtime node readiness")

                    def service_ready():
                        try:
                            for index in range(count):
                                rospy.wait_for_service(
                                    "/uav_%d/sim_c_neighbor_runtime/snapshot" %
                                    index, timeout=0.2)
                            return True
                        except rospy.ROSException:
                            return False

                    wait_for(service_ready, 30.0, "snapshot services")
                    publications, subscriptions, _ = system_state()
                    pub_map = {str(topic): set(str(owner)
                                               for owner in owners)
                               for topic, owners in publications}
                    sub_map = {str(topic): set(str(owner)
                                               for owner in owners)
                               for topic, owners in subscriptions}
                    topic = "/phase_offset/agent_state"
                    self.assertEqual(count, len(pub_map.get(topic, set())))
                    self.assertEqual(count, len(sub_map.get(topic, set())))
                    self.assertNotIn("/uav_50", node_names())
                    proxies = [rospy.ServiceProxy(
                        "/uav_%d/sim_c_neighbor_runtime/snapshot" % index,
                        Trigger) for index in range(count)]
                    wait_for(lambda: all(
                        json.loads(proxy().message)["cache_record_count"] >= count - 1
                        for proxy in proxies), 15.0, "neighbor cache population")
                    for index, proxy in enumerate(proxies):
                        report = json.loads(proxy().message)
                        expected = [item for item in range(count)
                                    if item != index]
                        self.assertEqual(expected, report["active_ids"],
                                         "active IDs for uav_%d" % index)
                        self.assertEqual(count - 1,
                                         report["cache_record_count"])
                    metrics[str(count)] = {
                        "runtime_count": len(runtime_names),
                        "shared_publishers": len(pub_map.get(topic, set())),
                        "shared_subscribers": len(sub_map.get(topic, set())),
                    }
                finally:
                    for process in (swarm, world):
                        if process is None or process.poll() is not None:
                            continue
                        try:
                            os.killpg(process.pid, signal.SIGINT)
                        except OSError:
                            pass
                        try:
                            process.wait(timeout=10.0)
                        except subprocess.TimeoutExpired:
                            try:
                                os.killpg(process.pid, signal.SIGTERM)
                            except OSError:
                                pass
                            process.wait(timeout=5.0)
        print("SIM_C_RUNTIME_METRICS " + json.dumps(metrics, sort_keys=True))

    @unittest.skipUnless(os.environ.get("SIM_C_RUN_N50_60S") == "1",
                         "set SIM_C_RUN_N50_60S=1 for the N=50 60-second gate")
    def test_n50_60s_liveness_and_performance(self):
        """Run the frozen N=50 AgentState rate/liveness/performance gate."""
        import rospy
        import rosgraph
        from std_srvs.srv import Trigger

        try:
            rospy.init_node("sim_c_n50_60s", anonymous=False)
        except rospy.exceptions.ROSException:
            # rostest may have initialized the process already.
            pass

        count = 50
        scenario = canonical_scenario(count, "sim_c_open_50")
        run_id = time.strftime("run_%Y%m%dT%H%M%S", time.gmtime())
        artifact_dir = os.path.join(N50_ACCEPTANCE_DIR, run_id)
        os.makedirs(artifact_dir, exist_ok=True)
        scenario_path = os.path.join(artifact_dir, "scenario.yaml")
        world_log = os.path.join(artifact_dir, "world.log")
        swarm_log = os.path.join(artifact_dir, "swarm.log")
        with open(scenario_path, "w") as stream:
            import yaml
            yaml.safe_dump({"scenario": scenario}, stream,
                           default_flow_style=False, sort_keys=False)

        world_log_stream = open(world_log, "w")
        swarm_log_stream = None
        world = subprocess.Popen(
            ["roslaunch", os.path.join(LAUNCH_DIR,
             "phase_offset_world.launch"),
             "scenario_file:=" + scenario_path,
             "enable_rviz:=false"], stdout=world_log_stream,
            stderr=subprocess.STDOUT, start_new_session=True)
        swarm = None
        trace = _AgentStateTrace(rospy)
        proxies = []
        gate_reports = []
        process_samples = []
        master = rosgraph.Master(rospy.get_name())
        runtime_names = {
            "/uav_%d/sim_c_neighbor_runtime" % index
            for index in range(count)}
        expected_nodes = runtime_names | {
            "/multi_quadrotor_simulator_so3",
            "/so3_nodelet_manager",
            "/phase_offset_swarm_orchestrator",
        }
        for index in range(count):
            expected_nodes.update({
                "/so3_control_nodelet_%d" % index,
                "/uav_%d/formation_planning" % index,
            })
        service_names = {
            "/uav_%d/sim_c_neighbor_runtime/snapshot" % index
            for index in range(count)}
        topic = "/phase_offset/agent_state"
        proxy_names = [
            (index, "/uav_%d/sim_c_neighbor_runtime/snapshot" % index)
            for index in range(count)]

        def system_state():
            return master.getSystemState()

        def node_names():
            state = system_state()
            names = set()
            for section in state:
                for _, owners in section:
                    names.update(str(owner) for owner in owners)
            return names

        def topic_owners():
            publications, subscriptions, _ = system_state()
            pubs = {str(name): set(str(owner) for owner in owners)
                    for name, owners in publications}
            subs = {str(name): set(str(owner) for owner in owners)
                    for name, owners in subscriptions}
            return pubs, subs

        def service_owner_names():
            _, _, services = system_state()
            return {str(name) for name, _ in services}

        def wait_for(predicate, timeout, description):
            deadline = time.monotonic() + float(timeout)
            while time.monotonic() < deadline and not rospy.is_shutdown():
                try:
                    if predicate():
                        return True
                except Exception:
                    pass
                time.sleep(0.1)
            try:
                live = sorted(node_names())
            except Exception as error:
                live = ["<master query failed: %s>" % error]
            try:
                with open(world_log, "r") as stream:
                    tail = stream.read()[-4000:]
            except (IOError, OSError):
                tail = ""
            print("SIM_C_N50_READINESS_DIAGNOSTIC " + json.dumps({
                "description": description,
                "world_poll": world.poll(),
                "swarm_poll": None if swarm is None else swarm.poll(),
                "live_nodes": live,
                "world_log_tail": tail,
            }, sort_keys=True))
            return False

        def service_reports():
            reports = []
            failures = []
            for index, proxy in proxies:
                try:
                    response = proxy()
                    if not response.success:
                        failures.append("uav_%d service returned failure" % index)
                        continue
                    payload = json.loads(response.message)
                    reports.append((index, payload))
                except Exception as error:
                    failures.append("uav_%d service call failed: %s" %
                                    (index, error))
            return reports, failures

        def cache_ready():
            reports, failures = service_reports()
            if failures or len(reports) != count:
                return False
            for index, payload in reports:
                expected_ids = [item for item in range(count)
                                if item != index]
                if payload.get("active_ids") != expected_ids:
                    return False
                if int(payload.get("cache_record_count", -1)) < count - 1:
                    return False
            return True

        def steady_gate(elapsed):
            errors = []
            live = set()
            pubs = {}
            subs = {}
            services = set()
            try:
                live = node_names()
            except Exception as error:
                errors.append("master node query failed: %s" % error)
            try:
                pubs, subs = topic_owners()
            except Exception as error:
                errors.append("master topic query failed: %s" % error)
            try:
                services = service_owner_names()
            except Exception as error:
                errors.append("master service query failed: %s" % error)

            if world.poll() is not None:
                errors.append("WORLD exited with code %s" % world.returncode)
            if swarm is None or swarm.poll() is not None:
                errors.append("SWARM exited")
            missing_nodes = sorted(expected_nodes - live)
            if missing_nodes:
                errors.append("missing nodes: %s" % missing_nodes)
            forbidden_nodes = sorted(
                name for name in live
                if name == "/uav_50" or name.startswith("/uav_50/"))
            if forbidden_nodes:
                errors.append("unexpected /uav_50 nodes: %s" % forbidden_nodes)

            runtime_publishers = set(pubs.get(topic, set())) & runtime_names
            runtime_subscribers = set(subs.get(topic, set())) & runtime_names
            if runtime_publishers != runtime_names:
                errors.append("AgentState publishers=%d, expected=%d" %
                              (len(runtime_publishers), count))
            if runtime_subscribers != runtime_names:
                errors.append("AgentState runtime subscribers=%d, expected=%d" %
                              (len(runtime_subscribers), count))
            expected_subscribers = runtime_names | {trace_name}
            if set(subs.get(topic, set())) != expected_subscribers:
                errors.append("AgentState subscribers=%s, expected=%s" %
                              (sorted(subs.get(topic, set())),
                               sorted(expected_subscribers)))
            if not service_names.issubset(services):
                errors.append("snapshot services missing=%s" %
                              sorted(service_names - services))

            all_topics = set(pubs) | set(subs)
            for index in range(count):
                prefix = "/uav_%d" % index
                for suffix in ("/sim/odom", "/sim/imu", "/sim/local_map",
                               "/goal", "/position_cmd", "/so3_cmd",
                               "/force_disturbance", "/moment_disturbance",
                               "/motors", "/corrections"):
                    if prefix + suffix not in all_topics:
                        errors.append("missing topic %s" % (prefix + suffix))
                for command_suffix in ("/position_cmd", "/so3_cmd", "/goal"):
                    owners = (set(pubs.get(prefix + command_suffix, set())) |
                              set(subs.get(prefix + command_suffix, set())))
                    if any(owner in runtime_names for owner in owners):
                        errors.append("SIM-C runtime owns %s" %
                                      (prefix + command_suffix))
                position_owners = set(pubs.get(prefix + "/position_cmd", set()))
                if position_owners != {prefix + "/formation_planning"}:
                    errors.append("position_cmd owners for uav_%d=%s" %
                                  (index, sorted(position_owners)))
                so3_owners = set(pubs.get(prefix + "/so3_cmd", set()))
                if so3_owners != {"/so3_nodelet_manager"}:
                    errors.append("so3_cmd owners for uav_%d=%s" %
                                  (index, sorted(so3_owners)))

            reports, failures = service_reports()
            errors.extend(failures)
            for index, payload in reports:
                expected_ids = [item for item in range(count)
                                if item != index]
                active_ids = payload.get("active_ids")
                cache_count = int(payload.get("cache_record_count", -1))
                if active_ids != expected_ids:
                    errors.append("uav_%d active_ids=%s" %
                                  (index, active_ids))
                if cache_count < 0 or cache_count > count - 1:
                    errors.append("uav_%d cache_record_count=%d" %
                                  (index, cache_count))

            simulator_pids = _find_processes(
                _owned_process_tree([world.pid, swarm.pid]),
                "multi_quadrotor_simulator_so3")
            if len(simulator_pids) != 1:
                errors.append("task-owned simulator process count=%d" %
                              len(simulator_pids))
            return {
                "elapsed_s": float(elapsed),
                "pass": not errors,
                "errors": errors,
                "live_node_count": len(live),
                "agentstate_publisher_count": len(runtime_publishers),
                "agentstate_runtime_subscriber_count": len(runtime_subscribers),
                "agentstate_subscriber_count": len(subs.get(topic, set())),
                "snapshot_service_count": len(service_names & services),
                "simulator_process_count": len(simulator_pids),
                "cache_record_max": max(
                    [int(payload.get("cache_record_count", -1))
                     for _, payload in reports] or [-1]),
            }

        def append_process_sample(previous_tree, previous_time,
                                  previous_external, now):
            sample, current_tree = _process_tree_sample(
                [world.pid, swarm.pid], previous_tree, previous_time, now)
            external_metric = _proc_metric(os.getpid())
            external_cpu = None
            if (external_metric is not None and previous_external is not None and
                    previous_time is not None and now > previous_time):
                hz = float(os.sysconf(os.sysconf_names["SC_CLK_TCK"]))
                external_cpu = max(
                    0, external_metric[0] - previous_external[0]) / hz / (
                        now - previous_time) * 100.0
            sample["external_cpu_percent"] = external_cpu
            sample["external_rss_kb"] = (None if external_metric is None else
                                          external_metric[1])
            sample["external_process_pid"] = os.getpid()
            return sample, current_tree, external_metric

        try:
            if not wait_for(lambda: {"/map_pub",
                                     "/phase_offset_swarm_visualizer"}.issubset(
                                         node_names()), 30.0,
                            "WORLD readiness"):
                self.fail("SIM-C N=50 WORLD readiness timed out")

            swarm_log_stream = open(swarm_log, "w")
            swarm = subprocess.Popen(
                ["roslaunch", os.path.join(LAUNCH_DIR,
                 "phase_offset_swarm.launch"),
                 "scenario_file:=" + scenario_path,
                 "enable_neighbor_transport:=true",
                 "all_visible_neighbors:=true"],
                stdout=swarm_log_stream, stderr=subprocess.STDOUT,
                start_new_session=True)
            if not wait_for(lambda: expected_nodes.issubset(node_names()),
                            120.0, "runtime node readiness"):
                self.fail("SIM-C N=50 runtime node readiness timed out")

            proxies = [(index, rospy.ServiceProxy(name, Trigger))
                       for index, name in proxy_names]
            if not wait_for(lambda: service_names.issubset(service_owner_names()),
                            30.0, "snapshot services"):
                self.fail("SIM-C N=50 snapshot services timed out")
            if not wait_for(cache_ready, 30.0, "neighbor cache population"):
                self.fail("SIM-C N=50 neighbor cache population timed out")

            trace_name = str(rospy.get_name())
            if not wait_for(lambda: trace_name in topic_owners()[1].get(
                    topic, set()), 15.0, "external AgentState trace"):
                self.fail("SIM-C N=50 external AgentState trace did not connect")

            # Freeze a ten-second post-readiness interval before recording.
            readiness_start = time.monotonic()
            readiness_deadline = readiness_start + 10.0
            while (time.monotonic() < readiness_deadline and
                   not rospy.is_shutdown()):
                self.assertIsNone(world.poll())
                self.assertIsNotNone(swarm)
                self.assertIsNone(swarm.poll())
                remaining = readiness_deadline - time.monotonic()
                if remaining > 0.0:
                    time.sleep(min(0.2, remaining))

            window_start = trace.start()
            window_deadline = window_start + 60.0
            previous_tree = None
            previous_tree_time = None
            previous_external = None
            next_sample = window_start
            next_gate = window_start + 5.0
            while not rospy.is_shutdown():
                now = time.monotonic()
                if now >= window_deadline:
                    break
                target = min(next_sample, next_gate, window_deadline)
                if now < target:
                    time.sleep(min(0.10, target - now))
                    continue
                now = time.monotonic()
                if now >= next_sample:
                    sample, previous_tree, previous_external = append_process_sample(
                        previous_tree, previous_tree_time, previous_external, now)
                    process_samples.append(sample)
                    previous_tree_time = now
                    next_sample += 1.0
                if now >= next_gate:
                    gate_reports.append(steady_gate(now - window_start))
                    next_gate += 5.0

            # Capture a final process sample and stop the trace at the exact
            # requested cutoff; callbacks arriving later are excluded.
            final_now = time.monotonic()
            if final_now < window_deadline:
                final_now = window_deadline
            if not process_samples or process_samples[-1]["monotonic"] < window_deadline:
                sample, previous_tree, previous_external = append_process_sample(
                    previous_tree, previous_tree_time, previous_external,
                    final_now)
                process_samples.append(sample)
            trace.stop(window_deadline)
            records = trace.snapshot()

            # Analyze the one external trace by robot id and source stamp.
            by_id = {index: [] for index in range(count)}
            unknown_ids = []
            invalid_records = []
            frame_errors = []
            rewind_count = 0
            for record in records:
                robot_id = record[1]
                if robot_id not in by_id:
                    unknown_ids.append(robot_id)
                    continue
                by_id[robot_id].append(record)
                if not record[10]:
                    invalid_records.append(record)
                if record[3] != "world":
                    frame_errors.append(record)

            rates = {}
            source_rewinds = {}
            for robot_id in range(count):
                samples = by_id[robot_id]
                stamps = [item[2] for item in samples]
                rewinds = sum(1 for prior, current in zip(stamps, stamps[1:])
                              if current < prior)
                source_rewinds[robot_id] = rewinds
                rewind_count += rewinds
                if len(samples) >= 2 and samples[-1][0] > samples[0][0]:
                    rates[robot_id] = float(len(samples) - 1) / (
                        samples[-1][0] - samples[0][0])
                else:
                    rates[robot_id] = 0.0

            rate_values = list(rates.values())
            cpu_values = [sample["cpu_percent"] for sample in process_samples
                          if sample.get("cpu_percent") is not None]
            rss_values = [sample["rss_kb"] for sample in process_samples
                          if sample.get("rss_kb") is not None and
                          sample.get("process_count", 0) > 0]
            external_cpu_values = [
                sample["external_cpu_percent"] for sample in process_samples
                if sample.get("external_cpu_percent") is not None]
            cpu_mean = (sum(cpu_values) / len(cpu_values)
                        if cpu_values else None)
            rss_mean = (sum(rss_values) / len(rss_values)
                        if rss_values else None)
            external_cpu_mean = (
                sum(external_cpu_values) / len(external_cpu_values)
                if external_cpu_values else None)
            liveness_failures = []
            if set(unknown_ids):
                liveness_failures.append("unknown robot ids: %s" %
                                         sorted(set(unknown_ids)))
            if invalid_records:
                liveness_failures.append(
                    "non-finite AgentState records: %d" % len(invalid_records))
            if frame_errors:
                liveness_failures.append(
                    "AgentState frame errors: %d" % len(frame_errors))
            if rewind_count:
                liveness_failures.append(
                    "source stamp rewinds: %d" % rewind_count)
            if not all(len(by_id[index]) >= 2 for index in range(count)):
                liveness_failures.append("one or more AgentState streams are empty")
            if not all(19.0 <= value <= 21.0 for value in rate_values):
                liveness_failures.append("AgentState rate outside [19,21] Hz")
            if len(gate_reports) < 11:
                liveness_failures.append(
                    "insufficient 5-second topology gates: %d" %
                    len(gate_reports))
            failed_gates = [gate for gate in gate_reports if not gate["pass"]]
            if failed_gates:
                liveness_failures.append(
                    "failed topology gates: %d" % len(failed_gates))
            if len(cpu_values) < 10:
                liveness_failures.append("insufficient process CPU samples")
            if len(rss_values) < 10:
                liveness_failures.append("insufficient process RSS samples")
            liveness_status = "PASS" if not liveness_failures else "FAIL"
            report = {
                "artifact_dir": artifact_dir,
                "window_requested_s": 60.0,
                "window_actual_s": 60.0,
                "readiness_wait_s": 10.0,
                "agentstate_records": len(records),
                "agentstate_counts": {
                    str(index): len(by_id[index]) for index in range(count)},
                "agentstate_rates_hz": {
                    str(index): rates[index] for index in range(count)},
                "agentstate_rate_min_hz": min(rate_values or [0.0]),
                "agentstate_rate_median_hz": (
                    sorted(rate_values)[len(rate_values) // 2]
                    if rate_values else 0.0),
                "agentstate_rate_max_hz": max(rate_values or [0.0]),
                "source_stamp_rewinds": rewind_count,
                "source_stamp_rewinds_by_robot": {
                    str(index): source_rewinds[index]
                    for index in range(count)},
                "unknown_robot_ids": unknown_ids,
                "invalid_records": len(invalid_records),
                "frame_errors": len(frame_errors),
                "topology_gates": gate_reports,
                "process_samples": len(process_samples),
                "total_sim_c_cpu_mean_percent": cpu_mean,
                "total_sim_c_cpu_p95_percent": _percentile(cpu_values, 0.95),
                "total_sim_c_cpu_max_percent": max(cpu_values or [0.0]),
                "total_sim_c_rss_mean_kib": rss_mean,
                "total_sim_c_rss_max_kib": max(rss_values or [0]),
                "external_collector_cpu_mean_percent": external_cpu_mean,
                "external_collector_cpu_p95_percent": _percentile(
                    external_cpu_values, 0.95),
                "external_collector_cpu_max_percent": max(
                    external_cpu_values or [0.0]),
                "sim_c_cpu_delta_from_sim_b": (
                    None if cpu_mean is None else
                    cpu_mean - SIM_B_CPU_REFERENCE_PERCENT),
                "sim_c_rss_delta_from_sim_b": (
                    None if rss_mean is None else
                    rss_mean - SIM_B_RSS_REFERENCE_KIB),
                "N50_STEADY_REALTIME": "NOT_PROVEN",
                "N50_SIM_C_60S_LIVENESS": liveness_status,
                "TOTAL_SIM_C_CPU_MEAN": cpu_mean,
                "TOTAL_SIM_C_CPU_P95": _percentile(cpu_values, 0.95),
                "TOTAL_SIM_C_CPU_MAX": max(cpu_values or [0.0]),
                "TOTAL_SIM_C_RSS_MEAN": rss_mean,
                "TOTAL_SIM_C_RSS_MAX": max(rss_values or [0]),
                "SIM_C_CPU_DELTA_FROM_SIM_B": (
                    None if cpu_mean is None else
                    cpu_mean - SIM_B_CPU_REFERENCE_PERCENT),
                "SIM_C_RSS_DELTA_FROM_SIM_B": (
                    None if rss_mean is None else
                    rss_mean - SIM_B_RSS_REFERENCE_KIB),
                "liveness_failures": liveness_failures,
            }
            with open(os.path.join(artifact_dir, "agent_state_trace.jsonl"),
                      "w") as stream:
                for record in records:
                    stream.write(json.dumps({
                        "arrival_monotonic": record[0],
                        "robot_id": record[1],
                        "source_stamp": record[2],
                        "frame_id": record[3],
                        "position": list(record[4:7]),
                        "velocity": list(record[7:10]),
                        "finite": record[10],
                    }, sort_keys=True) + "\n")
            with open(os.path.join(artifact_dir, "process_metrics.jsonl"),
                      "w") as stream:
                for sample in process_samples:
                    stream.write(json.dumps(sample, sort_keys=True) + "\n")
            with open(os.path.join(artifact_dir, "acceptance_report.json"),
                      "w") as stream:
                json.dump(report, stream, sort_keys=True, indent=2)
                stream.write("\n")
            print("SIM_C_N50_ACCEPTANCE " + json.dumps(
                report, sort_keys=True, allow_nan=False))

            self.assertFalse(liveness_failures, "; ".join(liveness_failures))
        finally:
            trace.close()
            for process in (swarm, world):
                if process is None or process.poll() is not None:
                    continue
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except OSError:
                    pass
                try:
                    process.wait(timeout=10.0)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(process.pid, signal.SIGTERM)
                    except OSError:
                        pass
                    try:
                        process.wait(timeout=5.0)
                    except subprocess.TimeoutExpired:
                        try:
                            os.killpg(process.pid, signal.SIGKILL)
                        except OSError:
                            pass
            if swarm_log_stream is not None:
                swarm_log_stream.close()
            world_log_stream.close()


if __name__ == "__main__":
    import rostest
    rostest.rosrun("phase_offset_sim_bringup", "sim_c_bringup",
                   SimCBringupTest)
