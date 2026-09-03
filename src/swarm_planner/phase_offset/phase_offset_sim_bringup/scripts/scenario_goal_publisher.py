#!/usr/bin/env python3
"""Publish one validated SIM-B ``PoseStamped`` goal for one UAV.

The process is intentionally short-lived.  A latched publisher waits for the
already-ready planner subscriber, emits exactly one message, keeps the
connection alive briefly for delivery, and exits.
"""

from __future__ import print_function

import argparse
import sys
import time

from swarm_orchestrator import (GOAL_TOPIC_TEMPLATE, ScenarioValidationError,
                                validate_scenario)


def _goal_subscriber_owners(rospy, master, topic):
    """Return the current master-registered subscribers for ``topic``."""
    _, subscriptions, _ = master.getSystemState()
    for registered_topic, owners in subscriptions:
        if str(registered_topic) == topic:
            return set(str(owner) for owner in owners)
    return set()


def _wait_for_goal_connections(rospy, publisher, topic, robot_id, timeout):
    """Wait for planner registration and a TCPROS connection to every owner.

    Master registration can precede the subscriber's TCPROS endpoint becoming
    reachable.  Checking only ``get_num_connections() > 0`` is unsafe when a
    second observer (such as an acceptance trace) connects first.  Require
    the protected planner owner explicitly, then wait until the publisher's
    connection count covers all owners currently registered on the topic.
    """
    import rosgraph

    planner_owner = "/uav_%d/formation_planning" % robot_id
    master = rosgraph.Master(rospy.get_name())
    deadline = time.monotonic() + float(timeout)
    stable_owners = None
    stable_since = None
    while time.monotonic() < deadline and not rospy.is_shutdown():
        try:
            owners = _goal_subscriber_owners(rospy, master, topic)
            connections = int(publisher.get_num_connections())
        except Exception:
            owners = set()
            connections = 0

        if owners != stable_owners:
            stable_owners = set(owners)
            stable_since = time.monotonic()

        if (planner_owner in owners and connections >= len(owners) and
                stable_since is not None and
                time.monotonic() - stable_since >= 0.10):
            return True
        time.sleep(0.02)
    raise RuntimeError(
        "goal subscriber TCPROS readiness exceeded %.1fs for %s "
        "(planner=%s, owners=%s, connections=%d)" %
        (float(timeout), topic, planner_owner,
         sorted(stable_owners or set()),
         int(publisher.get_num_connections())))


def publish_one(scenario_file, robot_id, wait_timeout=10.0):
    scenario = validate_scenario(scenario_file, require_goals=True)
    agent = next((item for item in scenario["agents"]
                  if item["robot_id"] == robot_id), None)
    if agent is None:
        raise ScenarioValidationError(
            "robot_id %d is not present in scenario" % robot_id)

    import rospy
    from geometry_msgs.msg import PoseStamped

    # This helper is deliberately short-lived and is started again on later
    # bringup rounds.  Let rospy append its PID/timestamp suffix so a stale
    # master NodeRef can never be mistaken for a different process that has
    # happened to reuse the same XML-RPC endpoint.  Keep the robot id in the
    # stem for diagnostics and graph inspection.
    rospy.init_node("scenario_goal_publisher_%d" % robot_id,
                    anonymous=True, disable_rosout=True)
    topic = GOAL_TOPIC_TEMPLATE.format(robot_id=robot_id)
    publisher = rospy.Publisher(topic, PoseStamped, queue_size=1, latch=True)
    _wait_for_goal_connections(rospy, publisher, topic, robot_id,
                               wait_timeout)

    goal = agent["goal_message"]
    message = PoseStamped()
    message.header.stamp = rospy.Time.now()
    message.header.frame_id = "world"
    message.pose.position.x = goal["x"]
    message.pose.position.y = goal["y"]
    message.pose.position.z = goal["z"]
    message.pose.orientation.x = 0.0
    message.pose.orientation.y = 0.0
    message.pose.orientation.z = 0.0
    message.pose.orientation.w = 1.0
    publisher.publish(message)
    # Do not use the rospy info logger here.  The rosout handler performs a cached
    # parameter registration that can outlive this short-lived publisher on
    # a busy master.  stdout is sufficient for the launch/test transcript.
    print("[SIM_B] published one goal on %s" % topic, flush=True)

    # Let TCPROS deliver the latched message before this short-lived process
    # exits.  No second publication is performed.
    time.sleep(0.15)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario-file", required=True)
    parser.add_argument("--robot-id", required=True, type=int)
    parser.add_argument("--wait-timeout", type=float, default=10.0)
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)
    try:
        if args.wait_timeout <= 0.0:
            raise ValueError("wait-timeout must be positive")
        return publish_one(args.scenario_file, args.robot_id,
                           args.wait_timeout)
    except (ScenarioValidationError, RuntimeError, ValueError) as exc:
        print("SIM_B_GOAL_INVALID: %s" % exc, file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
