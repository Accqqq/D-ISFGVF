#!/usr/bin/env python3
"""Scenario goal publisher for the phase-offset swarm simulation.

Publishes one PoseStamped goal per UAV to /uav_i/goal (once), reading the
scenario YAML:

  scenario:
    goals:
      - robot_id: 0
        x: 8.0
        y: 0.0
        z: 1.0

All new swarm scenarios use gvf/goal_z_mode=absolute_world, so z is the world
height.  Goals are sent once (plus a confirmation echo); the manager must not
receive periodic duplicates that would re-trigger path initialization.
"""

import math
import os
import sys
import time

import rospy
import yaml
from geometry_msgs.msg import PoseStamped


def main():
    rospy.init_node("swarm_scenario_publisher", anonymous=True)
    scenario_file = rospy.get_param("~scenario_file", "")
    if not scenario_file or not os.path.exists(scenario_file):
        rospy.logerr("scenario file not found: %s", scenario_file)
        return 2
    with open(scenario_file) as f:
        data = yaml.safe_load(f)
    goals = data.get("scenario", {}).get("goals", [])
    if not goals:
        rospy.logwarn("scenario %s has no goals", scenario_file)

    pubs = {}
    for g in goals:
        rid = int(g["robot_id"])
        topic = "/uav_%d/goal" % rid
        pubs[rid] = rospy.Publisher(topic, PoseStamped, queue_size=1)

    time.sleep(2.0)  # let the agents start
    for g in goals:
        rid = int(g["robot_id"])
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.pose.position.x = float(g["x"])
        msg.pose.position.y = float(g["y"])
        msg.pose.position.z = float(g["z"])
        msg.pose.orientation.w = 1.0
        pubs[rid].publish(msg)
        rospy.loginfo("published goal for uav_%d: (%.2f, %.2f, %.2f)",
                      rid, msg.pose.position.x, msg.pose.position.y,
                      msg.pose.position.z)
        time.sleep(0.2)

    rospy.loginfo("scenario publisher done (all goals sent once)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
