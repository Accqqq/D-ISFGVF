#!/usr/bin/env python3
"""Per-agent shared-click goal translation (distributed, leaderless).

Every agent runs one instance of this node.  All instances subscribe to the
*same* operator click topic (RViz's "2D Nav Goal" default
``/move_base_simple/goal``) and each applies the identical, symmetric rule
locally, using only its own initial position::

    goal_i = initial_i + bias + clicked

The caller passes one shared ``bias`` equal to the negative formation centroid,
so the rule reduces to

    goal_i = clicked + (initial_i - centroid)

and the operator's click becomes the formation centre: every UAV keeps its own
formation offset around the clicked point.  One click therefore reaches the
whole formation at once, every agent keeps its relative offset, and nobody is
special: the instances are interchangeable, the arithmetic is identical, and
there is no coordinator, no fan-out node and no leader.
"""

from __future__ import print_function

import rospy
from geometry_msgs.msg import PoseStamped


class ClickedGoalTranslator(object):
    def __init__(self):
        # Comma-separated so a stale RViz config that still points the "2D Nav
        # Goal" tool at a legacy topic name keeps working.  The canonical name
        # is RViz's default /move_base_simple/goal; /goal is the relative name
        # some saved configs resolve to.
        click_spec = str(rospy.get_param("~click_source_topic",
                                         "/move_base_simple/goal"))
        click_topics = [name.strip() for name in click_spec.split(",")
                        if name.strip()]
        if not click_topics:
            raise rospy.ROSInitException("click_source_topic must be non-empty")
        goal_topic = rospy.get_param("~goal_topic", "/goal")
        self.frame_id = str(rospy.get_param("~frame_id", "world"))
        self.initial_x = float(rospy.get_param("~initial_x", 0.0))
        self.initial_y = float(rospy.get_param("~initial_y", 0.0))
        self.initial_z = float(rospy.get_param("~initial_z", 1.0))
        self.bias_x = float(rospy.get_param("~bias_x", 0.0))
        self.bias_y = float(rospy.get_param("~bias_y", 0.0))
        self.bias_z = float(rospy.get_param("~bias_z", 0.0))

        self.publisher = rospy.Publisher(goal_topic, PoseStamped, queue_size=10)
        self.click_count = 0
        self.subscribers = [
            rospy.Subscriber(name, PoseStamped, self._on_click, queue_size=10)
            for name in click_topics]
        rospy.loginfo(
            "clicked_goal_translator: %s -> %s  goal = (%.3f, %.3f, %.3f) + "
            "(%.3f, %.3f, %.3f) + clicked",
            ",".join(click_topics), goal_topic, self.initial_x, self.initial_y,
            self.initial_z, self.bias_x, self.bias_y, self.bias_z)

    def _on_click(self, message):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = (message.header.frame_id
                                if message.header.frame_id else self.frame_id)
        goal.pose.position.x = (self.initial_x + self.bias_x +
                                message.pose.position.x)
        goal.pose.position.y = (self.initial_y + self.bias_y +
                                message.pose.position.y)
        goal.pose.position.z = (self.initial_z + self.bias_z +
                                message.pose.position.z)
        goal.pose.orientation = message.pose.orientation
        if (goal.pose.orientation.x == 0.0 and
                goal.pose.orientation.y == 0.0 and
                goal.pose.orientation.z == 0.0 and
                goal.pose.orientation.w == 0.0):
            goal.pose.orientation.w = 1.0
        self.publisher.publish(goal)
        self.click_count += 1
        rospy.loginfo(
            "clicked_goal_translator: click %d -> goal (%.3f, %.3f, %.3f)",
            self.click_count, goal.pose.position.x, goal.pose.position.y,
            goal.pose.position.z)


def main():
    rospy.init_node("clicked_goal_translator", anonymous=False)
    ClickedGoalTranslator()
    rospy.spin()


if __name__ == "__main__":
    main()
