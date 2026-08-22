#!/usr/bin/env python3

import math
import unittest

import geometry_msgs.msg
import nav_msgs.msg
import rospy
import rostest
from phase_offset_msgs.msg import AgentState


class SwarmIntentShadowAcceptance(unittest.TestCase):
    def setUp(self):
        self.coord = None
        self.sep = None
        self.coh = None
        self.conf = None
        self.odom_pub = rospy.Publisher(
            "/shadow/odom", nav_msgs.msg.Odometry, queue_size=10
        )
        self.state_pub = rospy.Publisher(
            "/shadow/agent_states", AgentState, queue_size=10
        )
        self.coord_sub = rospy.Subscriber(
            "/shadow/g_coord", geometry_msgs.msg.Vector3Stamped,
            lambda message: setattr(self, "coord", message)
        )
        self.sep_sub = rospy.Subscriber(
            "/shadow/g_sep", geometry_msgs.msg.Vector3Stamped,
            lambda message: setattr(self, "sep", message)
        )
        self.coh_sub = rospy.Subscriber(
            "/shadow/g_coh", geometry_msgs.msg.Vector3Stamped,
            lambda message: setattr(self, "coh", message)
        )
        self.conf_sub = rospy.Subscriber(
            "/shadow/g_conf", geometry_msgs.msg.Vector3Stamped,
            lambda message: setattr(self, "conf", message)
        )

    def publish_inputs(self):
        now = rospy.Time.now()
        odom = nav_msgs.msg.Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "world"
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0

        neighbor = AgentState()
        neighbor.header.stamp = now
        neighbor.header.frame_id = "world"
        neighbor.robot_id = 2
        neighbor.position_world.x = 0.5
        neighbor.position_world.y = 0.0
        neighbor.velocity_world.x = -0.5
        neighbor.velocity_world.y = 0.0
        self.odom_pub.publish(odom)
        self.state_pub.publish(neighbor)

    @staticmethod
    def finite_vector(message):
        return all(
            math.isfinite(value)
            for value in (message.vector.x, message.vector.y, message.vector.z)
        )

    def test_shadow_topics_and_output(self):
        rospy.sleep(1.0)
        deadline = rospy.Time.now() + rospy.Duration(8.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.publish_inputs()
            if all(value is not None for value in
                   (self.coord, self.sep, self.coh, self.conf)):
                break
            rospy.sleep(0.05)

        self.assertIsNotNone(self.coord)
        self.assertIsNotNone(self.sep)
        self.assertIsNotNone(self.coh)
        self.assertIsNotNone(self.conf)
        for message in (self.coord, self.sep, self.coh, self.conf):
            self.assertTrue(self.finite_vector(message))
        self.assertLess(self.coord.vector.x, 0.0)
        self.assertLess(self.sep.vector.x, 0.0)

        published_topics = rospy.get_published_topics()
        topics = dict(published_topics)
        topic_names = set(topics.keys())
        self.assertIn("/shadow/g_coord", topics)
        self.assertIn("/shadow/g_sep", topics)
        self.assertIn("/shadow/g_coh", topics)
        self.assertIn("/shadow/g_conf", topics)
        forbidden_command_names = {
            "/position_cmd", "/shadow/position_cmd", "/cmd", "/shadow/cmd"
        }
        self.assertEqual([], sorted(forbidden_command_names & topic_names))
        forbidden_command_types = [
            (name, topic_type) for name, topic_type in published_topics
            if topic_type.endswith("/PositionCommand")
        ]
        self.assertEqual([], forbidden_command_types)


if __name__ == "__main__":
    rospy.init_node("swarm_intent_shadow_acceptance")
    rostest.rosrun("phase_offset_swarm", "swarm_intent_shadow_acceptance",
                   SwarmIntentShadowAcceptance)
