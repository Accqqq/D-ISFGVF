#!/usr/bin/env python3
"""Isolated ROS transport proof for the SIM-C data plane."""

from __future__ import print_function

import json
import math
import threading
import time
import unittest

import rosgraph
import rospy
import rostest
from nav_msgs.msg import Odometry
from phase_offset_msgs.msg import AgentState
from std_srvs.srv import Trigger


TOPIC = "/sim_c_test/agent_state"


class AgentStateNeighborTransportTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.states = []
        self.odom_publishers = [
            rospy.Publisher("/sim_c_test/uav_0/odom", Odometry,
                            queue_size=10),
            rospy.Publisher("/sim_c_test/uav_1/odom", Odometry,
                            queue_size=10),
        ]
        self.synthetic = rospy.Publisher(TOPIC, AgentState, queue_size=100)
        self.subscription = rospy.Subscriber(TOPIC, AgentState,
                                              self._state_callback,
                                              queue_size=500)
        self.services = [
            rospy.ServiceProxy("/uav_0/sim_c_neighbor_runtime/snapshot",
                               Trigger),
            rospy.ServiceProxy("/uav_1/sim_c_neighbor_runtime/snapshot",
                               Trigger),
        ]
        for service in self.services:
            try:
                service.wait_for_service(timeout=10.0)
            except rospy.ROSException:
                self.fail("snapshot service did not become available")
        self._wait_for_connections()

    def tearDown(self):
        self.subscription.unregister()
        for publisher in self.odom_publishers:
            if publisher is not None:
                publisher.unregister()
        self.synthetic.unregister()

    def _state_callback(self, message):
        with self.lock:
            self.states.append(message)

    def _wait_for_connections(self):
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if (all(publisher.get_num_connections() > 0
                    for publisher in self.odom_publishers) and
                    self.synthetic.get_num_connections() >= 2):
                return
            time.sleep(0.05)
        self.fail("runtime odometry subscribers did not connect")

    @staticmethod
    def _odom(robot_id, stamp):
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = "world"
        message.child_frame_id = "uav_%d/base_link" % robot_id
        message.pose.pose.position.x = 10.0 + robot_id
        message.pose.pose.position.y = -2.0 - robot_id
        message.pose.pose.position.z = 3.0 + robot_id
        message.twist.twist.linear.x = 0.5 + robot_id
        message.twist.twist.linear.y = -0.25
        message.twist.twist.linear.z = 0.125
        return message

    @staticmethod
    def _state(robot_id, stamp, frame="world", x=1.0, nan=False):
        message = AgentState()
        message.header.stamp = stamp
        message.header.frame_id = frame
        message.robot_id = robot_id
        message.position_world.x = x
        message.position_world.y = 2.0
        message.position_world.z = 3.0
        message.velocity_world.x = 0.5
        message.velocity_world.y = -0.25
        message.velocity_world.z = 0.125
        if nan:
            message.position_world.z = float("nan")
        return message

    def _publish_odom_once(self):
        stamp = rospy.Time.now()
        for robot_id, publisher in enumerate(self.odom_publishers):
            if publisher is not None:
                publisher.publish(self._odom(robot_id, stamp))

    def _publish_odom_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline and not rospy.is_shutdown():
            self._publish_odom_once()
            time.sleep(0.02)

    def _snapshot(self, index):
        response = self.services[index]()
        self.assertTrue(response.success)
        try:
            return json.loads(response.message)
        except (TypeError, ValueError) as error:
            self.fail("snapshot service returned invalid JSON: %s" % error)

    def _wait_for(self, predicate, timeout=5.0, interval=0.05):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if predicate():
                return
            time.sleep(interval)
        self.fail("condition did not become true before %.1fs" % timeout)

    def test_transport_and_snapshot_contract(self):
        self._publish_odom_for(1.5)
        self._wait_for(lambda: len(self.states) >= 5)
        with self.lock:
            received = list(self.states)
        self.assertEqual({0, 1}, {int(message.robot_id) for message in received})
        for message in received:
            self.assertEqual("world", message.header.frame_id)
            self.assertTrue(message.header.stamp.to_sec() > 0.0)
            values = [message.position_world.x, message.position_world.y,
                      message.position_world.z, message.velocity_world.x,
                      message.velocity_world.y, message.velocity_world.z]
            self.assertTrue(all(math.isfinite(value) for value in values))

        stamp = rospy.Time.now()
        self.synthetic.publish(self._state(1, stamp, x=1.0))
        self.synthetic.publish(self._state(0, stamp, x=2.0))
        self._publish_odom_for(0.5)
        self._wait_for(lambda: self._snapshot(0)["counters"]["ACCEPTED"] >= 1)
        self._wait_for(lambda: self._snapshot(0)["active_ids"] == [1])
        self._wait_for(lambda: self._snapshot(1)["active_ids"] == [0])

        # Equal and older source stamps are rejected and cannot rewind the
        # selected state.  The service exposes deterministic counters.
        self.odom_publishers[1].unregister()
        self.odom_publishers[1] = None
        duplicate_stamp = rospy.Time.now()
        self.synthetic.publish(self._state(1, duplicate_stamp, x=99.0))
        self.synthetic.publish(self._state(1, duplicate_stamp, x=98.5))
        self.synthetic.publish(self._state(1, duplicate_stamp - rospy.Duration(0.1),
                                           x=98.0))
        self.synthetic.publish(self._state(3, rospy.Time.now(), x=97.0))
        self.synthetic.publish(self._state(1, rospy.Time.now(), frame="map",
                                           x=96.0))
        self.synthetic.publish(self._state(1, rospy.Time.now(), x=95.0,
                                           nan=True))
        self._publish_odom_for(0.1)
        self._wait_for(lambda: self._snapshot(0)["counters"]["ACCEPTED"] >= 2)
        counters = self._snapshot(0)["counters"]
        self.assertGreaterEqual(counters["DUPLICATE"], 1)
        self.assertGreaterEqual(counters["OUT_OF_ORDER"], 1)
        self.assertGreaterEqual(counters["INVALID_ID"], 1)
        self.assertGreaterEqual(counters["FRAME_MISMATCH"], 1)
        self.assertGreaterEqual(counters["NONFINITE_STATE"], 1)

        # Receive silence, rather than source clock offset, drives health and
        # retention.  Keep own odometry alive while withholding remote state.
        self.synthetic.unregister()
        self._publish_odom_for(0.35)
        self._wait_for(lambda: self._snapshot(0)["active_ids"] == [1])
        self._publish_odom_for(0.30)
        self._wait_for(lambda: self._snapshot(0)["active_ids"] == [])
        self.assertGreaterEqual(self._snapshot(0)["lost_count"], 1)
        self._publish_odom_for(1.0)
        self.assertEqual(0, self._snapshot(0)["cache_record_count"])

        # A newer valid publisher can repopulate the same keyed record.
        self.synthetic = rospy.Publisher(TOPIC, AgentState, queue_size=100)
        self._wait_for(lambda: self.synthetic.get_num_connections() >= 2)
        self.synthetic.publish(self._state(1, rospy.Time.now(), x=4.0))
        self._publish_odom_for(0.4)
        self._wait_for(lambda: self._snapshot(0)["active_ids"] == [1])

        publications, subscriptions, services = rosgraph.Master(
            rospy.get_name()).getSystemState()
        forbidden = {"/goal", "/position_cmd", "/so3_cmd", "/g_coord",
                     "/g_des"}
        owned_names = {"/uav_0/sim_c_neighbor_runtime",
                       "/uav_1/sim_c_neighbor_runtime"}
        for topic, owners in publications:
            if str(topic) in forbidden:
                self.assertTrue(owned_names.isdisjoint(set(owners)))
        for topic, owners in subscriptions:
            if str(topic) in forbidden:
                self.assertTrue(owned_names.isdisjoint(set(owners)))


if __name__ == "__main__":
    rospy.init_node("agent_state_neighbor_transport_test", anonymous=True)
    rostest.rosrun("phase_offset_swarm", "agent_state_neighbor_transport",
                   AgentStateNeighborTransportTest)
