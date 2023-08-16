# Copyright 2023 Ekumen, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Verify the initial pose is set accordingly in the localization stack."""

from math import atan2, radians
import os
import unittest

from ament_index_python.packages import get_package_share_directory
from angles import shortest_angular_distance
from geometry_msgs.msg import PoseWithCovarianceStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.markers
from lifecycle_msgs.srv import GetState
import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


def yaw_from_quaternion(q):
    """
    Extract yaw angle from quaternion.

    Formula for the Z-Y-X aerospace sequence https://doi.org/10.5772/61313
    :param q: Object with x,y,z,w properties of a quaternion.
    :returns: yaw angle
    """
    return atan2(2 * (q.w * q.z + q.x * q.y), q.w ** 2 + q.x ** 2 - q.y ** 2 - q.z ** 2)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    # Launch andino in initial navigation environment to test navigation is working.
    andino_empty_world_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('andino_navigation'),
                'launch',
                'andino_simulation_navigation.launch.py',
            )
        ),
        launch_arguments={
            'use_gazebo_ros_control': 'false',
            'gui': 'false',
            'rviz': 'false',
        }.items(),
    )

    return LaunchDescription(
        [
            andino_empty_world_gazebo,
            # Tell launch to start the test
            launch_testing.actions.ReadyToTest(),
        ]
    )


class AndinoNav2InGazeboTest(unittest.TestCase):
    AMCL_POSE_TOPIC = 'amcl_pose'
    INITIAL_POSE_TOPIC = 'initialpose'
    MAX_TOPIC_TIMEOUT = 30
    MAX_NODES_AVAILABLE_TIMEOUT = 20
    LINEAR_TOLERANCE = 0.1  # meters
    ANGULAR_TOLERANCE = 5  # degrees

    def setUp(self):
        # Initialize ROS 2 node
        rclpy.init()
        self.node = rclpy.create_node('test_andino_nav2_node')

    def tearDown(self):
        # Cleanup
        self.node.destroy_node()
        rclpy.shutdown()

    def wait_for_node_become_active(self, node_name: str, timeout_sec: int):
        """
        Service client to get_state service of the node until get active or time out.

        :param node_name: node name of the service get_state.
        :param timeout_sec: time out in seconds to get active state.
        :returns: The state of the node.
        """
        state_client = self.node.create_client(GetState, f'{node_name}/get_state')
        self.assertTrue(state_client.wait_for_service(timeout_sec=timeout_sec))
        req = GetState.Request()
        state = 'unknown'
        consumed_time_sec = 0
        while state != 'active' and consumed_time_sec < timeout_sec:
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.)
            if future.result() is not None:
                state = future.result().current_state.label
            self.node.get_clock().sleep_for(rel_time=Duration(seconds=1))
            consumed_time_sec += 1
        return state

    def test_amcl_start(self):
        # Wait for localization nodes to be active
        state = self.wait_for_node_become_active(
            'amcl', self.MAX_NODES_AVAILABLE_TIMEOUT
        )
        self.assertEqual(state, 'active', 'The amcl node is not active')

        # Publish the initial pose
        initial_pose_pub = self.node.create_publisher(
            PoseWithCovarianceStamped, self.INITIAL_POSE_TOPIC, 10
        )
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.node.get_clock().now().to_msg()
        initial_pose.pose.pose.position.x = -2.0
        initial_pose.pose.pose.position.y = -0.5
        initial_pose.pose.pose.orientation.z = 0.0
        initial_pose.pose.pose.orientation.w = 1.0
        initial_pose_pub.publish(initial_pose)
        is_published = initial_pose_pub.wait_for_all_acked(Duration(seconds=1))
        self.assertTrue(is_published, 'Initial pose is not published.')
        self.node.get_clock().sleep_for(rel_time=Duration(seconds=1))
        # Receive the localization pose
        amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Read the localization output from AMCL.
        pose_amcl = None

        def amcl_pose_callback(msg):
            nonlocal pose_amcl
            pose_amcl = msg

        # Subscribe to AMCL to obtain the pose, it should not be that different from
        # the one we set before.
        amcl_pose_subscriber = self.node.create_subscription(
            PoseWithCovarianceStamped,
            self.AMCL_POSE_TOPIC,
            amcl_pose_callback,
            amcl_pose_qos
        )
        timeout_t = self.node.get_clock().now() + Duration(seconds=self.MAX_TOPIC_TIMEOUT)
        while rclpy.ok() and self.node.get_clock().now() < timeout_t and pose_amcl is None:
            # Send again the initial pose, the amcl might not have received it yet.
            initial_pose_pub.publish(initial_pose)
            rclpy.spin_once(node=self.node, timeout_sec=0.5)
        amcl_pose_subscriber.destroy()

        # Evaluates that amcl report the estimated position.
        self.assertTrue(pose_amcl is not None, 'AMCL is not publishing estimated pose.')
        self.assertAlmostEqual(
            pose_amcl.pose.pose.position.x,
            initial_pose.pose.pose.position.x,
            delta=self.LINEAR_TOLERANCE,
            msg=f'Error in x estimation of amcl bigger than {self.LINEAR_TOLERANCE}m',
        )
        self.assertAlmostEqual(
            pose_amcl.pose.pose.position.y,
            initial_pose.pose.pose.position.y,
            delta=self.LINEAR_TOLERANCE,
            msg=f'Error in y estimation of amcl bigger than {self.LINEAR_TOLERANCE}m',
        )
        yaw = yaw_from_quaternion(pose_amcl.pose.pose.orientation)
        self.assertAlmostEqual(
            shortest_angular_distance(0.0, yaw),
            0.0,
            radians(self.ANGULAR_TOLERANCE),
            f'Error in yaw estimation of amcl bigger than {self.ANGULAR_TOLERANCE}grad',
        )
