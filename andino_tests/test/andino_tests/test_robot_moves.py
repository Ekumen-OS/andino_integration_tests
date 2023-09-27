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

"""Verifies the robot responds to cmd vels in an empty world."""

from math import atan2, pi
import os
from threading import Thread
import unittest

from ament_index_python.packages import get_package_share_directory
from angles import shortest_angular_distance_with_limits
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.markers
import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.signals import SignalHandlerGuardCondition
from sensor_msgs.msg import JointState

MOVE_FORWARD_TIME = 1.0

# Robot velocity
LINEAR_VELOCITY = 1.0
ANGULAR_VELOCITY = 0.0

# Robot displacement
DISPLACEMENT_X = 1.0
DISPLACEMENT_Y = 0.
DISPLACEMENT_YAW = 0.

# Movement tolerance
# Tolerance distance (x, y) [m], uncontrolled output
POSITION_TOLERANCE = 0.10
# Tolerance angle (yaw) [rad]
ANGLE_TOLERANCE = 0.18


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    # Launch andino in an empty world and verify it is publishing sensor data.
    andino_empty_world_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('andino_gz_classic'),
                'launch',
                'andino_one_robot.launch.py')
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
            launch_testing.actions.ReadyToTest()
        ]
    )


def yaw_from_quaternion(q):
    """
    Extract yaw angle from quaternion.

    Implements the formular from https://doi.org/10.5772/61313
    :param q -- Object with x,y,z,w properties of a quaternion
    :returns: The yaw angle from the quaternion.
    """
    return atan2(2 * (q.w * q.z + q.x * q.y),
                 q.w**2 + q.x**2 - q.y**2 - q.z**2)


class AndinoMobilityTest(unittest.TestCase):
    TIMEOUT_SEC = 10.0
    SERVICE_ENTITY_STATE = '/get_entity_state'
    TOPIC_CMD_VEL = '/cmd_vel'
    JOINT_STATES_TOPIC = '/joint_states'

    def setUp(self):
        # Initialize ROS 2 node
        rclpy.init()
        self.node = rclpy.create_node('andino_mobility_test_node')
        # Create client to get the entity state
        self.client = self.node.create_client(GetEntityState, self.SERVICE_ENTITY_STATE)
        # Create publisher to send velocity commands
        self.vel_pub = self.node.create_publisher(Twist, self.TOPIC_CMD_VEL, 10)
        # Create thread to receive messages
        self.spin_thread = Thread(target=rclpy.spin, args=(self.node,))
        self.spin_thread.start()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()
        self.spin_thread.join()

    # Note: the following function comes from
    # https://github.com/ros2/rclpy/blob/iron/rclpy/rclpy/wait_for_message.py
    # because it is not available for ROS 2 Humble. It was adapted
    # accordingly to the uses of this test class.
    #
    # Copyright: Open Source Robotics Foundation
    #
    # License note (can be found at https://github.com/ros2/rclpy/blob/iron/LICENSE):
    #
    # Licensed under the Apache License, Version 2.0 (the "License");
    # you may not use this file except in compliance with the License.
    # You may obtain a copy of the License at
    #
    #     http://www.apache.org/licenses/LICENSE-2.0
    #
    # Unless required by applicable law or agreed to in writing, software
    # distributed under the License is distributed on an "AS IS" BASIS,
    # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    # See the License for the specific language governing permissions and
    # limitations under the License.
    #
    # Changes:
    # - uses a node instance rather than an input argument.
    # - uses an inline time conversion instead of importing an extra
    # function to that end.
    def wait_for_message(
        self,
        msg_type,
        topic: str,
        time_to_wait: int = -1
    ):
        """
        Wait for the next incoming message.

        :param msg_type: message type
        :param topic: topic name to wait for message
        :param time_to_wait: seconds to wait before returning
        :returns: True if a message was successfully received, False if message
            could not be obtained or shutdown was triggered asynchronously on the context.
        """
        context = self.node.context
        wait_set = _rclpy.WaitSet(1, 1, 0, 0, 0, 0, context.handle)
        wait_set.clear_entities()

        sub = self.node.create_subscription(msg_type, topic, lambda _: None, 1)
        wait_set.add_subscription(sub.handle)
        sigint_gc = SignalHandlerGuardCondition(context=context)
        wait_set.add_guard_condition(sigint_gc.handle)

        timeout_nsec = int(float(time_to_wait) * 1e9)
        wait_set.wait(timeout_nsec)

        subs_ready = wait_set.get_ready_entities('subscription')
        guards_ready = wait_set.get_ready_entities('guard_condition')

        if guards_ready and sigint_gc.handle.pointer in guards_ready:
            return False

        if subs_ready and sub.handle.pointer in subs_ready:
            return True

        return False

    def move_robot_with_vel(self, twist, duration=0.0, rate=20.0):
        rate = self.node.create_rate(rate, self.node.get_clock())
        timeout_t = self.node.get_clock().now() + Duration(seconds=duration)
        while rclpy.ok() and self.node.get_clock().now() <= timeout_t:
            self.vel_pub.publish(twist)
            rate.sleep()

    def test_get_entity_state(self):
        # Wait for the service to be available
        service_availability = self.client.wait_for_service(timeout_sec=self.TIMEOUT_SEC)
        # Make sure that the robot is available
        robot_availability = self.wait_for_message(
                                            JointState,
                                            self.JOINT_STATES_TOPIC,
                                            time_to_wait=self.TIMEOUT_SEC)
        # Make sure that the robot is available
        self.assertTrue(service_availability, 'Service not available')
        self.assertTrue(robot_availability, 'Robot not available')

        request = GetEntityState.Request()
        request.name = 'andino'

        # Get initial robot state
        initial_state = self.client.call(request)
        initial_pose = initial_state.state.pose
        initial_yaw = yaw_from_quaternion(initial_pose.orientation)

        twist_msg = Twist()

        # Move robot
        twist_msg.linear.x = LINEAR_VELOCITY
        twist_msg.angular.z = ANGULAR_VELOCITY
        self.move_robot_with_vel(twist_msg, duration=MOVE_FORWARD_TIME)

        # Stop robot
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.vel_pub.publish(twist_msg)

        # Get final robot state
        final_state = self.client.call(request)
        final_pose = final_state.state.pose
        final_yaw = yaw_from_quaternion(final_pose.orientation)

        _, angle_diff = shortest_angular_distance_with_limits(final_yaw, initial_yaw, -pi, pi)

        self.assertAlmostEqual(
                DISPLACEMENT_X,
                abs(final_pose.position.x - initial_pose.position.x),
                delta=POSITION_TOLERANCE, msg='x failed')
        self.assertAlmostEqual(
                DISPLACEMENT_Y,
                abs(final_pose.position.y - initial_pose.position.y),
                delta=POSITION_TOLERANCE,
                msg='y failed')
        self.assertAlmostEqual(
                DISPLACEMENT_YAW, abs(angle_diff), delta=ANGLE_TOLERANCE, msg='yaw failed')
