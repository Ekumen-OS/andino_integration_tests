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

"""Launch Gazebo with a world that has Andino and verify that sensors are publishing."""

import os
from threading import Thread
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.markers
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.signals import SignalHandlerGuardCondition
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from sensor_msgs.msg import LaserScan


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    # Launch andino in an empty world and verify it is publishing sensor data.
    andino_empty_world_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('andino_gazebo'),
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
            # Tell launch to start the test
            launch_testing.actions.ReadyToTest()
        ]
    )


class AndinoSensorsInGazeboTest(unittest.TestCase):
    SCAN_TOPIC = '/scan'
    ODOM_TOPIC = '/odom'
    JOINT_STATES_TOPIC = '/joint_states'
    CAMERA_TOPIC = '/camera/image_raw'
    MAX_TOPIC_TIMEOUT = 10.0

    def setUp(self):
        # Initialize ROS 2 node
        rclpy.init()
        self.node = rclpy.create_node('test_andino_sensors_node')
        # Create thread to receive messages
        self.spin_thread = Thread(target=rclpy.spin, args=(self.node,))
        self.spin_thread.start()

    def tearDown(self):
        # Cleanup
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

    def test_sensors_are_reporting(self):
        """Evaluates that sensors are reporting messages."""
        is_odom_reporting = self.wait_for_message(
            Odometry, self.ODOM_TOPIC, time_to_wait=self.MAX_TOPIC_TIMEOUT)
        is_joint_states_reporting = self.wait_for_message(
            JointState, self.JOINT_STATES_TOPIC, time_to_wait=self.MAX_TOPIC_TIMEOUT)
        is_camera_reporting = self.wait_for_message(
            Image, self.CAMERA_TOPIC, time_to_wait=self.MAX_TOPIC_TIMEOUT)

        self.assertTrue(False, 'Lidar topic has not been received.')
        self.assertTrue(is_odom_reporting, 'Odom topic has not been received.')
        self.assertTrue(is_joint_states_reporting, 'Joint states topic has not been received.')
        self.assertTrue(is_camera_reporting, 'Camera topic has not been received.')
