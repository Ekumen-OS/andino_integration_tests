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

"""Launch Gazebo with a world that has Andino and verify that navigation is working."""

from math import atan2, cos, radians, sin
import os
import unittest

from ament_index_python.packages import get_package_share_directory
from angles import shortest_angular_distance
from geometry_msgs.msg import PoseStamped, Quaternion
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_testing.markers
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import pytest
import rclpy
from rclpy.duration import Duration


class TestNavigator(BasicNavigator):

    def _amclPoseCallback(self, msg):
        self.initial_pose_received = True
        self.amcl_pose = PoseStamped()
        self.amcl_pose.pose = msg.pose.pose
        return


def yaw_from_quaternion(q: Quaternion):
    """
    Extract yaw angle from quaternion.

    Formula for the Z-Y-X aerospace sequence https://doi.org/10.5772/61313
    :param q: Object with x,y,z,w properties of a quaternion.
    :returns: yaw angle
    """
    return atan2(2 * (q.w * q.z + q.x * q.y), q.w ** 2 + q.x ** 2 - q.y ** 2 - q.z ** 2)


def quaternion_from_rpy(
    roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0
) -> Quaternion:
    """
    Convert the roll, pitch, yaw angles to a quaternion.

    :param
        roll: (default: {0.0})
        pitch: (default: {0.0})
        yaw: (default: {0.0})
    :returns: quaternion message with x,y,z,w
    """
    q = Quaternion()
    q.x = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(
        pitch / 2
    ) * sin(yaw / 2)
    q.y = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(
        pitch / 2
    ) * sin(yaw / 2)
    q.z = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(
        pitch / 2
    ) * cos(yaw / 2)
    q.w = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(
        pitch / 2
    ) * sin(yaw / 2)
    return q


def get_robot_initial_2d_pose():
    """
    Define the initial position of the robot.

    :returns x,y,yaw
    """
    return (-2.0, -0.5, 0.0)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    # Launch andino in initial navigation environment to test navigation is working.
    x, y, yaw = get_robot_initial_2d_pose()
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
            'initial_pose_x': LaunchConfiguration('initial_pose_x', default=str(x)),
            'initial_pose_y': LaunchConfiguration('initial_pose_y', default=str(y)),
            'initial_pose_yaw': LaunchConfiguration(
                'initial_pose_yaw', default=str(yaw)
            ),
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
    # Should be near the xy_goal_tolerance of Controller server.
    LINEAR_TOLERANCE = 0.3  # meters
    ANGULAR_TOLERANCE = 10  # degrees
    MAX_NUMBER_OF_RECOVERIES = 3
    MAX_TIME_NAV_TASK = 20e9  # nanoseconds

    def setUp(self):
        # Initialize ROS 2 node.
        rclpy.init()
        self.node = TestNavigator('test_andino_nav2_node')
        # TODO: add a timeout to the test to validate that it does
        # not hang for ever in case of error.

    def tearDown(self):
        # Cleanup.
        self.node.destroy_node()
        rclpy.shutdown()

    def test_move_to_pose(self):
        # Move to a pose using nav2 stack.
        # Set initial pose of andino
        x, y, yaw = get_robot_initial_2d_pose()
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.node.get_clock().now().to_msg()
        initial_pose.pose.position.x = x
        initial_pose.pose.position.y = y
        initial_pose.pose.orientation = quaternion_from_rpy(yaw=yaw)
        # Receive the localization pose
        self.node.setInitialPose(initial_pose)

        self.node.waitUntilNav2Active()

        # Evaluates that amcl report the estimated position.
        self.assertTrue(
            self.node.initial_pose_received, 'AMCL is not publishing estimated pose.'
        )
        self.assertAlmostEqual(
            self.node.amcl_pose.pose.position.x,
            initial_pose.pose.position.x,
            delta=self.LINEAR_TOLERANCE,
            msg=f'error in x estimation of amcl bigger than {self.LINEAR_TOLERANCE}m',
        )
        self.assertAlmostEqual(
            self.node.amcl_pose.pose.position.y,
            initial_pose.pose.position.y,
            delta=self.LINEAR_TOLERANCE,
            msg=f'error in y estimation of amcl bigger than {self.LINEAR_TOLERANCE}m',
        )
        yaw = yaw_from_quaternion(self.node.amcl_pose.pose.orientation)
        self.assertAlmostEqual(
            shortest_angular_distance(0.0, yaw),
            0.0,
            radians(self.ANGULAR_TOLERANCE),
            f'Error in yaw estimation of amcl bigger than {radians(self.ANGULAR_TOLERANCE)}rad',
        )

        self.node.clearAllCostmaps()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_pose.pose.position.x = 2.0
        goal_pose.pose.position.y = 0.5
        goal_pose.pose.orientation = quaternion_from_rpy(yaw=0.0)
        # Change initial pose to PoseStamped
        path = self.node.getPath(initial_pose, goal_pose)

        self.assertIsNotNone(path, 'No available path to the goal')
        self.assertGreater(len(path.poses), 1, 'No available path to the goal')
        self.node.goToPose(goal_pose)
        current_pose = None
        while not self.node.isTaskComplete():
            feedback = self.node.getFeedback()
            current_pose = feedback.current_pose
            self.assertLess(
                feedback.number_of_recoveries,
                self.MAX_NUMBER_OF_RECOVERIES,
                'Has been exceeded the number of recoveries',
            )
            self.assertLess(
                Duration.from_msg(feedback.navigation_time).nanoseconds,
                self.MAX_TIME_NAV_TASK,
                'Time out to move to the goal',
            )
        self.assertEqual(
            self.node.getResult(), TaskResult.SUCCEEDED, 'Error to move to the goal'
        )

        # Evaluates that goal is correct
        self.assertAlmostEqual(
            goal_pose.pose.position.x,
            current_pose.pose.position.x,
            delta=self.LINEAR_TOLERANCE,
            msg=f'error in x goal error bigger than {self.LINEAR_TOLERANCE}m',
        )
        self.assertAlmostEqual(
            goal_pose.pose.position.y,
            current_pose.pose.position.y,
            delta=self.LINEAR_TOLERANCE,
            msg=f'Error in in y goal error bigger than {self.LINEAR_TOLERANCE}m',
        )
        self.assertAlmostEqual(
            shortest_angular_distance(
                yaw_from_quaternion(goal_pose.pose.orientation),
                yaw_from_quaternion(current_pose.pose.orientation),
            ),
            0.0,
            delta=radians(self.ANGULAR_TOLERANCE),
            msg=f'Error in yaw goal error bigger than {radians(self.ANGULAR_TOLERANCE)}rad',
        )
