# Copyright (c) 2024 TOYOTA MOTOR CORPORATION
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted (subject to the limitations in the disclaimer
# below) provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of the copyright holder nor the names of its contributors may be used
#   to endorse or promote products derived from this software without specific
#   prior written permission.
# NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
# LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
# OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.
# vim: fileencoding=utf-8
#
# Maintainer: Tomoaki Fujino (Hibikino-Musashi@Home)
"""Provide abstract inteface for a mobile base."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

from typing import Union, List, Tuple
import math
import warnings

import action_msgs.msg as action_msgs
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
import tf_transformations
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint

from . import exceptions
from . import geometry
from . import robot
from . import settings
from . import nav2_goal_canceller
from . import trajectory
from . import utils

# Timeout to receve enough tf transform [sec]
_TF_TIMEOUT = 5.0

_ACTION_WAIT_TIMEOUT = 3.0


def _validate_timeout(timeout):
    """Validate a given timeout value is meaning time value."""
    if timeout < 0.0 or math.isnan(timeout) or math.isinf(timeout):
        raise ValueError("Invalid timeout: {0}".format(timeout))


class MobileBase(robot.Item):
    """Abstract interface to control a mobile base.

    Example:

        .. sourcecode:: python

           with hsrb_interface.Robot() as robot:
               omni_base = robot.get('omni_base')
               omni_base.go(1, 2, math.pi / 2.0)
               print(omni_base.pose)
    """

    def __init__(self, name):
        """Initialize an instance with a resource which has given `name`.

        Args:
            name (str): A name of a target resource.
        """
        super(MobileBase, self).__init__()
        self._setting = settings.get_entry('mobile_base', name)

        self._tf2_buffer = robot._get_tf2_buffer()
        navigation_action_name = self._setting['navigation_action']

        self._joint_state_sub = utils.CachingSubscriber(
            self._setting["joint_states_topic"],
            JointState,
            default=JointState())
        self._joint_state_sub.wait_for_message(3.0)

        self._action_client = ActionClient(
            self._node, NavigateToPose, navigation_action_name)
        self._action_client.wait_for_server(_ACTION_WAIT_TIMEOUT)
        self._follow_client = trajectory.TrajectoryController(
            self._setting['follow_trajectory_action'], 'base_coordinates')
        self._current_client = None

        self.nav2_goal_canceller = nav2_goal_canceller.Nav2GoalCanceller(self._node)

    def go_pose(self, pose=geometry.pose(), timeout=0.0, ref_frame_id=None, sync=True) -> bool:
        """Move to a specified pose.

        Args:
            pose (Tuple[Vector3, Quaternion]):
                A pose from a ``ref_frame_id`` coordinate
            timeout (float): Timeout to movement [sec].
                If not specified, deafult is 0 and wait forever.
            ref_frame_id (str):
                A reference frame of a goal. Default is ``map`` frame.
            sync (bool):
                Not wait the result when this arg is ``False``

        Returns:
            bool: True if success

        Examples:

            .. sourcecode:: python

               with hsrb_interface.Robot() as robot:
                   base = robot.try_get('omni_base')
                   pose = (Vector3(0.1, 0.2, 0.0), Quaternion())
                   base.go_pose(pose, 10.0, 'base_footprint')
        """
        _validate_timeout(timeout)
        goal = self.create_go_pose_goal(pose, ref_frame_id)
        return self._send_goal_pose_and_wait(goal, timeout, sync)

    def go_rel(self, x=0.0, y=0.0, yaw=0.0, timeout=0.0, sync=True) -> bool:
        """Move base from current position.

        Args:
            x   (float): X-axis position on ``robot`` frame [m]
            y   (float): Y-axis position on ``robot`` frame [m]
            yaw (float): Yaw position on ``robot`` frame [rad]
            timeout (float): Timeout until movement finish [sec].
                Default is 0.0 and wait forever.
            sync (bool):
                Not wait the result when this arg is ``False``

        Returns:
            bool: True if success

        Examples:

            .. sourcecode:: python

               with hsrb_interface.Robot() as robot:
                   base = robot.try_get('omni_base')
                   base.go_rel(1.0, 0.0, 0.0)
        """
        _validate_timeout(timeout)
        pose = geometry.pose(x, y, 0.0, 0.0, 0.0, yaw)
        ref_frame_id = settings.get_frame('base')
        goal = self.create_go_pose_goal(pose, ref_frame_id)
        return self._send_goal_pose_and_wait(goal, timeout, sync)

    def go_abs(self, x=0.0, y=0.0, yaw=0.0, timeout=0.0, sync=True) -> bool:
        """Move to a specified pose on map.

        Args:
            x   (float): X-axis position on ``map`` frame [m]
            y   (float): Y-axis position on ``map`` frame [m]
            yaw (float): Yaw position on ``map`` frame [rad]
            timeout (float): Timeout until movement finish [sec].
                Default is 0.0 and wait forever.
            sync (bool):
                Not wait the result when this arg is ``False``

        Returns:
            bool: True if success

        Examples:

            .. sourcecode:: python

               with hsrb_interface.Robot() as robot:
                   base = robot.try_get('omni_base')
                   base.go_abs(1.0, 0.0, 0.0)
        """
        _validate_timeout(timeout)
        pose = geometry.pose(float(x), float(y), 0.0, 0.0, 0.0, float(yaw))
        ref_frame_id = settings.get_frame('map')
        goal = self.create_go_pose_goal(pose, ref_frame_id)
        return self._send_goal_pose_and_wait(goal, timeout, sync)

    def move(self, pose: Tuple[Vector3, Quaternion], timeout=0.0, ref_frame_id=None, sync=True) -> bool:
        """Move to a specified pose.

        Args:
            pose (Tuple[Vector3, Quaternion]):
                A pose from a ``ref_frame_id`` coordinate
            timeout (float): Timeout to movement [sec].
                If not specified, deafult is 0 and wait forever.
            ref_frame_id (str):
                A reference frame of a goal. Default is ``map`` frame.
            sync (bool):
                Not wait the result when this arg is ``False``

        Returns:
            bool: True if success

        Warning:
            This function is deprecated. Use :py:func:`go_pose()` instead.
        """
        msg = ' '.join(["MobileBase.move() is depreacated."
                        "Use MobileBase.go_pose() instead."])
        warnings.warn(msg, exceptions.DeprecationWarning)
        return self.go_pose(pose, timeout, ref_frame_id, sync)

    def go(self, x: float, y: float, yaw: float, timeout=0.0, relative=False, sync=True) -> bool:
        """Move base to a specified pose.

        Args:
            x   (float): X-axis position on ``map`` frame [m]
            y   (float): Y-axis position on ``map`` frame [m]
            yaw (float): Yaw position on ``map`` frame [rad]
            timeout (float): Timeout until movement finish [sec].
                Default is 0.0 and wait forever.
            relative (bool): If ``True``, a robot move on robot frame.
                Otherwise a robot move on ``map`` frame.
            sync (bool):
                Not wait the result when this arg is ``False``

        Returns:
            bool: True if success

        Warning:
            This function is deprecated.
            Use :py:func:`go_rel()` or :py:func:`go_abs()` instead.
        """
        msg = ' '.join(["MobileBase.go() is depreacated."
                        "Use MobileBase.go_rel() or"
                        "MobileBase.go_abs() instead."])
        warnings.warn(msg, exceptions.DeprecationWarning)
        if relative:
            return self.go_rel(x, y, yaw, timeout, sync)
        else:
            return self.go_abs(x, y, yaw, timeout, sync)

    def _send_goal_pose_and_wait(self, goal, timeout=0.0, sync=True) -> bool:

        self.execute(goal)
        if not sync:
            return True

        status_strings = {
            action_msgs.GoalStatus.STATUS_UNKNOWN: "STATUS_UNKNOWN",  # noqa
            action_msgs.GoalStatus.STATUS_ACCEPTED: "STATUS_ACCEPTED",  # noqa
            action_msgs.GoalStatus.STATUS_EXECUTING: "STATUS_EXECUTING",  # noqa
            action_msgs.GoalStatus.STATUS_CANCELING: "STATUS_CANCELING",  # noqa
            action_msgs.GoalStatus.STATUS_SUCCEEDED: "STATUS_SUCCEEDED",  # noqa
            action_msgs.GoalStatus.STATUS_CANCELED: "STATUS_CANCELED",  # noqa
            action_msgs.GoalStatus.STATUS_ABORTED: "STATUS_ABORTED"  # noqa
        }

        start_time = self._node.get_clock().now()

        try:
            while True:
                # goal handle
                rclpy.spin_until_future_complete(
                    self._node, self._send_goal_future, timeout_sec=0.1
                )

                if self._send_goal_future.result() is not None:
                    state = self.get_state()  # SUCCEEDED / EXECUTING / CANCELED / ABORTED...
                    if state == action_msgs.GoalStatus.STATUS_SUCCEEDED:
                        return self.is_succeeded()
                    elif state == action_msgs.GoalStatus.STATUS_EXECUTING:
                        # continue to wait for the goal to complete
                        pass
                    else:
                        msg = f"Failed to reach goal ({status_strings.get(state, str(state))})"
                        self._node.get_logger().error(msg)
                        return False

                # check timeout, if 0.0 then
                if timeout > 0.0:
                    elapsed = self._node.get_clock().now() - start_time
                    if elapsed.nanoseconds >= int(timeout * 1e9):
                        self._node.get_logger().error("Timed out")
                        return False

        except KeyboardInterrupt:
            self.cancel_goal()
            return False

    def follow_trajectory(
        self,
        poses: List[Tuple[Vector3, Quaternion]],
        time_from_starts=[],
        ref_frame_id=None,
        sync=True,
    ) -> bool:
        """Follow given poses and timing with ignoring the map.

        Args:
            poses (List[Tuple[Vector3, Quaternion]]):
                Target poses of the robot base.
            time_from_starts (List[float]):
                Times of each "poses" [sec]. If empty, the times are optimized
                by time-optimal trajectory filter.
            ref_frame_id (str):
                A reference frame of a goal. Default is ``map`` frame.
            sync (bool):
                Not wait the result when this arg is ``False``

        Returns:
            bool: True if success

        Examples:

            .. sourcecode:: python

               with hsrb_interface.Robot() as robot:
                   omni_base = robot.try_get('omni_base')
                   poses = [geometry.pose(x=1.0, y=0.0, ek=0.0),
                            geometry.pose(x=1.0, y=1.0, ek=math.pi)]
                   omni_base.follow_trajectory(poses)
        """
        goal = self.create_follow_trajectory_goal(
            poses, time_from_starts, ref_frame_id)
        self.execute(goal)
        if sync:
            try:
                trajectory.wait_controllers(self, [self._follow_client])
            except exceptions.FollowTrajectoryError as e:
                self._node.get_logger().error(f"[{self.__class__.__name__}]: {e}")
                return False
        return True

    @property
    def pose(self) -> List[float]:
        """Estimated pose of a robot on ``map`` frame.

        Returns:
            List[float]: A pose value structured as (x[m], y[m], yaw[rad]).
        """
        pos, ori = self.get_pose()
        quat = [ori.x, ori.y, ori.z, ori.w]
        euler_angles = tf_transformations.euler_from_quaternion(quat)
        yaw = euler_angles[2]
        return [pos.x, pos.y, yaw]

    def get_pose(self, ref_frame_id=None) -> Tuple[Vector3, Quaternion]:
        """Get estimated pose of a robot on ``ref_frame_id`` frame.

        Args:
             ref_frame_id (str):
                 A reference frame of estimated pose. (Default ``map`` frame)

        Returns:
             Tuple[Vector3, Quaternion]:
                 A pose of ``base_footprint`` frame from ``ref_frame_id``.
        """
        if ref_frame_id is None:
            ref_frame_id = settings.get_frame('map')

        trans = utils.get_transform(
            self._node,
            self._tf2_buffer,
            ref_frame_id,
            settings.get_frame('base'),
            _TF_TIMEOUT
        )

        return geometry.transform_to_tuples(trans.transform)

    def create_go_pose_goal(self, pose: Tuple[Vector3, Quaternion], ref_frame_id=None) -> PoseStamped:
        """Create goal pose to move to a specified pose

        Args:
            pose (Tuple[Vector3, Quaternion]):
                A pose from a ``ref_frame_id`` coordinate
            ref_frame_id (str):
                A reference frame of a goal. Default is ``map`` frame.

        Returns:
            geometry_msgs.msg.PoseStamped: A goal pose
        """
        if ref_frame_id is None:
            ref_frame_id = settings.get_frame('map')

        target_pose = PoseStamped()
        target_pose.header.frame_id = ref_frame_id
        target_pose.header.stamp = self._node.get_clock().now().to_msg()
        target_pose.pose = geometry.tuples_to_pose(pose)
        return target_pose

    def create_follow_trajectory_goal(self, poses: List[Tuple[Vector3, Quaternion]],
                                      time_from_starts=[], ref_frame_id=None) -> JointTrajectory:
        """Create trajectory to follow given poses and timing.

        Args:
            poses (List[Tuple[Vector3, Quaternion]]):
                Target poses of the robot base.
            time_from_starts (List[float]):
                Times of each "poses" [sec]. If empty, the times are optimized
                by time-optimal trajectory filter.
            ref_frame_id (str):
                A reference frame of a goal. Default is ``map`` frame.

        Returns:
            trajectory_msgs.msg.JointTrajectory: A base trajectory
        """
        num_poses = len(poses)
        num_times = len(time_from_starts)
        if (num_times != 0) and (num_poses != num_times):
            raise ValueError("The size of time_from_starts should be zero"
                             " or same as the size of poses")
        if ref_frame_id is None:
            ref_frame_id = settings.get_frame('map')

        ref_to_current = self.get_pose(ref_frame_id)
        input_trajectory = MultiDOFJointTrajectory()
        input_trajectory.header.frame_id = ref_frame_id
        input_trajectory.points = list(
            utils.iterate(MultiDOFJointTrajectoryPoint, num_poses + 1))
        input_trajectory.points[0].transforms.append(
            geometry.tuples_to_transform(ref_to_current))
        for index in range(num_poses):
            input_trajectory.points[index + 1].transforms.append(
                geometry.tuples_to_transform(poses[index]))

        transformed_trajectory = trajectory.transform_base_trajectory(
            input_trajectory, self._tf2_buffer, _TF_TIMEOUT,
            self._follow_client.joint_names, self._node)

        if num_times == 0:
            start_state = self._joint_state_sub.data
            start_state.name += self._follow_client.joint_names
            start_state.position += transformed_trajectory.points[0].positions
            base_trajectory = trajectory.hsr_timeopt_filter(transformed_trajectory, start_state, self._node)
        else:
            base_trajectory = transformed_trajectory
            for index in range(num_times):
                tfs = rclpy.duration.Duration(seconds=time_from_starts[index])
                base_trajectory.points[index + 1].time_from_start = tfs.to_msg()
        # Remove current point
        del base_trajectory.points[0]
        return base_trajectory

    def execute(self, goal: Union[PoseStamped, JointTrajectory]):
        """Send a goal and not wait the result.

        Args:
            goal (Union[geometry_msgs.msg.PoseStamped, trajectory_msgs.msg.JointTrajectory):
                A goal to move

        Examples:

            .. sourcecode:: python

               with hsrb_interface.Robot() as robot:
                   omni_base = robot.try_get('omni_base')
                   poses = [geometry.pose(x=1.0, y=0.0, ek=0.0),
                            geometry.pose(x=1.0, y=1.0, ek=math.pi)]
                   goal = omni_base.create_follow_trajectory_goal(poses)
                   omni_base.execute(goal)
                   while not rclpy.ok():
                       time.sleep(1.0)
                       if not omni_base.is_moving():
                           break
                   print("Result: " + str(omni_base.is_succeeded())
        """
        self.cancel_goal()
        if isinstance(goal, PoseStamped):
            action_goal = NavigateToPose.Goal()
            action_goal.pose = goal
            self._send_goal_future = self._action_client.send_goal_async(
                action_goal)
            self._current_client = self._action_client
        elif isinstance(goal, JointTrajectory):
            self._follow_client.submit(goal)
            self._current_client = self._follow_client
        else:
            raise ValueError("Invalid goal.")

    def is_moving(self) -> bool:
        """Get the state as if the robot is moving.

        Returns:
            bool: True if the robot is moving
        """
        if self._current_client is None:
            return False
        else:
            if self._current_client is self._action_client:
                state = self.get_state()
            elif self._current_client is self._follow_client:
                state = self._current_client.get_state()
                return state == action_msgs.GoalStatus.STATUS_EXECUTING

    def is_succeeded(self) -> bool:
        """Get the state as if the robot moving was succeeded.

        Returns:
            bool: True if success
        """
        if self._current_client is None:
            return False
        else:
            if self._current_client is self._action_client:
                state = self._send_goal_future.result().status
                return state == action_msgs.GoalStatus.STATUS_SUCCEEDED
            elif self._current_client is self._follow_client:
                state = self._current_client.get_state()
                return state == action_msgs.GoalStatus.STATUS_SUCCEEDED

    def get_state(self) -> int:
        """Get a status of the action client

        Returns:
            int: The status code of the action goal, corresponding to one of
                `action_msgs.msg.GoalStatus` constants (e.g. ``STATUS_SUCCEEDED``,
                ``STATUS_ABORTED``, ``STATUS_CANCELED``, ``STATUS_EXECUTING``, etc.).
        """
        goal_handle = self._send_goal_future.result()
        get_result_future = goal_handle.get_result_async()

        # NOTE (Fujino):バックグラウンドで spin を実行した際のエラー回避
        try:
            rclpy.spin_until_future_complete(
                self._node, get_result_future, timeout_sec=0.1)
        except (IndexError, rclpy._rclpy_pybind11.RCLError) as e:
            self._node.get_logger().warn(
                f"Ignored internal ROS2 error in get_state(): {type(e).__name__}: {e}")
            return action_msgs.GoalStatus.STATUS_EXECUTING
        except Exception as e:
            self._node.get_logger().error(
                f"Unexpected error in get_state(): {type(e).__name__}: {e}")
            return action_msgs.GoalStatus.STATUS_EXECUTING

        res = get_result_future.result()
        if res is None:
            return action_msgs.GoalStatus.STATUS_EXECUTING
        else:
            return res.status

        # NOTE (Fujino): 元コード
        # rclpy.spin_until_future_complete(
        #     self._node, get_result_future, timeout_sec=0.1)
        # res = get_result_future.result()
        # if res is None:
        #     return action_msgs.GoalStatus.STATUS_EXECUTING
        # else:
        #     return res.status

    def cancel_goal(self):
        """Cancel moving."""
        if not self.is_moving():
            return

        if self._current_client is self._action_client:
            goal_handle = self._send_goal_future.result()
            goal_handle.cancel_goal_async()
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self._node, result_future)
        elif self._current_client is self._follow_client:
            self._follow_client.cancel()
        self.nav2_goal_canceller.cancel_all()
