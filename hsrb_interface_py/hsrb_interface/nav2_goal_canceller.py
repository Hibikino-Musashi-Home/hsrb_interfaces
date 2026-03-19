#!/usr/bin/env python3
# -*-encoding:UTF-8-*-
#
# Copyright (c) 2025 Hibikino-Musashi@Home
#
# All rights reserved.
#
# This software and associated documentation files (the "Software") are provided to authorized users
# within Hibikino-Musashi@Home ("the Organization") for internal use only.
#
# Permission is granted to use, copy, and modify the Software solely for purposes directly related to
# the Organization’s internal operations.
#
# The following actions are strictly prohibited without prior written permission from the Organization:
#
# 1. Distributing, publishing, or otherwise making the Software available to any third party.
# 2. Using the Software for any commercial purpose outside the Organization.
# 3. Creating derivative works intended for public release.
#
# This Software is provided "AS IS", without warranty of any kind, express or implied, including but not
# limited to the warranties of merchantability, fitness for a particular purpose, and noninfringement.
#
# Violation of these terms may result in termination of access and legal action.
#
# Author: Ryohei Kobayashi (Hibikino-Musashi@Home)
# Maintainer: Tomoaki Fujino (Hibikino-Musashi@Home)
"""This module provides a helper class for cancelling Nav2 navigation goals."""

import rclpy
from rclpy.node import Node
from rclpy.action.client import ClientGoalHandle
from action_msgs.srv import CancelGoal as CancelGoalSrv
from action_msgs.msg import GoalStatus


class Nav2GoalCanceller:
    """A helper class to cancel goals in Nav2 (/navigate_to_pose).

    This class provides methods to cancel either a specific goal using its
    goal handle, or all active goals using the Nav2 CancelGoal service.
    """

    def __init__(self, node: Node) -> None:
        """
        Initialize the Nav2GoalCanceller.

        Args:
            node (Node): The ROS 2 node used to create the cancel goal client.
        """
        self._node = node
        self._cancel_cli = node.create_client(
            CancelGoalSrv, "/navigate_to_pose/_action/cancel_goal"
        )

    def cancel_by_handle(
        self,
        goal_handle: ClientGoalHandle,
        wait_result=True,
        timeout_sec=5.0,
    ) -> bool:
        """
        Cancel a goal using its goal handle.

        This method sends a cancel request for the given goal handle and optionally waits for
        confirmation that the goal was successfully canceled.

        Args:
            goal_handle (ClientGoalHandle): The goal handle obtained when sending the goal.
            wait_result (bool): Whether to wait for the cancel result. Default is True.
            timeout_sec (float): Maximum time (in seconds) to wait for the cancel result. Default is 5.0.

        Returns:
            bool: True if the goal was successfully canceled, False otherwise.
        """
        cf = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self._node, cf)
        resp = cf.result()
        if resp is None or resp.return_code != CancelGoalSrv.Response.ERROR_NONE:
            return False
        if wait_result:
            rf = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self._node, rf, timeout_sec=timeout_sec)
            if not rf.done():
                return False
            res = rf.result()
            return res is not None and res.status == GoalStatus.STATUS_CANCELED
        return True

    def cancel_all(self, wait_service_sec=5.0) -> bool:
        """
        Cancel all active goals via the CancelGoal service.

        This method directly calls the `/navigate_to_pose/_action/cancel_goal` service
        to request cancellation of all currently active goals. Useful when a specific
        goal handle is not available.

        Args:
            wait_service_sec (float): Time (in seconds) to wait for the cancel service to become available.
                Default is 5.0.

        Returns:
            bool: True if the cancel request was sent successfully, False otherwise.
        """
        if not self._cancel_cli.wait_for_service(timeout_sec=wait_service_sec):
            self._node.get_logger().error("CancelGoal service not available")
            return False
        req = CancelGoalSrv.Request()
        future = self._cancel_cli.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        res: CancelGoalSrv.Response = future.result()
        self._node.get_logger().error('nav2_goal_canceller. -> cancel all')
        return bool(res and res.goals_canceling)

