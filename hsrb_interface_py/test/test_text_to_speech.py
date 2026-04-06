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
"""Unittest for hsrb_interface.text_to_speech module"""
from unittest.mock import patch

from action_msgs.msg import GoalStatus
import hsrb_interface
import hsrb_interface.exceptions
import hsrb_interface.text_to_speech
from nose.tools import assert_false
from nose.tools import assert_true
from nose.tools import eq_
from nose.tools import raises
import rclpy
from tmc_voice_msgs.action import TalkRequest
from tmc_voice_msgs.msg import Voice


@patch('hsrb_interface.Robot._connecting')
@patch('hsrb_interface.settings.get_entry')
@patch('hsrb_interface.text_to_speech.ActionClient')
@patch('rclpy.node.Node.create_publisher')
def test_text_to_speech(mock_pub_class, mock_action_client_cls, mock_get_entry, mock_connecting):
    """Test simple usage of TTS object."""
    rclpy.init()
    robot = hsrb_interface.Robot()  # noqa: F841
    mock_connecting.return_value = True

    mock_get_entry.return_value = {"topic": "foo"}
    tts = hsrb_interface.text_to_speech.TextToSpeech('default_tts')

    mock_get_entry.assert_called_with("text_to_speech", "default_tts")
    mock_pub_class.assert_called_with(tts._node, Voice, "foo", 0)
    mock_action_client_cls.assert_called_with(tts._node, TalkRequest, "talk_request_action")
    mock_pub_instance = mock_pub_class.return_value

    eq_(tts.language, tts.JAPANESE)
    tts.language = tts.ENGLISH
    eq_(tts.language, tts.ENGLISH)

    expected_msg = Voice()
    expected_msg.interrupting = False
    expected_msg.queueing = False
    expected_msg.language = tts.language
    expected_msg.sentence = "Hello, World!"
    assert_true(tts.say(u"Hello, World!", sync=False))
    mock_pub_instance.publish.assert_called_with(expected_msg)


@raises(hsrb_interface.exceptions.InvalidLanguageError)
@patch('hsrb_interface.Robot._connecting')
@patch('hsrb_interface.settings.get_entry')
@patch('hsrb_interface.text_to_speech.ActionClient')
@patch('rclpy.node.Node.create_publisher')
def test_invalid_language_error(mock_pub_class, mock_action_client_cls, mock_get_entry,
                                mock_connecting):
    """TTS object should refuse invalid language."""
    robot = hsrb_interface.Robot()  # noqa: F841
    mock_connecting.return_value = True

    mock_get_entry.return_value = {"topic": "foo"}
    tts = hsrb_interface.text_to_speech.TextToSpeech('default_tts')

    tts.language = -1


@patch('rclpy.spin_until_future_complete')
@patch('hsrb_interface.Robot._connecting')
@patch('hsrb_interface.settings.get_entry')
@patch('hsrb_interface.text_to_speech.ActionClient')
@patch('rclpy.node.Node.create_publisher')
def test_text_to_speech_sync_success(mock_pub_class, mock_action_client_cls, mock_get_entry,
                                     mock_connecting, mock_spin):
    """sync=True should return True only after a successful action result."""
    robot = hsrb_interface.Robot()  # noqa: F841
    mock_connecting.return_value = True

    mock_get_entry.return_value = {"topic": "foo"}
    mock_action_client = mock_action_client_cls.return_value
    mock_action_client.wait_for_server.return_value = True

    goal_future = mock_action_client.send_goal_async.return_value
    goal_handle = goal_future.result.return_value
    goal_handle.accepted = True
    result_future = goal_handle.get_result_async.return_value
    result = result_future.result.return_value
    result.status = GoalStatus.STATUS_SUCCEEDED

    tts = hsrb_interface.text_to_speech.TextToSpeech('default_tts')

    assert_true(tts.say(u"Hello, World!", sync=True))
    mock_action_client.wait_for_server.assert_called_with(timeout_sec=5.0)
    goal_handle.get_result_async.assert_called_once_with()


@patch('rclpy.spin_until_future_complete')
@patch('hsrb_interface.Robot._connecting')
@patch('hsrb_interface.settings.get_entry')
@patch('hsrb_interface.text_to_speech.ActionClient')
@patch('rclpy.node.Node.create_publisher')
def test_text_to_speech_sync_failure(mock_pub_class, mock_action_client_cls, mock_get_entry,
                                     mock_connecting, mock_spin):
    """sync=True should return False when the action does not succeed."""
    robot = hsrb_interface.Robot()  # noqa: F841
    mock_connecting.return_value = True

    mock_get_entry.return_value = {"topic": "foo"}
    mock_action_client = mock_action_client_cls.return_value
    mock_action_client.wait_for_server.return_value = True

    goal_future = mock_action_client.send_goal_async.return_value
    goal_handle = goal_future.result.return_value
    goal_handle.accepted = True
    result_future = goal_handle.get_result_async.return_value
    result = result_future.result.return_value
    result.status = GoalStatus.STATUS_ABORTED

    tts = hsrb_interface.text_to_speech.TextToSpeech('default_tts')

    assert_false(tts.say(u"Hello, World!", sync=True))
