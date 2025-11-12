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
"""Text-to-speech interface"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import rclpy
from rclpy.action import ActionClient

from tmc_voice_msgs.msg import Voice
from tmc_voice_msgs.action import TalkRequest

from . import exceptions
from . import robot
from . import settings


class TextToSpeech(robot.Item):
    """Abstract interface for text-to-speech service

    Examples:

        .. sourcecode:: python

            with Robot() as robot:
                tts = robot.get('default', Items.TEXT_TO_SPEECH)
                tts.language = tts.JAPANESE
                tts.say(u"Hello, World!")
    """

    JAPANESE = Voice.JAPANESE
    ENGLISH = Voice.ENGLISH

    def __init__(self, name):
        """Initialize an instance

        Args:
            name (str): A resource name
        """
        super(TextToSpeech, self).__init__()
        self._setting = settings.get_entry('text_to_speech', name)
        topic = self._setting['topic']
        self._pub = self._node.create_publisher(Voice, topic, 0)
        self._language = TextToSpeech.JAPANESE
        self._ac_talk_request = ActionClient(self._node, TalkRequest, '/talk_request_action')

    @property
    def language(self) -> int:
        """(int): Language of speech"""
        return self._language

    @language.setter
    def language(self, value):
        if value not in (Voice.JAPANESE, Voice.ENGLISH):
            msg = 'Language code {0} is not supported'.format(value)
            raise exceptions.InvalidLanguageError(msg)
        self._language = value

    def say(self, text: str, queue=False, sync=True) -> bool:
        """Speak a given text

        Args:
            text (str): A text to be converted to voice sound (UTF-8)
            queue (bool):
                If True, the speech request is queued instead of interrupting
                    the current one. Default is ``False``.
            sync (bool):
                If True, wait for the TalkRequest action server to become
                    available before sending the goal. Default is ``True``.

        Returns:
            bool: True if success
        """

        if sync is True:
            if not self._ac_talk_request.wait_for_server(timeout_sec=5.0):
                self._node.get_logger().error('TalkRequest action server not available.')
                return False

        goal_msg = TalkRequest.Goal()
        goal_msg.data.interrupting = False
        goal_msg.data.queueing = queue
        goal_msg.data.language = self.language
        goal_msg.data.sentence = text

        future = self._ac_talk_request.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self._node, future)

        msg = Voice()
        msg.interrupting = False
        msg.queueing = queue
        msg.language = self._language
        msg.sentence = text
        self._pub.publish(msg)

        return True
