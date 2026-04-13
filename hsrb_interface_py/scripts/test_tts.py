#!/usr/bin/env python3
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

import argparse

import rclpy

from hsrb_interface import Robot
from rclpy.signals import SignalHandlerOptions

import time


def _build_parser():
    parser = argparse.ArgumentParser(description="Simple test script for hsrb_interface tts.say().")
    parser.add_argument(
        "text",
        nargs="?",
        default="This is a test of hsrb_interface tts.say.",
        help="sentence to speak",
    )
    parser.add_argument(
        "--language",
        choices=("ja", "en"),
        default="ja",
        help="speech language",
    )
    parser.add_argument(
        "--queue",
        action="store_true",
        help="queue the request instead of replacing the current speech",
    )
    parser.add_argument(
        "--async",
        dest="sync",
        action="store_false",
        help="publish the request without waiting for playback to finish",
    )
    return parser


def main(args=None):
    parser = _build_parser()
    parsed = parser.parse_args(args=args)

    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    try:
        with Robot() as hsrb:
            tts = hsrb.get("default_tts")

            tts.language = tts.ENGLISH
            success = tts.say("test test test", queue=parsed.queue, sync=True)
            success = tts.say("this is sync=true", queue=parsed.queue, sync=True)

            success = tts.say("test test test", queue=parsed.queue, sync=False)
            success = tts.say("this is sync=false", queue=parsed.queue, sync=False)

            time.sleep(5.0)

            tts.language = tts.JAPANESE
            success = tts.say("これはテストです．", queue=parsed.queue, sync=True)
            success = tts.say("これは同期しています．", queue=parsed.queue, sync=True)

            success = tts.say("これはテストです．", queue=parsed.queue, sync=False)
            success = tts.say("これは同期していません．", queue=parsed.queue, sync=False)

            if not success:
                raise RuntimeError("tts.say failed")

            print("tts.say succeeded")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
