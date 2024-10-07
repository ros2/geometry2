# Copyright 2024 Open Source Robotics Foundation, Inc.
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
#    * Neither the name of the Open Source Robotics Foundation, Inc. nor the names of its
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

from rclpy.time import Time
import PyKDL


class Header:
    def __init__(self, stamp_, frame_id_):
        self.stamp = stamp_
        self.frame_id = frame_id_


class StampedVector(PyKDL.Vector):
    def __init__(self, vector, timestamp: Time, frame_id: str):
        super().__init__(vector)
        self.vector = vector
        self.header = Header(timestamp, frame_id)

    def set_data(self, vector):
        self.vector = vector


class StampedFrame(PyKDL.Frame):
    def __init__(self, frame, timestamp: Time, frame_id: str):
        super().__init__(frame)
        self.frame = frame
        self.header = Header(timestamp, frame_id)

    def set_data(self, frame):
        self.frame = frame


class StampedTwist(PyKDL.Twist):
    def __init__(self, twist, timestamp: Time, frame_id: str):
        super().__init__(twist)
        self.twist = twist
        self.header = Header(timestamp, frame_id)

    def set_data(self, twist):
        self.twist = twist


class StampedWrench(PyKDL.Wrench):
    def __init__(self, wrench, timestamp: Time, frame_id: str):
        super().__init__(wrench)
        self.wrench = wrench
        self.header = Header(timestamp, frame_id)

    def set_data(self, wrench):
        self.wrench = wrench
