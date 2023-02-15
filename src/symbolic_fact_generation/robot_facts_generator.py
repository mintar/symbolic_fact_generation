#!/usr/bin/python3

# BSD 3-Clause License

# Copyright (c) 2022, DFKI Niedersachsen
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from typing import List

import rospy
import yaml
import tf
import math
import numpy
from sensor_msgs.msg import JointState

from symbolic_fact_generation.generator_interface import GeneratorInterface
from symbolic_fact_generation.common.fact import Fact


class HasArmPostureGenerator(GeneratorInterface):

    def __init__(self, fact_name: str = 'robot_has_arm_posture',
                 joint_states_topic: str = '/mobipick/joint_states',
                 arm_posture_param: str = 'arm_postures_yaml',
                 arm_tolerance: float = 0.01):
        rospy.Subscriber(joint_states_topic, JointState, self.jointStatesCB)

        self._joint_states_msg = None
        self._joint_states_msg_received = False
        self._fact_name = fact_name
        self._current_arm_posture = 'undefined'
        self._arm_tolerance = arm_tolerance

        try:
            with open(rospy.get_param(arm_posture_param), 'r') as arm_postures:
                self._arm_posture_yaml = yaml.safe_load(arm_postures)
        except FileNotFoundError as e:
            self._arm_posture_yaml = None
            print(f"Arm posture config file not found: {e}")

    def generate_facts(self) -> List[Fact]:
        arm_posture_facts = []

        if self._joint_states_msg_received and self._arm_posture_yaml is not None:
            self._joint_states_msg_received = False
            arm_posture = 'undefined'

            for posture in self._arm_posture_yaml:
                match = True
                for i, joint_position in enumerate(self._joint_states_msg.position):
                    if self._joint_states_msg.name[i] in list(self._arm_posture_yaml[posture]):
                        if abs(joint_position - self._arm_posture_yaml[posture][self._joint_states_msg.name[i]]) < self._arm_tolerance:
                            continue
                        else:
                            match = False
                            break
                if match:
                    arm_posture = posture
                    break

            arm_posture_facts.append(Fact(name=self._fact_name, values=[arm_posture]))
            self._current_arm_posture = arm_posture
        else:
            arm_posture_facts.append(Fact(name=self._fact_name, values=[self.current_arm_posture]))

        return arm_posture_facts

    def jointStatesCB(self, msg):
        self._joint_states_msg = msg
        self._joint_states_msg_received = True


class RobotAtGenerator(GeneratorInterface):

    def __init__(self, fact_name: str = 'robot_at',
                 global_frame: str = '/map',
                 robot_frame: str = '/mobipick/base_link',
                 waypoint_param: str = 'waypoints_yaml',
                 at_threshold: float = 0.2):
        self._robot_at = 'undefined'

        self._fact_name = fact_name
        self._global_frame = global_frame
        self._robot_frame = robot_frame
        self._at_threshold = at_threshold

        self._tf_listener = tf.TransformListener()

        try:
            with open(rospy.get_param(waypoint_param), 'r') as waypoints:
                self._waypoints = yaml.safe_load(waypoints)
        except FileNotFoundError as e:
            self._waypoints = None
            print(f"Robot waypoints config file not found: {e}")

    def generate_facts(self) -> List[Fact]:
        robot_at_facts = []
        if self._waypoints is not None:
            robot_at = "undefined"

            try:
                now = rospy.Time.now()
                self._tf_listener.waitForTransform(self._global_frame, self._robot_frame, now, rospy.Duration(4.0))
                trans, rot = self._tf_listener.lookupTransform(self._global_frame, self._robot_frame, now)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn('Failed to get robot pose')
                robot_at_facts.append(Fact(name=self._fact_name, values=[self._robot_at]))
                return robot_at_facts

            for waypoint in self._waypoints:
                if self.distance_to_waypoint(waypoint, trans) < self._at_threshold:
                    if self.has_correct_heading(waypoint, rot):
                        robot_at = waypoint
            robot_at_facts.append(Fact(name=self._fact_name, values=[robot_at]))
            self._robot_at = robot_at
        else:
            robot_at_facts.append(Fact(name=self._fact_name, values=[self.robot_at]))

        return robot_at_facts

    def distance_to_waypoint(self, waypoint, robot_position):

        x_1 = self._waypoints[waypoint][0][0]
        y_1 = self._waypoints[waypoint][0][1]
        x_2 = robot_position[0]
        y_2 = robot_position[1]

        d = math.sqrt((x_1 - x_2)**2 + (y_1 - y_2)**2)
        return d

    def has_correct_heading(self, waypoint, robot_rotation):
        # calculate difference between two rotations as quaternion
        d = numpy.dot(self._waypoints[waypoint][1], robot_rotation)

        return math.acos(min(abs(d), 1.0)) < self._at_threshold
