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
import yaml
import time
import numpy
import sys

import rospy
import rospkg
import rosgraph

from pose_selector.srv import ClassQuery, ClassQueryRequest
from symbolic_fact_generation.common.collision_checking import oriented_collision_check_with_obj_size
from symbolic_fact_generation.common.fact import Fact
from symbolic_fact_generation.generator_interface import GeneratorInterface
from symbolic_fact_generation.common.lib import split_object_class_from_id

from rospy_message_converter import message_converter


class OnGenerator(GeneratorInterface):

    def __init__(self, objects_of_interest: List[str] = [], container_objects: List[str] = [],
                 query_srv_str: str = "/pick_pose_selector_node/pose_selector_class_query",
                 planning_scene_param: str = "/mobipick/pick_object_node/planning_scene_boxes") -> None:
        try:
            if not rosgraph.is_master_online():
                print("Waiting for ROS master node to go online ...")
                while not rosgraph.is_master_online():
                    time.sleep(1.0)

            rospy.wait_for_service(query_srv_str, timeout=10.0)
            self._pose_selector_query_srv = rospy.ServiceProxy(query_srv_str, ClassQuery)

            self._objects_of_interest = objects_of_interest
            self._container_objects = container_objects

            package_path = rospkg.RosPack().get_path('symbolic_fact_generation')

            self._planning_scene_object_poses = []

            # if planning scene config file on parameter server
            if rospy.has_param(planning_scene_param):
                print(f"Using {planning_scene_param} parameter.")
                planning_scene_boxes = rospy.get_param(planning_scene_param)
                id_counter = {}
                for box in planning_scene_boxes:
                    class_id, instance_id = split_object_class_from_id(box['scene_name'])
                    if instance_id is None:
                        # create id for different class types starting from 1
                        id_counter[class_id] = id_counter.get(class_id, 1)
                        instance_id = id_counter[class_id]
                        id_counter[class_id] += 1
                    pose = {'class_id': class_id,
                            'instance_id': instance_id,
                            'pose':
                                {'position':
                                    {
                                        'x': box['box_position_x'],
                                        'y': box['box_position_y'],
                                        'z': box['box_position_z']
                                    },
                                 'orientation':
                                    {
                                        'x': box['box_orientation_x'],
                                        'y': box['box_orientation_y'],
                                        'z': box['box_orientation_z'],
                                        'w': box['box_orientation_w']
                                    }
                                 },
                            'size':
                                {
                                    'x': box['box_x_dimension'],
                                    'y': box['box_y_dimension'],
                                    'z': box['box_z_dimension']
                                },
                            'min':
                                {
                                    'x': -(box['box_x_dimension'] / 2.0),
                                    'y': -(box['box_y_dimension'] / 2.0),
                                    'z': -(box['box_z_dimension'] / 2.0)
                                },
                            'max':
                                {
                                    'x': (box['box_x_dimension'] / 2.0),
                                    'y': (box['box_y_dimension'] / 2.0),
                                    'z': (box['box_z_dimension'] / 2.0)
                                }
                            }
                    self._planning_scene_object_poses.append(
                        message_converter.convert_dictionary_to_ros_message('object_pose_msgs/ObjectPose', pose))
            else:
                # use default config file
                print("Using symbolic_fact_generation/config/tables_poses.yaml")
                table_poses_yaml = package_path + "/config/table_poses.yaml"
                yamlfile = open(table_poses_yaml, 'r')
                yaml_content = yaml.load(yamlfile, Loader=yaml.FullLoader)

                for pose in yaml_content["poses"]:
                    self._planning_scene_object_poses.append(
                        message_converter.convert_dictionary_to_ros_message('object_pose_msgs/ObjectPose', pose))

        except FileNotFoundError:
            print("[WARNING] No planning scene parameter is set and table_poses.yaml file is not found! Only objects on other objects facts can be generated!")
        except rospy.ROSInitException:
            print("ROS master was shutdown!")
            sys.exit(1)
        except rospy.ROSException:
            print(f"Timeout while waiting for pose_selector service: {query_srv_str}!")
            sys.exit(1)

    def generate_facts(self):
        # query pose_selector for all object classes
        obj_poses = []
        for obj in self._objects_of_interest:
            query_result = self._pose_selector_query_srv(ClassQueryRequest(class_id=obj))
            obj_poses.extend(query_result.poses)

        on_facts = []

        # create new list with planning scene objects and objects
        surface_objects = [*self._planning_scene_object_poses, *obj_poses]

        # iterate over all poses
        for surface_obj_i in surface_objects:
            for obj_j in obj_poses:
                surface_obj_name_i = surface_obj_i.class_id + "_" + str(surface_obj_i.instance_id)
                obj_name_j = obj_j.class_id + "_" + str(obj_j.instance_id)
                new_fact = None
                # no need to check with itself
                if surface_obj_name_i != obj_name_j:
                    if oriented_collision_check_with_obj_size(surface_obj_i.pose, surface_obj_i.size, obj_j.pose, obj_j.size):
                        # handle special "in" container object case
                        if surface_obj_i.class_id in self._container_objects or obj_j.class_id in self._container_objects:
                            # calculate euclidean distance to klt to check if obj_j is in klt
                            dist = numpy.linalg.norm((obj_j.pose.position.x - surface_obj_i.pose.position.x,
                                                      obj_j.pose.position.y - surface_obj_i.pose.position.y,
                                                      obj_j.pose.position.z - surface_obj_i.pose.position.z))
                            radius = max(surface_obj_i.max.x, surface_obj_i.max.y, surface_obj_i.max.z) if surface_obj_i.class_id in self._container_objects else max(
                                obj_j.max.x, obj_j.max.y, obj_j.max.z)
                            if dist < radius:
                                if surface_obj_i.class_id in self._container_objects:
                                    new_fact = Fact(name="in", values=[obj_name_j, surface_obj_name_i])
                                else:
                                    new_fact = Fact(name="in", values=[surface_obj_name_i, obj_name_j])
                            else:
                                if obj_j.pose.position.z > surface_obj_i.pose.position.z:
                                    new_fact = Fact(name="on", values=[obj_name_j, surface_obj_name_i])
                        else:
                            if obj_j.pose.position.z > surface_obj_i.pose.position.z:
                                new_fact = Fact(name="on", values=[obj_name_j, surface_obj_name_i])

                # add new fact to list if not already there
                if new_fact is not None and new_fact not in on_facts:
                    on_facts.append(new_fact)

        return on_facts
