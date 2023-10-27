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
import json
import rospy
import rospkg
import rosgraph

from copy import deepcopy
from daa_knowledge_base_ros.srv import Query, QueryRequest
from symbolic_fact_generation.common.collision_checking import oriented_collision_check_with_obj_size
from symbolic_fact_generation.common.fact import Fact
from symbolic_fact_generation.generator_interface import GeneratorInterface
from symbolic_fact_generation.common.lib import split_object_class_from_id

from rospy_message_converter import message_converter


class OnGenerator(GeneratorInterface):

    def __init__(self, fact_name: str = 'on', objects_of_interest: List[str] = [], container_objects: List[str] = [],
                 query_service_name: str = '/mobipick/daa_knowledge_base/query',
                 planning_scene_param: str = '/mobipick/pick_object_node/planning_scene_boxes') -> None:
        try:
            if not rosgraph.is_master_online():
                print("Waiting for ROS master node to go online ...")
                while not rosgraph.is_master_online():
                    time.sleep(1.0)

            rospy.wait_for_service(query_service_name, timeout=10.0)
            self._kb_query_service_proxy = rospy.ServiceProxy(query_service_name, Query)

            self._objects_of_interest = []
            for object_of_interest in objects_of_interest:
                obj_of_interest_class, _ = split_object_class_from_id(object_of_interest)
                if obj_of_interest_class not in self._objects_of_interest:
                    self._objects_of_interest.append(obj_of_interest_class)

            self._container_objects = container_objects
            self._fact_name = fact_name

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
            print("[WARNING] No planning scene parameter is set and table_poses.yaml file is not found!"
                  "Only objects on other objects facts can be generated!")
        except rospy.ROSInitException:
            print("ROS master was shutdown!")
            sys.exit(1)
        except rospy.ROSException:
            print(f"Timeout while waiting for anchoring service: {query_service_name}!")
            sys.exit(1)

    def generate_facts(self):
        # query anchoring for all object classes
        class Object:
            def __init__(self, symbol, **attr):
                self.name = symbol
                self.__dict__.update(attr)
        object_list: List[Object] = []
        try:
            query_result = self._kb_query_service_proxy(
                header=QueryRequest.GET_ALL_INSTANCES,
                data=None
                )
            query_result = json.loads(query_result.data)
            for symbol, percepts in query_result.items():
                object_list.append(
                    Object(
                        symbol=symbol,
                        **percepts
                    )
                )
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return []

        try:
            on_query = "is_on_top_of(Object,Surface)"
            # is_a_query = "is_a(Object, 'onto:Container')"
            query_result = self._kb_query_service_proxy(
                header=QueryRequest.PROLOG_QUERY,
                data=on_query
                )
            query_result = json.loads(query_result.data)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return []
        on_facts = []
        for r in query_result:
            new_fact = Fact(
                name="on",
                values=[r['Object'], r['Surface']])
            on_facts.append(new_fact)

        return on_facts


def check_in_condition(obj, container_obj) -> bool:
    if oriented_collision_check_with_obj_size(container_obj.pose, container_obj.size, obj.pose, obj.size):
        # calculate euclidean distance to check if obj is in container_obj
        dist = numpy.linalg.norm((obj.pose.position.x - container_obj.pose.position.x,
                                  obj.pose.position.y - container_obj.pose.position.y,
                                  obj.pose.position.z - container_obj.pose.position.z))
        radius = max(container_obj.max.x, container_obj.max.y, container_obj.max.z,
                     container_obj.size.x / 2.0, container_obj.size.y / 2.0, container_obj.size.z / 2.0)
        # remove 10% of radius for objects colliding with the outside wall
        # still detected as IN for rectangular container objects like klt if close to it
        radius = radius - radius * 0.1
        if dist < radius:
            return True
    return False


def check_on_condition(obj, surface_obj, z_threshold=0.1) -> bool:
    """
    Checks whether `obj` intersects with a box on top of `surface_obj`. That
    collision box has the same x and y dimensions as `surface_obj` and a z
    dimension of `z_threshold`.

    :param z_threshold: distance in meters above the table to count as "on" the table
    """
    # Assumption: surface_obj.pose.orientation is "upright"
    surface_pose = deepcopy(surface_obj.pose)
    surface_size = deepcopy(surface_obj.size)

    surface_pose.position.z = surface_pose.position.z + surface_size.z / 2 + z_threshold / 2
    surface_size.z = z_threshold

    return oriented_collision_check_with_obj_size(surface_pose, surface_size, obj.pose, obj.size)
