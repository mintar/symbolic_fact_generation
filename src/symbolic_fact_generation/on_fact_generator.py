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

import rospy
import rospkg
import rosgraph

from pose_selector.srv import ClassQuery, ClassQueryRequest, PoseDelete, PoseDeleteRequest
from symbolic_fact_generation.common.aabb import collision
from symbolic_fact_generation.common.Fact import Fact

from rospy_message_converter import message_converter

'''
Queries object poses from pose_selector, makes predicate on(obj, table) based on their poses.
'''

def _init_class(query_srv_str: str = "/pick_pose_selector_node/pose_selector_class_query",
                delete_srv_str: str = "/pick_pose_selector_node/pose_selector_delete") -> None:
    """Initialize class variables. """

    if not OnFactGenerator._pose_selector_query_srv:
        try:
            # initialize class
            if not rosgraph.is_master_online():
                print("Waiting for ROS master node to go online ...")
                while not rosgraph.is_master_online():
                    time.sleep(1.0)

            package_path = rospkg.RosPack().get_path('symbolic_fact_generation')

            # if tables_demo is running use config file parameter server
            if rospy.has_param("/mobipick/pick_object_node/planning_scene_boxes"):
                print("Using tables_demo_bringup/config/tables_planning_scene.yaml")
                planning_scene_boxes = rospy.get_param("/mobipick/pick_object_node/planning_scene_boxes")
                for table in planning_scene_boxes:
                    # only process tables
                    if table["scene_name"].startswith("table"):
                        class_id, instance_id = table["scene_name"].split("_")
                        pose = {'class_id': class_id,
                                'instance_id': int(instance_id),
                                'pose':
                                    {'position':
                                        {'x': table["box_position_x"],
                                        'y': table["box_position_y"],
                                        'z': table["box_position_z"]
                                        }
                                    }
                            }
                        OnFactGenerator._table_poses.append(message_converter.convert_dictionary_to_ros_message('object_pose_msgs/ObjectPose', pose))
            else:
                # use default config file
                print("Using symbolic_fact_generation/config/tables_poses.yaml")
                table_poses_yaml = package_path + "/config/table_poses.yaml"
                yamlfile = open(table_poses_yaml, 'r')
                yaml_content = yaml.load(yamlfile, Loader=yaml.FullLoader)

                for pose in yaml_content["poses"]:
                    OnFactGenerator._table_poses.append(message_converter.convert_dictionary_to_ros_message('object_pose_msgs/ObjectPose', pose))

            print("Waiting for pose_selector class query service!")
            rospy.wait_for_service(query_srv_str)
            print("pose_selector class query service found!")
            OnFactGenerator._pose_selector_query_srv = rospy.ServiceProxy(query_srv_str, ClassQuery)

            print("Waiting for pose_selector delete service!")
            rospy.wait_for_service(delete_srv_str)
            print("pose_selector delete service found!")
            OnFactGenerator._pose_selector_delete_srv = rospy.ServiceProxy(delete_srv_str, PoseDelete)

            OnFactGenerator._bb_yaml = package_path + "/config/boundingboxes.yaml"

            print("Initialized OnFactGenerator class!")
        except FileNotFoundError:
            print("[WARNING] table_positions.yaml not found!")
        except rospy.ROSInitException:
            print("ROS master was shutdown!")


class OnFactGenerator:

    _pose_selector_query_srv: rospy.ServiceProxy = None
    _pose_selector_delete_srv: rospy.ServiceProxy = None
    _bb_yaml: str = ""
    _table_poses: List = []
    _current_facts: List = []

    @classmethod
    def __make_predicate_on(cls, obj_poses: List) -> List:
        on_set = []

        # save poses of all klts
        klts = [obj for obj in obj_poses if obj.class_id == "klt"]

        for table_i in cls._table_poses:
            for obj_j in obj_poses:
                # Currenty assuming one object of each class (except tables, but no need to check them)
                if table_i.class_id != obj_j.class_id:
                    table_name_i = table_i.class_id + "_" + str(table_i.instance_id)
                    obj_name_j = obj_j.class_id + "_" + str(obj_j.instance_id)
                    if collision(table_name_i, table_i.pose, obj_name_j, obj_j.pose, cls._bb_yaml):
                        if obj_j.pose.position.z > table_i.pose.position.z:
                            if klts and obj_j not in klts:
                                # calculate euclidean distance to each klt to check if obj_j is in a klt
                                min_dist = 100
                                min_klt = None
                                for klt in klts:
                                    dist = numpy.linalg.norm((obj_j.pose.position.x - klt.pose.position.x, obj_j.pose.position.y - klt.pose.position.y, obj_j.pose.position.z - klt.pose.position.z))
                                    if min_dist > dist:
                                        min_dist = dist
                                        min_klt = klt
                                if min_dist >= 0.15:
                                    on_set.append(Fact(name="on", values=[obj_name_j, table_name_i]))
                                else:
                                    if min_klt:
                                        klt_name = min_klt.class_id + "_" + str(min_klt.instance_id)
                                        on_set.append(Fact(name="on", values=[obj_name_j, klt_name]))
                            else:
                                on_set.append(Fact(name="on", values=[obj_name_j, table_name_i]))

        return on_set

    @classmethod
    def process_observations(cls) -> None:
        # query pose_selector for all object classes
        obj_poses = []
        for obj in ["relay", "screwdriver", "multimeter", "klt", "power_drill", "power_drill_with_grip"]:
            query_result = cls._pose_selector_query_srv(ClassQueryRequest(class_id = obj))
            obj_poses.extend(query_result.poses)

        # iterate over all poses
        new_on = cls.__make_predicate_on(obj_poses)

        for on in list(cls._current_facts):
            if on in new_on:
                new_on.remove(on)
            else:
                cls._current_facts.remove(on)
        cls._current_facts.extend(new_on)


def clear_facts_and_poses_for_table(table_id: str) -> None:
    _init_class()

    objects_on_table = []

    for fact in list(OnFactGenerator._current_facts):
        if fact.values[1] == table_id:
            objects_on_table.append(fact.values[0])
            OnFactGenerator._current_facts.remove(fact)
            class_id, instance_id = fact.values[0].rsplit("_", 1)
            OnFactGenerator._pose_selector_delete_srv(PoseDeleteRequest(class_id = class_id, instance_id = int(instance_id)))

    while objects_on_table:
        for fact in list(OnFactGenerator._current_facts):
            if fact.values[1] == objects_on_table[0]:
                objects_on_table.append(fact.values[0])
                OnFactGenerator._current_facts.remove(fact)
                class_id, instance_id = fact.values[0].rsplit("_", 1)
                OnFactGenerator._pose_selector_delete_srv(PoseDeleteRequest(class_id = class_id, instance_id = int(instance_id)))
        objects_on_table.pop(0)

def get_current_facts() -> List:
    _init_class()
    OnFactGenerator.process_observations()
    return OnFactGenerator._current_facts.copy()