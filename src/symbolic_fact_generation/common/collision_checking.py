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

from tf.transformations import quaternion_matrix
from symbolic_fact_generation.common.lib import read_yaml_file


class Point:

    def __init__(self, x=None, y=None, z=None) -> None:

        self.x = x
        self.y = y
        self.z = z

    def __sub__(self, p):
        return Point(self.x - p.x, self.y - p.y, self.z - p.z)

    def __mul__(self, p):
        return self.x * p.x + self.y * p.y + self.z * p.z

    def __pow__(self, p):
        return Point(self.y * p.z - self.z * p.y, self.z * p.x - self.x * p.z, self.x * p.y - self.y * p.x)

    def product(self, val):
        return Point(self.x * val, self.y * val, self.z * val)

    def __str__(self) -> str:
        return f"({self.x}, {self.y}, {self.z})"


class BoundingBox:

    def __init__(self, center: Point, radius: Point, quaternion) -> None:

        self.center = center
        self.radius = radius

        rotation_matrix = quaternion_matrix([quaternion.x, quaternion.y, quaternion.z, quaternion.w])

        self.rotation_axis_x = Point(*rotation_matrix[0][:-1])
        self.rotation_axis_y = Point(*rotation_matrix[1][:-1])
        self.rotation_axis_z = Point(*rotation_matrix[2][:-1])

    def __str__(self) -> str:
        return (f"(Center: {self.center}, "
                f"Radius: {self.radius}, "
                f"Orientation: [{self.rotation_axis_x}, {self.rotation_axis_y}, {self.rotation_axis_z}])")


# Implementation of axis-aligned bounding boxes collision check
def aabb(bb_a, bb_b):

    if abs(bb_a.c.x - bb_b.c.x) > abs(bb_a.r.x + bb_b.r.x):
        return False
    if abs(bb_a.c.y - bb_b.c.y) > abs(bb_a.r.y + bb_b.r.y):
        return False
    if abs(bb_a.c.z - bb_b.c.z) > abs(bb_a.r.z + bb_b.r.z):
        return False

    return True


def get_separating_plane(r_pos: Point, plane: Point, bb_1: BoundingBox, bb_2: BoundingBox) -> bool:
    return (abs(r_pos * plane) > (abs(bb_1.rotation_axis_x.product(bb_1.radius.x) * plane)
                                  + abs(bb_1.rotation_axis_y.product(bb_1.radius.y) * plane)
                                  + abs(bb_1.rotation_axis_z.product(bb_1.radius.z) * plane)
                                  + abs(bb_2.rotation_axis_x.product(bb_2.radius.x) * plane)
                                  + abs(bb_2.rotation_axis_y.product(bb_2.radius.y) * plane)
                                  + abs(bb_2.rotation_axis_z.product(bb_2.radius.z) * plane)
                                 )
           )


# Implementation of the separating axis theorem (SAT) collision check
def SAT(bb_1: BoundingBox, bb_2: BoundingBox) -> bool:
    r_pos = bb_2.center - bb_1.center

    return not (get_separating_plane(r_pos, bb_1.rotation_axis_x, bb_1, bb_2) or
                get_separating_plane(r_pos, bb_1.rotation_axis_y, bb_1, bb_2) or
                get_separating_plane(r_pos, bb_1.rotation_axis_z, bb_1, bb_2) or
                get_separating_plane(r_pos, bb_2.rotation_axis_x, bb_1, bb_2) or
                get_separating_plane(r_pos, bb_2.rotation_axis_y, bb_1, bb_2) or
                get_separating_plane(r_pos, bb_2.rotation_axis_z, bb_1, bb_2) or
                get_separating_plane(r_pos, bb_1.rotation_axis_x ** bb_2.rotation_axis_x, bb_1, bb_2) or
                get_separating_plane(r_pos, bb_1.rotation_axis_x ** bb_2.rotation_axis_y, bb_1, bb_2) or
                get_separating_plane(r_pos, bb_1.rotation_axis_x ** bb_2.rotation_axis_z, bb_1, bb_2) or
                get_separating_plane(r_pos, bb_1.rotation_axis_y ** bb_2.rotation_axis_x, bb_1, bb_2) or
                get_separating_plane(r_pos, bb_1.rotation_axis_y ** bb_2.rotation_axis_y, bb_1, bb_2) or
                get_separating_plane(r_pos, bb_1.rotation_axis_y ** bb_2.rotation_axis_z, bb_1, bb_2) or
                get_separating_plane(r_pos, bb_1.rotation_axis_z ** bb_2.rotation_axis_x, bb_1, bb_2) or
                get_separating_plane(r_pos, bb_1.rotation_axis_z ** bb_2.rotation_axis_y, bb_1, bb_2) or
                get_separating_plane(r_pos, bb_1.rotation_axis_z ** bb_2.rotation_axis_z, bb_1, bb_2))


def frame_is_centered(a):

    if abs(a["x_max"]) != abs(a["x_min"]):
        return False
    if abs(a["y_max"]) != abs(a["y_min"]):
        return False
    if abs(a["z_max"]) != abs(a["z_min"]):
        return False

    return True


def get_halfwidths(value):

    return Point(abs(value["x_max"]), abs(value["y_max"]), abs(value["z_max"]))


def get_center(pose):

    return Point(pose.position.x, pose.position.y, pose.position.z)


def get_default_bounding_box(pose):

    c = get_center(pose)
    r = Point(0.1, 0.1, 0.1)
    o = pose.orientation

    return BoundingBox(c, r, o)


def get_centered_center_and_halfwidth(value, pose):

    r = Point()
    c = Point()

    if abs(value["x_min"]) != abs(value["x_max"]):
        width = value["x_max"] - value["x_min"]
        c.x = pose.position.x + value["x_min"] + width / 2.0
        r.x = width / 2.0
    else:
        c.x = pose.position.x
        r.x = abs(value["x_max"])

    if abs(value["y_min"]) != abs(value["y_max"]):
        width = value["y_max"] - value["y_min"]
        c.y = pose.position.y + value["y_min"] + width / 2.0
        r.y = width / 2.0
    else:
        c.y = pose.position.y
        r.y = abs(value["y_max"])

    if abs(value["z_min"]) != abs(value["z_max"]):
        width = value["z_max"] - value["z_min"]
        c.z = pose.position.z + value["z_min"] + width / 2.0
        r.z = width / 2.0
    else:
        c.z = pose.position.z
        r.z = abs(value["z_max"])

    return r, c


def get_object_bounding_box(name, pose, bb_config_path):

    try:
        yaml_content = read_yaml_file(bb_config_path)
    except FileNotFoundError:
        print("[WARNING] Boundingbox.yaml not found, using default BB!")
        return get_default_bounding_box(pose)

    try:
        value = yaml_content[name]
    except KeyError:
        print(f"[WARNING] {name} not in BoundingBox.yaml, using default BB!")
        return get_default_bounding_box(pose)

    if frame_is_centered(value):
        r = get_halfwidths(value)
        c = get_center(pose)
        o = pose.orientation
    else:
        r, c = get_centered_center_and_halfwidth(value, pose)
        o = pose.orientation

    return BoundingBox(c, r, o)


def collision_check_with_bb_config(obj1_name, obj1_pose, obj2_name, obj2_pose, bb_config_path):

    bb_1 = get_object_bounding_box(obj1_name, obj1_pose, bb_config_path)
    bb_2 = get_object_bounding_box(obj2_name, obj2_pose, bb_config_path)

    return aabb(bb_1, bb_2)


def oriented_collision_check_with_bb_config(obj1_name, obj1_pose, obj2_name, obj2_pose, bb_config_path):

    bb_1 = get_object_bounding_box(obj1_name, obj1_pose, bb_config_path)
    bb_2 = get_object_bounding_box(obj2_name, obj2_pose, bb_config_path)

    return SAT(bb_1, bb_2)


def create_bb_from_obj_size(obj_pose, obj_size, padding=0.0):
    # convert size to min max values and add a padding to bb
    obj_size_min_max = {'x_max': round(obj_size.x / 2.0 + padding, 4), 'x_min': -round(obj_size.x / 2.0 + padding, 4),
                        'y_max': round(obj_size.y / 2.0 + padding, 4), 'y_min': -round(obj_size.y / 2.0 + padding, 4),
                        'z_max': round(obj_size.z / 2.0 + padding, 4), 'z_min': -round(obj_size.z / 2.0 + padding, 4)}

    if frame_is_centered(obj_size_min_max):
        r = get_halfwidths(obj_size_min_max)
        c = get_center(obj_pose)
        o = obj_pose.orientation
    else:
        r, c = get_centered_center_and_halfwidth(obj_size_min_max, obj_pose)
        o = obj_pose.orientation

    return BoundingBox(c, r, o)


def collision_check_with_obj_size(obj1_pose, obj1_size, obj2_pose, obj2_size, padding=0.0):
    bb_1 = create_bb_from_obj_size(obj1_pose, obj1_size, padding)
    bb_2 = create_bb_from_obj_size(obj2_pose, obj2_size, padding)

    return aabb(bb_1, bb_2)


def oriented_collision_check_with_obj_size(obj1_pose, obj1_size, obj2_pose, obj2_size, padding=0.0):
    bb_1 = create_bb_from_obj_size(obj1_pose, obj1_size, padding)
    bb_2 = create_bb_from_obj_size(obj2_pose, obj2_size, padding)

    return SAT(bb_1, bb_2)
