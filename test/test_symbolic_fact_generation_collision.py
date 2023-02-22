import unittest

from symbolic_fact_generation.common.collision_checking import oriented_collision_check_with_obj_size
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion

class TestCollisionDetection(unittest.TestCase):
    def test_collision_between_non_rotated_objects_true(self):
        # Cube at origin with size 2, not rotated
        obj_1_pose = Pose(position=Point(0, 0, 0), orientation=Quaternion(0, 0, 0, 1))
        obj_1_size = Vector3(2, 2, 2)

        # Cube offset of origin in x direction by 1 with size 2, not rotated
        # Cubes are overlapping
        obj_2_pose = Pose(position=Point(1, 0, 0), orientation=Quaternion(0, 0, 0, 1))
        obj_2_size = Vector3(2, 2, 2)

        self.assertTrue(oriented_collision_check_with_obj_size(obj_1_pose, obj_1_size, obj_2_pose, obj_2_size))

    def test_collision_between_non_rotated_objects_false(self):
        # Cube at origin with size 2, not rotated
        obj_1_pose = Pose(position=Point(0, 0, 0), orientation=Quaternion(0, 0, 0, 1))
        obj_1_size = Vector3(2, 2, 2)

        # Cube offset of origin in x direction by 5 with size 2, not rotated
        # Gap between cubes of 1
        obj_2_pose = Pose(position=Point(5, 0, 0), orientation=Quaternion(0, 0, 0, 1))
        obj_2_size = Vector3(2, 2, 2)

        self.assertFalse(oriented_collision_check_with_obj_size(obj_1_pose, obj_1_size, obj_2_pose, obj_2_size))

    def test_collision_between_rotated_objects_true(self):
        # Cube at origin with size 2, not rotated
        obj_1_pose = Pose(position=Point(0.0, 0.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0))
        obj_1_size = Vector3(2.0, 2.0, 2.0)

        # Long stick like rectangle offset of origin in x direction by 3 with size (1, 4, 1), rotated by 90 degrees around z-axis
        obj_2_pose = Pose(position=Point(3.0, 0.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.707, 0.707))
        obj_2_size = Vector3(1.0, 4.0, 1.0)

        self.assertTrue(oriented_collision_check_with_obj_size(obj_1_pose, obj_1_size, obj_2_pose, obj_2_size))

    def test_collision_between_rotated_objects_false(self):
        # Cube at origin with size 2, not rotated
        obj_1_pose = Pose(position=Point(0, 0, 0), orientation=Quaternion(0, 0, 0, 1))
        obj_1_size = Vector3(2, 2, 2)

        # Long stick like rectangle offset of origin in x direction by 3 with size (1, 4, 1), rotated by 90 degrees around x-axis
        obj_2_pose = Pose(position=Point(3, 0, 0), orientation=Quaternion(0.707, 0, 0, 0.707))
        obj_2_size = Vector3(1, 4, 1)

        self.assertFalse(oriented_collision_check_with_obj_size(obj_1_pose, obj_1_size, obj_2_pose, obj_2_size))


PKG = 'symbolic_fact_generation'
NAME = 'test_symbolic_fact_generation_lib'

if __name__ == '__main__':
    import rosunit

    rosunit.unitrun(PKG, NAME, TestCollisionDetection)
