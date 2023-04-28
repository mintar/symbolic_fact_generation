import unittest

from symbolic_fact_generation.on_fact_generator import check_on_condition, check_in_condition
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion


class TestOnGenerator(unittest.TestCase):
    # Mockup class resembling the message returned by the pose_selector query
    class ObjectPose():
        def __init__(self, class_id, instance_id, pose, size, min=None, max=None):
            self.class_id = class_id
            self.instance_id = instance_id
            self.pose = pose
            self.size = size
            if min is None:
                self.min = Vector3(-(size.x / 2.0), -(size.y / 2.0), -(size.z / 2.0))
            else:
                self.min = min
            if max is None:
                self.max = Vector3(size.x / 2.0, size.y / 2.0, size.z / 2.0)
            else:
                self.max = max

    def test_on_condition_klt_table(self):
        klt_pose = Pose(position=Point(21.123515853417523, 13.95535470209441, 0.8024652163958597),
                        orientation=Quaternion(-0.008991460789160606, 0.031998891447492524, -0.8422055317592717, 0.5381310870532298))
        klt_size = Vector3(0.297, 0.197, 0.147)
        klt = self.ObjectPose('klt', 1, klt_pose, klt_size)

        table_pose = Pose(position=Point(21.265, 13.84, 0.3575),
                          orientation=Quaternion(0.0, 0.0, 0.0, 1.0))
        table_size = Vector3(1.6, 0.8, 0.715)
        table = self.ObjectPose('table', 1, table_pose, table_size)

        self.assertTrue(check_on_condition(klt, table), msg="klt_1 is on table_1")

    def test_in_condition_multimeter_klt(self):
        klt_pose = Pose(position=Point(21.123515853417523, 13.95535470209441, 0.8024652163958597),
                        orientation=Quaternion(-0.008991460789160606, 0.031998891447492524, -0.8422055317592717, 0.5381310870532298))
        klt_size = Vector3(0.297, 0.197, 0.147)
        klt = self.ObjectPose('klt', 1, klt_pose, klt_size)

        multimeter_pose = Pose(position=Point(21.11593840315666, 13.932851285216492, 0.7503829583854417),
                               orientation=Quaternion(-0.0422499770349587, 0.013006959666949392, 0.3903381678327803, 0.9196096308617674))
        multimeter_size = Vector3(0.17949999868869781, 0.08748800307512283, 0.04206399992108345)
        multimeter = self.ObjectPose('multimeter', 1, multimeter_pose, multimeter_size)

        self.assertTrue(check_in_condition(multimeter, klt), msg="multimeter_1 is inside klt_1")


PKG = 'symbolic_fact_generation'
NAME = 'test_symbolic_fact_generation_on_generator'

if __name__ == '__main__':
    import rosunit

    rosunit.unitrun(PKG, NAME, TestOnGenerator)
