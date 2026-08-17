import unittest
from unittest import mock

from icp_servoing.robot import CR5Robot


class FakeNode:
    def __init__(self, executor):
        self.executor = executor


class ExecutorOwnershipTests(unittest.TestCase):
    def make(self, executor):
        robot = object.__new__(CR5Robot)
        robot._node = FakeNode(executor)
        return robot

    @mock.patch('icp_servoing.robot.time.sleep')
    @mock.patch('icp_servoing.robot.rclpy.spin_once')
    def test_executor_owned_node_is_not_spun_again(
            self, spin_once, sleep):
        self.make(object())._settle_after_enable(0.25)
        spin_once.assert_not_called()
        sleep.assert_called_once_with(0.25)

    @mock.patch('icp_servoing.robot.time.sleep')
    @mock.patch('icp_servoing.robot.rclpy.spin_once')
    def test_standalone_node_still_spins_locally(
            self, spin_once, sleep):
        node = FakeNode(None)
        robot = object.__new__(CR5Robot)
        robot._node = node
        robot._settle_after_enable(0.25)
        spin_once.assert_called_once_with(node, timeout_sec=0.25)
        sleep.assert_not_called()

    def test_init_does_not_cycle_an_already_enabled_controller(self):
        robot = object.__new__(CR5Robot)
        robot._speed = 15
        robot._logger = mock.Mock()
        robot.get_robot_mode = mock.Mock(return_value=robot.MODE_ENABLED)
        robot.apply_motion_safety = mock.Mock(return_value=True)
        robot.use_base_tool0 = mock.Mock(return_value=True)
        robot._call = mock.Mock()

        self.assertTrue(robot.init())

        robot._call.assert_not_called()
        robot.apply_motion_safety.assert_called_once_with()
        robot.use_base_tool0.assert_called_once_with()


if __name__ == '__main__':
    unittest.main()
