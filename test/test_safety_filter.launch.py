import re
from typing import Union
from unittest import result
import rclpy
import unittest
import launch
import time
import launch_testing
import pytest
import subprocess
from launch_ros.actions import Node
import os


@pytest.mark.launch_test
def generate_test_description():
    """Generate launch description for node testing."""
    start_safety_filter_node = Node(
        package='safety_filter_mpc',
        executable='safety_filter_mpc_node',
        name='safety_filter_mpc',
        arguments=[
            '--ros-args', '--log-level', 'warn'],
    )

    return launch.LaunchDescription([
        start_safety_filter_node,
        launch.actions.TimerAction(period=8.0, actions=[launch_testing.actions.ReadyToTest()]),
    ])


class GeneratedNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('node_test')

    def tearDown(self):
        self.node.destroy_node()

    def test_set_collision_params(self, proc_info):
        """
        Test if collision compile-time declared default parameters.
        """
        self.__check_parameter_set(
            "safety_filter.collisions.radius", 
            0.2)
        self.__check_parameter_set(
            "safety_filter.collisions.front_offset",
            0.1)
        self.__check_parameter_set(
            "safety_filter.collisions.back_offset",
            0.1)
        
    def test_set_constraints_params(self, proc_info):
        """
        Test if constraints compile-time declared default parameters.
        """
        self.__check_parameter_set(
            "safety_filter.constraints.lbu", 
            [-1, -1.5])
        self.__check_parameter_set(
            "safety_filter.constraints.lbx", 
            [-1, -1.5])
        self.__check_parameter_set(
            "safety_filter.constraints.lsh", 
            [0, 0])
        self.__check_parameter_set(
            "safety_filter.constraints.ubu", 
            [1, 1.5])
        self.__check_parameter_set(
            "safety_filter.constraints.ubx", 
            [1, 1.5])
        self.__check_parameter_set(
            "safety_filter.constraints.uh", 
            [10000, 10000])
        self.__check_parameter_set(
            "safety_filter.constraints.ush", 
            [10000, 10000])

    def test_set_weights_params(self, proc_info):
        """
        Test if weights compile-time declared default parameters.
        """
        self.__check_parameter_set(
            "safety_filter.cost.W", 
            [3.0, 0.5])
        self.__check_parameter_set(
            "safety_filter.cost.W_0", 
            [15.0, 3.0, 3.0, 1.0])

    def test_set_slacks_params(self, proc_info):
        """
        Test if slacks compile-time declared default parameters.
        """
        self.__check_parameter_set(
            "safety_filter.cost.Zl", 
            [1000.0, 1000.0])
        self.__check_parameter_set(
            "safety_filter.cost.Zu", 
            [1000.0, 1000.0])
        self.__check_parameter_set(
            "safety_filter.cost.zl", 
            [800.0, 800.0])
        self.__check_parameter_set(
            "safety_filter.cost.zu", 
            [800.0, 800.0])

    def test_set_thers_params(self, proc_info):
        """
        Test if others compile-time declared default parameters.
        """
        self.__check_parameter_set(
            "safety_filter.ts", 
            0.1)


    def test_subscribing(self, proc_info):
        """Test if the node subscribes to all expected topics."""
        self.wait_for_subscription('/map', timeout=1.0)
        self.wait_for_subscription('/cmd_vel_unsafe', timeout=1.0)
        self.wait_for_subscription('/odom', timeout=1.0)

        
    def test_publishing(self, proc_info):
        """Test if the node publishes to all expected topics."""
        self.wait_for_publisher('/cmd_vel_safe', timeout=1.0)
        self.wait_for_publisher('/safety_filter_marker/obstacles', timeout=1.0)
        self.wait_for_publisher('/safety_filter_marker/trajectory', timeout=1.0)


    def wait_for_subscription(self, topic: str, timeout: float = 1.0, threshold: float = 0.5):
        end_time = time.time() + timeout + threshold
        while time.time() < end_time:
            subs = self.node.get_subscriptions_info_by_topic(topic)
            if subs:
                return True
            time.sleep(0.05)
        self.fail(f"Node has NOT subscribed to '{topic}'.")


    def wait_for_publisher(self, topic: str, timeout: float = 1.0, threshold: float = 0.5):
        end_time = time.time() + timeout + threshold
        while time.time() < end_time:
            pubs = self.node.get_publishers_info_by_topic(topic)
            if pubs:
                return True
            time.sleep(0.05)
        self.fail(f"Node has NOT published to '{topic}'.")


    def __check_parameter_get(self, param_name: str, expected_value: Union[list[float], float]):
        """Run a subprocess command and return its output."""
        output = get_parameter(param_name)
        numbers = [float(x) for x in re.findall(r"[-+]?\d*\.\d+|\d+", output)]
        if isinstance(expected_value, list):
            self.assertListEqual(numbers, expected_value, f"Parameter {param_name} has the wrong value! Got {numbers}")
        else:
            self.assertEqual(numbers[0], expected_value, f"Parameter {param_name} has the wrong value! Got {numbers[0]}")


    def __check_parameter_set(self, param_name: str, new_value: Union[list[float], float]):
        """Run a subprocess command and return its output."""
        try:
            set_parameter(param_name, new_value)
            self.__check_parameter_get(param_name, new_value)
        except subprocess.CalledProcessError as e:
            self.fail(f"Failed to set parameter {param_name}.\n"
                      f"Exit-Code: {e.returncode}\n"
                      f"Stderr: {e.stderr}\n"
                      f"Stdout: {e.stdout}")


def get_parameter(param_name: str):
    """Run a subprocess command and return its output."""
    cmd = ['ros2', 'param', 'get', 'safety_filter_mpc', param_name]
    result = subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=True
    )
    return result.stdout

def set_parameter(param_name: str, value: Union[list[float], float]):
    """Run a subprocess command to set a parameter."""
    if isinstance(value, list):
        value_str = "[" + ",".join(map(str, value)) + "]"
    else:
        value_str = str(value)

    cmd = ['ros2', 'param', 'set', 'safety_filter_mpc', param_name, value_str]
    print(f"CMD set: {cmd}")
    result = subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=True
    )
    return result.stderr