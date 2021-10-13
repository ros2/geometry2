# generated from buildfarm_perf_tests/test/test_performance.py.in
# generated code does not contain a copyright notice

import unittest

from launch import LaunchDescription
import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import launch_testing
import launch_testing.actions

def generate_test_description():
    node_under_test = Node(
        package='test_tf2',
        executable='test_static_publisher',
        output='screen',
        arguments=[],
    )
    node_static_transform_publisher_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=["--x", "1", "--y", "0", "--z", "0", "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1", "--frame-id", "a", "--child-frame-id", "b"]
    )
    node_static_transform_publisher_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=["--x", "0", "--y", "1", "--z", "0", "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1", "--frame-id", "b", "--child-frame-id", "c"]
    )

    return LaunchDescription([
        node_static_transform_publisher_1,
        node_static_transform_publisher_2,
        node_under_test,
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest(),
        ]), locals()


class TestBufferClient(unittest.TestCase):

    def test_termination(self, node_under_test, proc_info):
        proc_info.assertWaitForShutdown(process=node_under_test, timeout=(60))


@launch_testing.post_shutdown_test()
class BufferClientTestAfterShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        # Check that all processes in the launch (in this case, there's just one) exit
        # with code 0
        launch_testing.asserts.assertExitCodes(proc_info)
