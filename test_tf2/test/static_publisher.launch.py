# generated from buildfarm_perf_tests/test/test_performance.py.in
# generated code does not contain a copyright notice

import unittest

from launch import LaunchDescription
import launch
from launch_ros.actions import Node
import launch_testing
from launch.substitutions import LaunchConfiguration

def generate_test_description(ready_fn):
    node_under_test = Node(
        package='test_tf2',
        node_executable='test_static_publisher',
        output='screen',
        arguments=[],
    )
    node_static_transform_publisher_1 = Node(
        package='tf2_ros',
        node_executable='static_transform_publisher',
        output='screen',
        arguments=["1", "0", "0", "0", "0", "0", "1", "a", "b"]
    )
    node_static_transform_publisher_2 = Node(
        package='tf2_ros',
        node_executable='static_transform_publisher',
        output='screen',
        arguments=["0", "1", "0", "0", "0", "0", "1", "b", "c"]
    )

    return LaunchDescription([
        node_static_transform_publisher_1,
        node_static_transform_publisher_2,
        node_under_test,
        launch_testing.util.KeepAliveProc(),
        launch.actions.OpaqueFunction(function=lambda context: ready_fn()),
        ]), locals()


class TestBufferClient(unittest.TestCase):

    def test_termination(self, node_under_test, proc_info):
        proc_info.assertWaitForShutdown(process=node_under_test, timeout=(10))


@launch_testing.post_shutdown_test()
class BufferClientTestAfterShutdown(unittest.TestCase):

    def test_exit_code(self):
        # Check that all processes in the launch (in this case, there's just one) exit
        # with code 0
        launch_testing.asserts.assertExitCodes(self.proc_info)
