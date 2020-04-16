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
        executable='test_buffer_client.py',
        output='screen',
    )
    node_static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=["5", "6", "7", "0", "0", "0", "1", "a", "b"]
    )

    node_buffer_server = Node(
        package='test_tf2',
        executable='test_buffer_server',
        output='screen',
        arguments=[],
        sigterm_timeout=LaunchConfiguration('sigterm_timeout', default=2)
    )
    return LaunchDescription([
        node_static_transform_publisher,
        node_buffer_server,
        node_under_test,
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest(),
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
