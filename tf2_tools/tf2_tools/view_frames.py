# Copyright (c) 2008 Willow Garage Inc. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# author: Wim Meeussen

import argparse
import subprocess
import sys
import time

import rclpy
from tf2_msgs.srv import FrameGraph
import tf2_ros

import yaml


def main():
    rclpy.init(args=sys.argv)

    args_without_ros = rclpy.utilities.remove_ros_args(sys.argv)
    parser = argparse.ArgumentParser(
        description='Create a diagram of the TF frames being broadcast over ROS')
    parser.add_argument(
        '--wait-time', '-t', type=float, default=5.0,
        help='Listen to the /tf topic for this many seconds before rendering the frame tree')
    parser.add_argument('-o', '--output', help='Output filename')
    parsed_args = parser.parse_args(args=args_without_ros[1:])

    node = rclpy.create_node('view_frames')

    buf = tf2_ros.Buffer(node=node)
    listener = tf2_ros.TransformListener(buf, node, spin_thread=False)
    listener  # To quiet a flake8 warning

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    # listen to tf for 5 seconds
    node.get_logger().info(f'Listening to tf data for {parsed_args.wait_time} seconds...')
    start_time = time.time()
    while (time.time() - start_time) < parsed_args.wait_time:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info('Generating graph...')

    cli = node.create_client(FrameGraph, 'tf2_frames')
    req = FrameGraph.Request()
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    ret = 1
    try:
        result = future.result()
        ret = 0
    except Exception as e:
        node.get_logger().error('Service call failed %r' % (e,))
    else:
        node.get_logger().info('Result:' + str(result))
        data = yaml.safe_load(result.frame_yaml)

        if parsed_args.output is not None:
            frames_gv = '{:s}.gv'.format(parsed_args.output)
            frames_pdf = '{:s}.pdf'.format(parsed_args.output)
        else:
            datetime = time.strftime('%Y-%m-%d_%H.%M.%S')
            frames_gv = 'frames_{:s}.gv'.format(datetime)
            frames_pdf = 'frames_{:s}.pdf'.format(datetime)

        node.get_logger().info(f'Exporting graph in {frames_pdf} file...')
        with open(frames_gv, 'w') as f:
            f.write(generate_dot(data, node.get_clock().now().seconds_nanoseconds()))

        cmd = ['dot', '-Tpdf', frames_gv, '-o', frames_pdf]
        subprocess.Popen(cmd).communicate()
    finally:
        cli.destroy()
        node.destroy_node()
        rclpy.shutdown()
        return ret


def generate_dot(data, recorded_time):
    if len(data) == 0:
        return 'digraph G { "No tf data received" }'

    dot = 'digraph G {\n'
    for el in data:
        data_map = data[el]
        dot += '"' + data_map['parent'] + '" -> "' + str(el)+'"'
        dot += '[label=" '
        dot += 'Broadcaster: ' + data_map['broadcaster'] + '\\n'
        dot += 'Average rate: ' + str(data_map['rate']) + '\\n'
        dot += 'Buffer length: ' + str(data_map['buffer_length']) + '\\n'
        dot += 'Most recent transform: ' + str(data_map['most_recent_transform']) + '\\n'
        dot += 'Oldest transform: ' + str(data_map['oldest_transform']) + '\\n'
        dot += '"];\n'
        if not data_map['parent'] in data:
            root = data_map['parent']
    dot += 'edge [style=invis];\n'
    dot += ' subgraph cluster_legend { style=bold; color=black; label ="view_frames Result";\n'
    dot += '"Recorded at time: ' + str(recorded_time[0] + recorded_time[1] / 1e9) + \
        '"[ shape=plaintext ] ;\n'
    dot += '}->"' + root + '";\n}'
    return dot


if __name__ == '__main__':
    sys.exit(main())
