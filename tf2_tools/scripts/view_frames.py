#!/usr/bin/env python3
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# author: Wim Meeussen

import rclpy
import tf2_py as tf2
import yaml
import subprocess
from tf2_msgs.srv import FrameGraph
import tf2_ros
import time

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('view_frames')

    buffer = tf2_ros.Buffer(node=node)
    listener = tf2_ros.TransformListener(buffer, node, spin_thread=False)

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    # listen to tf for 5 seconds
    node.get_logger().info('Listening to tf data during 5 seconds...')
    start_time = time.time()
    while (time.time() - start_time) < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info('Generating graph in frames.pdf file...')

    cli = node.create_client(FrameGraph, 'tf2_frames')
    req = FrameGraph.Request()
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    raised = True
    try:
        result = future.result()
        raised = False
    except Exception as e:
        node.get_logger().error('Service call failed %r' % (e,))
    else:
        node.get_logger().info(
            'Result:'+ str(result) )
        data = yaml.load(result.frame_yaml)
        with open('frames.gv', 'w') as f:
           f.write(generate_dot(data, node.get_clock().now().seconds_nanoseconds()))
        subprocess.Popen('dot -Tpdf frames.gv -o frames.pdf'.split(' ')).communicate()
    finally:
        cli.destroy()
        node.destroy_node()
        rclpy.shutdown()
        return not raised

def generate_dot(data, recorded_time):
    if len(data) == 0:
        return 'digraph G { "No tf data received" }'

    dot = 'digraph G {\n'
    for el in data: 
        map = data[el]
        dot += '"'+map['parent']+'" -> "'+str(el)+'"'
        dot += '[label=" '
        dot += 'Broadcaster: '+map['broadcaster']+'\\n'
        dot += 'Average rate: '+str(map['rate'])+'\\n'
        dot += 'Buffer length: '+str(map['buffer_length'])+'\\n' 
        dot += 'Most recent transform: '+str(map['most_recent_transform'])+'\\n'
        dot += 'Oldest transform: '+str(map['oldest_transform'])+'\\n'
        dot += '"];\n'
        if not map['parent'] in data:
            root = map['parent']
    dot += 'edge [style=invis];\n'
    dot += ' subgraph cluster_legend { style=bold; color=black; label ="view_frames Result";\n'
    dot += '"Recorded at time: '+str(recorded_time[0]+recorded_time[1]/1e9)+'"[ shape=plaintext ] ;\n'
    dot += '}->"'+root+'";\n}'
    return dot


if __name__ == '__main__':
    main()
