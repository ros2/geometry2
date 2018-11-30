#! /usr/bin/env python3

from time import sleep

import rclpy

from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("tf2_example_pub")

    pub = node.create_publisher(TFMessage, "/tf")

    t1 = TransformStamped()
    t1.header.frame_id = "base_link"
    t1.child_frame_id = "arm1"
    t2 = TransformStamped()
    t2.header.frame_id = "base_link"
    t2.child_frame_id = "arm2"

    msg = TFMessage()
    msg.transforms = [t1, t2]

    while rclpy.ok():
        pub.publish(msg)
        sleep(0.9)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
