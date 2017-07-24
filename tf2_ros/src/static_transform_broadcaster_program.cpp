/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstdio>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/static_transform_broadcaster.h"

#include "builtin_interfaces/msg/time.hpp"

#include "rcutils/logging_macros.h"

//TODO(clalancette re-enable this)
// bool validateXmlRpcTf(XmlRpc::XmlRpcValue tf_data) {
//   // Validate a TF stored in XML RPC format: ensures the appropriate fields
//   // exist. Note this does not check data types.
//   return tf_data.hasMember("child_frame_id") &&
//          tf_data.hasMember("header") &&
//          tf_data["header"].hasMember("frame_id") &&
//          tf_data.hasMember("transform") &&
//          tf_data["transform"].hasMember("translation") &&
//          tf_data["transform"]["translation"].hasMember("x") &&
//          tf_data["transform"]["translation"].hasMember("y") &&
//          tf_data["transform"]["translation"].hasMember("z") &&
//          tf_data["transform"].hasMember("rotation") &&
//          tf_data["transform"]["rotation"].hasMember("x") &&
//          tf_data["transform"]["rotation"].hasMember("y") &&
//          tf_data["transform"]["rotation"].hasMember("z") &&
//          tf_data["transform"]["rotation"].hasMember("w");
// };

int main(int argc, char ** argv)
{
  //Initialize ROS
  rclcpp::init(argc, argv);

  // TODO(clalancette): Anonymize the node name like it is in ROS1.
  auto node = rclcpp::node::Node::make_shared("static_transform_publisher");

  tf2_ros::StaticTransformBroadcaster broadcaster(node);
  geometry_msgs::msg::TransformStamped msg;

  if(argc == 10)
  {
    msg.transform.translation.x = atof(argv[1]);
    msg.transform.translation.y = atof(argv[2]);
    msg.transform.translation.z = atof(argv[3]);
    msg.transform.rotation.x = atof(argv[4]);
    msg.transform.rotation.y = atof(argv[5]);
    msg.transform.rotation.z = atof(argv[6]);
    msg.transform.rotation.w = atof(argv[7]);
    msg.header.stamp = rclcpp::Time::now();
    msg.header.frame_id = argv[8];
    msg.child_frame_id = argv[9];
  }
  else if (argc == 9)
  {
    msg.transform.translation.x = atof(argv[1]);
    msg.transform.translation.y = atof(argv[2]);
    msg.transform.translation.z = atof(argv[3]);

    tf2::Quaternion quat;
    quat.setRPY(atof(argv[6]), atof(argv[5]), atof(argv[4]));
    msg.transform.rotation.x = quat.x();
    msg.transform.rotation.y = quat.y();
    msg.transform.rotation.z = quat.z();
    msg.transform.rotation.w = quat.w();

    msg.header.stamp = rclcpp::Time::now();
    msg.header.frame_id = argv[7];
    msg.child_frame_id = argv[8];
  }
  // else if (argc == 2) {
  //   const std::string param_name = argv[1];
  //   ROS_INFO_STREAM("Looking for TF in parameter: " << param_name);
  //   XmlRpc::XmlRpcValue tf_data;

  //   if (!ros::param::has(param_name) || !ros::param::get(param_name, tf_data)) {
  //     ROS_FATAL_STREAM("Could not read TF from parameter server: " << param_name);
  //     return -1;
  //   }

  //   // Check that all required members are present & of the right type.
  //   if (!validateXmlRpcTf(tf_data)) {
  //     ROS_FATAL_STREAM("Could not validate XmlRpcC for TF data: " << tf_data);
  //     return -1;
  //   }

  //   msg.transform.translation.x = (double) tf_data["transform"]["translation"]["x"];
  //   msg.transform.translation.y = (double) tf_data["transform"]["translation"]["y"];
  //   msg.transform.translation.z = (double) tf_data["transform"]["translation"]["z"];
  //   msg.transform.rotation.x = (double) tf_data["transform"]["rotation"]["x"];
  //   msg.transform.rotation.y = (double) tf_data["transform"]["rotation"]["y"];
  //   msg.transform.rotation.z = (double) tf_data["transform"]["rotation"]["z"];
  //   msg.transform.rotation.w = (double) tf_data["transform"]["rotation"]["w"];
  //   msg.header.stamp = ros::Time::now();
  //   msg.header.frame_id = (std::string) tf_data["header"]["frame_id"];
  //   msg.child_frame_id = (std::string) tf_data["child_frame_id"];
  // }
  else
  {
    printf("A command line utility for manually sending a transform.\n");
    //printf("It will periodicaly republish the given transform. \n");
    printf("Usage: static_transform_publisher x y z qx qy qz qw frame_id child_frame_id \n");
    printf("OR \n");
    printf("Usage: static_transform_publisher x y z yaw pitch roll frame_id child_frame_id \n");
    //printf("OR \n");
    //printf("Usage: static_transform_publisher /param_name \n");
    //printf("\nThis transform is the transform of the coordinate frame from frame_id into the coordinate frame \n");
    //printf("of the child_frame_id.  \n");
    RCUTILS_LOG_ERROR("static_transform_publisher exited due to not having the right number of arguments");
    return -1;
  }

  // Checks: frames should not be the same.
  if (msg.header.frame_id == msg.child_frame_id)
  {
    RCUTILS_LOG_FATAL("target_frame and source frame are the same (%s, %s) this cannot work",
                      msg.header.frame_id.c_str(), msg.child_frame_id.c_str());
    return 1;
  }

  rclcpp::WallRate loop_rate(0.2);
  while (rclcpp::ok())
  {
    RCUTILS_LOG_INFO_THROTTLE(RCUTILS_STEADY_TIME, 30000 /* ms */, "LOOPING due to no latching at the moment");
    broadcaster.sendTransform(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  // broadcaster.sendTransform(msg);
  // ROS_INFO("Spinning until killed publishing %s to %s", msg.header.frame_id.c_str(), msg.child_frame_id.c_str());
  // rclcpp::spin(node);
  return 0;
};
