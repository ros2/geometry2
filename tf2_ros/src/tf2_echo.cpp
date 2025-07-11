/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2015, Open Source Robotics Foundation, Inc.
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

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include <cmath>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "rclcpp/rclcpp.hpp"

class echoListener
{
public:
  tf2_ros::Buffer buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;

  explicit echoListener(rclcpp::Clock::SharedPtr clock)
  : buffer_(clock)
  {
    tfl_ = std::make_shared<tf2_ros::TransformListener>(buffer_);
  }

  ~echoListener()
  {
  }
};


int main(int argc, char ** argv)
{
  // Initialize ROS
  std::vector<std::string> args = rclcpp::init_and_remove_ros_arguments(argc, argv);

  double rate_hz;
  // Allow 2 or 3 command line arguments
  if (args.size() == 3) {
    rate_hz = 1.0;
  } else if (args.size() == 4) {
    size_t pos;
    try {
      rate_hz = std::stof(args[3], &pos);
    } catch (const std::invalid_argument &) {
      // If the user provided an argument that wasn't a number (like 'foo'), stof() will throw.
      fprintf(
        stderr, "Failed to convert rate argument '%s' to a floating-point number\n",
        args[3].c_str());
      return 2;
    }

    // If the user provide an floating-point argument with junk on the end (like '1.0foo'), the pos
    // argument will show it didn't convert the whole argument.
    if (pos != args[3].length()) {
      fprintf(
        stderr, "Failed to convert rate argument '%s' to a floating-point number\n",
        args[3].c_str());
      return 3;
    }
  } else {
    printf("Usage: tf2_echo source_frame target_frame [echo_rate]\n\n");
    printf("This will echo the transform from the coordinate frame of the source_frame\n");
    printf("to the coordinate frame of the target_frame. \n");
    printf("Note: This is the transform to get data from target_frame into the source_frame.\n");
    printf("Default echo rate is 1 if echo_rate is not given.\n");
    return 1;
  }
  // TODO(tfoote): restore parameter option
  // // read rate parameter
  // ros::NodeHandle p_nh("~");
  // p_nh.param("rate", rate_hz, 1.0);
  rclcpp::Rate rate(rate_hz);

  // TODO(tfoote): restore anonymous??
  // ros::init_options::AnonymousName);

  rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("tf2_echo");

  rclcpp::Clock::SharedPtr clock = nh->get_clock();
  // Instantiate a local listener
  echoListener echoListener(clock);

  std::string source_frameid = args[1];
  std::string target_frameid = args[2];

  // Wait for the first transforms to become avaiable.
  std::string warning_msg;
  while (rclcpp::ok() && !echoListener.buffer_.canTransform(
      source_frameid, target_frameid, tf2::TimePoint(), &warning_msg))
  {
    RCLCPP_INFO_THROTTLE(
      nh->get_logger(), *clock, 1000, "Waiting for transform %s ->  %s: %s",
      source_frameid.c_str(), target_frameid.c_str(), warning_msg.c_str());
    rate.sleep();
  }
  constexpr double rad_to_deg = 180.0 / M_PI;

  // Nothing needs to be done except wait for a quit
  // The callbacks within the listener class will take care of everything
  while (rclcpp::ok()) {
    try {
      geometry_msgs::msg::TransformStamped echo_transform;
      echo_transform = echoListener.buffer_.lookupTransform(
        source_frameid, target_frameid,
        tf2::TimePoint());
      std::cout.precision(3);
      std::cout.setf(std::ios::fixed, std::ios::floatfield);
      std::cout << "At time " << echo_transform.header.stamp.sec << "." <<
        echo_transform.header.stamp.nanosec << std::endl;
      auto translation = echo_transform.transform.translation;
      double translation_xyz[] = {translation.x, translation.y, translation.z};
      auto rotation = echo_transform.transform.rotation;
      std::cout << "- Translation: [" << translation.x << ", " << translation.y << ", " <<
        translation.z << "]" << std::endl;
      std::cout << "- Rotation: in Quaternion [" << rotation.x << ", " << rotation.y << ", " <<
        rotation.z << ", " << rotation.w << "]" << std::endl;

      tf2::Matrix3x3 mat(tf2::Quaternion{rotation.x, rotation.y, rotation.z, rotation.w});

      tf2Scalar yaw, pitch, roll;
      mat.getEulerYPR(yaw, pitch, roll);

      std::cout << "- Rotation: in RPY (radian) [" << roll << ", " << pitch << ", " << yaw << "]" <<
        std::endl;
      std::cout << "- Rotation: in RPY (degree) [" <<
        roll * rad_to_deg << ", " <<
        pitch * rad_to_deg << ", " <<
        yaw * rad_to_deg << "]" << std::endl;

      std::cout << "- Matrix:" << std::endl;
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          std::cout << " " << std::setw(6) << std::setprecision(3) << mat[i][j];
        }
        std::cout << " " << std::setw(6) << std::setprecision(3) << translation_xyz[i];
        std::cout << std::endl;
      }
      for (int j = 0; j < 3; j++) {
        std::cout << " " << std::setw(6) << std::setprecision(3) << 0.0;
      }
      std::cout << " " << std::setw(6) << std::setprecision(3) << 1.0 << std::endl;
    } catch (const tf2::TransformException & ex) {
      std::cout << "Failure at " << clock->now().seconds() << std::endl;
      std::cout << "Exception thrown:" << ex.what() << std::endl;
      std::cout << "The current list of frames is:" << std::endl;
      std::cout << echoListener.buffer_.allFramesAsString() << std::endl;
    }
    rate.sleep();
  }

  return 0;
}
