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

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

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

void print_usage()
{
  printf("Usage: tf2_echo source_frame target_frame [options]\n\n");
  printf("This will echo the transform from the coordinate frame of the source_frame\n");
  printf("to the coordinate frame of the target_frame. \n");
  printf("Note: This is the transform to get data from target_frame into the source_frame.\n\n");
  printf("Options:\n");
  printf("  -r <rate>       Echo rate in Hz (default: 1.0)\n");
  printf("  -t <time>       Fixed time to do the lookup (in seconds)\n");
  printf("  -p <precision>  Output precision (default: 3)\n");
}

int main(int argc, char ** argv)
{
  // Initialize ROS
  std::vector<std::string> args = rclcpp::init_and_remove_ros_arguments(argc, argv);

  double rate_hz = 1.0;
  double fixed_time = -1.0;  // -1 means use current time
  int precision = 3;
  std::string source_frameid;
  std::string target_frameid;

  // Parse arguments
  if (args.size() < 3) {
    print_usage();
    return 1;
  }

  source_frameid = args[1];
  target_frameid = args[2];

  // Parse optional arguments
  for (size_t i = 3; i < args.size(); i++) {
    if (args[i] == "-r" && i + 1 < args.size()) {
      try {
        rate_hz = std::stof(args[i + 1]);
        if (rate_hz <= 0.0) {
          fprintf(stderr, "Rate must be positive\n");
          return 2;
        }
        i++;  // Skip the next argument as it's the rate value
      } catch (const std::invalid_argument &) {
        fprintf(
          stderr, "Failed to convert rate argument '%s' to a floating-point number\n",
          args[i + 1].c_str());
        return 2;
      }
    } else if (args[i] == "-t" && i + 1 < args.size()) {
      try {
        fixed_time = std::stof(args[i + 1]);
        i++;  // Skip the next argument as it's the time value
      } catch (const std::invalid_argument &) {
        fprintf(
          stderr, "Failed to convert time argument '%s' to a floating-point number\n",
          args[i + 1].c_str());
        return 3;
      }
    } else if (args[i] == "-p" && i + 1 < args.size()) {
      try {
        precision = std::stoi(args[i + 1]);
        if (precision < 0) {
          fprintf(stderr, "Precision must be non-negative\n");
          return 4;
        }
        i++;  // Skip the next argument as it's the precision value
      } catch (const std::invalid_argument &) {
        fprintf(
          stderr, "Failed to convert precision argument '%s' to an integer\n",
          args[i + 1].c_str());
        return 4;
      }
    } else {
      fprintf(stderr, "Unknown argument: %s\n", args[i].c_str());
      print_usage();
      return 5;
    }
  }

  rclcpp::Rate rate(rate_hz);

  rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("tf2_echo");

  rclcpp::Clock::SharedPtr clock = nh->get_clock();
  // Instantiate a local listener
  echoListener echoListener(clock);

  // Wait for the first transforms to become available.
  std::string warning_msg;
  tf2::TimePoint lookup_time_point;

  if (fixed_time >= 0.0) {
    // Convert fixed time to tf2::TimePoint
    rclcpp::Time rclcpp_time(static_cast<int64_t>(fixed_time * 1e9));
    lookup_time_point = tf2_ros::fromRclcpp(rclcpp_time);
  } else {
    lookup_time_point = tf2::TimePoint();
  }

  while (rclcpp::ok() && !echoListener.buffer_.canTransform(
      source_frameid, target_frameid, lookup_time_point, &warning_msg))
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

      // Determine lookup time
      if (fixed_time >= 0.0) {
        // Use fixed time
        rclcpp::Time rclcpp_time(static_cast<int64_t>(fixed_time * 1e9));
        lookup_time_point = tf2_ros::fromRclcpp(rclcpp_time);
      } else {
        // Use current time (most recent transform)
        lookup_time_point = tf2::TimePoint();
      }

      echo_transform = echoListener.buffer_.lookupTransform(
        source_frameid, target_frameid, lookup_time_point);

      std::cout.precision(precision);
      std::cout.setf(std::ios::fixed, std::ios::floatfield);
      std::cout << "At time " << echo_transform.header.stamp.sec << "." <<
        echo_transform.header.stamp.nanosec << std::endl;
      auto translation = echo_transform.transform.translation;
      double translation_xyz[] = {translation.x, translation.y, translation.z};
      auto rotation = echo_transform.transform.rotation;
      std::cout << "- Translation: [" << translation.x << ", " << translation.y << ", " <<
        translation.z << "]" << std::endl;
      std::cout << "- Rotation: in Quaternion (xyzw) [" << rotation.x << ", " << rotation.y <<
        ", " << rotation.z << ", " << rotation.w << "]" << std::endl;

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
          std::cout << " " << std::setw(6) << std::setprecision(precision) << mat[i][j];
        }
        std::cout << " " << std::setw(6) << std::setprecision(precision) << translation_xyz[i];
        std::cout << std::endl;
      }
      for (int j = 0; j < 3; j++) {
        std::cout << " " << std::setw(6) << std::setprecision(precision) << 0.0;
      }
      std::cout << " " << std::setw(6) << std::setprecision(precision) << 1.0 << std::endl;
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
