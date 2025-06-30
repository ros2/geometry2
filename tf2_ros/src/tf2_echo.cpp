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
#include <getopt.h>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "rclcpp/rclcpp.hpp"

struct Arguments {
  std::string source_frame;
  std::string target_frame;
  double rate = 1.0;
  double cache_time = 10.0;
  double offset = 0.0;
  double time = 0.0;
  int limit = 0;
  int precision = 3;
  bool use_time = false;
  bool use_offset = false;
};

void print_usage(const char* program_name) {
  printf("Usage: %s source_frame target_frame [options]\n\n", program_name);
  printf("Options:\n");
  printf("  -r, --rate RATE        Update rate (default: 1.0)\n");
  printf("  -c, --cache-time TIME  Length of tf buffer cache in seconds (default: 10.0)\n");
  printf("  -o, --offset OFFSET    Offset the lookup from current time (ignored if using -t)\n");
  printf("  -t, --time TIME        Fixed time to do the lookup\n");
  printf("  -l, --limit LIMIT      Lookup fixed number of times\n");
  printf("  -p, --precision PREC   Output precision (default: 3)\n");
  printf("  -h, --help             Show this help message\n\n");
  printf("This will echo the transform from the coordinate frame of the source_frame\n");
  printf("to the coordinate frame of the target_frame.\n");
  printf("Note: This is the transform to get data from target_frame into the source_frame.\n");
}

Arguments parse_arguments(int argc, char** argv) {
  Arguments args;
  
  if (argc < 3) {
    print_usage(argv[0]);
    exit(1);
  }
  
  args.source_frame = argv[1];
  args.target_frame = argv[2];
  
  static struct option long_options[] = {
    {"rate", required_argument, 0, 'r'},
    {"cache-time", required_argument, 0, 'c'},
    {"offset", required_argument, 0, 'o'},
    {"time", required_argument, 0, 't'},
    {"limit", required_argument, 0, 'l'},
    {"precision", required_argument, 0, 'p'},
    {"help", no_argument, 0, 'h'},
    {0, 0, 0, 0}
  };
  
  int c;
  int option_index = 0;
  
  while ((c = getopt_long(argc, argv, "r:c:o:t:l:p:h", long_options, &option_index)) != -1) {
    switch (c) {
      case 'r':
        args.rate = std::stod(optarg);
        if (args.rate <= 0.0) {
          fprintf(stderr, "Rate must be > 0.0\n");
          exit(1);
        }
        break;
      case 'c':
        args.cache_time = std::stod(optarg);
        if (args.cache_time <= 0.0) {
          fprintf(stderr, "Cache time must be > 0.0\n");
          exit(1);
        }
        break;
      case 'o':
        args.offset = std::stod(optarg);
        args.use_offset = true;
        break;
      case 't':
        args.time = std::stod(optarg);
        args.use_time = true;
        break;
      case 'l':
        args.limit = std::stoi(optarg);
        if (args.limit <= 0) {
          fprintf(stderr, "Limit must be > 0\n");
          exit(1);
        }
        break;
      case 'p':
        args.precision = std::stoi(optarg);
        if (args.precision <= 0) {
          fprintf(stderr, "Precision must be > 0\n");
          exit(1);
        }
        break;
      case 'h':
        print_usage(argv[0]);
        exit(0);
      case '?':
        print_usage(argv[0]);
        exit(1);
      default:
        break;
    }
  }
  
  return args;
}

class echoListener
{
public:
  tf2_ros::Buffer buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;

  explicit echoListener(rclcpp::Clock::SharedPtr clock, double cache_time)
  : buffer_(clock, tf2::durationFromSec(cache_time))
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
  
  // Parse arguments
  Arguments parsed_args = parse_arguments(argc, argv);
  
  rclcpp::Rate rate(parsed_args.rate);
  rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("tf2_echo");
  rclcpp::Clock::SharedPtr clock = nh->get_clock();
  
  // Instantiate a local listener with custom cache time
  echoListener echoListener(clock, parsed_args.cache_time);

  std::string source_frameid = parsed_args.source_frame;
  std::string target_frameid = parsed_args.target_frame;

  // Wait for the first transforms to become available.
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
  int count = 0;

  // Main loop
  while (rclcpp::ok()) {
    count++;
    if (parsed_args.limit > 0 && count > parsed_args.limit) {
      break;
    }
    
    try {
      geometry_msgs::msg::TransformStamped echo_transform;
      
      // Determine lookup time based on arguments
      tf2::TimePoint lookup_time;
      if (parsed_args.use_time) {
        lookup_time = tf2::TimePoint(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(parsed_args.time)));
      } else if (parsed_args.use_offset) {
        auto now = clock->now();
        lookup_time = tf2::TimePoint(now.nanoseconds() + 
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(parsed_args.offset)));
      } else {
        lookup_time = tf2::TimePoint(); // Latest available
      }
      
      echo_transform = echoListener.buffer_.lookupTransform(
        source_frameid, target_frameid, lookup_time);
        
      // Set precision for output
      std::cout.precision(parsed_args.precision);
      std::cout.setf(std::ios::fixed, std::ios::floatfield);
      
      // Current time info
      auto current_time = clock->now();
      std::cout << "At time " << 
        echo_transform.header.stamp.sec + echo_transform.header.stamp.nanosec * 1e-9 << 
        ", (current time " << current_time.seconds() << ")" << std::endl;
        
      auto translation = echo_transform.transform.translation;
      auto rotation = echo_transform.transform.rotation;
      
      std::cout << "- Translation: [" << translation.x << ", " << translation.y << ", " <<
        translation.z << "]" << std::endl;
      std::cout << "- Rotation: in Quaternion [" << rotation.x << ", " << rotation.y <<
        ", " << rotation.z << ", " << rotation.w << "]" << std::endl;

      tf2::Matrix3x3 mat(tf2::Quaternion{rotation.x, rotation.y, rotation.z, rotation.w});

      tf2Scalar yaw, pitch, roll;
      mat.getEulerYPR(yaw, pitch, roll);

      std::cout << "            in RPY (radian) [" << roll << ", " << pitch << ", " << yaw << "]" <<
        std::endl;
      std::cout << "            in RPY (degree) [" <<
        roll * rad_to_deg << ", " <<
        pitch * rad_to_deg << ", " <<
        yaw * rad_to_deg << "]" << std::endl;

    } catch (const tf2::LookupException & ex) {
      std::cout << "At time " << clock->now().seconds() << ", (current time " << 
        clock->now().seconds() << ") " << ex.what() << std::endl;
    } catch (const tf2::ExtrapolationException & ex) {
      std::cout << "(current time " << clock->now().seconds() << ") " << ex.what() << std::endl;
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
