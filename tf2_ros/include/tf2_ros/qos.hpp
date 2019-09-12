/*
 * Copyright (c) 2019, Open Source Robotics Foundation, Inc.
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

#ifndef TF2_ROS__QOS_HPP_
#define TF2_ROS__QOS_HPP_

#include <rclcpp/qos.hpp>
#include <tf2_ros/visibility_control.h>

namespace tf2_ros
{

class TF2_ROS_PUBLIC DynamicListenerQoS: public rclcpp::QoS
{
public:
  explicit DynamicListenerQoS(size_t depth = 100) : rclcpp::QoS(depth) {}
};

class TF2_ROS_PUBLIC DynamicBroadcasterQoS : public rclcpp::QoS
{
public:
  explicit DynamicBroadcasterQoS(size_t depth = 100) : rclcpp::QoS(depth) {}
};

class TF2_ROS_PUBLIC StaticListenerQoS : public rclcpp::QoS
{
public:
  explicit StaticListenerQoS(size_t depth = 100) : rclcpp::QoS(depth)
  {
    transient_local();
  }
};

class TF2_ROS_PUBLIC StaticBroadcasterQoS : public rclcpp::QoS
{
public:
  explicit StaticBroadcasterQoS(size_t depth = 1) : rclcpp::QoS(depth)
  {
    transient_local();
  }
};
}  // namespace tf2_ros
#endif  // TF2_ROS__QOS_HPP_
