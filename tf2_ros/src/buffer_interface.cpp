/*
 * Copyright 2026, Open Source Robotics Foundation, Inc.
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
 *     * Neither the name of the copyright holder nor the names of its
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

#include <chrono>

#include "tf2_ros/buffer_interface.hpp"

namespace tf2_ros
{

builtin_interfaces::msg::Time toMsg(const tf2::TimePoint & t)
{
  std::chrono::nanoseconds ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch());
  std::chrono::seconds s =
    std::chrono::duration_cast<std::chrono::seconds>(t.time_since_epoch());
  builtin_interfaces::msg::Time time_msg;
  time_msg.sec = static_cast<int32_t>(s.count());
  time_msg.nanosec = static_cast<uint32_t>(ns.count() % 1000000000ull);
  return time_msg;
}

tf2::TimePoint fromMsg(const builtin_interfaces::msg::Time & time_msg)
{
  int64_t d = time_msg.sec * 1000000000ull + time_msg.nanosec;
  std::chrono::nanoseconds ns(d);
  return tf2::TimePoint(std::chrono::duration_cast<tf2::Duration>(ns));
}

builtin_interfaces::msg::Duration toMsg(const tf2::Duration & t)
{
  std::chrono::nanoseconds ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(t);
  std::chrono::seconds s =
    std::chrono::duration_cast<std::chrono::seconds>(t);
  builtin_interfaces::msg::Duration duration_msg;
  duration_msg.sec = static_cast<int32_t>(s.count());
  duration_msg.nanosec = static_cast<uint32_t>(ns.count() % 1000000000ull);
  return duration_msg;
}

tf2::Duration fromMsg(const builtin_interfaces::msg::Duration & duration_msg)
{
  int64_t d = duration_msg.sec * 1000000000ull + duration_msg.nanosec;
  std::chrono::nanoseconds ns(d);
  return tf2::Duration(std::chrono::duration_cast<tf2::Duration>(ns));
}

double timeToSec(const builtin_interfaces::msg::Time & time_msg)
{
  auto ns = std::chrono::duration<double, std::nano>(time_msg.nanosec);
  auto s = std::chrono::duration<double>(time_msg.sec);
  return (s + std::chrono::duration_cast<std::chrono::duration<double>>(ns)).count();
}

tf2::TimePoint fromRclcpp(const rclcpp::Time & time)
{
  // tf2::TimePoint is a typedef to a system time point, but rclcpp::Time may be ROS time.
  // Ignore that, and assume the clock used from rclcpp time points is consistent.
  return tf2::TimePoint(std::chrono::nanoseconds(time.nanoseconds()));
}

rclcpp::Time toRclcpp(const tf2::TimePoint & time)
{
  // tf2::TimePoint is a typedef to a system time point, but rclcpp::Time may be ROS time.
  // Use whatever the default clock is.
  return rclcpp::Time(std::chrono::nanoseconds(time.time_since_epoch()).count());
}

tf2::Duration fromRclcpp(const rclcpp::Duration & duration)
{
  return tf2::Duration(std::chrono::nanoseconds(duration.nanoseconds()));
}

rclcpp::Duration toRclcpp(const tf2::Duration & duration)
{
  return rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(duration));
}

BufferInterface::~BufferInterface()
{
}

}  // namespace tf2_ros
