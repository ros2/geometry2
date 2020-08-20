// Copyright 2010, Willow Garage, Inc. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <string>

#include "console_bridge/console.h"
#include "tf2/buffer_core.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

int main(int argc, char ** argv)
{
  uint32_t num_levels = 10;
  if (argc > 1) {
    num_levels = std::stoi(argv[1]);
  }

  tf2::BufferCore bc;
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = builtin_interfaces::msg::Time(1);
  t.header.frame_id = "root";
  t.child_frame_id = "0";
  t.transform.translation.x = 1;
  t.transform.rotation.w = 1.0;
  bc.setTransform(t, "me");
  t.header.stamp = builtin_interfaces::msg::Time(2);
  bc.setTransform(t, "me");

  for (uint32_t i = 1; i < num_levels / 2; ++i) {
    for (uint32_t j = 1; j < 3; ++j) {
      std::stringstream parent_ss;
      parent_ss << (i - 1);
      std::stringstream child_ss;
      child_ss << i;

      t.header.stamp = builtin_interfaces::msg::Time(j);
      t.header.frame_id = parent_ss.str();
      t.child_frame_id = child_ss.str();
      bc.setTransform(t, "me");
    }
  }

  t.header.frame_id = "root";
  std::stringstream ss;
  ss << num_levels / 2;
  t.header.stamp = builtin_interfaces::msg::Time(1);
  t.child_frame_id = ss.str();
  bc.setTransform(t, "me");
  t.header.stamp = builtin_interfaces::msg::Time(2);
  bc.setTransform(t, "me");

  for (uint32_t i = num_levels / 2 + 1; i < num_levels; ++i) {
    for (uint32_t j = 1; j < 3; ++j) {
      std::stringstream parent_ss;
      parent_ss << (i - 1);
      std::stringstream child_ss;
      child_ss << i;

      t.header.stamp = builtin_interfaces::msg::Time(j);
      t.header.frame_id = parent_ss.str();
      t.child_frame_id = child_ss.str();
      bc.setTransform(t, "me");
    }
  }

  std::string v_frame0 = std::to_string(num_levels - 1);
  std::string v_frame1 = std::to_string(num_levels / 2 - 1);
  logInform("%s to %s", v_frame0.c_str(), v_frame1.c_str());
  geometry_msgs::msg::TransformStamped out_t;

  const uint32_t count = 1000000;
  logInform("Doing %d %d-level tests", count, num_levels);

#if 01
  {
    ros::WallTime start = ros::WallTime::now();
    for (uint32_t i = 0; i < count; ++i) {
      out_t = bc.lookupTransform(v_frame1, v_frame0, tf2::TimePoint());
    }
    ros::WallTime end = ros::WallTime::now();
    ros::WallDuration dur = end - start;
    logInform(
      "lookupTransform at Time(0) took %f for an average of %.9f", dur.toSec(),
      dur.toSec() / static_cast<double>(count));
  }
#endif

#if 01
  {
    ros::WallTime start = ros::WallTime::now();
    for (uint32_t i = 0; i < count; ++i) {
      out_t = bc.lookupTransform(v_frame1, v_frame0, tf2::TimePoint(std::chrono::seconds(1)));
    }
    ros::WallTime end = ros::WallTime::now();
    ros::WallDuration dur = end - start;
    logInform(
      "lookupTransform at Time(1) took %f for an average of %.9f", dur.toSec(),
      dur.toSec() / static_cast<double>(count));
  }
#endif

#if 01
  {
    ros::WallTime start = ros::WallTime::now();
    for (uint32_t i = 0; i < count; ++i) {
      out_t =
        bc.lookupTransform(v_frame1, v_frame0, tf2::TimePoint(std::chrono::milliseconds(1500)));
    }
    ros::WallTime end = ros::WallTime::now();
    ros::WallDuration dur = end - start;
    logInform(
      "lookupTransform at Time(1.5) took %f for an average of %.9f", dur.toSec(),
      dur.toSec() / static_cast<double>(count));
  }
#endif

#if 01
  {
    ros::WallTime start = ros::WallTime::now();
    for (uint32_t i = 0; i < count; ++i) {
      out_t = bc.lookupTransform(v_frame1, v_frame0, tf2::TimePoint(std::chrono::seconds(2)));
    }
    ros::WallTime end = ros::WallTime::now();
    ros::WallDuration dur = end - start;
    logInform(
      "lookupTransform at Time(2) took %f for an average of %.9f", dur.toSec(),
      dur.toSec() / static_cast<double>(count));
  }
#endif

#if 01
  {
    ros::WallTime start = ros::WallTime::now();
    for (uint32_t i = 0; i < count; ++i) {
      bc.canTransform(v_frame1, v_frame0, tf2::TimePoint());
    }
    ros::WallTime end = ros::WallTime::now();
    ros::WallDuration dur = end - start;
    logInform(
      "canTransform at Time(0) took %f for an average of %.9f", dur.toSec(),
      dur.toSec() / static_cast<double>(count));
  }
#endif

#if 01
  {
    ros::WallTime start = ros::WallTime::now();
    for (uint32_t i = 0; i < count; ++i) {
      bc.canTransform(v_frame1, v_frame0, tf2::TimePoint(std::chrono::seconds(1)));
    }
    ros::WallTime end = ros::WallTime::now();
    ros::WallDuration dur = end - start;
    logInform(
      "canTransform at Time(1) took %f for an average of %.9f", dur.toSec(),
      dur.toSec() / static_cast<double>(count));
  }
#endif

#if 01
  {
    ros::WallTime start = ros::WallTime::now();
    for (uint32_t i = 0; i < count; ++i) {
      bc.canTransform(v_frame1, v_frame0, tf2::TimePoint(std::chrono::milliseconds(1500)));
    }
    ros::WallTime end = ros::WallTime::now();
    ros::WallDuration dur = end - start;
    logInform(
      "canTransform at Time(1.5) took %f for an average of %.9f", dur.toSec(),
      dur.toSec() / static_cast<double>(count));
  }
#endif

#if 01
  {
    ros::WallTime start = ros::WallTime::now();
    for (uint32_t i = 0; i < count; ++i) {
      bc.canTransform(v_frame1, v_frame0, tf2::TimePoint(std::chrono::seconds(2)));
    }
    ros::WallTime end = ros::WallTime::now();
    ros::WallDuration dur = end - start;
    logInform(
      "canTransform at Time(2) took %f for an average of %.9f", dur.toSec(),
      dur.toSec() / static_cast<double>(count));
  }
#endif
}
