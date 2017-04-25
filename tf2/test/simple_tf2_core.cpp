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

#include <chrono>
#include <gtest/gtest.h>
#include <tf2/buffer_core.h>
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"
#include "tf2/time.h"

TEST(tf2, setTransformFail)
{
  tf2::BufferCore tfc;
  geometry_msgs::msg::TransformStamped st;
  st.header.frame_id = "foo";
  st.header.stamp = builtin_interfaces::msg::Time();
  st.header.stamp.sec = 1;
  st.header.stamp.nanosec = 1;
  st.child_frame_id = "foo";
  st.transform.translation.x = 0;
  st.transform.translation.y = 0;
  st.transform.translation.z = 0;
  st.transform.rotation.x = 0;
  st.transform.rotation.y = 0;
  st.transform.rotation.z = 0;
  st.transform.rotation.w = 1;
  EXPECT_FALSE(tfc.setTransform(st, "authority1"));
  
}

TEST(tf2, setTransformValid)
{
  tf2::BufferCore tfc;
  geometry_msgs::msg::TransformStamped st;
  st.header.frame_id = "foo";
  st.header.stamp = builtin_interfaces::msg::Time();
  st.header.stamp.sec = 1;
  st.header.stamp.nanosec = 1;
  st.child_frame_id = "child";
  st.transform.translation.x = 0;
  st.transform.translation.y = 0;
  st.transform.translation.z = 0;
  st.transform.rotation.x = 0;
  st.transform.rotation.y = 0;
  st.transform.rotation.z = 0;
  st.transform.rotation.w = 1;
  EXPECT_TRUE(tfc.setTransform(st, "authority1"));
  
}

TEST(tf2, setTransformInvalidQuaternion)
{
  tf2::BufferCore tfc;
  geometry_msgs::msg::TransformStamped st;
  st.header.frame_id = "foo";
  st.header.stamp = builtin_interfaces::msg::Time();
  st.header.stamp.sec = 1;
  st.header.stamp.nanosec = 1;
  st.child_frame_id = "child";
  st.transform.translation.x = 0;
  st.transform.translation.y = 0;
  st.transform.translation.z = 0;
  st.transform.rotation.x = 0;
  st.transform.rotation.y = 0;
  st.transform.rotation.z = 0;
  st.transform.rotation.w = 0;
  EXPECT_FALSE(tfc.setTransform(st, "authority1"));
  
}

TEST(tf2_lookupTransform, LookupException_Nothing_Exists)
{
  tf2::BufferCore tfc;
  EXPECT_THROW(tfc.lookupTransform("a", "b", tf2::TimePoint(std::chrono::seconds(1))), tf2::LookupException);
  
}

TEST(tf2_canTransform, Nothing_Exists)
{
  tf2::BufferCore tfc;
  EXPECT_FALSE(tfc.canTransform("a", "b", tf2::TimePoint(std::chrono::seconds(1))));
  
}

TEST(tf2_lookupTransform, LookupException_One_Exists)
{
  tf2::BufferCore tfc;
  geometry_msgs::msg::TransformStamped st;
  st.header.frame_id = "foo";
  st.header.stamp = builtin_interfaces::msg::Time();
  st.header.stamp.sec = 1;
  st.header.stamp.nanosec = 0;
  st.child_frame_id = "child";
  st.transform.translation.x = 0;
  st.transform.translation.y = 0;
  st.transform.translation.z = 0;
  st.transform.rotation.x = 0;
  st.transform.rotation.y = 0;
  st.transform.rotation.z = 0;
  st.transform.rotation.w = 1;
  EXPECT_TRUE(tfc.setTransform(st, "authority1"));
  EXPECT_THROW(tfc.lookupTransform("foo", "bar", tf2::TimePoint(std::chrono::seconds(1))), tf2::LookupException);
  
}

TEST(tf2_canTransform, One_Exists)
{
  tf2::BufferCore tfc;
  geometry_msgs::msg::TransformStamped st;
  st.header.frame_id = "foo";
  st.header.stamp = builtin_interfaces::msg::Time();
  st.header.stamp.sec = 1;
  st.header.stamp.nanosec = 0;
  st.child_frame_id = "child";
  st.transform.translation.x = 0;
  st.transform.translation.y = 0;
  st.transform.translation.z = 0;
  st.transform.rotation.x = 0;
  st.transform.rotation.y = 0;
  st.transform.rotation.z = 0;
  st.transform.rotation.w = 1;
  EXPECT_TRUE(tfc.setTransform(st, "authority1"));
  EXPECT_FALSE(tfc.canTransform("foo", "bar", tf2::TimePoint(std::chrono::seconds(1))));
}

TEST(tf2_time, Display_Time_Point)
{
  tf2::TimePoint t = tf2::get_now();
  // Check ability to stringify
  std::string s = tf2::displayTimePoint(t);
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
