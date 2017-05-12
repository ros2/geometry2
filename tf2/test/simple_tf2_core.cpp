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

TEST(tf2_time, To_From_Sec)
{
  // Exact representation of a time.
  tf2::TimePoint t1 = tf2::get_now();

  // Approximate representation of the time in seconds as a double (floating point error introduced).
  double t1_sec = tf2::timeToSec(t1);

  // Time point from the t1_sec approximation.
  tf2::TimePoint t2 = tf2::timeFromSec(t1_sec);

  // Check that the difference due to t1_sec being approximate is small.
  tf2::Duration diff = t2 > t1 ? t2 - t1 : t1 - t2;
  std::cout << "Difference (ns): " << diff.count() << std::endl;
  EXPECT_TRUE(diff < tf2::Duration(std::chrono::nanoseconds(200)));

  // No new floating point errors are expected after converting to and from time points.
  double t2_sec = tf2::timeToSec(t2);
  EXPECT_EQ(t1_sec, t2_sec);
  EXPECT_EQ(tf2::timeFromSec(t1_sec), tf2::timeFromSec(t2_sec));
}

TEST(tf2_time, Negative_Durations)
{
  tf2::TimePoint t1 = tf2::get_now();
  // Create a time in the past.
  double expected_diff_sec = -0.7;
  tf2::TimePoint t2 = tf2::timeFromSec(tf2::timeToSec(t1) + expected_diff_sec);

  // Check durationToSec.
  // Create a negative duration.
  tf2::Duration duration = t2 - t1;
  EXPECT_TRUE(duration.count() < 0);
  std::cout << "Difference (ns): " << duration.count() << std::endl;
  std::cout << "Difference (s): " << tf2::durationToSec(duration) << std::endl;

  double error_sec = tf2::durationToSec(duration) - expected_diff_sec;
  EXPECT_TRUE(std::abs(error_sec) < 0.001);

  // Check durationFromSec.
  tf2::Duration duration2 = tf2::durationFromSec(tf2::durationToSec(duration));
  tf2::Duration error_duration = duration - duration2;
  // Get the absolute difference between Durations.
  error_duration = error_duration > tf2::Duration(0) ? error_duration : -error_duration;
  std::cout << "Error (ns): " << error_duration.count() << std::endl;
  EXPECT_TRUE(error_duration < tf2::Duration(std::chrono::nanoseconds(1)));
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
