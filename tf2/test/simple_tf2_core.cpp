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
  EXPECT_FALSE(tfc.setTransform(st, "authority1"));
  
}

TEST(tf2, setTransformValid)
{
  tf2::BufferCore tfc;
  geometry_msgs::msg::TransformStamped st;
  st.header.frame_id = "foo";
  st.header.stamp = builtin_interfaces::msg::Time();
  st.header.stamp.sec = 1;
  st.header.stamp.nanosec = 0;
  st.child_frame_id = "child";
  st.transform.rotation.w = 1;
  EXPECT_TRUE(tfc.setTransform(st, "authority1"));
  
}

TEST(tf2, setTransformValidWithCallback)
{
  tf2::BufferCore buffer;

  // Input
  const std::string target_frame = "foo";
  const std::string source_frame = "bar";
  const tf2::TimePoint time_point = tf2::timeFromSec(1.0);

  tf2::TransformableRequestHandle received_request_handle;
  std::string received_target_frame = "";
  std::string received_source_frame = "";
  tf2::TimePoint received_time_point;
  bool transform_available = false;

  auto cb_handle = buffer.addTransformableCallback(
    [&received_request_handle, &received_target_frame, &received_source_frame, &received_time_point, &transform_available](
      tf2::TransformableRequestHandle request_handle,
      const std::string & target_frame,
      const std::string & source_frame,
      tf2::TimePoint time,
      tf2::TransformableResult result)
    {
      received_request_handle = request_handle;
      received_target_frame = target_frame;
      received_source_frame = source_frame;
      received_time_point = time;
      transform_available = tf2::TransformAvailable == result;
    });

  tf2::TransformableRequestHandle request_handle = buffer.addTransformableRequest(
    cb_handle, target_frame, source_frame, time_point);
  ASSERT_NE(request_handle, 0u);
  
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.frame_id = target_frame;
  transform_msg.header.stamp = builtin_interfaces::msg::Time();
  transform_msg.header.stamp.sec = 1;
  transform_msg.header.stamp.nanosec = 0;
  transform_msg.child_frame_id = source_frame;
  transform_msg.transform.rotation.w = 1;
  EXPECT_TRUE(buffer.setTransform(transform_msg, "authority1"));

  // Setting the transform should trigger the callback
  EXPECT_EQ(received_target_frame, target_frame);
  EXPECT_EQ(received_source_frame, source_frame);
  EXPECT_EQ(received_time_point, time_point);
  EXPECT_TRUE(transform_available);
}

TEST(tf2, setTransformInvalidQuaternion)
{
  tf2::BufferCore tfc;
  geometry_msgs::msg::TransformStamped st;
  st.header.frame_id = "foo";
  st.header.stamp = builtin_interfaces::msg::Time();
  st.header.stamp.sec = 1;
  st.header.stamp.nanosec = 0;
  st.child_frame_id = "child";
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
  st.transform.rotation.w = 1;
  EXPECT_TRUE(tfc.setTransform(st, "authority1"));
  EXPECT_FALSE(tfc.canTransform("foo", "bar", tf2::TimePoint(std::chrono::seconds(1))));
}

TEST(tf2_clear, LookUp_Static_Transfrom_Succeed)
{
  tf2::BufferCore tfc;
  geometry_msgs::msg::TransformStamped st;
  st.header.frame_id = "foo";
  st.header.stamp = builtin_interfaces::msg::Time();
  st.header.stamp.sec = 1;
  st.header.stamp.nanosec = 0;
  st.child_frame_id = "bar";
  st.transform.rotation.w = 1;
  tfc.clear();
  EXPECT_TRUE(tfc.setTransform(st, "authority1", true));
  EXPECT_NO_THROW(
    auto trans = tfc.lookupTransform("foo", "bar", tf2::TimePoint(std::chrono::seconds(2)));
  );
}

TEST(tf2_clear, LookUp_Static_Transfrom_Fail)
{
  tf2::BufferCore tfc;
  geometry_msgs::msg::TransformStamped st;
  st.header.frame_id = "foo";
  st.header.stamp = builtin_interfaces::msg::Time();
  st.header.stamp.sec = 1;
  st.header.stamp.nanosec = 0;
  st.child_frame_id = "bar";
  st.transform.rotation.w = 1;
  EXPECT_TRUE(tfc.setTransform(st, "authority1"));
  tfc.clear();
  EXPECT_TRUE(tfc.setTransform(st, "authority1", true));
  EXPECT_NO_THROW(
    auto trans = tfc.lookupTransform("foo", "bar", tf2::TimePoint(std::chrono::seconds(2)));
  );
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
  EXPECT_TRUE(diff < tf2::Duration(std::chrono::nanoseconds(200)));

  // No new floating point errors are expected after converting to and from time points.
  double t2_sec = tf2::timeToSec(t2);
  EXPECT_EQ(t1_sec, t2_sec);
  EXPECT_EQ(tf2::timeFromSec(t1_sec), tf2::timeFromSec(t2_sec));
}

TEST(tf2_time, To_From_Duration)
{
  tf2::TimePoint t1 = tf2::get_now();

  std::vector<double> values = {
    -0.01, -0.2, -0.5, -0.7, -0.99, -5.7, -1000000, -123456789.123456789,
    0.01, 0.2, 0.5, 0.7, 0.99, 5.7, 10000000, 123456789.123456789,
    0.0,
  };

  for (double expected_diff_sec : values )
  {
    tf2::TimePoint t2 = tf2::timeFromSec(tf2::timeToSec(t1) + expected_diff_sec);

    // Check durationToSec.
    tf2::Duration duration1 = t2 - t1;

    // Approximate representation of the duration in seconds as a double (floating point error introduced).
    double duration1_sec = tf2::durationToSec(duration1);

    // Check that the difference due to duration_sec being approximate is small.
    double error_sec = duration1_sec - expected_diff_sec;
    EXPECT_TRUE(std::abs(error_sec) < 200 * 1e-9);

    // Check durationFromSec.
    tf2::Duration duration2 = tf2::durationFromSec(tf2::durationToSec(duration1));
    tf2::Duration error_duration = duration1 - duration2;
    // Get the absolute difference between Durations.
    error_duration = error_duration > tf2::Duration(0) ? error_duration : -error_duration;
    // Increased tolerance for larger differences.
    int32_t tol_ns = std::abs(expected_diff_sec) > 100000 ? 10 : 1;

    EXPECT_TRUE(error_duration < tf2::Duration(std::chrono::nanoseconds(tol_ns)));
  }
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
