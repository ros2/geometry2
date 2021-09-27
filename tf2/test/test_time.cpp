/*
 * Copyright (c) 2020, Open Source Robotics Foundation, Inc.
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

#include <gtest/gtest.h>
#include <chrono>
#include "tf2/time.h"

using namespace std::literals::chrono_literals;

TEST(TestTime, durationFromSec)
{
  EXPECT_EQ(tf2::durationFromSec(0.0), 0ns);
  EXPECT_EQ(tf2::durationFromSec(-0.0), 0ns);
  EXPECT_EQ(tf2::durationFromSec(1.0), 1e9ns);
  EXPECT_EQ(tf2::durationFromSec(-1.0), -1e9ns);
  EXPECT_EQ(tf2::durationFromSec(1.000000001), 1s + 1ns);
  EXPECT_EQ(tf2::durationFromSec(1.0000000012345), 1s + 1ns);
  EXPECT_EQ(tf2::durationFromSec(0.5), 5e8ns);
  EXPECT_EQ(tf2::durationFromSec(1.5), 1s + 5e8ns);
  EXPECT_EQ(tf2::durationFromSec(2.5), 2s + 5e8ns);
  EXPECT_EQ(tf2::durationFromSec(-0.5), -5e8ns);
  EXPECT_EQ(tf2::durationFromSec(-1.5), -1s - 5e8ns);
  EXPECT_EQ(tf2::durationFromSec(-2.5), -2s - 5e8ns);
  EXPECT_EQ(tf2::durationFromSec(1e-40), 0s);
  EXPECT_EQ(tf2::durationFromSec(-1e-40), 0s);
}

TEST(TestTime, timeFromSec)
{
  auto time_point = tf2::TimePoint();
  EXPECT_EQ(tf2::timeFromSec(0.0), time_point);
  EXPECT_EQ(tf2::timeFromSec(1.0), time_point + 1s);
  EXPECT_EQ(tf2::timeFromSec(-1.0), time_point - 1s);
  EXPECT_EQ(tf2::timeFromSec(0.0), time_point);
  EXPECT_EQ(tf2::timeFromSec(1e-9), time_point + 1ns);
  EXPECT_EQ(tf2::timeFromSec(-1e-9), time_point - 1ns);
}

TEST(TestTime, durationToSec)
{
  EXPECT_EQ(tf2::durationToSec(0s), 0.0);
  EXPECT_EQ(tf2::durationToSec(1s), 1.0);
  EXPECT_EQ(tf2::durationToSec(-1s), -1.0);
  EXPECT_EQ(tf2::durationToSec(1ns), 1e-9);
  EXPECT_EQ(tf2::durationToSec(-1ns), -1e-9);
  EXPECT_EQ(tf2::durationToSec(1s + 1ns), 1.000000001);
  EXPECT_EQ(tf2::durationToSec(1s - 1ns), 0.999999999);

  // Check rounding of seconds component
  EXPECT_EQ(tf2::durationToSec(500ms), 0.5);
  EXPECT_EQ(tf2::durationToSec(1s + 500ms), 1.5);
  EXPECT_EQ(tf2::durationToSec(2s + 500ms), 2.5);
  EXPECT_EQ(tf2::durationToSec(-500ms), -0.5);
  EXPECT_EQ(tf2::durationToSec(-1s - 500ms), -1.5);
  EXPECT_EQ(tf2::durationToSec(-2s - 500ms), -2.5);
}

TEST(TestTime, timeToSec)
{
  auto time_point = tf2::TimePoint();
  EXPECT_EQ(tf2::timeToSec(time_point), 0.0);
  EXPECT_EQ(tf2::timeToSec(time_point + 1s), 1.0);
  EXPECT_EQ(tf2::timeToSec(time_point - 1s), -1.0);
  EXPECT_EQ(tf2::timeToSec(time_point + 1ns), 0.000000001);
}

TEST(TestTime, displayTimePoint)
{
  EXPECT_EQ(tf2::displayTimePoint(tf2::TimePoint(0s)), "0.000000");
  EXPECT_EQ(tf2::displayTimePoint(tf2::TimePoint(-0s)), "0.000000");
  EXPECT_EQ(tf2::displayTimePoint(tf2::TimePoint(1s)), "1.000000");
  EXPECT_EQ(tf2::displayTimePoint(tf2::TimePoint(10s)), "10.000000");
  EXPECT_EQ(tf2::displayTimePoint(tf2::TimePoint(-1s)), "-1.000000");
  EXPECT_EQ(tf2::displayTimePoint(tf2::TimePoint(-10s)), "-10.000000");
  EXPECT_EQ(tf2::displayTimePoint(tf2::TimePoint(1ns)), "0.000000");
  EXPECT_EQ(tf2::displayTimePoint(tf2::TimePoint(-1ns)), "-0.000000");
  EXPECT_EQ(tf2::displayTimePoint(tf2::TimePoint(1000ns)), "0.000001");
  EXPECT_EQ(tf2::displayTimePoint(tf2::TimePoint(-1000ns)), "-0.000001");
  EXPECT_EQ(tf2::displayTimePoint(tf2::TimePoint(4ms)), "0.004000");
}

TEST(TestTime, To_From_Sec)
{
  // Exact representation of a time.
  tf2::TimePoint t1 = tf2::get_now();

  // Approximate representation of the time in seconds as a double (floating point
  // error introduced).
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

TEST(TestTime, To_From_Duration)
{
  tf2::TimePoint t1 = tf2::get_now();

  std::vector<double> values = {
    -0.01, -0.2, -0.5, -0.7, -0.99, -5.7, -1000000, -123456789.123456789,
    0.01, 0.2, 0.5, 0.7, 0.99, 5.7, 10000000, 123456789.123456789,
    0.0,
  };

  for (double expected_diff_sec : values) {
    tf2::TimePoint t2 = tf2::timeFromSec(tf2::timeToSec(t1) + expected_diff_sec);

    // Check durationToSec.
    tf2::Duration duration1 = t2 - t1;

    // Approximate representation of the duration in seconds as a double (floating point
    // error introduced).
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
