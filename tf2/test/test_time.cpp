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

#include <chrono>
#include <gtest/gtest.h>
#include "tf2/time.h"

using namespace std::literals::chrono_literals;

TEST(TestTime, durationFromSec) {
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

TEST(TestTime, timeFromSec) {
  auto time_point = tf2::TimePoint();
  EXPECT_EQ(tf2::timeFromSec(0.0), time_point);
  EXPECT_EQ(tf2::timeFromSec(1.0), time_point + 1s);
  EXPECT_EQ(tf2::timeFromSec(-1.0), time_point - 1s);
  EXPECT_EQ(tf2::timeFromSec(0.0), time_point);
  EXPECT_EQ(tf2::timeFromSec(1e-9), time_point + 1ns);
  EXPECT_EQ(tf2::timeFromSec(-1e-9), time_point - 1ns);
}

TEST(TestTime, durationToSec) {
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

TEST(TestTime, timeToSec) {
  auto time_point = tf2::TimePoint();
  EXPECT_EQ(tf2::timeToSec(time_point), 0.0);
  EXPECT_EQ(tf2::timeToSec(time_point + 1s), 1.0);
  EXPECT_EQ(tf2::timeToSec(time_point - 1s), -1.0);
  EXPECT_EQ(tf2::timeToSec(time_point + 1ns), 0.000000001);
}

TEST(TestTime, displayTimePoint) {
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
}
