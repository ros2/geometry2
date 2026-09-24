// Copyright 2023, Your Name. All rights reserved.
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

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/buffer_core.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"

TEST(tf2_chainAsVector, ValidChain)
{
  tf2::BufferCore buffer;

  // Populate buffer with transforms: C -> B -> A (fixed) and D -> A
  geometry_msgs::msg::TransformStamped transform;

  // A -> B
  transform.header.stamp.sec = 1;
  transform.header.frame_id = "A";
  transform.child_frame_id = "B";
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(buffer.setTransform(transform, "test"));

  // B -> C
  transform.header.frame_id = "B";
  transform.child_frame_id = "C";
  ASSERT_TRUE(buffer.setTransform(transform, "test"));

  // A -> D
  transform.header.frame_id = "A";
  transform.child_frame_id = "D";
  ASSERT_TRUE(buffer.setTransform(transform, "test"));

  // Call _chainAsVector for source=C, target=D, fixed=A
  std::vector<std::string> output;
  buffer._chainAsVector(
      "D", tf2::timeFromSec(1.0), // target
      "C", tf2::timeFromSec(1.0), // source
      "A",                        // fixed
      output);

  // Verify the resulting chain
  std::vector<std::string> expected{"C", "B", "A", "D"};
  EXPECT_EQ(expected, output);
}

TEST(tf2_chainAsVector, LookupError_SourceFrameNotFound)
{
  tf2::BufferCore buffer;
  std::vector<std::string> output;

  // Attempt to get chain with non-existent source frame
  EXPECT_THROW(
      buffer._chainAsVector(
          "target", tf2::timeFromSec(1.0),
          "non_existent", tf2::timeFromSec(1.0),
          "fixed",
          output),
      tf2::LookupException);
}

TEST(tf2_chainAsVector, ConnectivityError)
{
  tf2::BufferCore buffer;

  // Add isolated transform C -> D (no connection to fixed frame A)
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp.sec = 1;
  transform.header.frame_id = "C";
  transform.child_frame_id = "D";
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(buffer.setTransform(transform, "test"));

  std::vector<std::string> output;
  // Attempt to get chain from D to A (fixed) which is disconnected
  EXPECT_THROW(
      buffer._chainAsVector(
          "A", tf2::timeFromSec(1.0),
          "D", tf2::timeFromSec(1.0),
          "A",
          output),
      tf2::ConnectivityException);
}

TEST(tf2_chainAsVector, ExtrapolationError)
{
  tf2::BufferCore buffer;

  // Add transform with timestamp at 2 seconds
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp.sec = 2;
  transform.header.frame_id = "A";
  transform.child_frame_id = "B";
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(buffer.setTransform(transform, "test"));

  std::vector<std::string> output;
  // Request transform at 1 second (before available time)
  EXPECT_THROW(
      buffer._chainAsVector(
          "B", tf2::timeFromSec(1.0),
          "A", tf2::timeFromSec(1.0),
          "A",
          output),
      tf2::ExtrapolationException);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}