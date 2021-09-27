// Copyright 2008, Willow Garage, Inc. All rights reserved.
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
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "tf2/time_cache.h"
#include "tf2/LinearMath/Quaternion.h"

std::vector<double> values;
unsigned int step = 0;

void seed_rand()
{
  values.clear();
  for (unsigned int i = 0; i < 2000; i++) {
    int pseudo_rand = static_cast<int>(std::floor(static_cast<double>(i * 3.141592653589793)));
    values.push_back(( pseudo_rand % 100) / 50.0 - 1.0);
  }
}

double get_rand()
{
  if (values.size() == 0) {throw std::runtime_error("you need to call seed_rand first");}
  if (step >= values.size()) {
    step = 0;
  } else {
    step++;
  }
  return values[step];
}

void setIdentity(tf2::TransformStorage & stor)
{
  stor.translation_.setValue(0.0, 0.0, 0.0);
  stor.rotation_.setValue(0.0, 0.0, 0.0, 1.0);
}

TEST(TimeCache, Repeatability)
{
  unsigned int runs = 100;

  tf2::TimeCache cache;

  tf2::TransformStorage stor;
  setIdentity(stor);

  for (uint64_t i = 1; i < runs; i++) {
    stor.frame_id_ = tf2::CompactFrameID(i);
    stor.stamp_ = tf2::TimePoint(std::chrono::nanoseconds(i));

    EXPECT_TRUE(cache.insertData(stor));
  }

  for (uint64_t i = 1; i < runs; i++) {
    EXPECT_TRUE(cache.getData(tf2::TimePoint(std::chrono::nanoseconds(i)), stor));
    EXPECT_EQ(stor.frame_id_, i);
    EXPECT_EQ(stor.stamp_, tf2::TimePoint(std::chrono::nanoseconds(i)));
  }
}

TEST(TimeCache, RepeatabilityReverseInsertOrder)
{
  unsigned int runs = 100;

  tf2::TimeCache cache;

  tf2::TransformStorage stor;
  setIdentity(stor);

  for (int i = runs - 1; i >= 0; i--) {
    stor.frame_id_ = i;
    stor.stamp_ = tf2::TimePoint(std::chrono::nanoseconds(i));

    EXPECT_TRUE(cache.insertData(stor));
  }
  for (uint64_t i = 1; i < runs; i++) {
    EXPECT_TRUE(cache.getData(tf2::TimePoint(std::chrono::nanoseconds(i)), stor));
    EXPECT_EQ(stor.frame_id_, i);
    EXPECT_EQ(stor.stamp_, tf2::TimePoint(std::chrono::nanoseconds(i)));
  }
}

TEST(TimeCache, ZeroAtFront)
{
  uint64_t runs = 100;

  tf2::TimeCache cache;

  tf2::TransformStorage stor;
  setIdentity(stor);

  for (uint64_t i = 1; i < runs; i++) {
    stor.frame_id_ = tf2::CompactFrameID(i);
    stor.stamp_ = tf2::TimePoint(std::chrono::nanoseconds(i));

    EXPECT_TRUE(cache.insertData(stor));
  }

  stor.frame_id_ = tf2::CompactFrameID(runs);
  stor.stamp_ = tf2::TimePoint(std::chrono::nanoseconds(runs));
  EXPECT_TRUE(cache.insertData(stor));

  for (uint64_t i = 1; i < runs; i++) {
    EXPECT_TRUE(cache.getData(tf2::TimePoint(std::chrono::nanoseconds(i)), stor));
    EXPECT_EQ(stor.frame_id_, i);
    EXPECT_EQ(stor.stamp_, tf2::TimePoint(std::chrono::nanoseconds(i)));
  }

  EXPECT_TRUE(cache.getData(tf2::TimePoint(), stor));
  EXPECT_EQ(stor.frame_id_, runs);
  EXPECT_EQ(stor.stamp_, tf2::TimePoint(std::chrono::nanoseconds(runs)));

  stor.frame_id_ = tf2::CompactFrameID(runs);
  stor.stamp_ = tf2::TimePoint(std::chrono::nanoseconds(runs + 1));
  EXPECT_TRUE(cache.insertData(stor));

  // Make sure we get a different value now that a new value is added at the front
  EXPECT_TRUE(cache.getData(tf2::TimePoint(), stor));
  EXPECT_EQ(stor.frame_id_, runs);
  EXPECT_EQ(stor.stamp_, tf2::TimePoint(std::chrono::nanoseconds(runs + 1)));
}

TEST(TimeCache, CartesianInterpolation)
{
  uint64_t runs = 100;
  double epsilon = 2e-6;
  seed_rand();

  tf2::TimeCache cache;
  std::vector<double> xvalues(2);
  std::vector<double> yvalues(2);
  std::vector<double> zvalues(2);

  uint64_t offset = 200;

  tf2::TransformStorage stor;
  setIdentity(stor);

  for (uint64_t i = 1; i < runs; i++) {
    for (uint64_t step = 0; step < 2; step++) {
      xvalues[step] = 10.0 * get_rand();
      yvalues[step] = 10.0 * get_rand();
      zvalues[step] = 10.0 * get_rand();

      stor.translation_.setValue(xvalues[step], yvalues[step], zvalues[step]);
      stor.frame_id_ = 2;
      stor.stamp_ = tf2::TimePoint(std::chrono::nanoseconds(step * 100 + offset));
      EXPECT_TRUE(cache.insertData(stor));
    }

    for (int pos = 0; pos < 100; pos++) {
      EXPECT_TRUE(cache.getData(tf2::TimePoint(std::chrono::nanoseconds(offset + pos)), stor));
      double x_out = stor.translation_.x();
      double y_out = stor.translation_.y();
      double z_out = stor.translation_.z();
      EXPECT_NEAR(xvalues[0] + (xvalues[1] - xvalues[0]) * (double)pos / 100.0, x_out, epsilon);
      EXPECT_NEAR(yvalues[0] + (yvalues[1] - yvalues[0]) * (double)pos / 100.0, y_out, epsilon);
      EXPECT_NEAR(zvalues[0] + (zvalues[1] - zvalues[0]) * (double)pos / 100.0, z_out, epsilon);
    }
    cache.clearList();
  }
}

/** \brief Make sure we dont' interpolate across reparented data */
TEST(TimeCache, ReparentingInterpolationProtection)
{
  double epsilon = 1e-6;
  uint64_t offset = 555;

  seed_rand();

  tf2::TimeCache cache;
  std::vector<double> xvalues(2);
  std::vector<double> yvalues(2);
  std::vector<double> zvalues(2);

  tf2::TransformStorage stor;
  setIdentity(stor);

  for (uint64_t step = 0; step < 2; step++) {
    xvalues[step] = 10.0 * get_rand();
    yvalues[step] = 10.0 * get_rand();
    zvalues[step] = 10.0 * get_rand();

    stor.translation_.setValue(xvalues[step], yvalues[step], zvalues[step]);
    stor.frame_id_ = tf2::CompactFrameID(step + 4);
    stor.stamp_ = tf2::TimePoint(std::chrono::nanoseconds(step * 100 + offset));
    EXPECT_TRUE(cache.insertData(stor));
  }

  for (int pos = 0; pos < 100; pos++) {
    EXPECT_TRUE(cache.getData(tf2::TimePoint(std::chrono::nanoseconds(offset + pos)), stor));
    double x_out = stor.translation_.x();
    double y_out = stor.translation_.y();
    double z_out = stor.translation_.z();
    EXPECT_NEAR(xvalues[0], x_out, epsilon);
    EXPECT_NEAR(yvalues[0], y_out, epsilon);
    EXPECT_NEAR(zvalues[0], z_out, epsilon);
  }
}

TEST(TimeCache, AngularInterpolation)
{
  uint64_t runs = 100;
  double epsilon = 1e-6;
  seed_rand();

  tf2::TimeCache cache;
  std::vector<double> yawvalues(2);
  std::vector<double> pitchvalues(2);
  std::vector<double> rollvalues(2);
  uint64_t offset = 200;

  std::vector<tf2::Quaternion> quats(2);

  tf2::TransformStorage stor;
  setIdentity(stor);

  for (uint64_t i = 1; i < runs; i++) {
    for (uint64_t step = 0; step < 2; step++) {
      yawvalues[step] = 10.0 * get_rand() / 100.0;
      pitchvalues[step] = 0;  // 10.0 * get_rand();
      rollvalues[step] = 0;  // 10.0 * get_rand();
      quats[step].setRPY(yawvalues[step], pitchvalues[step], rollvalues[step]);
      stor.rotation_ = quats[step];
      stor.frame_id_ = 3;
      // step = 0 or 1
      stor.stamp_ = tf2::TimePoint(std::chrono::nanoseconds(offset + (step * 100)));
      EXPECT_TRUE(cache.insertData(stor));
    }

    for (int pos = 0; pos < 100; pos++) {
      // get the transform for the position
      EXPECT_TRUE(cache.getData(tf2::TimePoint(std::chrono::nanoseconds(offset + pos)), stor));
      tf2::Quaternion quat(stor.rotation_);

      // Generate a ground truth quaternion directly calling slerp
      tf2::Quaternion ground_truth = quats[0].slerp(quats[1], pos / 100.0);

      // Make sure the transformed one and the direct call match
      EXPECT_NEAR(0, angle(ground_truth, quat), epsilon);
    }
    cache.clearList();
  }
}

TEST(TimeCache, DuplicateEntries)
{
  tf2::TimeCache cache;

  tf2::TransformStorage stor;
  setIdentity(stor);
  stor.frame_id_ = 3;
  stor.stamp_ = tf2::TimePoint(std::chrono::nanoseconds(1));

  EXPECT_TRUE(cache.insertData(stor));

  EXPECT_TRUE(cache.insertData(stor));

  EXPECT_TRUE(cache.getData(tf2::TimePoint(std::chrono::nanoseconds(1)), stor));

  EXPECT_TRUE(!std::isnan(stor.translation_.x()));
  EXPECT_TRUE(!std::isnan(stor.translation_.y()));
  EXPECT_TRUE(!std::isnan(stor.translation_.z()));
  EXPECT_TRUE(!std::isnan(stor.rotation_.x()));
  EXPECT_TRUE(!std::isnan(stor.rotation_.y()));
  EXPECT_TRUE(!std::isnan(stor.rotation_.z()));
  EXPECT_TRUE(!std::isnan(stor.rotation_.w()));
}

TEST(TimeCache, GetDataNotPresent)
{
  tf2::TimeCache cache;

  tf2::TransformStorage stor;

  EXPECT_FALSE(cache.getData(tf2::TimePoint(std::chrono::nanoseconds(1)), stor));
}

TEST(TimeCache, GetDataSingleEntry)
{
  tf2::TimeCache cache;

  tf2::TransformStorage stor;
  setIdentity(stor);
  stor.frame_id_ = 3;
  stor.stamp_ = tf2::TimePoint(std::chrono::nanoseconds(1));

  EXPECT_TRUE(cache.insertData(stor));

  tf2::TransformStorage fetch;
  EXPECT_TRUE(cache.getData(tf2::TimePoint(std::chrono::nanoseconds(1)), fetch));
}

TEST(TimeCache, GetDataSingleEntryNotExist)
{
  tf2::TimeCache cache;

  tf2::TransformStorage stor;
  setIdentity(stor);
  stor.frame_id_ = 3;
  stor.stamp_ = tf2::TimePoint(std::chrono::milliseconds(1));

  EXPECT_TRUE(cache.insertData(stor));

  tf2::TransformStorage fetch;
  std::string error;
  EXPECT_FALSE(cache.getData(tf2::TimePoint(std::chrono::milliseconds(2)), fetch, &error));
  EXPECT_EQ(error, "Lookup would require extrapolation at time 0.002000, but only time 0.001000 is in the buffer");
}

TEST(TimeCache, GetDataExtrapolationToFuture)
{
  tf2::TimeCache cache;

  tf2::TransformStorage stor1;
  setIdentity(stor1);
  stor1.frame_id_ = 3;
  stor1.stamp_ = tf2::TimePoint(std::chrono::milliseconds(1));
  EXPECT_TRUE(cache.insertData(stor1));

  tf2::TransformStorage stor2;
  setIdentity(stor2);
  stor2.frame_id_ = 3;
  stor2.stamp_ = tf2::TimePoint(std::chrono::milliseconds(2));
  EXPECT_TRUE(cache.insertData(stor2));

  tf2::TransformStorage fetch;
  std::string error;
  EXPECT_FALSE(cache.getData(tf2::TimePoint(std::chrono::milliseconds(3)), fetch, &error));
  EXPECT_EQ(error, "Lookup would require extrapolation into the future.  Requested time 0.003000 but the latest data is at time 0.002000");
}

TEST(TimeCache, GetDataExtrapolationToPast)
{
  tf2::TimeCache cache;

  tf2::TransformStorage stor1;
  setIdentity(stor1);
  stor1.frame_id_ = 3;
  stor1.stamp_ = tf2::TimePoint(std::chrono::milliseconds(4));
  EXPECT_TRUE(cache.insertData(stor1));

  tf2::TransformStorage stor2;
  setIdentity(stor2);
  stor2.frame_id_ = 3;
  stor2.stamp_ = tf2::TimePoint(std::chrono::milliseconds(5));
  EXPECT_TRUE(cache.insertData(stor2));

  tf2::TransformStorage fetch;
  std::string error;
  EXPECT_FALSE(cache.getData(tf2::TimePoint(std::chrono::milliseconds(2)), fetch, &error));
  EXPECT_EQ(error, "Lookup would require extrapolation into the past.  Requested time 0.002000 but the earliest data is at time 0.004000");
}

TEST(TimeCache, GetParentNotExist)
{
  tf2::TimeCache cache;

  std::string error;
  EXPECT_EQ(cache.getParent(tf2::TimePoint(std::chrono::milliseconds(1)), &error), 0u);
  EXPECT_EQ(error, "");
}

TEST(TimeCache, GetParent)
{
  tf2::TimeCache cache;

  tf2::TransformStorage stor1;
  setIdentity(stor1);
  stor1.frame_id_ = 3;
  stor1.stamp_ = tf2::TimePoint(std::chrono::milliseconds(4));
  EXPECT_TRUE(cache.insertData(stor1));

  std::string error;
  EXPECT_EQ(cache.getParent(tf2::TimePoint(std::chrono::milliseconds(4)), &error), 3u);
  EXPECT_EQ(error, "");
}

TEST(TimeCache, InsertDataTooOld)
{
  tf2::TimeCache cache;

  tf2::TransformStorage stor1;
  setIdentity(stor1);
  stor1.frame_id_ = 3;
  stor1.stamp_ = tf2::TimePoint(std::chrono::seconds(12));
  EXPECT_TRUE(cache.insertData(stor1));

  tf2::TransformStorage stor2;
  setIdentity(stor2);
  stor2.frame_id_ = 4;
  stor2.stamp_ = tf2::TimePoint(std::chrono::seconds(1));
  EXPECT_FALSE(cache.insertData(stor2));
}

TEST(TimeCache, ClearList)
{
  tf2::TimeCache cache;

  EXPECT_EQ(cache.getListLength(), 0u);

  tf2::TransformStorage stor1;
  setIdentity(stor1);
  stor1.frame_id_ = 3;
  stor1.stamp_ = tf2::TimePoint(std::chrono::seconds(12));
  EXPECT_TRUE(cache.insertData(stor1));

  EXPECT_EQ(cache.getListLength(), 1u);

  cache.clearList();

  EXPECT_EQ(cache.getListLength(), 0u);
}

TEST(TimeCache, GetLatestTimeAndParentEmpty)
{
  tf2::TimeCache cache;

  std::pair<tf2::TimePoint, tf2::CompactFrameID> tp = cache.getLatestTimeAndParent();
  EXPECT_EQ(tp.first, tf2::TimePointZero);
  EXPECT_EQ(tp.second, 0u);
}

TEST(TimeCache, GetLatestTimeAndParent)
{
  tf2::TimeCache cache;

  tf2::TransformStorage stor1;
  setIdentity(stor1);
  stor1.frame_id_ = 3;
  stor1.stamp_ = tf2::TimePoint(std::chrono::seconds(12));
  EXPECT_TRUE(cache.insertData(stor1));

  std::pair<tf2::TimePoint, tf2::CompactFrameID> tp = cache.getLatestTimeAndParent();
  EXPECT_EQ(tp.first, tf2::TimePoint(std::chrono::seconds(12)));
  EXPECT_EQ(tp.second, 3u);
}

TEST(TimeCache, GetLatestTimestampEmpty)
{
  tf2::TimeCache cache;

  tf2::TimePoint time = cache.getLatestTimestamp();
  EXPECT_EQ(time, tf2::TimePointZero);
}

TEST(TimeCache, GetLatestTimestamp)
{
  tf2::TimeCache cache;

  tf2::TransformStorage stor1;
  setIdentity(stor1);
  stor1.frame_id_ = 3;
  stor1.stamp_ = tf2::TimePoint(std::chrono::seconds(12));
  EXPECT_TRUE(cache.insertData(stor1));

  tf2::TimePoint time = cache.getLatestTimestamp();
  EXPECT_EQ(time, tf2::TimePoint(std::chrono::seconds(12)));
}

TEST(TimeCache, GetOldestTimestampEmpty)
{
  tf2::TimeCache cache;

  tf2::TimePoint time = cache.getOldestTimestamp();
  EXPECT_EQ(time, tf2::TimePointZero);
}

TEST(TimeCache, GetOldestTimestamp)
{
  tf2::TimeCache cache;

  tf2::TransformStorage stor1;
  setIdentity(stor1);
  stor1.frame_id_ = 3;
  stor1.stamp_ = tf2::TimePoint(std::chrono::seconds(12));
  EXPECT_TRUE(cache.insertData(stor1));

  tf2::TimePoint time = cache.getOldestTimestamp();
  EXPECT_EQ(time, tf2::TimePoint(std::chrono::seconds(12)));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
