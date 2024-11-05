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
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "tf2/time_cache.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

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

class TimeCacheInternal : public tf2::TimeCache
{
public:
  using tf2::TimeCache::TimeCache;
  using tf2::TimeCache::getAllItems;
};

// Shorthand for making incomplete but unique transforms.
tf2::TransformStorage makeItem(uint32_t nanosec, uint32_t frame_id)
{
  tf2::TransformStorage stor{};
  stor.frame_id_ = frame_id;
  stor.stamp_ = tf2::TimePoint(std::chrono::nanoseconds(nanosec));
  // Initialize remaining elements.
  stor.child_frame_id_ = 0;
  setIdentity(stor);
  return stor;
}

std::string toMakeItemString(const tf2::TransformStorage & item)
{
  std::stringstream out;
  std::chrono::duration nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(
    item.stamp_.time_since_epoch());
  out << "makeItem(" << nanosec.count() << ", " << item.frame_id_ << ")";
  return out.str();
}

// Reformat list for ease of debugging pursuant to this test point.
std::string listToMakeItemStrings(const std::list<tf2::TransformStorage> & storage)
{
  std::stringstream out;
  out << "{\n";
  for (auto & item : storage) {
    out << "  " << toMakeItemString(item) << ",\n";
  }
  out << "}";
  return out.str();
}

// Tests the behavior of sorting and pruning relevant to the implementation for
// performance concerns. Certain details may change as the implementation
// evolves.
TEST(TimeCache, GetAllItems)
{
  tf2::Duration max_storage_time(std::chrono::nanoseconds(10));
  TimeCacheInternal cache(max_storage_time);

  const auto item_a = makeItem(0, 0);
  const auto item_b = makeItem(10, 1);
  const auto item_c = makeItem(5, 2);
  const auto item_d = makeItem(3, 3);
  // Same timestamp, different id.
  const auto item_e = makeItem(8, 4);
  const auto item_f = makeItem(8, 5);
  const auto item_g = makeItem(8, 6);

  // Insert in order.
  cache.insertData(item_a);
  cache.insertData(item_b);
  cache.insertData(item_c);
  cache.insertData(item_d);
  cache.insertData(item_e);
  cache.insertData(item_f);
  cache.insertData(item_g);

  // Note that the difference between the oldest and newest timestamp is exactly equal
  // to the max storage duration.
  EXPECT_EQ(
      cache.getLatestTimestamp() - cache.getOldestTimestamp(),
      max_storage_time);

  // Expect that storage is descending.
  const std::list<tf2::TransformStorage> & storage_expected{
    item_b,
    // Same timestamps have effectively reversed insertion order.
    item_g,
    item_f,
    item_e,
    // Remaining are in descending order.
    item_c,
    item_d,
    item_a};

  const std::list<tf2::TransformStorage> storage = cache.getAllItems();
  EXPECT_EQ(storage, storage_expected)
    << "storage: " << listToMakeItemStrings(storage) << "\n"
    << "storage_expected: " << listToMakeItemStrings(storage_expected) << "\n";

  // Insert repeated, in reverse. Nothing should change.
  cache.insertData(item_g);
  cache.insertData(item_f);
  cache.insertData(item_e);
  cache.insertData(item_d);
  cache.insertData(item_c);
  cache.insertData(item_b);
  cache.insertData(item_a);

  const std::list<tf2::TransformStorage> storage_repeat = cache.getAllItems();
  EXPECT_EQ(storage_repeat, storage_expected)
    << "storage_repeat: " << listToMakeItemStrings(storage_repeat) << "\n"
    << "storage_expected: " << listToMakeItemStrings(storage_expected) << "\n";

  // Insert newer data, and expect stale data to be pruned, even if newly inserted.
  const auto item_h = makeItem(15, 7);
  const auto item_i = makeItem(0, 8);  // This will be dropped.
  const auto item_j = makeItem(5, 9);

  cache.insertData(item_h);
  cache.insertData(item_i);
  cache.insertData(item_j);

  const std::list<tf2::TransformStorage> & storage_expected_new{
    item_h,
    item_b,
    item_g,
    item_f,
    item_e,
    item_j,
    item_c};
  // item_a, item_d, and item_i are pruned.
  const std::list<tf2::TransformStorage> storage_new = cache.getAllItems();
  EXPECT_EQ(storage_new, storage_expected_new)
    << "storage_new: " << listToMakeItemStrings(storage_new) << "\n"
    << "storage_expected_new: " << listToMakeItemStrings(storage_expected_new) << "\n";
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

    cache.insertData(stor);
  }
  EXPECT_EQ(cache.getListLength(), runs - 1);

  for (uint64_t i = 1; i < runs; i++) {
    cache.getData(tf2::TimePoint(std::chrono::nanoseconds(i)), stor);
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

    cache.insertData(stor);
  }
  EXPECT_EQ(cache.getListLength(), runs);
  for (uint64_t i = 1; i < runs; i++) {
    cache.getData(tf2::TimePoint(std::chrono::nanoseconds(i)), stor);
    EXPECT_EQ(stor.frame_id_, i);
    EXPECT_EQ(stor.stamp_, tf2::TimePoint(std::chrono::nanoseconds(i)));
  }
}

TEST(TimeCache, RepeatedElements)
{
  constexpr uint64_t runs = 100;

  tf2::TimeCache cache;
  EXPECT_EQ(cache.getListLength(), 0);

  tf2::TransformStorage stor;
  setIdentity(stor);
  stor.frame_id_ = tf2::CompactFrameID(0);
  stor.stamp_ = tf2::TimePoint(std::chrono::nanoseconds(0));

  // Attempt to insert the same element 100 times
  for (uint64_t i = 1; i < runs; ++i) {
    cache.insertData(stor);
  }

  // Even after 100 insertions, there should be one unique element in the internal list
  EXPECT_EQ(cache.getListLength(), 1);
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

    cache.insertData(stor);
  }

  stor.frame_id_ = tf2::CompactFrameID(runs);
  stor.stamp_ = tf2::TimePoint(std::chrono::nanoseconds(runs));
  cache.insertData(stor);

  for (uint64_t i = 1; i < runs; i++) {
    cache.getData(tf2::TimePoint(std::chrono::nanoseconds(i)), stor);
    EXPECT_EQ(stor.frame_id_, i);
    EXPECT_EQ(stor.stamp_, tf2::TimePoint(std::chrono::nanoseconds(i)));
  }

  cache.getData(tf2::TimePoint(), stor);
  EXPECT_EQ(stor.frame_id_, runs);
  EXPECT_EQ(stor.stamp_, tf2::TimePoint(std::chrono::nanoseconds(runs)));

  stor.frame_id_ = tf2::CompactFrameID(runs);
  stor.stamp_ = tf2::TimePoint(std::chrono::nanoseconds(runs + 1));
  cache.insertData(stor);

  // Make sure we get a different value now that a new values is added at the front
  cache.getData(tf2::TimePoint(), stor);
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
      cache.insertData(stor);
    }

    for (int pos = 0; pos < 100; pos++) {
      cache.getData(tf2::TimePoint(std::chrono::nanoseconds(offset + pos)), stor);
      double x_out = stor.translation_.x();
      double y_out = stor.translation_.y();
      double z_out = stor.translation_.z();
      //      printf("pose %d, %f %f %f, expected %f %f %f\n", pos, x_out, y_out, z_out,
      //       xvalues[0] + (xvalues[1] - xvalues[0]) * (double)pos/100.,
      //       yvalues[0] + (yvalues[1] - yvalues[0]) * (double)pos/100.0,
      //       zvalues[0] + (xvalues[1] - zvalues[0]) * (double)pos/100.0);
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
    cache.insertData(stor);
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

TEST(Bullet, Slerp)
{
  uint64_t runs = 100;
  seed_rand();

  tf2::Quaternion q1, q2;
  q1.setEuler(0, 0, 0);

  for (uint64_t i = 0; i < runs; i++) {
    q2.setEuler(
      1.0 * get_rand(),
      1.0 * get_rand(),
      1.0 * get_rand());
    tf2::Quaternion q3 = slerp(q1, q2, 0.5);

    EXPECT_NEAR(q3.angle(q1), q2.angle(q3), 1e-5);
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
      cache.insertData(stor);
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

  cache.insertData(stor);
  EXPECT_EQ(cache.getListLength(), 1);

  cache.insertData(stor);
  // Exact repeated element, should not grow in length.
  EXPECT_EQ(cache.getListLength(), 1);

  cache.getData(tf2::TimePoint(std::chrono::nanoseconds(1)), stor);

  EXPECT_TRUE(!std::isnan(stor.translation_.x()));
  EXPECT_TRUE(!std::isnan(stor.translation_.y()));
  EXPECT_TRUE(!std::isnan(stor.translation_.z()));
  EXPECT_TRUE(!std::isnan(stor.rotation_.x()));
  EXPECT_TRUE(!std::isnan(stor.rotation_.y()));
  EXPECT_TRUE(!std::isnan(stor.rotation_.z()));
  EXPECT_TRUE(!std::isnan(stor.rotation_.w()));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
