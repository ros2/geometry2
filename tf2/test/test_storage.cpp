// Copyright (c) 2024 Dexory

#include <gtest/gtest.h>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/time.h"
#include "tf2/transform_storage.h"

class TransformStorageTest : public ::testing::Test
{
protected:
  tf2::TransformStorage createTransformStorage()
  {
    const tf2::CompactFrameID frame_id(0);
    const tf2::CompactFrameID child_frame_id(1);
    const tf2::TimePoint stamp(tf2::TimePointZero);
    const tf2::Quaternion rotation(0.0, 0.0, 0.0, 1.0);
    const tf2::Vector3 translation(0.0, 0.0, 0.0);
    return tf2::TransformStorage(stamp, rotation, translation, frame_id, child_frame_id);
  }
};

TEST_F(TransformStorageTest, EqualityOperator) {
  // Create a dummy storage, set to identity
  tf2::TransformStorage transformStorage1 = createTransformStorage();

  // tf2::Quaternion rotation_;
  {
    tf2::TransformStorage transformStorage2 = createTransformStorage();
    ASSERT_TRUE(transformStorage1 == transformStorage2);
    transformStorage2.rotation_.setValue(1.0, 0.0, 0.0, 0.0);
    ASSERT_FALSE(transformStorage1 == transformStorage2);
  }
  // tf2::Vector3 translation_;
  {
    tf2::TransformStorage transformStorage2 = createTransformStorage();
    ASSERT_TRUE(transformStorage1 == transformStorage2);
    transformStorage2.translation_.setValue(1.0, 0.0, 0.0);
    ASSERT_FALSE(transformStorage1 == transformStorage2);
  }
  // TimePoint stamp_;
  {
    tf2::TransformStorage transformStorage2 = createTransformStorage();
    ASSERT_TRUE(transformStorage1 == transformStorage2);
    transformStorage2.stamp_ = tf2::TimePoint(tf2::durationFromSec(1.0));
    ASSERT_FALSE(transformStorage1 == transformStorage2);
  }
  // CompactFrameID frame_id_;
  {
    tf2::TransformStorage transformStorage2 = createTransformStorage();
    ASSERT_TRUE(transformStorage1 == transformStorage2);
    transformStorage2.frame_id_ = 55;
    ASSERT_FALSE(transformStorage1 == transformStorage2);
  }
  // CompactFrameID child_frame_id_;
  {
    tf2::TransformStorage transformStorage2 = createTransformStorage();
    ASSERT_TRUE(transformStorage1 == transformStorage2);
    transformStorage2.translation_.setValue(1.0, 0.0, 0.0);
    ASSERT_FALSE(transformStorage1 == transformStorage2);
  }
}

TEST_F(TransformStorageTest, InequalityOperator) {
  // Create a dummy storage, set to identity
  tf2::TransformStorage transformStorage1 = createTransformStorage();

  // tf2::Quaternion rotation_;
  {
    tf2::TransformStorage transformStorage2 = createTransformStorage();
    ASSERT_TRUE(transformStorage1 == transformStorage2);
    transformStorage2.rotation_.setValue(1.0, 0.0, 0.0, 0.0);
    ASSERT_TRUE(transformStorage1 != transformStorage2);
  }
  // tf2::Vector3 translation_;
  {
    tf2::TransformStorage transformStorage2 = createTransformStorage();
    ASSERT_TRUE(transformStorage1 == transformStorage2);
    transformStorage2.translation_.setValue(1.0, 0.0, 0.0);
    ASSERT_TRUE(transformStorage1 != transformStorage2);
  }
  // TimePoint stamp_;
  {
    tf2::TransformStorage transformStorage2 = createTransformStorage();
    ASSERT_TRUE(transformStorage1 == transformStorage2);
    transformStorage2.stamp_ = tf2::TimePoint(tf2::durationFromSec(1.0));
    ASSERT_TRUE(transformStorage1 != transformStorage2);
  }
  // CompactFrameID frame_id_;
  {
    tf2::TransformStorage transformStorage2 = createTransformStorage();
    ASSERT_TRUE(transformStorage1 == transformStorage2);
    transformStorage2.frame_id_ = 55;
    ASSERT_TRUE(transformStorage1 != transformStorage2);
  }
  // CompactFrameID child_frame_id_;
  {
    tf2::TransformStorage transformStorage2 = createTransformStorage();
    ASSERT_TRUE(transformStorage1 == transformStorage2);
    transformStorage2.translation_.setValue(1.0, 0.0, 0.0);
    ASSERT_TRUE(transformStorage1 != transformStorage2);
  }
}
