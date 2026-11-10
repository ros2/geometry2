// Copyright 2014 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <rclcpp/utilities.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/utils.hpp>
#include <tf2_kdl/tf2_kdl.hpp>

double epsilon = 1e-9;

template<typename T>
void yprTest(const T & t, double yaw1, double pitch1, double roll1)
{
  double yaw2, pitch2, roll2;

  tf2::getEulerYPR(t, yaw2, pitch2, roll2);

  EXPECT_NEAR(yaw1, yaw2, epsilon);
  EXPECT_NEAR(pitch1, pitch2, epsilon);
  EXPECT_NEAR(roll1, roll2, epsilon);
  EXPECT_NEAR(tf2::getYaw(t), yaw1, epsilon);
}

TEST(tf2Utils, yaw)
{
  double x, y, z, w;
  x = 0.4;
  y = 0.5;
  z = 0.6;
  w = 0.7;

  double yaw1, pitch1, roll1;
  // Compute results one way with KDL
  KDL::Rotation::Quaternion(x, y, z, w).GetRPY(roll1, pitch1, yaw1);
  {
    // geometry_msgs::msg::Quaternion
    geometry_msgs::msg::Quaternion q;
    q.x = x; q.y = y; q.z = z; q.w = w;
    yprTest(q, yaw1, pitch1, roll1);

    // geometry_msgs::msg::QuaternionStamped
    geometry_msgs::msg::QuaternionStamped qst;
    qst.quaternion = q;
    yprTest(qst, yaw1, pitch1, roll1);
  }


  {
    // tf2::Quaternion
    tf2::Quaternion q(x, y, z, w);
    yprTest(q, yaw1, pitch1, roll1);

    // TODO (ahcorde): This PR fix this issue https://github.com/ros/geometry2/pull/357
    // // tf2::Stamped<tf2::Quaternion>
    // tf2::Stamped<tf2::Quaternion> sq;
    // sq.setData(q);
    // yprTest(sq, yaw1, pitch1, roll1);
  }
}

// Test fixture for rpyConversion tests
class RpyConversionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    yaw = 0.0;
    pitch = 0.0;
    roll = 0.0;
  }

  tf2::Quaternion q_input;
  double yaw, pitch, roll;
  const double test_epsilon = 1e-6;
};

TEST_F(RpyConversionTest, singularityOnlyGimbalLockPositive)
{
  // Test gimbal lock at exactly only positive 90 degrees pitch
  q_input.setRPY(0.0, TF2SIMD_HALF_PI, 0.0);  // Roll=0.0, Pitch=90°, Yaw=0.0
  
  tf2::getEulerYPR(q_input, yaw, pitch, roll);
  EXPECT_NEAR(roll, 0.0, test_epsilon);
  EXPECT_NEAR(pitch, TF2SIMD_HALF_PI, test_epsilon);
  EXPECT_NEAR(yaw, 0.0, test_epsilon);
  EXPECT_NEAR(tf2::getYaw(q_input), yaw, test_epsilon);

  // Compare with Matrix3x3 equivalent getEulerYPR
  tf2::Matrix3x3 m_pos_gimbal_exact(q_input);
  double roll_m, pitch_m, yaw_m;
  m_pos_gimbal_exact.getEulerYPR(yaw_m, pitch_m, roll_m);
  EXPECT_NEAR(roll_m, roll, test_epsilon);
  EXPECT_NEAR(pitch_m, pitch, test_epsilon);
  EXPECT_NEAR(yaw_m, yaw, test_epsilon);
}

TEST_F(RpyConversionTest, singularityOnlyGimbalLockNegative)
{
  // Test gimbal lock at exactly only negative 90 degrees pitch
  q_input.setRPY(0.0, -TF2SIMD_HALF_PI, 0.0);  // Roll=0.0, Pitch=-90°, Yaw=0.0
  
  tf2::getEulerYPR(q_input, yaw, pitch, roll);
  EXPECT_NEAR(roll, 0.0, test_epsilon);
  EXPECT_NEAR(pitch, -TF2SIMD_HALF_PI, test_epsilon);
  EXPECT_NEAR(yaw, 0.0, test_epsilon);
  EXPECT_NEAR(tf2::getYaw(q_input), yaw, test_epsilon);

  // Compare with Matrix3x3 equivalent getEulerYPR
  tf2::Matrix3x3 m_pos_gimbal_exact(q_input);
  double roll_m, pitch_m, yaw_m;
  m_pos_gimbal_exact.getEulerYPR(yaw_m, pitch_m, roll_m);
  EXPECT_NEAR(roll_m, roll, test_epsilon);
  EXPECT_NEAR(pitch_m, pitch, test_epsilon);
  EXPECT_NEAR(yaw_m, yaw, test_epsilon);
}

TEST_F(RpyConversionTest, singularityGimbalLockWithRollYawPositive)
{
  // Test gimbal lock at positive 90 degrees pitch including non-zero roll and yaw
  q_input.setRPY(tf2Radians(40.0), TF2SIMD_HALF_PI, tf2Radians(50.0));  // Roll=40°, Pitch=90°, Yaw=50°
  
  tf2::getEulerYPR(q_input, yaw, pitch, roll);
  EXPECT_NEAR(pitch, TF2SIMD_HALF_PI, test_epsilon);
  // At gimbal lock, roll and yaw combine - exact values depend on implementation
  EXPECT_TRUE(std::isfinite(yaw) && std::isfinite(roll));
  // Below values are expected according to #796
  // Note that for the gimbal lock case yaw is expected to be zero and that rotation is only represented by roll
  EXPECT_NEAR(roll, tf2Radians(-10.0), test_epsilon);  // roll - yaw
  EXPECT_NEAR(pitch, TF2SIMD_HALF_PI, test_epsilon);
  EXPECT_NEAR(yaw, 0.0, test_epsilon);
  EXPECT_NEAR(tf2::getYaw(q_input), yaw, test_epsilon);

  // Compare with Matrix3x3 equivalent getEulerYPR
  tf2::Matrix3x3 m_pos_gimbal_exact(q_input);
  double roll_m, pitch_m, yaw_m;
  m_pos_gimbal_exact.getEulerYPR(yaw_m, pitch_m, roll_m);
  EXPECT_NEAR(roll_m, roll, test_epsilon);
  EXPECT_NEAR(pitch_m, pitch, test_epsilon);
  EXPECT_NEAR(yaw_m, yaw, test_epsilon);
}

TEST_F(RpyConversionTest, singularityGimbalLockWithRollYawNegative)
{
  // Test gimbal lock at negative 90 degrees pitch including non-zero roll and yaw
  q_input.setRPY(tf2Radians(40.0), -TF2SIMD_HALF_PI, tf2Radians(50.0));  // Roll=40°, Pitch=-90°, Yaw=50°
  
  tf2::getEulerYPR(q_input, yaw, pitch, roll);
  EXPECT_NEAR(pitch, -TF2SIMD_HALF_PI, test_epsilon);
  // At gimbal lock, roll and yaw combine - exact values depend on implementation
  EXPECT_TRUE(std::isfinite(yaw) && std::isfinite(roll));
  // Below values are expected according to #796
  // Note that for the gimbal lock case yaw is expected to be zero and that rotation is only represented by roll
  EXPECT_NEAR(roll, tf2Radians(90.0), test_epsilon);  // roll + yaw combined
  EXPECT_NEAR(pitch, -TF2SIMD_HALF_PI, test_epsilon);
  EXPECT_NEAR(yaw, 0.0, test_epsilon);
  EXPECT_NEAR(tf2::getYaw(q_input), yaw, test_epsilon);

  // Compare with Matrix3x3 equivalent getEulerYPR
  tf2::Matrix3x3 m_pos_gimbal_exact(q_input);
  double roll_m, pitch_m, yaw_m;
  m_pos_gimbal_exact.getEulerYPR(yaw_m, pitch_m, roll_m);
  EXPECT_NEAR(roll_m, roll, test_epsilon);
  EXPECT_NEAR(pitch_m, pitch, test_epsilon);
  EXPECT_NEAR(yaw_m, yaw, test_epsilon);
}

TEST_F(RpyConversionTest, singularityBelowGimbalLockWithRollYaw)
{
  // Test near-gimbal lock cases (just under threshold)
  q_input.setRPY(0.1, TF2SIMD_HALF_PI - 1e-5, 0.2);
  
  tf2::getEulerYPR(q_input, yaw, pitch, roll);
  EXPECT_NEAR(roll, 0.1, test_epsilon);
  EXPECT_NEAR(pitch, TF2SIMD_HALF_PI - 1e-5, test_epsilon);
  EXPECT_NEAR(yaw, 0.2, test_epsilon);
  EXPECT_NEAR(tf2::getYaw(q_input), yaw, test_epsilon);

  // Compare with Matrix3x3 equivalent getEulerYPR
  tf2::Matrix3x3 m_pos_gimbal_exact(q_input);
  double roll_m, pitch_m, yaw_m;
  m_pos_gimbal_exact.getEulerYPR(yaw_m, pitch_m, roll_m);
  EXPECT_NEAR(roll_m, roll, test_epsilon);
  EXPECT_NEAR(pitch_m, pitch, test_epsilon);
  EXPECT_NEAR(yaw_m, yaw, test_epsilon);
}

TEST_F(RpyConversionTest, smallAngles)
{
  // Test quaternions that should produce very small angles (near epsilon threshold)
  tf2::Quaternion q_small_angles;
  q_small_angles.setRPY(1e-9, 1e-9, 1e-9);  // Very small rotations
  
  tf2::getEulerYPR(q_small_angles, yaw, pitch, roll);
  EXPECT_NEAR(yaw, 0.0, test_epsilon);
  EXPECT_NEAR(pitch, 0.0, test_epsilon);
  EXPECT_NEAR(roll, 0.0, test_epsilon);
  EXPECT_NEAR(tf2::getYaw(q_small_angles), yaw, test_epsilon);

  // Compare with Matrix3x3 equivalent getEulerYPR
  tf2::Matrix3x3 m_pos_gimbal_exact(q_small_angles);
  double roll_m, pitch_m, yaw_m;
  m_pos_gimbal_exact.getEulerYPR(yaw_m, pitch_m, roll_m);
  EXPECT_NEAR(roll_m, roll, test_epsilon);
  EXPECT_NEAR(pitch_m, pitch, test_epsilon);
  EXPECT_NEAR(yaw_m, yaw, test_epsilon);
}

TEST(tf2Utils, identity)
{
  geometry_msgs::msg::Transform t;
  t.translation.x = 0.1;
  t.translation.y = 0.2;
  t.translation.z = 0.3;
  t.rotation.x = 0.4;
  t.rotation.y = 0.5;
  t.rotation.z = 0.6;
  t.rotation.w = 0.7;

  // Test identity
  t = tf2::getTransformIdentity<geometry_msgs::msg::Transform>();

  EXPECT_EQ(t.translation.x, 0);
  EXPECT_EQ(t.translation.y, 0);
  EXPECT_EQ(t.translation.z, 0);
  EXPECT_EQ(t.rotation.x, 0);
  EXPECT_EQ(t.rotation.y, 0);
  EXPECT_EQ(t.rotation.z, 0);
  EXPECT_EQ(t.rotation.w, 1);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
