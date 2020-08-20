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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_kdl/tf2_kdl.h>

double epsilon = 1e-9;

template<typename T>
void yprTest(const T& t, double yaw1, double pitch1, double roll1) {
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
    q.x = x; q.y =y; q.z = z; q.w = w;
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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
