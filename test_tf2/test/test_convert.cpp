/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <gtest/gtest.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_kdl/tf2_kdl.hpp>
#include <tf2_bullet/tf2_bullet.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Geometry>

TEST(tf2Convert, kdlToBullet)
{
  double epsilon = 1e-9;

  tf2::Stamped<btVector3> b(btVector3(1, 2, 3), tf2::timeFromSec(0), "my_frame");

  tf2::Stamped<btVector3> b1 = b;
  tf2::Stamped<KDL::Vector> k1;
  tf2::convert(b1, k1);

  tf2::Stamped<btVector3> b2;
  tf2::convert(k1, b2);

  EXPECT_EQ(b.frame_id_, b2.frame_id_);
  EXPECT_NEAR(tf2::timeToSec(b.stamp_), tf2::timeToSec(b2.stamp_), epsilon);
  EXPECT_NEAR(b.x(), b2.x(), epsilon);
  EXPECT_NEAR(b.y(), b2.y(), epsilon);
  EXPECT_NEAR(b.z(), b2.z(), epsilon);


  EXPECT_EQ(b1.frame_id_, b2.frame_id_);
  EXPECT_NEAR(tf2::timeToSec(b1.stamp_), tf2::timeToSec(b2.stamp_), epsilon);
  EXPECT_NEAR(b1.x(), b2.x(), epsilon);
  EXPECT_NEAR(b1.y(), b2.y(), epsilon);
  EXPECT_NEAR(b1.z(), b2.z(), epsilon);
}

TEST(tf2Convert, kdlBulletROSConversions)
{
  double epsilon = 1e-9;

  tf2::Stamped<btVector3> b1(btVector3(1, 2, 3), tf2::timeFromSec(0), "my_frame"), b2, b3, b4;
  geometry_msgs::msg::PointStamped r1, r2, r3;
  tf2::Stamped<KDL::Vector> k1, k2, k3;

  // Do bullet -> self -> bullet -> KDL -> self -> KDL -> ROS -> self
  //   -> ROS -> KDL -> bullet -> ROS -> bullet
  tf2::convert(b1, b1);
  tf2::convert(b1, b2);
  tf2::convert(b2, k1);
  tf2::convert(k1, k1);
  tf2::convert(k1, k2);
  tf2::convert(k2, r1);
  tf2::convert(r1, r1);
  tf2::convert(r1, r2);
  tf2::convert(r2, k3);
  tf2::convert(k3, b3);
  tf2::convert(b3, r3);
  tf2::convert(r3, b4);

  EXPECT_EQ(b1.frame_id_, b4.frame_id_);
  EXPECT_NEAR(tf2::timeToSec(b1.stamp_), tf2::timeToSec(b4.stamp_), epsilon);
  EXPECT_NEAR(b1.x(), b4.x(), epsilon);
  EXPECT_NEAR(b1.y(), b4.y(), epsilon);
  EXPECT_NEAR(b1.z(), b4.z(), epsilon);
}

TEST(tf2Convert, ConvertTf2Quaternion)
{
  double epsilon = 1e-9;

  tf2::Quaternion tq(1, 2, 3, 4);
  tq.normalize();
  Eigen::Quaterniond eq;
  // TODO(gleichdick): switch to tf2::convert() when it's working
  tf2::fromMsg(tf2::toMsg(tq), eq);

  EXPECT_NEAR(tq.w(), eq.w(), epsilon);
  EXPECT_NEAR(tq.x(), eq.x(), epsilon);
  EXPECT_NEAR(tq.y(), eq.y(), epsilon);
  EXPECT_NEAR(tq.z(), eq.z(), epsilon);
}

TEST(tf2Convert, PointVectorDefaultMessagetype)
{
  // Verify the return type of `toMsg()`
  // as it can return a Vector3 or a Point for certain datatypes
  {
    // Bullet
    const tf2::Stamped<btVector3> b1{btVector3{1.0, 3.0, 4.0}, tf2::TimePoint(), "my_frame"};
    const geometry_msgs::msg::PointStamped msg = tf2::toMsg(b1);

    EXPECT_EQ(msg.point.x, 1.0);
    EXPECT_EQ(msg.point.y, 3.0);
    EXPECT_EQ(msg.point.z, 4.0);
    EXPECT_EQ(msg.header.frame_id, b1.frame_id_);
  }
  {
    // Eigen
    const Eigen::Vector3d e1{2.0, 4.0, 5.0};
    const geometry_msgs::msg::Point msg = tf2::toMsg(e1);

    EXPECT_EQ(msg.x, 2.0);
    EXPECT_EQ(msg.y, 4.0);
    EXPECT_EQ(msg.z, 5.0);
  }
  {
    // tf2
    const tf2::Vector3 t1{2.0, 4.0, 5.0};
    const geometry_msgs::msg::Vector3 msg = tf2::toMsg(t1);

    EXPECT_EQ(msg.x, 2.0);
    EXPECT_EQ(msg.y, 4.0);
    EXPECT_EQ(msg.z, 5.0);
  }
  {
    // KDL
    const tf2::Stamped<KDL::Vector> k1{KDL::Vector{1.0, 3.0, 4.0}, tf2::TimePoint(), "my_frame"};
    const geometry_msgs::msg::PointStamped msg = tf2::toMsg(k1);

    EXPECT_EQ(msg.point.x, 1.0);
    EXPECT_EQ(msg.point.y, 3.0);
    EXPECT_EQ(msg.point.z, 4.0);
    EXPECT_EQ(msg.header.frame_id, k1.frame_id_);
  }
}

TEST(tf2Convert, PointVectorOtherMessagetype)
{
  {
    const tf2::Vector3 t1{2.0, 4.0, 5.0};
    geometry_msgs::msg::Point msg;
    const geometry_msgs::msg::Point & msg2 = tf2::toMsg(t1, msg);

    // returned reference is second argument
    EXPECT_EQ(&msg2, &msg);
    EXPECT_EQ(msg.x, 2.0);
    EXPECT_EQ(msg.y, 4.0);
    EXPECT_EQ(msg.z, 5.0);
  }
  {
    // Eigen
    const Eigen::Vector3d e1{2.0, 4.0, 5.0};
    geometry_msgs::msg::Vector3 msg;
    const geometry_msgs::msg::Vector3 & msg2 = tf2::toMsg(e1, msg);

    // returned reference is second argument
    EXPECT_EQ(&msg2, &msg);
    EXPECT_EQ(msg.x, 2.0);
    EXPECT_EQ(msg.y, 4.0);
    EXPECT_EQ(msg.z, 5.0);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
