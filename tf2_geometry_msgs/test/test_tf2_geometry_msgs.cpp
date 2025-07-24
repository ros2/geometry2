// Copyright 2008 Willow Garage, Inc.
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
//    * Neither the name of the Willow Garage, Inc. nor the names of its
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


/** \author Wim Meeussen */

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include <cmath>
#include <memory>
#include <string>

#include "gtest/gtest.h"

#include "rclcpp/clock.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include <geometry_msgs/msg/velocity_stamped.hpp>

std::unique_ptr<tf2_ros::Buffer> tf_buffer = nullptr;
static const double EPS = 1e-3;

geometry_msgs::msg::TransformStamped generate_stamped_transform()
{
  geometry_msgs::msg::TransformStamped t;
  t.transform.translation.x = 10;
  t.transform.translation.y = 20;
  t.transform.translation.z = 30;
  t.transform.rotation.w = 0;
  t.transform.rotation.x = 1;
  t.transform.rotation.y = 0;
  t.transform.rotation.z = 0;
  t.header.stamp = tf2_ros::toMsg(tf2::timeFromSec(2));
  t.header.frame_id = "A";
  t.child_frame_id = "B";
  return t;
}

TEST(TfGeometry, Conversions)
{
  // Quaternion
  {
    auto rotation = tf2::Quaternion(1.0, 2.0, 3.0, 4.0).normalized();
    tf2::Quaternion quat(rotation);
    geometry_msgs::msg::Quaternion quat_msg;
    tf2::convert(quat, quat_msg);

    EXPECT_NEAR(rotation.getX(), quat_msg.x, EPS);
    EXPECT_NEAR(rotation.getY(), quat_msg.y, EPS);
    EXPECT_NEAR(rotation.getZ(), quat_msg.z, EPS);
    EXPECT_NEAR(rotation.getW(), quat_msg.w, EPS);

    tf2::Quaternion quat_from_msg;
    tf2::convert(quat_msg, quat_from_msg);

    EXPECT_NEAR(quat_from_msg.getX(), quat_msg.x, EPS);
    EXPECT_NEAR(quat_from_msg.getY(), quat_msg.y, EPS);
    EXPECT_NEAR(quat_from_msg.getZ(), quat_msg.z, EPS);
    EXPECT_NEAR(quat_from_msg.getW(), quat_msg.w, EPS);
  }

  // QuaternionStamped
  {
    auto rotation = tf2::Quaternion(1.0, 2.0, 3.0, 4.0).normalized();
    auto stamp = tf2::timeFromSec(2);
    std::string frame_id = "test_frame_id";
    tf2::Stamped<tf2::Quaternion> quat_stamped(rotation, stamp, frame_id);
    geometry_msgs::msg::QuaternionStamped quat_stamped_msg;
    tf2::convert(quat_stamped, quat_stamped_msg);

    EXPECT_NEAR(rotation.getX(), quat_stamped_msg.quaternion.x, EPS);
    EXPECT_NEAR(rotation.getY(), quat_stamped_msg.quaternion.y, EPS);
    EXPECT_NEAR(rotation.getZ(), quat_stamped_msg.quaternion.z, EPS);
    EXPECT_NEAR(rotation.getW(), quat_stamped_msg.quaternion.w, EPS);
    EXPECT_EQ(frame_id, quat_stamped_msg.header.frame_id);

    tf2::Stamped<tf2::Quaternion> quat_from_msg;
    tf2::convert(quat_stamped_msg, quat_from_msg);

    EXPECT_NEAR(quat_from_msg.getX(), quat_stamped_msg.quaternion.x, EPS);
    EXPECT_NEAR(quat_from_msg.getY(), quat_stamped_msg.quaternion.y, EPS);
    EXPECT_NEAR(quat_from_msg.getZ(), quat_stamped_msg.quaternion.z, EPS);
    EXPECT_NEAR(quat_from_msg.getW(), quat_stamped_msg.quaternion.w, EPS);
    EXPECT_EQ(quat_from_msg.frame_id_, quat_stamped_msg.header.frame_id);
  }

  // Transform
  {
    auto rotation = tf2::Quaternion(1.0, 2.0, 3.0, 4.0).normalized();
    tf2::Vector3 translation(1.0, 2.0, 3.0);
    tf2::Transform tf_(tf2::Transform(rotation, translation));
    geometry_msgs::msg::Transform tf_msg;
    tf2::convert(tf_, tf_msg);

    EXPECT_NEAR(rotation.getX(), tf_msg.rotation.x, EPS);
    EXPECT_NEAR(rotation.getY(), tf_msg.rotation.y, EPS);
    EXPECT_NEAR(rotation.getZ(), tf_msg.rotation.z, EPS);
    EXPECT_NEAR(rotation.getW(), tf_msg.rotation.w, EPS);
    EXPECT_NEAR(translation.getX(), tf_msg.translation.x, EPS);
    EXPECT_NEAR(translation.getY(), tf_msg.translation.y, EPS);
    EXPECT_NEAR(translation.getZ(), tf_msg.translation.z, EPS);

    tf2::Transform tf_from_msg;
    tf2::convert(tf_msg, tf_from_msg);

    EXPECT_NEAR(tf_from_msg.getRotation().getX(), tf_msg.rotation.x, EPS);
    EXPECT_NEAR(tf_from_msg.getRotation().getY(), tf_msg.rotation.y, EPS);
    EXPECT_NEAR(tf_from_msg.getRotation().getZ(), tf_msg.rotation.z, EPS);
    EXPECT_NEAR(tf_from_msg.getRotation().getW(), tf_msg.rotation.w, EPS);
    EXPECT_NEAR(tf_from_msg.getOrigin().getX(), tf_msg.translation.x, EPS);
    EXPECT_NEAR(tf_from_msg.getOrigin().getY(), tf_msg.translation.y, EPS);
    EXPECT_NEAR(tf_from_msg.getOrigin().getZ(), tf_msg.translation.z, EPS);
  }

  // TransformStamped
  {
    auto rotation = tf2::Quaternion(1.0, 2.0, 3.0, 4.0).normalized();
    tf2::Vector3 translation(1.0, 2.0, 3.0);
    auto stamp = tf2::timeFromSec(2);
    std::string frame_id = "test_frame_id";
    tf2::Stamped<tf2::Transform> tf_stamped(tf2::Transform(rotation, translation), stamp, frame_id);
    geometry_msgs::msg::TransformStamped tf_stamped_msg;
    tf2::convert(tf_stamped, tf_stamped_msg);

    EXPECT_NEAR(rotation.getX(), tf_stamped_msg.transform.rotation.x, EPS);
    EXPECT_NEAR(rotation.getY(), tf_stamped_msg.transform.rotation.y, EPS);
    EXPECT_NEAR(rotation.getZ(), tf_stamped_msg.transform.rotation.z, EPS);
    EXPECT_NEAR(rotation.getW(), tf_stamped_msg.transform.rotation.w, EPS);
    EXPECT_NEAR(translation.getX(), tf_stamped_msg.transform.translation.x, EPS);
    EXPECT_NEAR(translation.getY(), tf_stamped_msg.transform.translation.y, EPS);
    EXPECT_NEAR(translation.getZ(), tf_stamped_msg.transform.translation.z, EPS);
    EXPECT_EQ(frame_id, tf_stamped_msg.header.frame_id);

    tf2::Stamped<tf2::Transform> tf_from_msg;
    tf2::convert(tf_stamped_msg, tf_from_msg);

    EXPECT_NEAR(tf_from_msg.getRotation().getX(), tf_stamped_msg.transform.rotation.x, EPS);
    EXPECT_NEAR(tf_from_msg.getRotation().getY(), tf_stamped_msg.transform.rotation.y, EPS);
    EXPECT_NEAR(tf_from_msg.getRotation().getZ(), tf_stamped_msg.transform.rotation.z, EPS);
    EXPECT_NEAR(tf_from_msg.getRotation().getW(), tf_stamped_msg.transform.rotation.w, EPS);
    EXPECT_NEAR(tf_from_msg.getOrigin().getX(), tf_stamped_msg.transform.translation.x, EPS);
    EXPECT_NEAR(tf_from_msg.getOrigin().getY(), tf_stamped_msg.transform.translation.y, EPS);
    EXPECT_NEAR(tf_from_msg.getOrigin().getZ(), tf_stamped_msg.transform.translation.z, EPS);
    EXPECT_EQ(tf_from_msg.frame_id_, tf_stamped_msg.header.frame_id);
  }
}

TEST(TfGeometry, Frame)
{
  // non-stamped
  {
    geometry_msgs::msg::Pose v1, res;
    v1.position.x = 1;
    v1.position.y = 2;
    v1.position.z = 3;
    v1.orientation.w = 0;
    v1.orientation.x = 1;
    v1.orientation.y = 0;
    v1.orientation.z = 0;

    geometry_msgs::msg::TransformStamped t = generate_stamped_transform();

    tf2::doTransform(v1, res, t);
    EXPECT_NEAR(res.position.x, 11, EPS);
    EXPECT_NEAR(res.position.y, 18, EPS);
    EXPECT_NEAR(res.position.z, 27, EPS);
    EXPECT_NEAR(res.orientation.x, 0.0, EPS);
    EXPECT_NEAR(res.orientation.y, 0.0, EPS);
    EXPECT_NEAR(res.orientation.z, 0.0, EPS);
    EXPECT_NEAR(res.orientation.w, 1.0, EPS);
  }

  // stamped
  {
    geometry_msgs::msg::PoseStamped v1;
    v1.pose.position.x = 1;
    v1.pose.position.y = 2;
    v1.pose.position.z = 3;
    v1.pose.orientation.w = 0;
    v1.pose.orientation.x = 1;
    v1.pose.orientation.y = 0;
    v1.pose.orientation.z = 0;
    v1.header.stamp = tf2_ros::toMsg(tf2::timeFromSec(2));
    v1.header.frame_id = "A";

    // simple api
    geometry_msgs::msg::PoseStamped v_simple =
      tf_buffer->transform(v1, "B", tf2::durationFromSec(2.0));
    EXPECT_NEAR(v_simple.pose.position.x, -9, EPS);
    EXPECT_NEAR(v_simple.pose.position.y, 18, EPS);
    EXPECT_NEAR(v_simple.pose.position.z, 27, EPS);
    EXPECT_NEAR(v_simple.pose.orientation.x, 0.0, EPS);
    EXPECT_NEAR(v_simple.pose.orientation.y, 0.0, EPS);
    EXPECT_NEAR(v_simple.pose.orientation.z, 0.0, EPS);
    EXPECT_NEAR(v_simple.pose.orientation.w, 1.0, EPS);


    // advanced api
    geometry_msgs::msg::PoseStamped v_advanced =
      tf_buffer->transform(v1, "B", tf2::timeFromSec(2.0), "A", tf2::durationFromSec(3.0));
    EXPECT_NEAR(v_advanced.pose.position.x, -9, EPS);
    EXPECT_NEAR(v_advanced.pose.position.y, 18, EPS);
    EXPECT_NEAR(v_advanced.pose.position.z, 27, EPS);
    EXPECT_NEAR(v_advanced.pose.orientation.x, 0.0, EPS);
    EXPECT_NEAR(v_advanced.pose.orientation.y, 0.0, EPS);
    EXPECT_NEAR(v_advanced.pose.orientation.z, 0.0, EPS);
    EXPECT_NEAR(v_advanced.pose.orientation.w, 1.0, EPS);
  }
}

TEST(TfGeometry, FrameWithCovariance)
{
  // non-stamped
  {
    geometry_msgs::msg::PoseWithCovariance v1, res;
    v1.pose.position.x = 1;
    v1.pose.position.y = 2;
    v1.pose.position.z = 3;
    v1.pose.orientation.w = 0;
    v1.pose.orientation.x = 1;
    v1.pose.orientation.y = 0;
    v1.pose.orientation.z = 0;
    v1.covariance = {
      1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
      1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
      1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
      1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
      1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
      1.0, 2.0, 3.0, 4.0, 5.0, 6.0
    };

    geometry_msgs::msg::PoseWithCovariance::_covariance_type v1_expected_covariance = {
      1.0, -2.0, -3.0, 4.0, -5.0, -6.0,
      -1.0, 2.0, 3.0, -4.0, 5.0, 6.0,
      -1.0, 2.0, 3.0, -4.0, 5.0, 6.0,
      1.0, -2.0, -3.0, 4.0, -5.0, -6.0,
      -1.0, 2.0, 3.0, -4.0, 5.0, 6.0,
      -1.0, 2.0, 3.0, -4.0, 5.0, 6.0
    };

    geometry_msgs::msg::TransformStamped t = generate_stamped_transform();

    tf2::doTransform(v1, res, t);
    EXPECT_NEAR(res.pose.position.x, 11, EPS);
    EXPECT_NEAR(res.pose.position.y, 18, EPS);
    EXPECT_NEAR(res.pose.position.z, 27, EPS);
    EXPECT_NEAR(res.pose.orientation.x, 0.0, EPS);
    EXPECT_NEAR(res.pose.orientation.y, 0.0, EPS);
    EXPECT_NEAR(res.pose.orientation.z, 0.0, EPS);
    EXPECT_NEAR(res.pose.orientation.w, 1.0, EPS);
    EXPECT_EQ(res.covariance, v1_expected_covariance);
  }

  // stamped
  {
    geometry_msgs::msg::PoseWithCovarianceStamped v1;
    v1.pose.pose.position.x = 1;
    v1.pose.pose.position.y = 2;
    v1.pose.pose.position.z = 3;
    v1.pose.pose.orientation.w = 0;
    v1.pose.pose.orientation.x = 1;
    v1.pose.pose.orientation.y = 0;
    v1.pose.pose.orientation.z = 0;
    v1.header.stamp = tf2_ros::toMsg(tf2::timeFromSec(2));
    v1.header.frame_id = "A";
    v1.pose.covariance = {
      1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
      1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
      1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
      1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
      1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
      1.0, 2.0, 3.0, 4.0, 5.0, 6.0
    };

    geometry_msgs::msg::PoseWithCovariance::_covariance_type v1_expected_covariance = {
      1.0, -2.0, -3.0, 4.0, -5.0, -6.0,
      -1.0, 2.0, 3.0, -4.0, 5.0, 6.0,
      -1.0, 2.0, 3.0, -4.0, 5.0, 6.0,
      1.0, -2.0, -3.0, 4.0, -5.0, -6.0,
      -1.0, 2.0, 3.0, -4.0, 5.0, 6.0,
      -1.0, 2.0, 3.0, -4.0, 5.0, 6.0
    };

    // simple api
    geometry_msgs::msg::PoseWithCovarianceStamped v_simple =
      tf_buffer->transform(v1, "B", tf2::durationFromSec(2.0));
    EXPECT_NEAR(v_simple.pose.pose.position.x, -9, EPS);
    EXPECT_NEAR(v_simple.pose.pose.position.y, 18, EPS);
    EXPECT_NEAR(v_simple.pose.pose.position.z, 27, EPS);
    EXPECT_NEAR(v_simple.pose.pose.orientation.x, 0.0, EPS);
    EXPECT_NEAR(v_simple.pose.pose.orientation.y, 0.0, EPS);
    EXPECT_NEAR(v_simple.pose.pose.orientation.z, 0.0, EPS);
    EXPECT_NEAR(v_simple.pose.pose.orientation.w, 1.0, EPS);
    EXPECT_EQ(v_simple.pose.covariance, v1_expected_covariance);


    // advanced api
    geometry_msgs::msg::PoseWithCovarianceStamped v_advanced =
      tf_buffer->transform(v1, "B", tf2::timeFromSec(2.0), "A", tf2::durationFromSec(3.0));
    EXPECT_NEAR(v_advanced.pose.pose.position.x, -9, EPS);
    EXPECT_NEAR(v_advanced.pose.pose.position.y, 18, EPS);
    EXPECT_NEAR(v_advanced.pose.pose.position.z, 27, EPS);
    EXPECT_NEAR(v_advanced.pose.pose.orientation.x, 0.0, EPS);
    EXPECT_NEAR(v_advanced.pose.pose.orientation.y, 0.0, EPS);
    EXPECT_NEAR(v_advanced.pose.pose.orientation.z, 0.0, EPS);
    EXPECT_NEAR(v_advanced.pose.pose.orientation.w, 1.0, EPS);
    EXPECT_EQ(v_advanced.pose.covariance, v1_expected_covariance);
  }
}

TEST(TfGeometry, Vector)
{
  // non-stamped
  {
    geometry_msgs::msg::Vector3 v1, res;
    v1.x = 1;
    v1.y = 2;
    v1.z = 3;

    geometry_msgs::msg::TransformStamped t = generate_stamped_transform();

    tf2::doTransform(v1, res, t);
    EXPECT_NEAR(res.x, 1, EPS);
    EXPECT_NEAR(res.y, -2, EPS);
    EXPECT_NEAR(res.z, -3, EPS);
  }

  // stamped
  {
    geometry_msgs::msg::Vector3Stamped v1, res;
    v1.vector.x = 1;
    v1.vector.y = 2;
    v1.vector.z = 3;
    v1.header.stamp = tf2_ros::toMsg(tf2::timeFromSec(2));
    v1.header.frame_id = "A";

    // simple api
    geometry_msgs::msg::Vector3Stamped v_simple =
      tf_buffer->transform(v1, "B", tf2::durationFromSec(2.0));
    EXPECT_NEAR(v_simple.vector.x, 1, EPS);
    EXPECT_NEAR(v_simple.vector.y, -2, EPS);
    EXPECT_NEAR(v_simple.vector.z, -3, EPS);

    // advanced api
    geometry_msgs::msg::Vector3Stamped v_advanced =
      tf_buffer->transform(v1, "B", tf2::timeFromSec(2.0), "A", tf2::durationFromSec(3.0));
    EXPECT_NEAR(v_advanced.vector.x, 1, EPS);
    EXPECT_NEAR(v_advanced.vector.y, -2, EPS);
    EXPECT_NEAR(v_advanced.vector.z, -3, EPS);
  }
}


TEST(TfGeometry, Point)
{
  // non-stamped
  {
    geometry_msgs::msg::Point v1, res;
    v1.x = 1;
    v1.y = 2;
    v1.z = 3;

    geometry_msgs::msg::TransformStamped t = generate_stamped_transform();

    tf2::doTransform(v1, res, t);
    EXPECT_NEAR(res.x, 11, EPS);
    EXPECT_NEAR(res.y, 18, EPS);
    EXPECT_NEAR(res.z, 27, EPS);
  }

  // non-stamped 32
  {
    geometry_msgs::msg::Point32 v1, res;
    v1.x = 1;
    v1.y = 2;
    v1.z = 3;

    geometry_msgs::msg::TransformStamped t = generate_stamped_transform();

    tf2::doTransform(v1, res, t);
    EXPECT_NEAR(res.x, 11, EPS);
    EXPECT_NEAR(res.y, 18, EPS);
    EXPECT_NEAR(res.z, 27, EPS);
  }

  // stamped
  {
    geometry_msgs::msg::PointStamped v1, res;
    v1.point.x = 1;
    v1.point.y = 2;
    v1.point.z = 3;
    v1.header.stamp = tf2_ros::toMsg(tf2::timeFromSec(2));
    v1.header.frame_id = "A";

    // simple api
    geometry_msgs::msg::PointStamped v_simple =
      tf_buffer->transform(v1, "B", tf2::durationFromSec(2.0));
    EXPECT_NEAR(v_simple.point.x, -9, EPS);
    EXPECT_NEAR(v_simple.point.y, 18, EPS);
    EXPECT_NEAR(v_simple.point.z, 27, EPS);

    // advanced api
    geometry_msgs::msg::PointStamped v_advanced =
      tf_buffer->transform(v1, "B", tf2::timeFromSec(2.0), "A", tf2::durationFromSec(3.0));
    EXPECT_NEAR(v_advanced.point.x, -9, EPS);
    EXPECT_NEAR(v_advanced.point.y, 18, EPS);
    EXPECT_NEAR(v_advanced.point.z, 27, EPS);
  }
}

TEST(TfGeometry, Polygon)
{
  // non-stamped
  {
    geometry_msgs::msg::Polygon v1, res;
    geometry_msgs::msg::Point32 p;
    p.x = 1;
    p.y = 2;
    p.z = 3;
    v1.points.push_back(p);

    geometry_msgs::msg::TransformStamped t = generate_stamped_transform();

    tf2::doTransform(v1, res, t);
    EXPECT_NEAR(res.points[0].x, 11, EPS);
    EXPECT_NEAR(res.points[0].y, 18, EPS);
    EXPECT_NEAR(res.points[0].z, 27, EPS);
  }

  // stamped
  {
    geometry_msgs::msg::PolygonStamped v1, res;
    geometry_msgs::msg::Point32 p;
    p.x = 1;
    p.y = 2;
    p.z = 3;
    v1.polygon.points.push_back(p);
    v1.header.stamp = tf2_ros::toMsg(tf2::timeFromSec(2));
    v1.header.frame_id = "A";

    // simple api
    geometry_msgs::msg::PolygonStamped v_simple =
      tf_buffer->transform(v1, "B", tf2::durationFromSec(2.0));
    EXPECT_NEAR(v_simple.polygon.points[0].x, -9, EPS);
    EXPECT_NEAR(v_simple.polygon.points[0].y, 18, EPS);
    EXPECT_NEAR(v_simple.polygon.points[0].z, 27, EPS);

    // advanced api
    geometry_msgs::msg::PolygonStamped v_advanced =
      tf_buffer->transform(v1, "B", tf2::timeFromSec(2.0), "A", tf2::durationFromSec(3.0));
    EXPECT_NEAR(v_simple.polygon.points[0].x, -9, EPS);
    EXPECT_NEAR(v_simple.polygon.points[0].y, 18, EPS);
    EXPECT_NEAR(v_simple.polygon.points[0].z, 27, EPS);
  }
}

TEST(TfGeometry, Quaternion)
{
  // rotated by -90° around y
  // 0, 0, -1
  // 0, 1, 0,
  // 1, 0, 0

  // non-stamped
  {
    geometry_msgs::msg::Quaternion q1, res;
    q1.x = 0;
    q1.y = -1 * M_SQRT1_2;
    q1.z = 0;
    q1.w = M_SQRT1_2;

    geometry_msgs::msg::TransformStamped t = generate_stamped_transform();

    tf2::doTransform(q1, res, t);
    EXPECT_NEAR(res.x, M_SQRT1_2, EPS);
    EXPECT_NEAR(res.y, 0, EPS);
    EXPECT_NEAR(res.z, -1 * M_SQRT1_2, EPS);
    EXPECT_NEAR(res.w, 0, EPS);
  }

  // stamped
  {
    geometry_msgs::msg::QuaternionStamped q1, res;
    q1.quaternion.x = 0;
    q1.quaternion.y = -1 * M_SQRT1_2;
    q1.quaternion.z = 0;
    q1.quaternion.w = M_SQRT1_2;
    q1.header.stamp = tf2_ros::toMsg(tf2::timeFromSec(2));
    q1.header.frame_id = "A";

    // simple api
    const geometry_msgs::msg::QuaternionStamped q_simple = tf_buffer->transform(
      q1, "B", tf2::durationFromSec(
        2.0));
    EXPECT_NEAR(q_simple.quaternion.x, M_SQRT1_2, EPS);
    EXPECT_NEAR(q_simple.quaternion.y, 0, EPS);
    EXPECT_NEAR(q_simple.quaternion.z, -1 * M_SQRT1_2, EPS);
    EXPECT_NEAR(q_simple.quaternion.w, 0, EPS);

    // advanced api
    const geometry_msgs::msg::QuaternionStamped q_advanced = tf_buffer->transform(
      q1, "B", tf2::timeFromSec(2.0),
      "A", tf2::durationFromSec(3.0));
    EXPECT_NEAR(q_advanced.quaternion.x, M_SQRT1_2, EPS);
    EXPECT_NEAR(q_advanced.quaternion.y, 0, EPS);
    EXPECT_NEAR(q_advanced.quaternion.z, -1 * M_SQRT1_2, EPS);
    EXPECT_NEAR(q_advanced.quaternion.w, 0, EPS);
  }
}

TEST(TfGeometry, Transform)
{
  // non-stamped
  {
    geometry_msgs::msg::Transform v1, res;
    v1.translation.x = 1;
    v1.translation.y = 2;
    v1.translation.z = 3;
    v1.rotation.w = 0;
    v1.rotation.x = 1;
    v1.rotation.y = 0;
    v1.rotation.z = 0;

    geometry_msgs::msg::TransformStamped t = generate_stamped_transform();

    tf2::doTransform(v1, res, t);
    EXPECT_NEAR(res.translation.x, 11, EPS);
    EXPECT_NEAR(res.translation.y, 18, EPS);
    EXPECT_NEAR(res.translation.z, 27, EPS);
    EXPECT_NEAR(res.rotation.x, 0.0, EPS);
    EXPECT_NEAR(res.rotation.y, 0.0, EPS);
    EXPECT_NEAR(res.rotation.z, 0.0, EPS);
    EXPECT_NEAR(res.rotation.w, 1.0, EPS);
  }

  // stamped
  {
    geometry_msgs::msg::TransformStamped v1;
    v1.transform.translation.x = 1;
    v1.transform.translation.y = 2;
    v1.transform.translation.z = 3;
    v1.transform.rotation.w = 0;
    v1.transform.rotation.x = 1;
    v1.transform.rotation.y = 0;
    v1.transform.rotation.z = 0;
    v1.header.stamp = tf2_ros::toMsg(tf2::timeFromSec(2));
    v1.header.frame_id = "A";

    // simple api
    geometry_msgs::msg::TransformStamped v_simple =
      tf_buffer->transform(v1, "B", tf2::durationFromSec(2.0));
    EXPECT_NEAR(v_simple.transform.translation.x, -9, EPS);
    EXPECT_NEAR(v_simple.transform.translation.y, 18, EPS);
    EXPECT_NEAR(v_simple.transform.translation.z, 27, EPS);
    EXPECT_NEAR(v_simple.transform.rotation.x, 0.0, EPS);
    EXPECT_NEAR(v_simple.transform.rotation.y, 0.0, EPS);
    EXPECT_NEAR(v_simple.transform.rotation.z, 0.0, EPS);
    EXPECT_NEAR(v_simple.transform.rotation.w, 1.0, EPS);

    // advanced api
    geometry_msgs::msg::TransformStamped v_advanced =
      tf_buffer->transform(v1, "B", tf2::timeFromSec(2.0), "A", tf2::durationFromSec(3.0));
    EXPECT_NEAR(v_advanced.transform.translation.x, -9, EPS);
    EXPECT_NEAR(v_advanced.transform.translation.y, 18, EPS);
    EXPECT_NEAR(v_advanced.transform.translation.z, 27, EPS);
    EXPECT_NEAR(v_advanced.transform.rotation.x, 0.0, EPS);
    EXPECT_NEAR(v_advanced.transform.rotation.y, 0.0, EPS);
    EXPECT_NEAR(v_advanced.transform.rotation.z, 0.0, EPS);
    EXPECT_NEAR(v_advanced.transform.rotation.w, 1.0, EPS);
  }
}

TEST(TfGeometry, Wrench)
{
  geometry_msgs::msg::Wrench v1, res;
  v1.force.x = 2;
  v1.force.y = 1;
  v1.force.z = 3;
  v1.torque.x = 2;
  v1.torque.y = 1;
  v1.torque.z = 3;

  geometry_msgs::msg::TransformStamped trafo;
  trafo.transform.translation.x = 0;
  trafo.transform.translation.y = -2;
  trafo.transform.translation.z = 0;
  trafo.transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), -M_PI / 2.0));

  tf2::doTransform(v1, res, trafo);
  EXPECT_NEAR(res.force.x, 1, EPS);
  EXPECT_NEAR(res.force.y, -2, EPS);
  EXPECT_NEAR(res.force.z, 3, EPS);

  EXPECT_NEAR(res.torque.x, -5, EPS);
  EXPECT_NEAR(res.torque.y, -2, EPS);
  EXPECT_NEAR(res.torque.z, 5, EPS);
}

TEST(TfGeometry, Velocity)
{
  geometry_msgs::msg::VelocityStamped v1, res;
  v1.header.frame_id = "world";
  v1.body_frame_id = "base_link";

  geometry_msgs::msg::TransformStamped trafo;

  tf2::doTransform(v1, res, trafo);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf_buffer = std::make_unique<tf2_ros::Buffer>(clock);
  tf_buffer->setUsingDedicatedThread(true);

  // populate buffer
  geometry_msgs::msg::TransformStamped t = generate_stamped_transform();
  tf_buffer->setTransform(t, "test");

  int ret = RUN_ALL_TESTS();
  return ret;
}
