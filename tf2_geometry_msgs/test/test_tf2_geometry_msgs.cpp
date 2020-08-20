/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

/** \author Wim Meeussen */


#include <rclcpp/clock.hpp>
#include <tf2_ros/transform_listener.h>
#include <gtest/gtest.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

std::unique_ptr<tf2_ros::Buffer> tf_buffer = nullptr;
static const double EPS = 1e-3;

TEST(TfGeometry, Conversions)
{
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
  geometry_msgs::msg::PoseStamped v_simple = tf_buffer->transform(v1, "B", tf2::durationFromSec(2.0));
  EXPECT_NEAR(v_simple.pose.position.x, -9, EPS);
  EXPECT_NEAR(v_simple.pose.position.y, 18, EPS);
  EXPECT_NEAR(v_simple.pose.position.z, 27, EPS);
  EXPECT_NEAR(v_simple.pose.orientation.x, 0.0, EPS);
  EXPECT_NEAR(v_simple.pose.orientation.y, 0.0, EPS);
  EXPECT_NEAR(v_simple.pose.orientation.z, 0.0, EPS);
  EXPECT_NEAR(v_simple.pose.orientation.w, 1.0, EPS);


  // advanced api
  geometry_msgs::msg::PoseStamped v_advanced = tf_buffer->transform(v1, "B", tf2::timeFromSec(2.0),
							      "A", tf2::durationFromSec(3.0));
  EXPECT_NEAR(v_advanced.pose.position.x, -9, EPS);
  EXPECT_NEAR(v_advanced.pose.position.y, 18, EPS);
  EXPECT_NEAR(v_advanced.pose.position.z, 27, EPS);
  EXPECT_NEAR(v_advanced.pose.orientation.x, 0.0, EPS);
  EXPECT_NEAR(v_advanced.pose.orientation.y, 0.0, EPS);
  EXPECT_NEAR(v_advanced.pose.orientation.z, 0.0, EPS);
  EXPECT_NEAR(v_advanced.pose.orientation.w, 1.0, EPS);
}



TEST(TfGeometry, Vector)
{
  geometry_msgs::msg::Vector3Stamped v1, res;
  v1.vector.x = 1;
  v1.vector.y = 2;
  v1.vector.z = 3;
  v1.header.stamp = tf2_ros::toMsg(tf2::timeFromSec(2));
  v1.header.frame_id = "A";

  // simple api
  geometry_msgs::msg::Vector3Stamped v_simple = tf_buffer->transform(v1, "B", tf2::durationFromSec(2.0));
  EXPECT_NEAR(v_simple.vector.x, 1, EPS);
  EXPECT_NEAR(v_simple.vector.y, -2, EPS);
  EXPECT_NEAR(v_simple.vector.z, -3, EPS);

  // advanced api
  geometry_msgs::msg::Vector3Stamped v_advanced = tf_buffer->transform(v1, "B", tf2::timeFromSec(2.0),
								 "A", tf2::durationFromSec(3.0));
  EXPECT_NEAR(v_advanced.vector.x, 1, EPS);
  EXPECT_NEAR(v_advanced.vector.y, -2, EPS);
  EXPECT_NEAR(v_advanced.vector.z, -3, EPS);
}


TEST(TfGeometry, Point)
{
  geometry_msgs::msg::PointStamped v1, res;
  v1.point.x = 1;
  v1.point.y = 2;
  v1.point.z = 3;
  v1.header.stamp = tf2_ros::toMsg(tf2::timeFromSec(2));
  v1.header.frame_id = "A";

  // simple api
  geometry_msgs::msg::PointStamped v_simple = tf_buffer->transform(v1, "B", tf2::durationFromSec(2.0));
  EXPECT_NEAR(v_simple.point.x, -9, EPS);
  EXPECT_NEAR(v_simple.point.y, 18, EPS);
  EXPECT_NEAR(v_simple.point.z, 27, EPS);

  // advanced api
  geometry_msgs::msg::PointStamped v_advanced = tf_buffer->transform(v1, "B", tf2::timeFromSec(2.0),
								 "A", tf2::durationFromSec(3.0));
  EXPECT_NEAR(v_advanced.point.x, -9, EPS);
  EXPECT_NEAR(v_advanced.point.y, 18, EPS);
  EXPECT_NEAR(v_advanced.point.z, 27, EPS);
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf_buffer = std::make_unique<tf2_ros::Buffer>(clock);
  tf_buffer->setUsingDedicatedThread(true);

  // populate buffer
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
  tf_buffer->setTransform(t, "test");

  int ret = RUN_ALL_TESTS();
  return ret;
}
