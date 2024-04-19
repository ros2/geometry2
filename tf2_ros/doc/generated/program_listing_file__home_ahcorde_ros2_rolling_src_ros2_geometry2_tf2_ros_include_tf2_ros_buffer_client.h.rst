
.. _program_listing_file__home_ahcorde_ros2_rolling_src_ros2_geometry2_tf2_ros_include_tf2_ros_buffer_client.h:

Program Listing for File buffer_client.h
========================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ahcorde_ros2_rolling_src_ros2_geometry2_tf2_ros_include_tf2_ros_buffer_client.h>` (``/home/ahcorde/ros2_rolling/src/ros2/geometry2/tf2_ros/include/tf2_ros/buffer_client.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

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
   
   #ifndef TF2_ROS__BUFFER_CLIENT_H_
   #define TF2_ROS__BUFFER_CLIENT_H_
   
   #include <stdexcept>
   #include <string>
   
   #include "tf2_ros/buffer_interface.h"
   #include "tf2_ros/visibility_control.h"
   #include "tf2/time.h"
   
   #include "geometry_msgs/msg/transform_stamped.hpp"
   #include "rclcpp_action/rclcpp_action.hpp"
   #include "tf2_msgs/action/lookup_transform.hpp"
   
   namespace tf2_ros
   {
   class LookupTransformGoalException : public std::runtime_error
   {
   public:
     TF2_ROS_PUBLIC
     explicit LookupTransformGoalException(const std::string & message)
     : std::runtime_error(message)
     {
     }
   };
   
   class GoalRejectedException : public LookupTransformGoalException
   {
   public:
     TF2_ROS_PUBLIC
     explicit GoalRejectedException(const std::string & message)
     : LookupTransformGoalException(message)
     {
     }
   };
   
   class GoalAbortedException : public LookupTransformGoalException
   {
   public:
     TF2_ROS_PUBLIC
     explicit GoalAbortedException(const std::string & message)
     : LookupTransformGoalException(message)
     {
     }
   };
   
   class GoalCanceledException : public LookupTransformGoalException
   {
   public:
     TF2_ROS_PUBLIC
     explicit GoalCanceledException(const std::string & message)
     : LookupTransformGoalException(message)
     {
     }
   };
   
   class UnexpectedResultCodeException : public LookupTransformGoalException
   {
   public:
     TF2_ROS_PUBLIC
     explicit UnexpectedResultCodeException(const std::string & message)
     : LookupTransformGoalException(message)
     {
     }
   };
   
   class BufferClient : public BufferInterface
   {
   public:
     using LookupTransformAction = tf2_msgs::action::LookupTransform;
   
     template<typename NodePtr>
     BufferClient(
       NodePtr node,
       const std::string ns,
       const double & check_frequency = 10.0,
       const tf2::Duration & timeout_padding = tf2::durationFromSec(2.0))
     : check_frequency_(check_frequency),
       timeout_padding_(timeout_padding)
     {
       client_ = rclcpp_action::create_client<LookupTransformAction>(node, ns);
     }
   
     virtual ~BufferClient() = default;
   
     TF2_ROS_PUBLIC
     geometry_msgs::msg::TransformStamped
     lookupTransform(
       const std::string & target_frame,
       const std::string & source_frame,
       const tf2::TimePoint & time,
       const tf2::Duration timeout = tf2::durationFromSec(0.0)) const override;
   
     TF2_ROS_PUBLIC
     geometry_msgs::msg::TransformStamped
     lookupTransform(
       const std::string & target_frame,
       const tf2::TimePoint & target_time,
       const std::string & source_frame,
       const tf2::TimePoint & source_time,
       const std::string & fixed_frame,
       const tf2::Duration timeout = tf2::durationFromSec(0.0)) const override;
   
     TF2_ROS_PUBLIC
     bool
     canTransform(
       const std::string & target_frame,
       const std::string & source_frame,
       const tf2::TimePoint & time,
       const tf2::Duration timeout = tf2::durationFromSec(0.0),
       std::string * errstr = nullptr) const override;
   
     TF2_ROS_PUBLIC
     bool
     canTransform(
       const std::string & target_frame,
       const tf2::TimePoint & target_time,
       const std::string & source_frame,
       const tf2::TimePoint & source_time,
       const std::string & fixed_frame,
       const tf2::Duration timeout = tf2::durationFromSec(0.0),
       std::string * errstr = nullptr) const override;
   
     TF2_ROS_PUBLIC
     bool waitForServer(const tf2::Duration & timeout = tf2::durationFromSec(0))
     {
       return client_->wait_for_action_server(timeout);
     }
   
   private:
     geometry_msgs::msg::TransformStamped
     processGoal(const LookupTransformAction::Goal & goal) const;
   
     geometry_msgs::msg::TransformStamped
     processResult(const LookupTransformAction::Result::SharedPtr & result) const;
   
     rclcpp_action::Client<LookupTransformAction>::SharedPtr client_;
     double check_frequency_;
     tf2::Duration timeout_padding_;
   };
   }  // namespace tf2_ros
   
   #endif  // TF2_ROS__BUFFER_CLIENT_H_
