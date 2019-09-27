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
#ifndef TF2_ROS_BUFFER_CLIENT_H_
#define TF2_ROS_BUFFER_CLIENT_H_

#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_msgs/action/lookup_transform.hpp>

#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/visibility_control.h>

namespace tf2_ros
{
  /**
   * \ brief Base class for lookup transform action goal exceptions.
   */
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

  /** \brief Action client-based implementation of the tf2_ros::BufferInterface abstract data type.
   *
   * BufferClient uses actions to coordinate waiting for available transforms.
   *
   * You can use this class with a tf2_ros::BufferServer and tf2_ros::TransformListener in a separate process.
   */
  class BufferClient : public BufferInterface
  {
    public:
      using LookupTransformAction = tf2_msgs::action::LookupTransform;

      /** \brief BufferClient constructor
       * \param node The node to add the buffer client to
       * \param ns The namespace in which to look for a BufferServer
       * \param check_frequency The frequency in Hz to check whether the BufferServer has completed a request
       * \param timeout_padding The amount of time to allow passed the desired timeout on the client side for communication lag
       */
      template<typename NodePtr>
      BufferClient(
        NodePtr node,
        const std::string ns,
        const double& check_frequency = 10.0,
        const tf2::Duration& timeout_padding = tf2::durationFromSec(2.0))
      : check_frequency_(check_frequency),
        timeout_padding_(timeout_padding)
      {
        client_ = rclcpp_action::create_client<LookupTransformAction>(node, ns);
      }
      virtual ~BufferClient() = default;

      /** \brief Get the transform between two frames by frame ID.
       *
       * If there is a communication failure, timeout, or transformation error,
       * an exception is thrown.
       *
       * \param target_frame The frame to which data should be transformed
       * \param source_frame The frame where the data originated
       * \param time The time at which the value of the transform is desired. (0 will get the latest)
       * \param timeout How long to block before failing
       * \return The transform between the frames
       *
       * \throws tf2::TransformException One of the following
       *   - tf2::LookupException
       *   - tf2::ConnectivityException
       *   - tf2::ExtrapolationException
       *   - tf2::InvalidArgumentException
       * \throws tf2_ros::LookupTransformGoalException One of the following
       *   - tf2_ros::GoalRejectedException
       *   - tf2_ros::GoalAbortedException
       *   - tf2_ros::GoalCanceledException
       *   - tf2_ros::UnexpectedResultCodeException
       */
      TF2_ROS_PUBLIC
      geometry_msgs::msg::TransformStamped
      lookupTransform(
        const std::string& target_frame,
        const std::string& source_frame,
        const tf2::TimePoint& time,
        const tf2::Duration timeout = tf2::durationFromSec(0.0)) const override;

      /** \brief Get the transform between two frames by frame ID assuming fixed frame.
       *
       * If there is a communication failure, timeout, or transformation error,
       * an exception is thrown.
       *
       * \param target_frame The frame to which data should be transformed
       * \param target_time The time to which the data should be transformed. (0 will get the latest)
       * \param source_frame The frame where the data originated
       * \param source_time The time at which the source_frame should be evaluated. (0 will get the latest)
       * \param fixed_frame The frame in which to assume the transform is constant in time.
       * \param timeout How long to block before failing
       * \return The transform between the frames
       *
       * \throws tf2::TransformException One of the following
       *   - tf2::LookupException
       *   - tf2::ConnectivityException
       *   - tf2::ExtrapolationException
       *   - tf2::InvalidArgumentException
       * \throws tf2_ros::LookupTransformGoalException One of the following
       *   - tf2_ros::GoalRejectedException
       *   - tf2_ros::GoalAbortedException
       *   - tf2_ros::GoalCanceledException
       *   - tf2_ros::UnexpectedResultCodeException
       */
      TF2_ROS_PUBLIC
      geometry_msgs::msg::TransformStamped
      lookupTransform(
        const std::string& target_frame,
        const tf2::TimePoint& target_time,
        const std::string& source_frame,
        const tf2::TimePoint& source_time,
        const std::string& fixed_frame,
        const tf2::Duration timeout = tf2::durationFromSec(0.0)) const override;

      /** \brief Test if a transform is possible
       * \param target_frame The frame into which to transform
       * \param source_frame The frame from which to transform
       * \param time The time at which to transform
       * \param timeout How long to block before failing
       * \param errstr A pointer to a string which will be filled with why the transform failed, if not NULL
       * \return True if the transform is possible, false otherwise
       */
      TF2_ROS_PUBLIC
      bool
      canTransform(
        const std::string& target_frame,
        const std::string& source_frame,
        const tf2::TimePoint& time,
        const tf2::Duration timeout = tf2::durationFromSec(0.0),
        std::string* errstr = nullptr) const override;

      /** \brief Test if a transform is possible
       * \param target_frame The frame into which to transform
       * \param target_time The time into which to transform
       * \param source_frame The frame from which to transform
       * \param source_time The time from which to transform
       * \param fixed_frame The frame in which to treat the transform as constant in time
       * \param timeout How long to block before failing
       * \param errstr A pointer to a string which will be filled with why the transform failed, if not NULL
       * \return True if the transform is possible, false otherwise
       */
      TF2_ROS_PUBLIC
      bool
      canTransform(
        const std::string& target_frame,
        const tf2::TimePoint& target_time,
        const std::string& source_frame,
        const tf2::TimePoint& source_time,
        const std::string& fixed_frame,
        const tf2::Duration timeout = tf2::durationFromSec(0.0),
        std::string* errstr = nullptr) const override;

      /** \brief Block until the action server is ready to respond to requests.
       * \param timeout Time to wait for the server.
       * \return True if the server is ready, false otherwise.
       */
      TF2_ROS_PUBLIC
      bool waitForServer(const tf2::Duration& timeout = tf2::durationFromSec(0))
      {
        return client_->wait_for_action_server(timeout);
      }

    private:
      geometry_msgs::msg::TransformStamped
      processGoal(const LookupTransformAction::Goal& goal) const;

      geometry_msgs::msg::TransformStamped
      processResult(const LookupTransformAction::Result::SharedPtr& result) const;

      rclcpp_action::Client<LookupTransformAction>::SharedPtr client_;
      double check_frequency_;
      tf2::Duration timeout_padding_;
  };
}
#endif
