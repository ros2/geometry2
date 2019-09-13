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
#include <tf2_ros/buffer_client.h>

namespace tf2_ros
{
  geometry_msgs::msg::TransformStamped BufferClient::lookupTransform(
    const std::string& target_frame,
    const std::string& source_frame,
    const tf2::TimePoint& time,
    const tf2::Duration timeout) const
  {
    // populate the goal message
    LookupTransformAction::Goal goal;
    goal.target_frame = target_frame;
    goal.source_frame = source_frame;
    goal.source_time = tf2_ros::toMsg(time);
    goal.timeout = tf2_ros::toMsg(timeout);
    goal.advanced = false;

    return processGoal(goal);
  }

  geometry_msgs::msg::TransformStamped BufferClient::lookupTransform(
    const std::string& target_frame,
    const tf2::TimePoint& target_time,
    const std::string& source_frame,
    const tf2::TimePoint& source_time,
    const std::string& fixed_frame,
    const tf2::Duration timeout) const
  {
    // populate the goal message
    LookupTransformAction::Goal goal;
    goal.target_frame = target_frame;
    goal.source_frame = source_frame;
    goal.source_time = tf2_ros::toMsg(source_time);
    goal.timeout = tf2_ros::toMsg(timeout);
    goal.target_time = tf2_ros::toMsg(target_time);
    goal.fixed_frame = fixed_frame;
    goal.advanced = true;

    return processGoal(goal);
  }

  geometry_msgs::msg::TransformStamped BufferClient::processGoal(
    const LookupTransformAction::Goal& goal) const
  {
    if (!client_->wait_for_action_server(tf2_ros::fromMsg(goal.timeout))) {
      throw tf2::ConnectivityException("Failed find available action server");
    }

    auto goal_handle_future = client_->async_send_goal(goal);

    const std::chrono::milliseconds period(static_cast<int>((1.0 / check_frequency_) * 1000));
    bool ready = false;
    bool timed_out = false;
    tf2::TimePoint start_time = tf2::get_now();
    while (rclcpp::ok() && !ready && !timed_out) {
      ready = (std::future_status::ready == goal_handle_future.wait_for(period));
      timed_out = tf2::get_now() > start_time + tf2_ros::fromMsg(goal.timeout) + timeout_padding_;
    }

    if (timed_out) {
      throw tf2::TimeoutException("Did not receive the goal response for the goal sent to "
         "the action server. Something is likely wrong with the server.");
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      throw GoalRejectedException("Goal rejected by action server");
    }

    auto result_future = client_->async_get_result(goal_handle);

    ready = false;
    while (rclcpp::ok() && !ready && !timed_out) {
      ready = (std::future_status::ready == result_future.wait_for(period));
      timed_out = tf2::get_now() > start_time + tf2_ros::fromMsg(goal.timeout) + timeout_padding_;
    }

    if (timed_out) {
      throw tf2::TimeoutException("Did not receive the result for the goal sent to "
         "the action server. Something is likely wrong with the server.");
    }

    auto wrapped_result = result_future.get();

    switch (wrapped_result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        throw GoalAbortedException("LookupTransform action was aborted");
      case rclcpp_action::ResultCode::CANCELED:
        throw GoalCanceledException("LookupTransform action was canceled");
      default:
        throw UnexpectedResultCodeException("Unexpected result code returned from server");
    }

    // process the result for errors and return it
    return processResult(wrapped_result.result);
  }

  geometry_msgs::msg::TransformStamped BufferClient::processResult(
    const LookupTransformAction::Result::SharedPtr& result) const
  {
    // if there's no error, then we'll just return the transform
    if (result->error.error != result->error.NO_ERROR) {
      // otherwise, we'll have to throw the appropriate exception
      if (result->error.error == result->error.LOOKUP_ERROR)
        throw tf2::LookupException(result->error.error_string);

      if (result->error.error == result->error.CONNECTIVITY_ERROR)
        throw tf2::ConnectivityException(result->error.error_string);

      if (result->error.error == result->error.EXTRAPOLATION_ERROR)
        throw tf2::ExtrapolationException(result->error.error_string);

      if (result->error.error == result->error.INVALID_ARGUMENT_ERROR)
        throw tf2::InvalidArgumentException(result->error.error_string);

      if (result->error.error == result->error.TIMEOUT_ERROR)
        throw tf2::TimeoutException(result->error.error_string);

      throw tf2::TransformException(result->error.error_string);
    }

    return result->transform;
  }

  bool BufferClient::canTransform(
    const std::string& target_frame,
    const std::string& source_frame,
    const tf2::TimePoint& time,
    const tf2::Duration timeout,
    std::string* errstr) const
  {
    try {
      lookupTransform(target_frame, source_frame, time, timeout);
      return true;
    } catch (tf2::TransformException& ex) {
      if (errstr)
        *errstr = ex.what();
      return false;
    } catch (LookupTransformGoalException& ex) {
      if (errstr)
        *errstr = ex.what();
      return false;
    }

  }

  bool BufferClient::canTransform(
    const std::string& target_frame,
    const tf2::TimePoint& target_time,
    const std::string& source_frame,
    const tf2::TimePoint& source_time,
    const std::string& fixed_frame,
    const tf2::Duration timeout,
    std::string* errstr) const
  {
    try {
      lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame, timeout);
      return true;
    } catch (tf2::TransformException& ex) {
      if(errstr)
        *errstr = ex.what();
      return false;
    } catch (LookupTransformGoalException& ex) {
      if (errstr)
        *errstr = ex.what();
      return false;
    }
  }
}  // namespace tf2_ros
