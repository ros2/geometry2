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
#include <string>

#include <tf2/exceptions.h>

#include <tf2_ros/buffer.h>  // Only needed for toMsg() and fromMsg()
#include <tf2_ros/buffer_server.h>

namespace tf2_ros
{
  void BufferServer::checkTransforms()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    for(std::list<GoalInfo>::iterator it = active_goals_.begin(); it != active_goals_.end();)
    {
      GoalInfo& info = *it;

      //we want to lookup a transform if the time on the goal
      //has expired, or a transform is available
      if(canTransform(info.handle))
      {
        auto result = std::make_shared<LookupTransformAction::Result>();

        //try to populate the result, catching exceptions if they occur
        try
        {
          result->transform = lookupTransform(info.handle);

          RCLCPP_DEBUG(
            logger_,
            "Can transform for goal %s",
            rclcpp_action::to_string(info.handle->get_goal_id()).c_str());

          info.handle->succeed(result);
        }
        catch (tf2::ConnectivityException &ex)
        {
          result->error.error = result->error.CONNECTIVITY_ERROR;
          result->error.error_string = ex.what();
          info.handle->abort(result);
        }
        catch (tf2::LookupException &ex)
        {
          result->error.error = result->error.LOOKUP_ERROR;
          result->error.error_string = ex.what();
          info.handle->abort(result);
        }
        catch (tf2::ExtrapolationException &ex)
        {
          result->error.error = result->error.EXTRAPOLATION_ERROR;
          result->error.error_string = ex.what();
          info.handle->abort(result);
        }
        catch (tf2::InvalidArgumentException &ex)
        {
          result->error.error = result->error.INVALID_ARGUMENT_ERROR;
          result->error.error_string = ex.what();
          info.handle->abort(result);
        }
        catch (tf2::TimeoutException &ex)
        {
          result->error.error = result->error.TIMEOUT_ERROR;
          result->error.error_string = ex.what();
          info.handle->abort(result);
        }
        catch (tf2::TransformException &ex)
        {
          result->error.error = result->error.TRANSFORM_ERROR;
          result->error.error_string = ex.what();
          info.handle->abort(result);
        }

      } else if (info.end_time < tf2::get_now()) {
        // Timeout
        auto result = std::make_shared<LookupTransformAction::Result>();
        info.handle->abort(result);
      } else {
        ++it;
      }

      // Remove goal if it has terminated
      if (!info.handle->is_active()) {
        it = active_goals_.erase(it);
      }
    }
  }

  rclcpp_action::CancelResponse BufferServer::cancelCB(GoalHandle gh)
  {
    RCLCPP_DEBUG(
      logger_,
      "Cancel request for goal %s",
      rclcpp_action::to_string(gh->get_goal_id()).c_str());

    std::unique_lock<std::mutex> lock(mutex_);
    //we need to find the goal in the list and remove it... also setting it as canceled
    //if its not in the list, we won't do anything since it will have already been set
    //as completed
    for(std::list<GoalInfo>::iterator it = active_goals_.begin(); it != active_goals_.end();)
    {
      GoalInfo& info = *it;
      if(info.handle == gh)
      {
        RCLCPP_DEBUG(
          logger_,
          "Accept cancel request for goal %s",
          rclcpp_action::to_string(gh->get_goal_id()).c_str());
        it = active_goals_.erase(it);
        auto result = std::make_shared<LookupTransformAction::Result>();
        info.handle->canceled(result);
        return rclcpp_action::CancelResponse::ACCEPT;
      }
      else
        ++it;
    }

    RCLCPP_DEBUG(
      logger_,
      "Reject cancel request for goal %s",
      rclcpp_action::to_string(gh->get_goal_id()).c_str());

    return rclcpp_action::CancelResponse::REJECT;
  }

  rclcpp_action::GoalResponse BufferServer::goalCB(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const LookupTransformAction::Goal> goal)
  {
    (void)uuid;
    (void)goal;
    // accept all goals we get
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void BufferServer::acceptedCB(GoalHandle gh)
  {
    RCLCPP_DEBUG(
      logger_,
      "New goal accepted with ID %s",
      rclcpp_action::to_string(gh->get_goal_id()).c_str());
    //if the transform isn't immediately available, we'll push it onto our list to check
    //along with the time that the goal will end
    GoalInfo goal_info;
    goal_info.handle = gh;
    goal_info.end_time = tf2::get_now() + tf2_ros::fromMsg(gh->get_goal()->timeout);

    //we can do a quick check here to see if the transform is valid
    //we'll also do this if the end time has been reached
    if(canTransform(gh) || goal_info.end_time <= tf2::get_now())
    {
      auto result = std::make_shared<LookupTransformAction::Result>();
      try
      {
        result->transform = lookupTransform(gh);
      }
      catch (tf2::ConnectivityException &ex)
      {
        result->error.error = result->error.CONNECTIVITY_ERROR;
        result->error.error_string = ex.what();
      }
      catch (tf2::LookupException &ex)
      {
        result->error.error = result->error.LOOKUP_ERROR;
        result->error.error_string = ex.what();
      }
      catch (tf2::ExtrapolationException &ex)
      {
        result->error.error = result->error.EXTRAPOLATION_ERROR;
        result->error.error_string = ex.what();
      }
      catch (tf2::InvalidArgumentException &ex)
      {
        result->error.error = result->error.INVALID_ARGUMENT_ERROR;
        result->error.error_string = ex.what();
      }
      catch (tf2::TimeoutException &ex)
      {
        result->error.error = result->error.TIMEOUT_ERROR;
        result->error.error_string = ex.what();
      }
      catch (tf2::TransformException &ex)
      {
        result->error.error = result->error.TRANSFORM_ERROR;
        result->error.error_string = ex.what();
      }

      RCLCPP_DEBUG(logger_, "Transform available immediately for new goal");
      gh->succeed(result);
      return;
    }

    std::unique_lock<std::mutex> lock(mutex_);
    active_goals_.push_back(goal_info);
  }

  bool BufferServer::canTransform(GoalHandle gh)
  {
    const auto goal = gh->get_goal();

    tf2::TimePoint source_time_point = tf2_ros::fromMsg(goal->source_time);

    // check whether we need to used the advanced or simple api
    if(!goal->advanced)
      return buffer_.canTransform(goal->target_frame, goal->source_frame, source_time_point, nullptr);

    tf2::TimePoint target_time_point = tf2_ros::fromMsg(goal->target_time);
    return buffer_.canTransform(goal->target_frame, target_time_point,
        goal->source_frame, source_time_point, goal->fixed_frame, nullptr);
  }

  geometry_msgs::msg::TransformStamped BufferServer::lookupTransform(GoalHandle gh)
  {
    const auto goal = gh->get_goal();

    //check whether we need to used the advanced or simple api
    if(!goal->advanced)
      return buffer_.lookupTransform(goal->target_frame, goal->source_frame, tf2_ros::fromMsg(goal->source_time));

    return buffer_.lookupTransform(goal->target_frame, tf2_ros::fromMsg(goal->target_time),
        goal->source_frame, tf2_ros::fromMsg(goal->source_time), goal->fixed_frame);
  }
}  // namespace tf2_ros
