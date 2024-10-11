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

#include "tf2_ros/buffer_client.h"

#include <chrono>
#include <future>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"

namespace tf2_ros
{
geometry_msgs::msg::TransformStamped BufferClient::lookupTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  const tf2::TimePoint & time,
  const tf2::Duration timeout) const
{
  // populate the request
  LookupTransformService::Request::SharedPtr request;
  request->target_frame = target_frame;
  request->source_frame = source_frame;
  request->source_time = tf2_ros::toMsg(time);
  request->timeout = tf2_ros::toMsg(timeout);
  request->advanced = false;

  return processRequest(request);
}

geometry_msgs::msg::TransformStamped BufferClient::lookupTransform(
  const std::string & target_frame,
  const tf2::TimePoint & target_time,
  const std::string & source_frame,
  const tf2::TimePoint & source_time,
  const std::string & fixed_frame,
  const tf2::Duration timeout) const
{
  // populate the request
  LookupTransformService::Request::SharedPtr request;
  request->target_frame = target_frame;
  request->source_frame = source_frame;
  request->source_time = tf2_ros::toMsg(source_time);
  request->timeout = tf2_ros::toMsg(timeout);
  request->target_time = tf2_ros::toMsg(target_time);
  request->fixed_frame = fixed_frame;
  request->advanced = true;

  return processRequest(request);
}

geometry_msgs::msg::TransformStamped BufferClient::processRequest(
  const LookupTransformService::Request::SharedPtr & request) const
{
  if (!service_client_->wait_for_service(tf2_ros::fromMsg(request->timeout))) {
    throw tf2::ConnectivityException("Failed find available service server");
  }

  auto response_future = service_client_->async_send_request(request);

  // Wait for the result
  if (rclcpp::spin_until_future_complete(
      node_->get_node_base_interface(),
      response_future, tf2_ros::fromMsg(request->timeout)) == rclcpp::FutureReturnCode::SUCCESS)
  {
    return processResponse(response_future.get());
  } else {
    throw tf2::TimeoutException(
            "Did not receive the response for the request sent to "
            "the service server. Something is likely wrong with the server.");
  }
}

geometry_msgs::msg::TransformStamped BufferClient::processResponse(
  const LookupTransformService::Response::SharedPtr & response) const
{
  // if there's no error, then we'll just return the transform
  if (response->error.error != response->error.NO_ERROR) {
    // otherwise, we'll have to throw the appropriate exception
    if (response->error.error == response->error.LOOKUP_ERROR) {
      throw tf2::LookupException(response->error.error_string);
    }

    if (response->error.error == response->error.CONNECTIVITY_ERROR) {
      throw tf2::ConnectivityException(response->error.error_string);
    }

    if (response->error.error == response->error.EXTRAPOLATION_ERROR) {
      throw tf2::ExtrapolationException(response->error.error_string);
    }

    if (response->error.error == response->error.INVALID_ARGUMENT_ERROR) {
      throw tf2::InvalidArgumentException(response->error.error_string);
    }

    if (response->error.error == response->error.TIMEOUT_ERROR) {
      throw tf2::TimeoutException(response->error.error_string);
    }

    throw tf2::TransformException(response->error.error_string);
  }

  return response->transform;
}

bool BufferClient::canTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  const tf2::TimePoint & time,
  const tf2::Duration timeout,
  std::string * errstr) const
{
  try {
    lookupTransform(target_frame, source_frame, time, timeout);
    return true;
  } catch (const tf2::TransformException & ex) {
    if (errstr) {
      *errstr = ex.what();
    }
    return false;
  } catch (const LookupTransformGoalException & ex) {
    if (errstr) {
      *errstr = ex.what();
    }
    return false;
  }
}

bool BufferClient::canTransform(
  const std::string & target_frame,
  const tf2::TimePoint & target_time,
  const std::string & source_frame,
  const tf2::TimePoint & source_time,
  const std::string & fixed_frame,
  const tf2::Duration timeout,
  std::string * errstr) const
{
  try {
    lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame, timeout);
    return true;
  } catch (const tf2::TransformException & ex) {
    if (errstr) {
      *errstr = ex.what();
    }
    return false;
  } catch (const LookupTransformGoalException & ex) {
    if (errstr) {
      *errstr = ex.what();
    }
    return false;
  }
}

}  // namespace tf2_ros
