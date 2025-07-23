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


#include "tf2_ros/buffer.hpp"

#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

namespace tf2_ros
{
inline
tf2::Duration
from_rclcpp(const rclcpp::Duration & rclcpp_duration)
{
  return tf2::Duration(std::chrono::nanoseconds(rclcpp_duration.nanoseconds()));
}

inline
rclcpp::Duration
to_rclcpp(const tf2::Duration & duration)
{
  return rclcpp::Duration(std::chrono::nanoseconds(duration));
}

geometry_msgs::msg::TransformStamped
Buffer::lookupTransform(
  const std::string & target_frame, const std::string & source_frame,
  const tf2::TimePoint & lookup_time, const tf2::Duration timeout) const
{
  // Pass error string to suppress console spam
  std::string error;
  canTransform(target_frame, source_frame, lookup_time, timeout, &error);
  return lookupTransform(target_frame, source_frame, lookup_time);
}

void Buffer::onTimeJump(const rcl_time_jump_t & jump)
{
  if (RCL_ROS_TIME_ACTIVATED == jump.clock_change ||
    RCL_ROS_TIME_DEACTIVATED == jump.clock_change)
  {
    RCLCPP_WARN(getLogger(), "Detected time source change. Clearing TF buffer.");
    clear();
  } else if (jump.delta.nanoseconds < 0) {
    RCLCPP_WARN(getLogger(), "Detected jump back in time. Clearing TF buffer.");
    clear();
  }
}

geometry_msgs::msg::TransformStamped
Buffer::lookupTransform(
  const std::string & target_frame, const tf2::TimePoint & target_time,
  const std::string & source_frame, const tf2::TimePoint & source_time,
  const std::string & fixed_frame, const tf2::Duration timeout) const
{
  // Pass error string to suppress console spam
  std::string error;
  canTransform(target_frame, target_time, source_frame, source_time, fixed_frame, timeout, &error);
  return lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame);
}

void conditionally_append_timeout_info(
  std::string * errstr, const rclcpp::Time & start_time,
  const rclcpp::Time & current_time,
  const rclcpp::Duration & timeout)
{
  if (errstr) {
    std::stringstream ss;
    ss << ". canTransform returned after " <<
      tf2::durationToSec(from_rclcpp(current_time - start_time)) <<
      " timeout was " << tf2::durationToSec(from_rclcpp(timeout)) << ".";
    (*errstr) += ss.str();
  }
}

bool
Buffer::canTransform(
  const std::string & target_frame, const std::string & source_frame,
  const tf2::TimePoint & time, const tf2::Duration timeout, std::string * errstr) const
{
  if (timeout != tf2::durationFromSec(0.0) && !checkAndErrorDedicatedThreadPresent(errstr)) {
    return false;
  }

  rclcpp::Duration rclcpp_timeout(to_rclcpp(timeout));

  // poll for transform if timeout is set
  rclcpp::Time start_time = clock_->now();
  while (clock_->now() < start_time + rclcpp_timeout &&
    !canTransform(
      target_frame, source_frame, time,
      tf2::Duration(std::chrono::nanoseconds::zero()), errstr) &&
    (clock_->now() + rclcpp::Duration(3, 0) >= start_time) &&  // don't wait bag loop detected
    (rclcpp::ok()))  // Make sure we haven't been stopped (won't work for pytf)
  {
    // TODO(sloretz) sleep using clock_->sleep_for when implemented
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  bool retval = canTransform(target_frame, source_frame, time, errstr);
  rclcpp::Time current_time = clock_->now();
  conditionally_append_timeout_info(errstr, start_time, current_time, rclcpp_timeout);
  return retval;
}

bool
Buffer::canTransform(
  const std::string & target_frame, const tf2::TimePoint & target_time,
  const std::string & source_frame, const tf2::TimePoint & source_time,
  const std::string & fixed_frame, const tf2::Duration timeout, std::string * errstr) const
{
  if (timeout != tf2::durationFromSec(0.0) && !checkAndErrorDedicatedThreadPresent(errstr)) {
    return false;
  }

  rclcpp::Duration rclcpp_timeout(to_rclcpp(timeout));

  // poll for transform if timeout is set
  rclcpp::Time start_time = clock_->now();
  while (clock_->now() < start_time + rclcpp_timeout &&
    !canTransform(
      target_frame, target_time, source_frame, source_time, fixed_frame,
      tf2::Duration(std::chrono::nanoseconds::zero()), errstr) &&
    (clock_->now() + rclcpp::Duration(3, 0) >= start_time) &&  // don't wait bag loop detected
    (rclcpp::ok()))  // Make sure we haven't been stopped (won't work for pytf)
  {
    // TODO(sloretz) sleep using clock_->sleep_for when implemented
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  bool retval = canTransform(
    target_frame, target_time,
    source_frame, source_time, fixed_frame, errstr);
  rclcpp::Time current_time = clock_->now();
  conditionally_append_timeout_info(errstr, start_time, current_time, rclcpp_timeout);
  return retval;
}

TransformStampedFuture
Buffer::waitForTransform(
  const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
  const tf2::Duration & timeout, TransformReadyCallback callback)
{
  if (nullptr == timer_interface_) {
    throw CreateTimerInterfaceException("timer interface not set before call to waitForTransform");
  }

  auto promise = std::make_shared<std::promise<geometry_msgs::msg::TransformStamped>>();
  TransformStampedFuture future(promise->get_future());

  auto cb = [this, promise, callback, future](
    tf2::TransformableRequestHandle request_handle, const std::string & target_frame,
    const std::string & source_frame, tf2::TimePoint time, tf2::TransformableResult result)
    {
      (void) request_handle;

      bool timeout_occurred = true;
      {
        std::lock_guard<std::mutex> lock(this->timer_to_request_map_mutex_);
        // Check if a timeout already occurred
        for (auto it = timer_to_request_map_.begin(); it != timer_to_request_map_.end(); ++it) {
          if (request_handle == it->second) {
            // The request handle was found, so a timeout has not occurred
            this->timer_interface_->remove(it->first);
            this->timer_to_request_map_.erase(it->first);
            timeout_occurred = false;
            break;
          }
        }
      }

      if (timeout_occurred) {
        return;
      }

      if (result == tf2::TransformAvailable) {
        geometry_msgs::msg::TransformStamped msg_stamped = this->lookupTransform(
          target_frame, source_frame, time);
        promise->set_value(msg_stamped);
      } else {
        promise->set_exception(
          std::make_exception_ptr(
            tf2::LookupException(
              "Failed to transform from " + source_frame + " to " + target_frame)));
      }
      callback(future);
    };

  auto handle = addTransformableRequest(cb, target_frame, source_frame, time);
  future.setHandle(handle);
  if (0 == handle) {
    // Immediately transformable
    geometry_msgs::msg::TransformStamped msg_stamped = lookupTransform(
      target_frame, source_frame, time);
    promise->set_value(msg_stamped);
    callback(future);
  } else if (0xffffffffffffffffULL == handle) {
    // Never transformable
    promise->set_exception(
      std::make_exception_ptr(
        tf2::LookupException(
          "Failed to transform from " + source_frame + " to " + target_frame)));
    callback(future);
  } else {
    std::lock_guard<std::mutex> lock(timer_to_request_map_mutex_);
    auto timer_handle = timer_interface_->createTimer(
      clock_,
      timeout,
      std::bind(&Buffer::timerCallback, this, std::placeholders::_1, promise, future, callback));

    // Save association between timer and request handle
    timer_to_request_map_[timer_handle] = handle;
  }
  return future;
}

void
Buffer::cancel(const TransformStampedFuture & ts_future)
{
  cancelTransformableRequest(ts_future.getHandle());

  std::lock_guard<std::mutex> lock(timer_to_request_map_mutex_);
  auto iter = timer_to_request_map_.find(ts_future.getHandle());
  if (iter != timer_to_request_map_.end()) {
    timer_to_request_map_.erase(iter);
  }
}

void
Buffer::timerCallback(
  const TimerHandle & timer_handle,
  std::shared_ptr<std::promise<geometry_msgs::msg::TransformStamped>> promise,
  TransformStampedFuture future,
  TransformReadyCallback callback)
{
  bool timer_is_valid = false;
  tf2::TransformableRequestHandle request_handle = 0u;
  {
    std::lock_guard<std::mutex> lock(timer_to_request_map_mutex_);
    auto timer_and_request_it = timer_to_request_map_.find(timer_handle);
    timer_is_valid = (timer_to_request_map_.end() != timer_and_request_it);
    if (timer_is_valid) {
      request_handle = timer_and_request_it->second;
      timer_to_request_map_.erase(timer_handle);
      timer_interface_->remove(timer_handle);
    }
  }

  if (timer_is_valid) {
    cancelTransformableRequest(request_handle);
    promise->set_exception(
      std::make_exception_ptr(
        tf2::TimeoutException(std::string("Timed out waiting for transform"))));
    callback(future);
  }
}

bool Buffer::getFrames(
  const tf2_msgs::srv::FrameGraph::Request::SharedPtr req,
  tf2_msgs::srv::FrameGraph::Response::SharedPtr res)
{
  (void)req;
  res->frame_yaml = allFramesAsYAML();
  return true;
}

bool Buffer::checkAndErrorDedicatedThreadPresent(std::string * error_str) const
{
  if (isUsingDedicatedThread()) {
    return true;
  }

  if (error_str) {
    *error_str = tf2_ros::threading_error;
  }

  RCLCPP_ERROR(getLogger(), "%s", tf2_ros::threading_error);
  return false;
}

rclcpp::Logger Buffer::getLogger() const
{
  return node_logging_interface_ ? node_logging_interface_->get_logger() : rclcpp::get_logger(
    "tf2_buffer");
}

}  // namespace tf2_ros
