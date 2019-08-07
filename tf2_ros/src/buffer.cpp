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


#include "tf2_ros/buffer.h"

#include <exception>
#include <limits>
#include <mutex>
#include <sstream>
#include <thread>

//TODO(tfoote replace these terrible macros)
#define ROS_ERROR printf
#define ROS_FATAL printf
#define ROS_INFO printf
#define ROS_WARN printf


namespace tf2_ros
{

Buffer::Buffer(rclcpp::Clock::SharedPtr clock, tf2::Duration cache_time) :
  BufferCore(cache_time), clock_(clock), timer_interface_(nullptr)
{
  if (nullptr == clock_)
  {
    throw std::invalid_argument("clock must be a valid instance");
  }

  auto post_jump_cb = [this](const rcl_time_jump_t & jump_info) { onTimeJump(jump_info); };

  rcl_jump_threshold_t jump_threshold;
  // Disable forward jump callbacks
  jump_threshold.min_forward.nanoseconds = 0;
  // Anything backwards is a jump
  jump_threshold.min_backward.nanoseconds = -1;
  // Callback if the clock changes too
  jump_threshold.on_clock_change = true;

  jump_handler_ = clock_->create_jump_callback(nullptr, post_jump_cb, jump_threshold);

  // TODO(tfoote) reenable 
  // if(debug && !ros::exists("~tf2_frames", false))
  // {
  //   ros::NodeHandle n("~");
  //   frames_server_ = n.advertiseService("tf2_frames", &Buffer::getFrames, this);
  // }
}

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
  return rclcpp::Duration(std::chrono::nanoseconds(duration).count());
}

geometry_msgs::msg::TransformStamped 
Buffer::lookupTransform(const std::string& target_frame, const std::string& source_frame,
                        const tf2::TimePoint& lookup_time, const tf2::Duration timeout) const
{
  canTransform(target_frame, source_frame, lookup_time, timeout);
  return lookupTransform(target_frame, source_frame, lookup_time);
}

void Buffer::onTimeJump(const struct rcl_time_jump_t & time_jump)
{
  if (RCL_ROS_TIME_ACTIVATED == time_jump.clock_change ||
      RCL_ROS_TIME_DEACTIVATED == time_jump.clock_change)
  {
    ROS_WARN("Detected time source change. Clearing TF buffer.");
    clear();
  }
  else if (time_jump.delta.nanoseconds < 0)
  {
    ROS_WARN("Detected jump back in time. Clearing TF buffer.");
    clear();
  }
}

geometry_msgs::msg::TransformStamped 
Buffer::lookupTransform(const std::string& target_frame, const tf2::TimePoint& target_time,
                        const std::string& source_frame, const tf2::TimePoint& source_time,
                        const std::string& fixed_frame, const tf2::Duration timeout) const
{
  canTransform(target_frame, target_time, source_frame, source_time, fixed_frame, timeout);
  return lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame);
}

void conditionally_append_timeout_info(std::string * errstr, const rclcpp::Time& start_time,
                                       const rclcpp::Time & current_time,
                                       const rclcpp::Duration& timeout)
{
  if (errstr)
  {
    std::stringstream ss;
    ss << ". canTransform returned after "
       << tf2::durationToSec(from_rclcpp(current_time - start_time))
       <<" timeout was " << tf2::durationToSec(from_rclcpp(timeout)) << ".";
    (*errstr) += ss.str();
  }
}

bool
Buffer::canTransform(const std::string& target_frame, const std::string& source_frame, 
                     const tf2::TimePoint& time, const tf2::Duration timeout, std::string* errstr) const
{
  if (!checkAndErrorDedicatedThreadPresent(errstr))
    return false;

  rclcpp::Duration rclcpp_timeout(to_rclcpp(timeout));

  // poll for transform if timeout is set
  rclcpp::Time start_time = clock_->now();
  while (clock_->now() < start_time + rclcpp_timeout &&
         !canTransform(target_frame, source_frame, time) &&
         (clock_->now() + rclcpp::Duration(3, 0) >= start_time) &&  //don't wait when we detect a bag loop
         (rclcpp::ok()// || !ros::isInitialized() //TODO(tfoote) restore
       )) // Make sure we haven't been stopped (won't work for pytf)
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
Buffer::canTransform(const std::string& target_frame, const tf2::TimePoint& target_time,
                     const std::string& source_frame, const tf2::TimePoint& source_time,
                     const std::string& fixed_frame, const tf2::Duration timeout, std::string* errstr) const
{
  if (!checkAndErrorDedicatedThreadPresent(errstr))
    return false;

  rclcpp::Duration rclcpp_timeout(to_rclcpp(timeout));

  // poll for transform if timeout is set
  rclcpp::Time start_time = clock_->now();
  while (clock_->now() < start_time + rclcpp_timeout &&
         !canTransform(target_frame, target_time, source_frame, source_time, fixed_frame) &&
         (clock_->now() + rclcpp::Duration(3, 0) >= start_time) &&  //don't wait when we detect a bag loop
         (rclcpp::ok() //|| !ros::isInitialized() //TODO(tfoote) restore
          )
        ) // Make sure we haven't been stopped (won't work for pytf)
         {  
          // TODO(sloretz) sleep using clock_->sleep_for when implemented
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
         }
  bool retval = canTransform(target_frame, target_time, source_frame, source_time, fixed_frame, errstr);
  rclcpp::Time current_time = clock_->now();
  conditionally_append_timeout_info(errstr, start_time, current_time, rclcpp_timeout);
  return retval; 
}

TransformStampedFuture
Buffer::waitForTransform(const std::string& target_frame, const std::string& source_frame, const tf2::TimePoint& time,
                         const tf2::Duration& timeout, TransformReadyCallback callback)
{
  if (nullptr == timer_interface_) {
    throw CreateTimerInterfaceException("timer interface not set before call to waitForTransform");
  }

  auto promise = std::make_shared<std::promise<geometry_msgs::msg::TransformStamped>>();
  TransformStampedFuture future(promise->get_future());

  auto cb_handle = addTransformableCallback([this, promise, callback, future](
    tf2::TransformableRequestHandle request_handle, const std::string& target_frame,
    const std::string& source_frame, tf2::TimePoint time, tf2::TransformableResult result)
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
        geometry_msgs::msg::TransformStamped msg_stamped = this->lookupTransform(target_frame, source_frame, time);
        promise->set_value(msg_stamped);
      } else {
        promise->set_exception(std::make_exception_ptr<tf2::LookupException>(
            "Failed to transform from " + source_frame + " to " + target_frame));
      }
      callback(future);
    });

  auto handle = addTransformableRequest(cb_handle, target_frame, source_frame, time);
  if (0 == handle) {
    // Immediately transformable
    geometry_msgs::msg::TransformStamped msg_stamped = lookupTransform(target_frame, source_frame, time);
    promise->set_value(msg_stamped);
  } else if (0xffffffffffffffffULL == handle) {
    // Never transformable
    promise->set_exception(std::make_exception_ptr<tf2::LookupException>(
          "Failed to transform from " + source_frame + " to " + target_frame));
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
Buffer::timerCallback(const TimerHandle & timer_handle,
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
    }
    timer_to_request_map_.erase(timer_handle);
    timer_interface_->remove(timer_handle);
  }

  if (timer_is_valid) {
    cancelTransformableRequest(request_handle);
    promise->set_exception(
      std::make_exception_ptr<tf2::TimeoutException>(std::string("Timed out waiting for transform")));
    callback(future);
  }
}

bool Buffer::getFrames(tf2_msgs::srv::FrameGraph::Request& req, tf2_msgs::srv::FrameGraph::Response& res) 
{
  (void)req;
  res.frame_yaml = allFramesAsYAML();
  return true;
}



bool Buffer::checkAndErrorDedicatedThreadPresent(std::string* error_str) const
{
  if (isUsingDedicatedThread())
    return true;
  


  if (error_str)
    *error_str = tf2_ros::threading_error;

  ROS_ERROR("%s", tf2_ros::threading_error.c_str());
  return false;
}



}
