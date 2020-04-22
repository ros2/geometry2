/*
 * Copyright (c) 2019, Open Source Robotics Foundation, Inc.
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
#include <functional>

#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2/time.h>

#include <tf2_ros/create_timer_ros.h>

namespace tf2_ros
{

CreateTimerROS::CreateTimerROS(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers)
: node_base_(node_base), node_timers_(node_timers), next_timer_handle_index_(0)
{
}

TimerHandle
CreateTimerROS::createTimer(
  rclcpp::Clock::SharedPtr clock,
  const tf2::Duration & period,
  TimerCallbackType callback)
{
  std::lock_guard<std::mutex> lock(timers_map_mutex_);
  auto timer_handle_index = next_timer_handle_index_++;
  auto timer = rclcpp::create_timer<std::function<void()>>(
    node_base_,
    node_timers_,
    clock,
    period,
    std::bind(&CreateTimerROS::timerCallback, this, timer_handle_index, callback));
  timers_map_[timer_handle_index] = timer;
  return timer_handle_index;
}

void
CreateTimerROS::cancel(const TimerHandle & timer_handle)
{
  std::lock_guard<std::mutex> lock(timers_map_mutex_);
  cancelNoLock(timer_handle);
}

void
CreateTimerROS::reset(const TimerHandle & timer_handle)
{
  std::lock_guard<std::mutex> lock(timers_map_mutex_);
  try {
    timers_map_.at(timer_handle)->reset();
  } catch (const std::out_of_range &) {
    throw InvalidTimerHandleException("Invalid timer handle in reset()");
  }
}

void
CreateTimerROS::remove(const TimerHandle & timer_handle)
{
  std::lock_guard<std::mutex> lock(timers_map_mutex_);
  try {
    cancelNoLock(timer_handle);
  } catch (const InvalidTimerHandleException &) {
    throw InvalidTimerHandleException("Invalid timer handle in remove()");
  }
  timers_map_.erase(timer_handle);
}

void
CreateTimerROS::cancelNoLock(const TimerHandle & timer_handle)
{
  try {
    timers_map_.at(timer_handle)->cancel();
  } catch (const std::out_of_range &) {
    throw InvalidTimerHandleException("Invalid timer handle in cancel()");
  }
}

void
CreateTimerROS::timerCallback(
  const TimerHandle & timer_handle,
  TimerCallbackType callback)
{
  callback(timer_handle);
}

}  // namespace tf2_ros
