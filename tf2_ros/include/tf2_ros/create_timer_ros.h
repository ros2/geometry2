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

#ifndef TF2_ROS__CREATE_TIMER_ROS_H
#define TF2_ROS__CREATE_TIMER_ROS_H

#include <mutex>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/create_timer_interface.h>
#include <tf2_ros/visibility_control.h>

namespace tf2_ros
{

/**
 * \brief Create and manage ROS timers.
 *
 * This class is thread safe.
 */
class CreateTimerROS : public CreateTimerInterface
{
public:
  TF2_ROS_PUBLIC
  CreateTimerROS(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers);

  virtual ~CreateTimerROS() = default;

  /**
   * \brief Create a new timer.
   *
   * After creation, the timer will periodically execute the user-provided callback.
   *
   * \param clock The clock providing the current time
   * \param period The interval at which the timer fires
   * \param callback The callback function to execute every interval
   */
  TF2_ROS_PUBLIC
  virtual TimerHandle
  createTimer(
    rclcpp::Clock::SharedPtr clock,
    const tf2::Duration & period,
    TimerCallbackType callback) override;

  /**
   * \brief Cancel a timer.
   *
   * The timer will stop executing user callbacks.
   *
   * \param timer_handle Handle to the timer to cancel
   * \raises tf2_ros::InvalidTimerHandleException if the timer does not exist
   */
  TF2_ROS_PUBLIC
  virtual void
  cancel(const TimerHandle & timer_handle) override;

  /**
   * \brief Reset the timer.
   *
   * The timer will reset and continue to execute user callbacks periodically.
   *
   * \param timer_handle Handle to the timer to reset
   * \raises tf2_ros::InvalidTimerHandleException if the timer does not exist
   */
  TF2_ROS_PUBLIC
  virtual void
  reset(const TimerHandle & timer_handle) override;

  /**
   * \brief Remove a timer.
   *
   * The timer will be canceled and removed from internal storage.
   *
   * \param timer_handle Handle to the timer to reset.
   * \raises tf2_ros::InvalidTimerHandleException if the timer does not exist
   */
  TF2_ROS_PUBLIC
  virtual void
  remove(const TimerHandle & timer_handle) override;

private:

  void
  cancelNoLock(const TimerHandle & timer_handle);

  void
  timerCallback(
    const TimerHandle & timer_handle,
    TimerCallbackType callback);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers_;
  TimerHandle next_timer_handle_index_;
  std::unordered_map<TimerHandle, rclcpp::TimerBase::SharedPtr> timers_map_;
  std::mutex timers_map_mutex_;
};  // class CreateTimerROS

}  // namespace tf2_ros

#endif // TF2_ROS__CREATE_TIMER_INTERFACE_H
