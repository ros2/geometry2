// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <tf2_ros/async_wait_for_transform.hpp>

namespace tf2_ros
{
/// \brief Helper class that creates a timer to poll for a transform
/// \internal
class TransformWaiter
{
public:

  TransformWaiter(
    rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers,
    rclcpp::Clock::SharedPtr clock,
    std::shared_ptr<tf2_ros::BufferInterface> buffer,
    std::string target_frame,
    std::string source_frame,
    rclcpp::Time transform_time,
    rclcpp::Duration timeout,
    TransformCallback callback,
    rclcpp::Duration period,
    rclcpp::callback_group::CallbackGroup::SharedPtr group,
    rclcpp::Context::SharedPtr context)
    :
    promise_(),
    future_(promise_.get_future()),
    buffer_(buffer),
    clock_(clock),
    target_frame_(target_frame),
    source_frame_(source_frame),
    tf_time_(tf2::timeFromSec(transform_time.seconds())),
    callback_(callback)
  {
    end_time_ = clock_->now() + timeout;
    // create a timer and add it to the node
    timer_ = rclcpp::GenericTimer<std::function<void ()>>::make_shared(
      clock,
      period.to_chrono<std::chrono::nanoseconds>(),
      std::bind(&TransformWaiter::on_timer, this),
      context);

    node_timers->add_timer(timer_, group);
  }

private:
  // must put this before future_ so it is constructed first
  std::promise<geometry_msgs::msg::TransformStamped> promise_;

public:
  std::shared_future<geometry_msgs::msg::TransformStamped> future_;

private:
  void on_timer()
  {
    try {
      // Get the transform
      geometry_msgs::msg::TransformStamped transform = buffer_->lookupTransform(
        target_frame_, source_frame_, tf_time_, std::chrono::nanoseconds(0));

      promise_.set_value(transform);
      timer_->cancel();

      if (callback_) {
        callback_(transform);
      }
    } catch (const tf2::TransformException &) {
      // transform not available
      if (clock_->now() >= end_time_) {
        // Timeout reached
        timer_->cancel();
        promise_.set_exception(std::current_exception());
      }
    }
  }

  std::shared_ptr<tf2_ros::BufferInterface> buffer_;
  rclcpp::Clock::SharedPtr clock_;
  std::string target_frame_;
  std::string source_frame_;
  tf2::TimePoint tf_time_;
  TransformCallback callback_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time end_time_;
};

std::shared_ptr<std::shared_future<geometry_msgs::msg::TransformStamped>>
async_wait_for_transform(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base_interface,
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr timer_interface,
  rclcpp::Clock::SharedPtr clock,
  std::shared_ptr<tf2_ros::BufferInterface> buffer,
  std::string target_frame,
  std::string source_frame,
  rclcpp::Time transform_time,
  rclcpp::Duration timeout,
  TransformCallback callback,
  rclcpp::Duration period,
  rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  auto waiter = std::make_shared<TransformWaiter>(
    timer_interface,
    clock,
    buffer,
    target_frame,
    source_frame,
    transform_time,
    timeout,
    callback,
    period,
    group,
    base_interface->get_context());

  // return shared_ptr to waiter aliased to the future
  return std::shared_ptr<std::shared_future<geometry_msgs::msg::TransformStamped>>(
    waiter, &(waiter->future_));
}
}  // namespace tf2_ros
