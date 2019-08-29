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
 *     * Neither the name of the copyright holder nor the names of its
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

#include <chrono>
#include <future>

#include <gtest/gtest.h>

#include <tf2_msgs/action/lookup_transform.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_server.h>

static const std::string ACTION_NAME = "test_tf2_buffer_action";

class MockBufferClient : public rclcpp::Node
{
  using LookupTransformAction = tf2_msgs::action::LookupTransform;
  using GoalHandle = rclcpp_action::ClientGoalHandle<LookupTransformAction>;

public:
  MockBufferClient()
    : rclcpp::Node("mock_buffer_client"),
      accepted_(false)
  {
    action_client_ = rclcpp_action::create_client<LookupTransformAction>(
      get_node_base_interface(),
      get_node_graph_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      ACTION_NAME);
  }

  std::shared_future<bool> send_goal(
    const std::string target_frame,
    const std::string source_frame,
    const std::chrono::seconds timeout,
    const std::string fixed_frame = "",
    const bool advanced = false)
  {
    // Promise is set to true when the result is received
    auto promise = std::make_shared<std::promise<bool>>();

    auto goal = LookupTransformAction::Goal();
    goal.target_frame = target_frame;
    goal.source_frame = source_frame;
    goal.source_time.sec = 0;
    goal.source_time.nanosec = 0u;
    goal.timeout.sec = static_cast<int32_t>(timeout.count());
    goal.target_time.sec = 0;
    goal.target_time.nanosec = 0u;
    goal.fixed_frame = fixed_frame;
    goal.advanced = advanced;

    auto send_goal_options = rclcpp_action::Client<LookupTransformAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](std::shared_future<GoalHandle::SharedPtr> future) {
        auto goal_handle = future.get();
        if (!goal_handle) {
          this->accepted_ = false;
        } else {
          this->accepted_ = true;
        }
      };

    send_goal_options.result_callback = [this, promise](const GoalHandle::WrappedResult & result) {
      this->result_ = result;
      promise->set_value(true);
    };
    action_client_->async_send_goal(goal, send_goal_options);
    return std::shared_future<bool>(promise->get_future());
  }

  bool accepted_;
  GoalHandle::WrappedResult result_;
  rclcpp_action::Client<LookupTransformAction>::SharedPtr action_client_;
};

class TestBufferServer : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp()
  {
    node_ = std::make_shared<rclcpp::Node>("tf_buffer");
    buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    server_ = std::make_unique<tf2_ros::BufferServer>(*buffer_, node_, ACTION_NAME);
    mock_client_ = std::make_shared<MockBufferClient>();

    executor_.add_node(node_);
    executor_.add_node(mock_client_);

    // Wait for discovery
    ASSERT_TRUE(mock_client_->action_client_->wait_for_action_server(std::chrono::seconds(10)));
  }

  void TearDown()
  {
    mock_client_.reset();
    server_.reset();
    buffer_.reset();
    node_.reset();
  }

  void setTransform(
    const std::string target_frame,
    const std::string source_frame,
    geometry_msgs::msg::Transform transform)
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.frame_id = target_frame;
    transform_stamped.child_frame_id = source_frame;
    transform_stamped.transform = transform;
    buffer_->setTransform(transform_stamped, "mock_tf_authority");
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::BufferServer> server_;
  std::shared_ptr<MockBufferClient> mock_client_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(TestBufferServer, lookup_transform_available)
{
  EXPECT_FALSE(mock_client_->accepted_);

  // Send goal with 1 second timeout
  auto result_ready_future = mock_client_->send_goal(
    "test_target_link", "test_source_link", std::chrono::seconds(1));

  // Make transform available
  geometry_msgs::msg::Transform transform;
  transform.translation.x = 1.0;
  transform.translation.y = -2.0;
  transform.translation.z = 3.0;
  transform.rotation.w = 1.0;
  setTransform("test_target_link", "test_source_link", transform);

  auto spin_result = executor_.spin_until_future_complete(
    result_ready_future, std::chrono::seconds(3));
  ASSERT_EQ(spin_result, rclcpp::executor::FutureReturnCode::SUCCESS);

  EXPECT_TRUE(mock_client_->accepted_);
  EXPECT_EQ(mock_client_->result_.code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(TestBufferServer, lookup_transform_timeout)
{
  EXPECT_FALSE(mock_client_->accepted_);

  // Send goal with 1 second timeout
  auto result_ready_future = mock_client_->send_goal(
    "test_target_link", "test_source_link", std::chrono::seconds(1));

  auto start_time = std::chrono::system_clock::now();
  auto spin_result = executor_.spin_until_future_complete(
    result_ready_future, std::chrono::seconds(3));
  ASSERT_EQ(spin_result, rclcpp::executor::FutureReturnCode::SUCCESS);
  auto end_time = std::chrono::system_clock::now();

  auto time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  EXPECT_NEAR(static_cast<double>(time_ms.count()), 1000.0, 100.0);

  EXPECT_TRUE(mock_client_->accepted_);
  EXPECT_EQ(mock_client_->result_.code, rclcpp_action::ResultCode::ABORTED);
}

TEST_F(TestBufferServer, lookup_transform_delayed)
{
  EXPECT_FALSE(mock_client_->accepted_);

  // Send goal with 5 second timeout
  auto result_ready_future = mock_client_->send_goal(
    "test_target_link", "test_source_link", std::chrono::seconds(5));

  // Expect executor to timeout since transform is not available yet
  auto spin_result = executor_.spin_until_future_complete(
    result_ready_future, std::chrono::seconds(1));
  EXPECT_EQ(spin_result, rclcpp::executor::FutureReturnCode::TIMEOUT);

  EXPECT_TRUE(mock_client_->accepted_);

  // Make transform available
  geometry_msgs::msg::Transform transform;
  transform.translation.x = 1.0;
  transform.translation.y = -2.0;
  transform.translation.z = 3.0;
  transform.rotation.w = 1.0;
  setTransform("test_target_link", "test_source_link", transform);

  // Wait some more
  spin_result = executor_.spin_until_future_complete(
    result_ready_future, std::chrono::seconds(3));
  ASSERT_EQ(spin_result, rclcpp::executor::FutureReturnCode::SUCCESS);

  EXPECT_EQ(mock_client_->result_.code, rclcpp_action::ResultCode::SUCCEEDED);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

