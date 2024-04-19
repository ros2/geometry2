
.. _program_listing_file__home_ahcorde_ros2_rolling_src_ros2_geometry2_tf2_ros_include_tf2_ros_buffer.h:

Program Listing for File buffer.h
=================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ahcorde_ros2_rolling_src_ros2_geometry2_tf2_ros_include_tf2_ros_buffer.h>` (``/home/ahcorde/ros2_rolling/src/ros2/geometry2/tf2_ros/include/tf2_ros/buffer.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

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
   
   #ifndef TF2_ROS__BUFFER_H_
   #define TF2_ROS__BUFFER_H_
   
   #include <future>
   #include <memory>
   #include <mutex>
   #include <string>
   #include <unordered_map>
   
   #include "tf2_ros/async_buffer_interface.h"
   #include "tf2_ros/buffer_interface.h"
   #include "tf2_ros/create_timer_interface.h"
   #include "tf2_ros/visibility_control.h"
   #include "tf2/buffer_core.h"
   #include "tf2/time.h"
   
   #include "geometry_msgs/msg/transform_stamped.hpp"
   #include "tf2_msgs/srv/frame_graph.hpp"
   #include "rclcpp/rclcpp.hpp"
   
   namespace tf2_ros
   {
   
   class Buffer : public BufferInterface, public AsyncBufferInterface, public tf2::BufferCore
   {
   public:
     using tf2::BufferCore::lookupTransform;
     using tf2::BufferCore::canTransform;
     using SharedPtr = std::shared_ptr<tf2_ros::Buffer>;
   
     TF2_ROS_PUBLIC Buffer(
       rclcpp::Clock::SharedPtr clock,
       tf2::Duration cache_time = tf2::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME),
       rclcpp::Node::SharedPtr node = rclcpp::Node::SharedPtr());
   
     TF2_ROS_PUBLIC
     geometry_msgs::msg::TransformStamped
     lookupTransform(
       const std::string & target_frame, const std::string & source_frame,
       const tf2::TimePoint & time, const tf2::Duration timeout) const override;
   
     TF2_ROS_PUBLIC
     geometry_msgs::msg::TransformStamped
     lookupTransform(
       const std::string & target_frame, const std::string & source_frame,
       const rclcpp::Time & time,
       const rclcpp::Duration timeout = rclcpp::Duration::from_nanoseconds(0)) const
     {
       return lookupTransform(target_frame, source_frame, fromRclcpp(time), fromRclcpp(timeout));
     }
   
     TF2_ROS_PUBLIC
     geometry_msgs::msg::TransformStamped
     lookupTransform(
       const std::string & target_frame, const tf2::TimePoint & target_time,
       const std::string & source_frame, const tf2::TimePoint & source_time,
       const std::string & fixed_frame, const tf2::Duration timeout) const override;
   
     TF2_ROS_PUBLIC
     geometry_msgs::msg::TransformStamped
     lookupTransform(
       const std::string & target_frame, const rclcpp::Time & target_time,
       const std::string & source_frame, const rclcpp::Time & source_time,
       const std::string & fixed_frame,
       const rclcpp::Duration timeout = rclcpp::Duration::from_nanoseconds(0)) const
     {
       return lookupTransform(
         target_frame, fromRclcpp(target_time),
         source_frame, fromRclcpp(source_time),
         fixed_frame, fromRclcpp(timeout));
     }
   
     TF2_ROS_PUBLIC
     bool
     canTransform(
       const std::string & target_frame, const std::string & source_frame,
       const tf2::TimePoint & target_time, const tf2::Duration timeout,
       std::string * errstr = NULL) const override;
   
     TF2_ROS_PUBLIC
     bool
     canTransform(
       const std::string & target_frame, const std::string & source_frame,
       const rclcpp::Time & time,
       const rclcpp::Duration timeout = rclcpp::Duration::from_nanoseconds(0),
       std::string * errstr = NULL) const
     {
       return canTransform(target_frame, source_frame, fromRclcpp(time), fromRclcpp(timeout), errstr);
     }
   
     TF2_ROS_PUBLIC
     bool
     canTransform(
       const std::string & target_frame, const tf2::TimePoint & target_time,
       const std::string & source_frame, const tf2::TimePoint & source_time,
       const std::string & fixed_frame, const tf2::Duration timeout,
       std::string * errstr = NULL) const override;
   
     TF2_ROS_PUBLIC
     bool
     canTransform(
       const std::string & target_frame, const rclcpp::Time & target_time,
       const std::string & source_frame, const rclcpp::Time & source_time,
       const std::string & fixed_frame,
       const rclcpp::Duration timeout = rclcpp::Duration::from_nanoseconds(0),
       std::string * errstr = NULL) const
     {
       return canTransform(
         target_frame, fromRclcpp(target_time),
         source_frame, fromRclcpp(source_time),
         fixed_frame, fromRclcpp(timeout),
         errstr);
     }
   
     TF2_ROS_PUBLIC
     TransformStampedFuture
     waitForTransform(
       const std::string & target_frame, const std::string & source_frame,
       const tf2::TimePoint & time, const tf2::Duration & timeout,
       TransformReadyCallback callback) override;
   
     TF2_ROS_PUBLIC
     TransformStampedFuture
     waitForTransform(
       const std::string & target_frame, const std::string & source_frame,
       const rclcpp::Time & time,
       const rclcpp::Duration & timeout, TransformReadyCallback callback)
     {
       return waitForTransform(
         target_frame, source_frame,
         fromRclcpp(time), fromRclcpp(timeout),
         callback);
     }
   
     TF2_ROS_PUBLIC
     void
     cancel(const TransformStampedFuture & ts_future) override;
   
     TF2_ROS_PUBLIC
     inline void
     setCreateTimerInterface(CreateTimerInterface::SharedPtr create_timer_interface)
     {
       timer_interface_ = create_timer_interface;
     }
   
   private:
     void timerCallback(
       const TimerHandle & timer_handle,
       std::shared_ptr<std::promise<geometry_msgs::msg::TransformStamped>> promise,
       TransformStampedFuture future,
       TransformReadyCallback callback);
   
     bool getFrames(
       const tf2_msgs::srv::FrameGraph::Request::SharedPtr req,
       tf2_msgs::srv::FrameGraph::Response::SharedPtr res);
   
     void onTimeJump(const rcl_time_jump_t & jump);
   
     // conditionally error if dedicated_thread unset.
     bool checkAndErrorDedicatedThreadPresent(std::string * errstr) const;
   
     rclcpp::Logger getLogger() const;
   
     // framegraph service
     rclcpp::Service<tf2_msgs::srv::FrameGraph>::SharedPtr frames_server_;
   
     rclcpp::Clock::SharedPtr clock_;
   
     rclcpp::Node::SharedPtr node_;
   
     CreateTimerInterface::SharedPtr timer_interface_;
   
     std::unordered_map<TimerHandle, tf2::TransformableRequestHandle> timer_to_request_map_;
   
     std::mutex timer_to_request_map_mutex_;
   
     rclcpp::JumpHandler::SharedPtr jump_handler_;
   };
   
   static const char threading_error[] = "Do not call canTransform or lookupTransform with a timeout "
     "unless you are using another thread for populating data. Without a dedicated thread it will "
     "always timeout.  If you have a separate thread servicing tf messages, call "
     "setUsingDedicatedThread(true) on your Buffer instance.";
   
   }  // namespace tf2_ros
   
   #endif  // TF2_ROS__BUFFER_H_
