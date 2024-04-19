
.. _program_listing_file__home_ahcorde_ros2_rolling_src_ros2_geometry2_tf2_ros_include_tf2_ros_create_timer_interface.h:

Program Listing for File create_timer_interface.h
=================================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ahcorde_ros2_rolling_src_ros2_geometry2_tf2_ros_include_tf2_ros_create_timer_interface.h>` (``/home/ahcorde/ros2_rolling/src/ros2/geometry2/tf2_ros/include/tf2_ros/create_timer_interface.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

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
   
   #ifndef TF2_ROS__CREATE_TIMER_INTERFACE_H_
   #define TF2_ROS__CREATE_TIMER_INTERFACE_H_
   
   #include <functional>
   #include <memory>
   #include <stdexcept>
   #include <string>
   
   #include "tf2/time.h"
   
   #include "tf2_ros/visibility_control.h"
   
   #include "rclcpp/rclcpp.hpp"
   
   namespace tf2_ros
   {
   
   using TimerHandle = uint64_t;
   using TimerCallbackType = std::function<void (const TimerHandle &)>;
   
   class CreateTimerInterfaceException : public std::runtime_error
   {
   public:
     TF2_ROS_PUBLIC
     explicit CreateTimerInterfaceException(const std::string & errorDescription)
     : std::runtime_error(errorDescription)
     {
     }
   };
   
   class InvalidTimerHandleException : public std::runtime_error
   {
   public:
     TF2_ROS_PUBLIC
     explicit InvalidTimerHandleException(const std::string & description)
     : std::runtime_error(description)
     {
     }
   };
   
   class CreateTimerInterface
   {
   public:
     using SharedPtr = std::shared_ptr<CreateTimerInterface>;
     using ConstSharedPtr = std::shared_ptr<const CreateTimerInterface>;
     using UniquePtr = std::unique_ptr<CreateTimerInterface>;
   
     TF2_ROS_PUBLIC
     virtual
     ~CreateTimerInterface() = default;
   
     TF2_ROS_PUBLIC
     virtual TimerHandle
     createTimer(
       rclcpp::Clock::SharedPtr clock,
       const tf2::Duration & period,
       TimerCallbackType callback) = 0;
   
     TF2_ROS_PUBLIC
     virtual void
     cancel(const TimerHandle & timer_handle) = 0;
   
     TF2_ROS_PUBLIC
     virtual void
     reset(const TimerHandle & timer_handle) = 0;
   
     TF2_ROS_PUBLIC
     virtual void
     remove(const TimerHandle & timer_handle) = 0;
   };  // class CreateTimerInterface
   
   }  // namespace tf2_ros
   
   #endif  // TF2_ROS__CREATE_TIMER_INTERFACE_H_
