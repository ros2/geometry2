
.. _program_listing_file__home_ahcorde_ros2_rolling_src_ros2_geometry2_tf2_ros_include_tf2_ros_buffer_interface.h:

Program Listing for File buffer_interface.h
===========================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ahcorde_ros2_rolling_src_ros2_geometry2_tf2_ros_include_tf2_ros_buffer_interface.h>` (``/home/ahcorde/ros2_rolling/src/ros2/geometry2/tf2_ros/include/tf2_ros/buffer_interface.h``)

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
   
   #ifndef TF2_ROS__BUFFER_INTERFACE_H_
   #define TF2_ROS__BUFFER_INTERFACE_H_
   
   #include <chrono>
   #include <functional>
   #include <future>
   #include <string>
   
   #include "tf2_ros/visibility_control.h"
   #include "tf2/transform_datatypes.h"
   #include "tf2/exceptions.h"
   #include "tf2/convert.h"
   
   #include "builtin_interfaces/msg/duration.hpp"
   #include "builtin_interfaces/msg/time.hpp"
   #include "geometry_msgs/msg/transform_stamped.hpp"
   #include "rclcpp/rclcpp.hpp"
   
   namespace tf2_ros
   {
   
   inline builtin_interfaces::msg::Time toMsg(const tf2::TimePoint & t)
   {
     std::chrono::nanoseconds ns =
       std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch());
     std::chrono::seconds s =
       std::chrono::duration_cast<std::chrono::seconds>(t.time_since_epoch());
     builtin_interfaces::msg::Time time_msg;
     time_msg.sec = static_cast<int32_t>(s.count());
     time_msg.nanosec = static_cast<uint32_t>(ns.count() % 1000000000ull);
     return time_msg;
   }
   
   inline tf2::TimePoint fromMsg(const builtin_interfaces::msg::Time & time_msg)
   {
     int64_t d = time_msg.sec * 1000000000ull + time_msg.nanosec;
     std::chrono::nanoseconds ns(d);
     return tf2::TimePoint(std::chrono::duration_cast<tf2::Duration>(ns));
   }
   
   inline builtin_interfaces::msg::Duration toMsg(const tf2::Duration & t)
   {
     std::chrono::nanoseconds ns =
       std::chrono::duration_cast<std::chrono::nanoseconds>(t);
     std::chrono::seconds s =
       std::chrono::duration_cast<std::chrono::seconds>(t);
     builtin_interfaces::msg::Duration duration_msg;
     duration_msg.sec = static_cast<int32_t>(s.count());
     duration_msg.nanosec = static_cast<uint32_t>(ns.count() % 1000000000ull);
     return duration_msg;
   }
   
   inline tf2::Duration fromMsg(const builtin_interfaces::msg::Duration & duration_msg)
   {
     int64_t d = duration_msg.sec * 1000000000ull + duration_msg.nanosec;
     std::chrono::nanoseconds ns(d);
     return tf2::Duration(std::chrono::duration_cast<tf2::Duration>(ns));
   }
   
   inline double timeToSec(const builtin_interfaces::msg::Time & time_msg)
   {
     auto ns = std::chrono::duration<double, std::nano>(time_msg.nanosec);
     auto s = std::chrono::duration<double>(time_msg.sec);
     return (s + std::chrono::duration_cast<std::chrono::duration<double>>(ns)).count();
   }
   
   inline tf2::TimePoint fromRclcpp(const rclcpp::Time & time)
   {
     // tf2::TimePoint is a typedef to a system time point, but rclcpp::Time may be ROS time.
     // Ignore that, and assume the clock used from rclcpp time points is consistent.
     return tf2::TimePoint(std::chrono::nanoseconds(time.nanoseconds()));
   }
   
   inline rclcpp::Time toRclcpp(const tf2::TimePoint & time)
   {
     // tf2::TimePoint is a typedef to a system time point, but rclcpp::Time may be ROS time.
     // Use whatever the default clock is.
     return rclcpp::Time(std::chrono::nanoseconds(time.time_since_epoch()).count());
   }
   
   inline tf2::Duration fromRclcpp(const rclcpp::Duration & duration)
   {
     return tf2::Duration(std::chrono::nanoseconds(duration.nanoseconds()));
   }
   
   inline rclcpp::Duration toRclcpp(const tf2::Duration & duration)
   {
     return rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(duration));
   }
   
   class BufferInterface
   {
   public:
     TF2_ROS_PUBLIC
     virtual geometry_msgs::msg::TransformStamped
     lookupTransform(
       const std::string & target_frame, const std::string & source_frame,
       const tf2::TimePoint & time, const tf2::Duration timeout) const = 0;
   
     TF2_ROS_PUBLIC
     virtual geometry_msgs::msg::TransformStamped
     lookupTransform(
       const std::string & target_frame, const tf2::TimePoint & target_time,
       const std::string & source_frame, const tf2::TimePoint & source_time,
       const std::string & fixed_frame, const tf2::Duration timeout) const = 0;
   
   
     TF2_ROS_PUBLIC
     virtual bool
     canTransform(
       const std::string & target_frame, const std::string & source_frame,
       const tf2::TimePoint & time, const tf2::Duration timeout,
       std::string * errstr = NULL) const = 0;
   
     TF2_ROS_PUBLIC
     virtual bool
     canTransform(
       const std::string & target_frame, const tf2::TimePoint & target_time,
       const std::string & source_frame, const tf2::TimePoint & source_time,
       const std::string & fixed_frame, const tf2::Duration timeout,
       std::string * errstr = NULL) const = 0;
   
     template<class T>
     T & transform(
       const T & in, T & out,
       const std::string & target_frame, tf2::Duration timeout = tf2::durationFromSec(0.0)) const
     {
       // do the transform
       tf2::doTransform(
         in, out, lookupTransform(target_frame, tf2::getFrameId(in), tf2::getTimestamp(in), timeout));
       return out;
     }
   
     template<class T>
     T transform(
       const T & in,
       const std::string & target_frame, tf2::Duration timeout = tf2::durationFromSec(0.0)) const
     {
       T out;
       return this->transform(in, out, target_frame, timeout);
     }
   
     template<class A, class B>
     B & transform(
       const A & in, B & out,
       const std::string & target_frame, tf2::Duration timeout = tf2::durationFromSec(0.0)) const
     {
       A copy = this->transform(in, target_frame, timeout);
       tf2::convert(copy, out);
       return out;
     }
   
     template<class T>
     T & transform(
       const T & in, T & out,
       const std::string & target_frame, const tf2::TimePoint & target_time,
       const std::string & fixed_frame, tf2::Duration timeout = tf2::durationFromSec(0.0)) const
     {
       // do the transform
       tf2::doTransform(
         in, out, lookupTransform(
           target_frame, target_time,
           tf2::getFrameId(in), tf2::getTimestamp(in),
           fixed_frame, timeout));
       return out;
     }
   
     template<class T>
     T transform(
       const T & in,
       const std::string & target_frame, const tf2::TimePoint & target_time,
       const std::string & fixed_frame, tf2::Duration timeout = tf2::durationFromSec(0.0)) const
     {
       T out;
       return this->transform(in, out, target_frame, target_time, fixed_frame, timeout);
     }
   
     template<class A, class B>
     B & transform(
       const A & in, B & out,
       const std::string & target_frame, const tf2::TimePoint & target_time,
       const std::string & fixed_frame, tf2::Duration timeout = tf2::durationFromSec(0.0)) const
     {
       // do the transform
       A copy = this->transform(in, target_frame, target_time, fixed_frame, timeout);
       tf2::convert(copy, out);
       return out;
     }
   
     virtual ~BufferInterface()
     {
     }
   };
   
   }  // namespace tf2_ros
   
   #endif  // TF2_ROS__BUFFER_INTERFACE_H_
