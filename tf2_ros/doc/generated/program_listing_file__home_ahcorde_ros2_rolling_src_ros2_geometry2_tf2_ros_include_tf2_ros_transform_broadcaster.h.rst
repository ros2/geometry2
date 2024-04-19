
.. _program_listing_file__home_ahcorde_ros2_rolling_src_ros2_geometry2_tf2_ros_include_tf2_ros_transform_broadcaster.h:

Program Listing for File transform_broadcaster.h
================================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ahcorde_ros2_rolling_src_ros2_geometry2_tf2_ros_include_tf2_ros_transform_broadcaster.h>` (``/home/ahcorde/ros2_rolling/src/ros2/geometry2/tf2_ros/include/tf2_ros/transform_broadcaster.h``)

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
   
   
   #ifndef TF2_ROS__TRANSFORM_BROADCASTER_H_
   #define TF2_ROS__TRANSFORM_BROADCASTER_H_
   
   #include <memory>
   #include <vector>
   
   #include "tf2_ros/visibility_control.h"
   
   #include "rclcpp/node_interfaces/get_node_parameters_interface.hpp"
   #include "rclcpp/node_interfaces/get_node_topics_interface.hpp"
   #include "rclcpp/rclcpp.hpp"
   #include "geometry_msgs/msg/transform_stamped.hpp"
   #include "tf2_msgs/msg/tf_message.hpp"
   #include "tf2_ros/qos.hpp"
   
   namespace tf2_ros
   {
   
   class TransformBroadcaster
   {
   public:
     template<class NodeT, class AllocatorT = std::allocator<void>>
     TransformBroadcaster(
       NodeT && node,
       const rclcpp::QoS & qos = DynamicBroadcasterQoS(),
       const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options = [] () {
         rclcpp::PublisherOptionsWithAllocator<AllocatorT> options;
         options.qos_overriding_options = rclcpp::QosOverridingOptions{
           rclcpp::QosPolicyKind::Depth,
           rclcpp::QosPolicyKind::Durability,
           rclcpp::QosPolicyKind::History,
           rclcpp::QosPolicyKind::Reliability};
         return options;
       } ())
       : TransformBroadcaster(
         rclcpp::node_interfaces::get_node_parameters_interface(node),
         rclcpp::node_interfaces::get_node_topics_interface(node),
         qos,
         options)
     {}
   
     template<class AllocatorT = std::allocator<void>>
     TransformBroadcaster(
       rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
       rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
       const rclcpp::QoS & qos = DynamicBroadcasterQoS(),
       const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options = [] () {
         rclcpp::PublisherOptionsWithAllocator<AllocatorT> options;
         options.qos_overriding_options = rclcpp::QosOverridingOptions{
           rclcpp::QosPolicyKind::Depth,
           rclcpp::QosPolicyKind::Durability,
           rclcpp::QosPolicyKind::History,
           rclcpp::QosPolicyKind::Reliability};
         return options;
       } ())
     {
       publisher_ = rclcpp::create_publisher<tf2_msgs::msg::TFMessage>(
         node_parameters, node_topics, "/tf", qos, options);
     }
   
     TF2_ROS_PUBLIC
     void sendTransform(const geometry_msgs::msg::TransformStamped & transform);
   
     TF2_ROS_PUBLIC
     void sendTransform(const std::vector<geometry_msgs::msg::TransformStamped> & transforms);
   
   private:
     rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_;
   };
   
   }  // namespace tf2_ros
   
   #endif  // TF2_ROS__TRANSFORM_BROADCASTER_H_
