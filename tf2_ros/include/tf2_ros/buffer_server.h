/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/

#ifndef TF2_ROS__BUFFER_SERVER_H_
#define TF2_ROS__BUFFER_SERVER_H_

#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <string>

#include "tf2/time.h"
#include "tf2/buffer_core_interface.h"
#include "tf2_ros/visibility_control.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "tf2_msgs/srv/lookup_transform.hpp"

namespace tf2_ros
{
/** \brief Service server for the service-based implementation of tf2::BufferCoreInterface.
 *
 * Use this class with a tf2_ros::TransformListener in the same process.
 * You can use this class with a tf2_ros::BufferClient in a different process.
 */
class BufferServer
{
  using LookupTransformService = tf2_msgs::srv::LookupTransform;

public:
  /** \brief Constructor
   * \param buffer The Buffer that this BufferServer will wrap.
   * \param node The node to add the buffer server to.
   * \param ns The namespace in which to look for service clients.
   */
  template<typename NodePtr>
  BufferServer(
    const tf2::BufferCoreInterface & buffer,
    NodePtr node,
    const std::string & ns)
  : buffer_(buffer),
    logger_(node->get_logger())
  {
    service_server_ = rclcpp::create_service<LookupTransformService>(
      node->get_node_base_interface(),
      node->get_node_services_interface(),
      ns,
      std::bind(&BufferServer::serviceCB, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      nullptr);
    RCLCPP_DEBUG(logger_, "Buffer server started");
  }

private:
  TF2_ROS_PUBLIC
  void serviceCB(
    const std::shared_ptr<LookupTransformService::Request> request,
    std::shared_ptr<LookupTransformService::Response> response);

  TF2_ROS_PUBLIC
  bool canTransform(const std::shared_ptr<LookupTransformService::Request> request);

  const tf2::BufferCoreInterface & buffer_;
  rclcpp::Logger logger_;
  rclcpp::Service<LookupTransformService>::SharedPtr service_server_;
};

}  // namespace tf2_ros

#endif  // TF2_ROS__BUFFER_SERVER_H_
