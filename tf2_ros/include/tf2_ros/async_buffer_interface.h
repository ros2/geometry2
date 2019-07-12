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

#ifndef TF2_ROS__ASYNC_BUFFER_INTERFACE_H
#define TF2_ROS__ASYNC_BUFFER_INTERFACE_H

#include <functional>
#include <future>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/visibility_control.h>
#include <tf2/transform_datatypes.h>

namespace tf2_ros
{

using TransformStampedFuture = std::shared_future<geometry_msgs::msg::TransformStamped>;
using TransformReadyCallback = std::function<void(const TransformStampedFuture&)>;

/**
 * \brief Abstract interface for asynchronous operations on a `tf2::BufferCoreInterface`.
 * Implementations include tf2_ros::Buffer.
 */
class AsyncBufferInterface
{
public:
  /**
   * \brief Wait for a transform between two frames to become available.
   * \param target_frame The frame into which to transform.
   * \param source_frame The frame from which to tranform.
   * \param time The time at which to transform.
   * \param timeout Duration after which waiting will be stopped.
   * \param callback The function to be called when the transform becomes available or a timeout
   *   occurs. In the case of timeout, an exception will be set on the future.
   * \return A future to the requested transform. If a timeout occurs a `tf2::LookupException`
   *    will be set on the future.
   */
  TF2_ROS_PUBLIC
  virtual TransformStampedFuture
  waitForTransform(
    const std::string& target_frame,
    const std::string& source_frame,
    const tf2::TimePoint& time,
    const tf2::Duration& timeout,
    TransformReadyCallback callback) = 0;
};  // class AsyncBufferInterface

}  // namespace tf2_ros

#endif // TF2_ROS__ASYNC_BUFFER_INTERFACE_H
