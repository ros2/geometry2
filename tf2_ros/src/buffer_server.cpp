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

#include <tf2/exceptions.h>

#include <tf2_ros/buffer.h>  // Only needed for toMsg() and fromMsg()
#include <tf2_ros/buffer_server.h>

#include <algorithm>
#include <list>
#include <memory>
#include <string>

namespace tf2_ros
{

void BufferServer::serviceCB(
  const std::shared_ptr<LookupTransformService::Request> request,
  std::shared_ptr<LookupTransformService::Response> response)
{
  // TODO: implement timeout
  try {
    // check whether we need to use the advanced or simple api
    if (request->advanced) {
      response->transform = buffer_.lookupTransform(
        request->target_frame, tf2_ros::fromMsg(request->target_time),
        request->source_frame, tf2_ros::fromMsg(request->source_time), request->fixed_frame);
    }
    else {
      response->transform = buffer_.lookupTransform(
      request->target_frame, request->source_frame,
      tf2_ros::fromMsg(request->source_time));
    }

  } catch (const tf2::ConnectivityException & ex) {
    response->error.error = response->error.CONNECTIVITY_ERROR;
    response->error.error_string = ex.what();
  } catch (const tf2::LookupException & ex) {
    response->error.error = response->error.LOOKUP_ERROR;
    response->error.error_string = ex.what();
  } catch (const tf2::ExtrapolationException & ex) {
    response->error.error = response->error.EXTRAPOLATION_ERROR;
    response->error.error_string = ex.what();
  } catch (const tf2::InvalidArgumentException & ex) {
    response->error.error = response->error.INVALID_ARGUMENT_ERROR;
    response->error.error_string = ex.what();
  } catch (const tf2::TimeoutException & ex) {
    response->error.error = response->error.TIMEOUT_ERROR;
    response->error.error_string = ex.what();
  } catch (const tf2::TransformException & ex) {
    response->error.error = response->error.TRANSFORM_ERROR;
    response->error.error_string = ex.what();
  } 
}

bool BufferServer::canTransform(const std::shared_ptr<LookupTransformService::Request> request)
{
  tf2::TimePoint source_time_point = tf2_ros::fromMsg(request->source_time);

  // check whether we need to used the advanced or simple api
  if (!request->advanced) {
    return buffer_.canTransform(
      request->target_frame, request->source_frame, source_time_point, nullptr);
  }

  tf2::TimePoint target_time_point = tf2_ros::fromMsg(request->target_time);
  return buffer_.canTransform(
    request->target_frame, target_time_point,
    request->source_frame, source_time_point, request->fixed_frame, nullptr);
}

}  // namespace tf2_ros
