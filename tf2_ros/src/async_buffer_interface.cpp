// Copyright 2027, Open Source Robotics Foundation, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <utility>

#include "tf2_ros/async_buffer_interface.hpp"

namespace tf2_ros
{

TransformStampedFuture::TransformStampedFuture(BaseType && future) noexcept
: BaseType(std::move(future))
{
}

TransformStampedFuture::TransformStampedFuture(const TransformStampedFuture & ts_future) noexcept
: BaseType(ts_future),
  handle_(ts_future.handle_)
{
}

TransformStampedFuture::TransformStampedFuture(TransformStampedFuture && ts_future) noexcept
: BaseType(std::move(ts_future)),
  handle_(std::move(ts_future.handle_))
{
}

void TransformStampedFuture::setHandle(const tf2::TransformableRequestHandle handle)
{
  handle_ = handle;
}

tf2::TransformableRequestHandle TransformStampedFuture::getHandle() const
{
  return handle_;
}

}  // namespace tf2_ros
