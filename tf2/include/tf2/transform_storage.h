// Copyright 2010, Willow Garage, Inc. All rights reserved.
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
//    * Neither the name of the Willow Garage nor the names of its
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
/** \author Tully Foote */

#ifndef TF2__TRANSFORM_STORAGE_H_
#define TF2__TRANSFORM_STORAGE_H_

#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/time.h"
#include "tf2/visibility_control.h"

namespace tf2
{
typedef uint32_t CompactFrameID;

/** \brief Storage for transforms and their parent */
class TransformStorage
{
public:
  TF2_PUBLIC
  TransformStorage();
  TF2_PUBLIC
  TransformStorage(
    const TimePoint & stamp, const Quaternion & q, const Vector3 & t, CompactFrameID frame_id,
    CompactFrameID child_frame_id);

  TF2_PUBLIC
  TransformStorage(const TransformStorage & rhs)
  {
    *this = rhs;
  }

  TF2_PUBLIC
  TransformStorage & operator=(const TransformStorage & rhs)
  {
    rotation_ = rhs.rotation_;
    translation_ = rhs.translation_;
    stamp_ = rhs.stamp_;
    frame_id_ = rhs.frame_id_;
    child_frame_id_ = rhs.child_frame_id_;
    return *this;
  }

  tf2::Quaternion rotation_;
  tf2::Vector3 translation_;
  TimePoint stamp_;
  CompactFrameID frame_id_;
  CompactFrameID child_frame_id_;
};
}  // namespace tf2
#endif  // TF2__TRANSFORM_STORAGE_H_
