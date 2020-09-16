// Copyright 2008, Willow Garage, Inc. All rights reserved.
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

#include <string>
#include <utility>

#include "tf2/time_cache.h"
#include "tf2/exceptions.h"

#include "tf2/LinearMath/Transform.h"

bool tf2::StaticCache::getData(
  tf2::TimePoint time,
  tf2::TransformStorage & data_out, std::string * error_str)
{
  (void)error_str;
  data_out = storage_;
  data_out.stamp_ = time;
  return true;
}

bool tf2::StaticCache::insertData(const tf2::TransformStorage & new_data)
{
  storage_ = new_data;
  return true;
}

void tf2::StaticCache::clearList() {}

unsigned tf2::StaticCache::getListLength() {return 1;}

tf2::CompactFrameID tf2::StaticCache::getParent(tf2::TimePoint time, std::string * error_str)
{
  (void)time;
  (void)error_str;
  return storage_.frame_id_;
}

tf2::P_TimeAndFrameID tf2::StaticCache::getLatestTimeAndParent()
{
  return std::make_pair(TimePoint(), storage_.frame_id_);
}

tf2::TimePoint tf2::StaticCache::getLatestTimestamp()
{
  return tf2::TimePoint();
}

tf2::TimePoint tf2::StaticCache::getOldestTimestamp()
{
  return tf2::TimePoint();
}
