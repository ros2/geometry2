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

#ifndef TF2__STATIC_CACHE_H_
#define TF2__STATIC_CACHE_H_

#include <string>

#include "tf2/time.h"
#include "tf2/time_cache_interface.h"
#include "tf2/transform_storage.h"
#include "tf2/visibility_control.h"

namespace tf2
{

class StaticCache : public TimeCacheInterface
{
public:
  /// Virtual methods
  TF2_PUBLIC
  virtual bool getData(TimePoint time, TransformStorage & data_out, std::string * error_str = 0);
  // returns false if data unavailable (should be thrown as lookup exception
  TF2_PUBLIC
  virtual bool insertData(const TransformStorage & new_data);
  TF2_PUBLIC
  virtual void clearList();
  TF2_PUBLIC
  virtual CompactFrameID getParent(TimePoint time, std::string * error_str);
  TF2_PUBLIC
  virtual P_TimeAndFrameID getLatestTimeAndParent();

  /// Debugging information methods
  TF2_PUBLIC
  virtual unsigned int getListLength();
  TF2_PUBLIC
  virtual TimePoint getLatestTimestamp();
  TF2_PUBLIC
  virtual TimePoint getOldestTimestamp();

private:
  TransformStorage storage_;
};

}  // namespace tf2

#endif  // TF2__STATIC_CACHE_H_
