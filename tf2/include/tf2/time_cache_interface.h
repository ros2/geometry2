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

#include <memory>
#include <string>
#include <utility>

#include "tf2/time.h"
#include "tf2/transform_storage.h"
#include "tf2/visibility_control.h"

#ifndef TF2__TIME_CACHE_INTERFACE_H_
#define TF2__TIME_CACHE_INTERFACE_H_

namespace tf2
{
typedef std::pair<tf2::TimePoint, tf2::CompactFrameID> P_TimeAndFrameID;

class TimeCacheInterface
{
public:
  TF2_PUBLIC
  virtual ~TimeCacheInterface() = default;

  /** \brief Access data from the cache
   * returns false if data unavailable (should be thrown as lookup exception)
   */
  TF2_PUBLIC
  virtual bool getData(
    tf2::TimePoint time, tf2::TransformStorage & data_out,
    std::string * error_str = 0) = 0;

  /** \brief Insert data into the cache */
  TF2_PUBLIC
  virtual bool insertData(const tf2::TransformStorage & new_data) = 0;

  /** @brief Clear the list of stored values */
  TF2_PUBLIC
  virtual void clearList() = 0;

  /** \brief Retrieve the parent at a specific time */
  TF2_PUBLIC
  virtual CompactFrameID getParent(tf2::TimePoint time, std::string * error_str) = 0;

  /**
   * \brief Get the latest time stored in this cache, and the parent associated with it.  Returns parent = 0 if no data.
   */
  TF2_PUBLIC
  virtual P_TimeAndFrameID getLatestTimeAndParent() = 0;

  /// Debugging information methods
  /** @brief Get the length of the stored list */
  TF2_PUBLIC
  virtual unsigned int getListLength() = 0;

  /** @brief Get the latest timestamp cached */
  TF2_PUBLIC
  virtual tf2::TimePoint getLatestTimestamp() = 0;

  /** @brief Get the oldest timestamp cached */
  TF2_PUBLIC
  virtual tf2::TimePoint getOldestTimestamp() = 0;
};

using TimeCacheInterfacePtr = std::shared_ptr<TimeCacheInterface>;

}  // namespace tf2

#endif  // TF2__TIME_CACHE_INTERFACE_H_
