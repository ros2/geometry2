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

#include <assert.h>

#include <string>
#include <utility>

#include "tf2/time_cache.h"
#include "tf2/exceptions.h"

#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"

namespace tf2
{

TransformStorage::TransformStorage()
{
}

TransformStorage::TransformStorage(
  const TimePoint & stamp, const Quaternion & q, const Vector3 & t,
  CompactFrameID frame_id, CompactFrameID child_frame_id)
: rotation_(q),
  translation_(t),
  stamp_(stamp),
  frame_id_(frame_id),
  child_frame_id_(child_frame_id)
{
}

TimeCache::TimeCache(tf2::Duration max_storage_time)
: max_storage_time_(max_storage_time)
{}

// Avoid ODR collisions https://github.com/ros/geometry2/issues/175
namespace cache
{
// hoisting these into separate functions causes an ~8% speedup.
// Removing calling them altogether adds another ~10%
void createExtrapolationException1(TimePoint t0, TimePoint t1, std::string * error_str)
{
  if (error_str) {
    std::stringstream ss;
    ss << "Lookup would require extrapolation at time " << displayTimePoint(t0) <<
      ", but only time " << displayTimePoint(t1) << " is in the buffer";
    *error_str = ss.str();
  }
}

void createExtrapolationException2(TimePoint t0, TimePoint t1, std::string * error_str)
{
  if (error_str) {
    std::stringstream ss;
    ss << "Lookup would require extrapolation into the future.  Requested time " <<
      displayTimePoint(t0) << " but the latest data is at time " << displayTimePoint(t1);
    *error_str = ss.str();
  }
}

void createExtrapolationException3(TimePoint t0, TimePoint t1, std::string * error_str)
{
  if (error_str) {
    std::stringstream ss;
    ss << "Lookup would require extrapolation into the past.  Requested time " << displayTimePoint(
      t0) << " but the earliest data is at time " << displayTimePoint(t1);
    *error_str = ss.str();
  }
}
}  // namespace cache

uint8_t TimeCache::findClosest(
  TransformStorage * & one, TransformStorage * & two,
  TimePoint target_time, std::string * error_str)
{
  // No values stored
  if (storage_.empty()) {
    return 0;
  }

  // If time == 0 return the latest
  if (target_time == TimePointZero) {
    one = &storage_.front();
    return 1;
  }

  // One value stored
  if (++storage_.begin() == storage_.end()) {
    TransformStorage & ts = *storage_.begin();
    if (ts.stamp_ == target_time) {
      one = &ts;
      return 1;
    } else {
      cache::createExtrapolationException1(target_time, ts.stamp_, error_str);
      return 0;
    }
  }

  TimePoint latest_time = (*storage_.begin()).stamp_;
  TimePoint earliest_time = (*(storage_.rbegin())).stamp_;

  if (target_time == latest_time) {
    one = &(*storage_.begin());
    return 1;
  } else if (target_time == earliest_time) {
    one = &(*storage_.rbegin());
    return 1;
  } else {   // Catch cases that would require extrapolation
    if (target_time > latest_time) {
      cache::createExtrapolationException2(target_time, latest_time, error_str);
      return 0;
    } else {
      if (target_time < earliest_time) {
        cache::createExtrapolationException3(target_time, earliest_time, error_str);
        return 0;
      }
    }
  }

  // At least 2 values stored
  // Find the first value less than the target value
  L_TransformStorage::iterator storage_it = storage_.begin();
  while (storage_it != storage_.end()) {
    if (storage_it->stamp_ <= target_time) {
      break;
    }
    storage_it++;
  }

  // Finally the case were somewhere in the middle  Guarenteed no extrapolation :-)
  one = &*(storage_it);  // Older
  two = &*(--storage_it);  // Newer
  return 2;
}

void TimeCache::interpolate(
  const TransformStorage & one, const TransformStorage & two,
  TimePoint time, TransformStorage & output)
{
  // Check for zero distance case
  if (two.stamp_ == one.stamp_) {
    output = two;
    return;
  }
  // Calculate the ratio
  tf2Scalar ratio = static_cast<double>((time - one.stamp_).count()) /
    static_cast<double>((two.stamp_ - one.stamp_).count());

  // Interpolate translation
  output.translation_.setInterpolate3(one.translation_, two.translation_, ratio);

  // Interpolate rotation
  output.rotation_ = slerp(one.rotation_, two.rotation_, ratio);

  output.stamp_ = one.stamp_;
  output.frame_id_ = one.frame_id_;
  output.child_frame_id_ = one.child_frame_id_;
}

bool TimeCache::getData(
  TimePoint time, TransformStorage & data_out,
  std::string * error_str)
{
  // returns false if data not available
  TransformStorage * p_temp_1;
  TransformStorage * p_temp_2;

  int num_nodes = findClosest(p_temp_1, p_temp_2, time, error_str);
  if (num_nodes == 0) {
    return false;
  } else if (num_nodes == 1) {
    data_out = *p_temp_1;
  } else if (num_nodes == 2) {
    if (p_temp_1->frame_id_ == p_temp_2->frame_id_) {
      interpolate(*p_temp_1, *p_temp_2, time, data_out);
    } else {
      data_out = *p_temp_1;
    }
  } else {
    assert(0);
  }
  return true;
}

CompactFrameID TimeCache::getParent(TimePoint time, std::string * error_str)
{
  TransformStorage * p_temp_1;
  TransformStorage * p_temp_2;

  int num_nodes = findClosest(p_temp_1, p_temp_2, time, error_str);
  if (num_nodes == 0) {
    return 0;
  }

  return p_temp_1->frame_id_;
}

bool TimeCache::insertData(const TransformStorage & new_data)
{
  L_TransformStorage::iterator storage_it = storage_.begin();

  if (storage_it != storage_.end()) {
    if (storage_it->stamp_ > new_data.stamp_ + max_storage_time_) {
      return false;
    }
  }

  while (storage_it != storage_.end()) {
    if (storage_it->stamp_ <= new_data.stamp_) {
      break;
    }
    storage_it++;
  }
  storage_.insert(storage_it, new_data);

  pruneList();
  return true;
}

void TimeCache::clearList()
{
  storage_.clear();
}

unsigned int TimeCache::getListLength()
{
  return (unsigned int)storage_.size();
}

P_TimeAndFrameID TimeCache::getLatestTimeAndParent()
{
  if (storage_.empty()) {
    return std::make_pair(TimePoint(), 0);
  }

  const TransformStorage & ts = storage_.front();
  return std::make_pair(ts.stamp_, ts.frame_id_);
}

TimePoint TimeCache::getLatestTimestamp()
{
  // empty list case
  if (storage_.empty()) {
    return TimePoint();
  }
  return storage_.front().stamp_;
}

TimePoint TimeCache::getOldestTimestamp()
{
  // empty list case
  if (storage_.empty()) {
    return TimePoint();
  }
  return storage_.back().stamp_;
}

void TimeCache::pruneList()
{
  TimePoint latest_time = storage_.begin()->stamp_;

  while (!storage_.empty() && storage_.back().stamp_ + max_storage_time_ < latest_time) {
    storage_.pop_back();
  }
}
}  // namespace tf2
