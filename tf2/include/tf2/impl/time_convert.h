// Copyright 2021, Bjarne von Horn. All rights reserved.
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
//    * Neither the name of the Open Source Robotics Foundation nor the names of its
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

/** \author dhood, Bjarne von Horn */

#ifndef TF2__IMPL__TIME_CONVERT_H_
#define TF2__IMPL__TIME_CONVERT_H_

#include <builtin_interfaces/msg/time.hpp>

#include "../time.h"
#include "forward.h"

namespace tf2
{
namespace impl
{

template <>
struct ImplDetails<tf2::TimePoint, builtin_interfaces::msg::Time>
{
  static void toMsg(const tf2::TimePoint & t, builtin_interfaces::msg::Time & time_msg)
  {
    std::chrono::nanoseconds ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch());
    std::chrono::seconds s = std::chrono::duration_cast<std::chrono::seconds>(t.time_since_epoch());

    time_msg.sec = static_cast<int32_t>(s.count());
    time_msg.nanosec = static_cast<uint32_t>(ns.count() % 1000000000ull);
  }

  static void fromMsg(const builtin_interfaces::msg::Time & time_msg, tf2::TimePoint & t)
  {
    int64_t d = time_msg.sec * 1000000000ull + time_msg.nanosec;
    std::chrono::nanoseconds ns(d);
    t = tf2::TimePoint(std::chrono::duration_cast<tf2::Duration>(ns));
  }
};
}  // namespace impl

inline tf2::TimePoint TimePointFromMsg(const builtin_interfaces::msg::Time & time_msg)
{
  TimePoint tp;
  impl::ImplDetails<TimePoint, builtin_interfaces::msg::Time, void>::fromMsg(time_msg, tp);
  return tp;
}
}  // namespace tf2

#endif  // TF2__IMPL__TIME_CONVERT_H_
