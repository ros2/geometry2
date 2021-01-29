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

#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>

#include "../time.h"
#include "forward.h"

namespace tf2
{
namespace impl
{
/// \brief Conversion implementation for builtin_interfaces::msg::Time and tf2::TimePoint.
template<>
struct ConversionImplementation<tf2::TimePoint, builtin_interfaces::msg::Time>
{
  /** \brief Convert a tf2::TimePoint to a Time message.
   * \param[in] t The tf2::TimePoint to convert.
   * \param[out] time_msg The converted tf2::TimePoint, as a Time message.
   */
  static void toMsg(const tf2::TimePoint & t, builtin_interfaces::msg::Time & time_msg)
  {
    std::chrono::nanoseconds ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch());
    std::chrono::seconds s = std::chrono::duration_cast<std::chrono::seconds>(t.time_since_epoch());

    time_msg.sec = static_cast<int32_t>(s.count());
    time_msg.nanosec = static_cast<uint32_t>(ns.count() % 1000000000ull);
  }

  /** \brief Convert a Time message to a tf2::TimePoint.
   * \param[in] time_msg The Time message to convert.
   * \param[out] t The converted message, as a tf2::TimePoint.
   */
  static void fromMsg(const builtin_interfaces::msg::Time & time_msg, tf2::TimePoint & t)
  {
    int64_t d = time_msg.sec * 1000000000ull + time_msg.nanosec;
    std::chrono::nanoseconds ns(d);
    t = tf2::TimePoint(std::chrono::duration_cast<tf2::Duration>(ns));
  }
};

/// \brief Conversion implementation for builtin_interfaces::msg::Duration and tf2::Duration.
template<>
struct ConversionImplementation<tf2::Duration, builtin_interfaces::msg::Duration>
{
  /** \brief Convert a tf2::Duration to a Duration message.
   * \param[in] t The tf2::Duration to convert.
   * \param[out] duration_msg The converted tf2::Duration, as a Duration message.
   */
  static void toMsg(const tf2::Duration & t, builtin_interfaces::msg::Duration & duration_msg)
  {
    std::chrono::nanoseconds ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t);
    std::chrono::seconds s = std::chrono::duration_cast<std::chrono::seconds>(t);
    duration_msg.sec = static_cast<int32_t>(s.count());
    duration_msg.nanosec = static_cast<uint32_t>(ns.count() % 1000000000ull);
  }

  /** \brief Convert a Duration message to a tf2::Duration.
   * \param[in] duration_msg The Duration message to convert.
   * \param[out] duration The converted message, as a tf2::Duration.
   */
  static void fromMsg(
    const builtin_interfaces::msg::Duration & duration_msg, tf2::Duration & duration)
  {
    int64_t d = duration_msg.sec * 1000000000ull + duration_msg.nanosec;
    std::chrono::nanoseconds ns(d);
    duration = tf2::Duration(std::chrono::duration_cast<tf2::Duration>(ns));
  }
};

/// \brief Default return type of tf2::toMsg() for tf2::TimePoint.
template<>
struct DefaultMessageForDatatype<tf2::TimePoint>
{
  /// \brief Default return type of tf2::toMsg() for tf2::TimePoint.
  using type = builtin_interfaces::msg::Time;
};

/// \brief Default return type of tf2::toMsg() for tf2::Duration.
template<>
struct DefaultMessageForDatatype<tf2::Duration>
{
  /// \brief Default return type of tf2::toMsg() for tf2::Duration.
  using type = builtin_interfaces::msg::Duration;
};
}  // namespace impl

/** \brief Convert a Time message to a tf2::TimePoint.
 * \param[in] time_msg The Time message to convert.
 * \return The converted message, as a tf2::TimePoint.
 */
inline tf2::TimePoint TimePointFromMsg(const builtin_interfaces::msg::Time & time_msg)
{
  TimePoint tp;
  tf2::fromMsg<>(time_msg, tp);
  return tp;
}

/** \brief Convert a Duration message to a tf2::Duration.
 * \param[in] duration_msg The Duration message to convert.
 * \return The converted message, as a tf2::Duration.
 */
inline tf2::Duration DurationFromMsg(const builtin_interfaces::msg::Duration & duration_msg)
{
  Duration d;
  tf2::fromMsg<>(duration_msg, d);
  return d;
}
}  // namespace tf2

#endif  // TF2__IMPL__TIME_CONVERT_H_
