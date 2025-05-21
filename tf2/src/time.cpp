// Copyright 2008, Open Source Robotics Foundation, Inc. All rights reserved.
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

/** \author Tully Foote */

#include <chrono>
#include <stdexcept>
#include <string>

#include "rcutils/snprintf.h"
#include "rcutils/strerror.h"
#include "tf2/time.hpp"

tf2::TimePoint tf2::get_now()
{
  return std::chrono::system_clock::now();
}

tf2::Duration tf2::durationFromSec(double t_sec)
{
  if (t_sec > std::numeric_limits<int32_t>::max() || t_sec < std::numeric_limits<int32_t>::min()) {
    throw std::overflow_error("Input t_sec is too large or too small for tf2::Duration");
  }
  int32_t sec, nsec;
  sec = static_cast<int32_t>(floor(t_sec));
  nsec = static_cast<int32_t>(std::round((t_sec - sec) * 1e9));
  // avoid rounding errors
  sec += (nsec / 1000000000l);
  nsec %= 1000000000l;
  return std::chrono::seconds(sec) + std::chrono::nanoseconds(nsec);
}

tf2::TimePoint tf2::timeFromSec(double t_sec)
{
  return tf2::TimePoint(durationFromSec(t_sec));
}

double tf2::durationToSec(const tf2::Duration & input)
{
  int64_t count = input.count();

  // scale the nanoseconds separately for improved accuracy
  int32_t sec, nsec;
  nsec = static_cast<int32_t>(count % 1000000000l);
  sec = static_cast<int32_t>((count - nsec) / 1000000000l);

  double sec_double, nsec_double;
  nsec_double = 1e-9 * static_cast<double>(nsec);
  sec_double = static_cast<double>(sec);
  return sec_double + nsec_double;
}

double tf2::timeToSec(const tf2::TimePoint & timepoint)
{
  return durationToSec(tf2::Duration(timepoint.time_since_epoch()));
}

std::string tf2::displayTimePoint(const tf2::TimePoint & stamp)
{
  const char * format_str = "%.6f";
  double current_time = tf2::timeToSec(stamp);

  // Determine how many bytes to allocate for the string. If successful, buff_size does not count
  // null terminating character. http://www.cplusplus.com/reference/cstdio/snprintf/
  int buff_size = rcutils_snprintf(nullptr, 0, format_str, current_time);
  if (buff_size < 0) {
    char errmsg[200];
    rcutils_strerror(errmsg, sizeof(errmsg));
    throw std::runtime_error(errmsg);
  }

  // Increase by one for null-terminating character
  ++buff_size;
  char * buffer = new char[buff_size];

  // Write to the string. buffer size must accommodate the null-terminating character
  int bytes_written = rcutils_snprintf(buffer, buff_size, format_str, current_time);
  if (bytes_written < 0) {
    delete[] buffer;
    char errmsg[200];
    rcutils_strerror(errmsg, sizeof(errmsg));
    throw std::runtime_error(errmsg);
  }
  std::string result = std::string(buffer);
  delete[] buffer;
  return result;
}
