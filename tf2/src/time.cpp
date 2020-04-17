/*
 * Copyright (c) 2016, Open Source Robotics Foundation, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Tully Foote */

#include <stdexcept>
#include <string>

#include "rcutils/snprintf.h"
#include "rcutils/strerror.h"
#include "tf2/time.h"

std::string tf2::displayTimePoint(const tf2::TimePoint& stamp)
{
  const char * format_str = "%.6f";
  double current_time = tf2::timeToSec(stamp);

  // Determine how many bytes to allocate for the string. If successful, buff_size does not count
  // null terminating character. http://www.cplusplus.com/reference/cstdio/snprintf/
  int buff_size = rcutils_snprintf(NULL, 0, format_str, current_time);
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
