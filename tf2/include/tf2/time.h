/*
 * Copyright (c) 2015-2016, Open Source Robotics Foundation, Inc.
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

 #ifndef TF2_TIME_H
 #define TF2_TIME_H

#include <chrono>
#include <cmath>
#include <stdio.h>
#include <string>
#include <thread>
#include <iostream>

#include <tf2/visibility_control.h>

namespace tf2
{
  using Duration = std::chrono::nanoseconds;
  using TimePoint = std::chrono::time_point<std::chrono::system_clock, Duration>;


  using IDuration = std::chrono::duration<int, std::nano>;
  // This is the zero time in ROS
  static const TimePoint TimePointZero = TimePoint(IDuration::zero());

  inline TimePoint get_now()
  {
    return std::chrono::system_clock::now();
  }

  inline Duration durationFromSec(double t_sec)
  {
    int32_t sec, nsec;
    sec = static_cast<int32_t>(floor(t_sec));
    nsec = static_cast<int32_t>(std::round((t_sec - sec) * 1e9));
    // avoid rounding errors
    sec += (nsec / 1000000000l);
    nsec %= 1000000000l;
    return std::chrono::seconds(sec) + std::chrono::nanoseconds(nsec);
  }

  inline TimePoint timeFromSec(double t_sec)
  {
    return tf2::TimePoint(durationFromSec(t_sec));
  }

  inline double durationToSec(const tf2::Duration& input){
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

  inline double timeToSec(const TimePoint& timepoint)
  {
    return durationToSec(Duration(timepoint.time_since_epoch()));
  }

  TF2_PUBLIC
  std::string displayTimePoint(const TimePoint& stamp);

}

#endif // TF2_TIME_H
