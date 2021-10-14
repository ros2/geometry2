// Copyright 2015-2016, Open Source Robotics Foundation, Inc. All rights reserved.
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

#ifndef TF2__TIME_H_
#define TF2__TIME_H_

#include <chrono>
#include <cmath>
#include <string>

#include "tf2/visibility_control.h"

namespace tf2
{
using Duration = std::chrono::nanoseconds;
using TimePoint = std::chrono::time_point<std::chrono::system_clock, Duration>;

using IDuration = std::chrono::duration<int, std::nano>;
// This is the zero time in ROS
static const TimePoint TimePointZero = TimePoint(IDuration::zero());

TF2_PUBLIC
TimePoint get_now();

TF2_PUBLIC
Duration durationFromSec(double t_sec);

TF2_PUBLIC
TimePoint timeFromSec(double t_sec);

TF2_PUBLIC
double durationToSec(const tf2::Duration & input);

TF2_PUBLIC
double timeToSec(const TimePoint & timepoint);

TF2_PUBLIC
std::string displayTimePoint(const TimePoint & stamp);

}  // namespace tf2

#endif  // TF2__TIME_H_
