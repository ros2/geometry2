// Copyright 2026, Open Source Robotics Foundation, Inc.
// All rights reserved.
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
//    * Neither the name of the copyright holder nor the names of its
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

#include <string>

#include "tf2/exceptions.hpp"

namespace tf2
{

TransformException::TransformException(const std::string errorDescription)
: std::runtime_error(errorDescription)
{
}

ConnectivityException::ConnectivityException(const std::string errorDescription)
: tf2::TransformException(errorDescription)
{
}

LookupException::LookupException(const std::string errorDescription)
: tf2::TransformException(errorDescription)
{
}

ExtrapolationException::ExtrapolationException(const std::string errorDescription)
: tf2::TransformException(errorDescription)
{
}

BackwardExtrapolationException::BackwardExtrapolationException(const std::string errorDescription)
: ExtrapolationException(errorDescription)
{
}

ForwardExtrapolationException::ForwardExtrapolationException(const std::string errorDescription)
: ExtrapolationException(errorDescription)
{
}

NoDataForExtrapolationException::NoDataForExtrapolationException(
  const std::string errorDescription)
: ExtrapolationException(errorDescription)
{
}

InvalidArgumentException::InvalidArgumentException(const std::string errorDescription)
: tf2::TransformException(errorDescription)
{
}

TimeoutException::TimeoutException(const std::string errorDescription)
: tf2::TransformException(errorDescription)
{
}

}  // namespace tf2
