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

#ifndef TF2__EXCEPTIONS_H_
#define TF2__EXCEPTIONS_H_

#include <stdexcept>
#include <cstdint>
#include <string>

#include "tf2/visibility_control.h"

namespace tf2
{

enum class TF2Error : std::uint8_t
{
  NO_ERROR = 0,
  LOOKUP_ERROR = 1,
  CONNECTIVITY_ERROR = 2,
  EXTRAPOLATION_ERROR = 3,
  INVALID_ARGUMENT_ERROR = 4,
  TIMEOUT_ERROR = 5,
  TRANSFORM_ERROR = 6
};

/** \brief A base class for all tf2 exceptions
 * This inherits from ros::exception
 * which inherits from std::runtime_exception
 */
class TransformException : public std::runtime_error
{
public:
  TF2_PUBLIC
  explicit TransformException(const std::string errorDescription)
  : std::runtime_error(errorDescription)
  {
  }
};


/** \brief An exception class to notify of no connection
 *
 * This is an exception class to be thrown in the case
 * that the Reference Frame tree is not connected between
 * the frames requested. */
class ConnectivityException : public TransformException
{
public:
  TF2_PUBLIC
  explicit ConnectivityException(const std::string errorDescription)
  : tf2::TransformException(errorDescription)
  {
  }
};


/** \brief An exception class to notify of bad frame number
 *
 * This is an exception class to be thrown in the case that
 * a frame not in the graph has been attempted to be accessed.
 * The most common reason for this is that the frame is not
 * being published, or a parent frame was not set correctly
 * causing the tree to be broken.
 */
class LookupException : public TransformException
{
public:
  TF2_PUBLIC
  explicit LookupException(const std::string errorDescription)
  : tf2::TransformException(errorDescription)
  {
  }
};

/** \brief An exception class to notify that the requested value would have required extrapolation beyond current limits.
 *
 */
class ExtrapolationException : public TransformException
{
public:
  TF2_PUBLIC
  explicit ExtrapolationException(const std::string errorDescription)
  : tf2::TransformException(errorDescription)
  {
  }
};

/** \brief An exception class to notify that one of the arguments is invalid
 *
 * usually it's an uninitalized Quaternion (0,0,0,0)
 *
 */
class InvalidArgumentException : public TransformException
{
public:
  TF2_PUBLIC
  explicit InvalidArgumentException(const std::string errorDescription)
  : tf2::TransformException(errorDescription) {}
};

/** \brief An exception class to notify that a timeout has occured
 *
 *
 */
class TimeoutException : public TransformException
{
public:
  TF2_PUBLIC
  explicit TimeoutException(const std::string errorDescription)
  : tf2::TransformException(errorDescription)
  {
  }
};
}  // namespace tf2
#endif  // TF2__EXCEPTIONS_H_
