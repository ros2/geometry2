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

#ifndef TF2__TRANSFORM_DATATYPES_H_
#define TF2__TRANSFORM_DATATYPES_H_

#include <array>
#include <chrono>
#include <string>

#include "tf2/time.h"

namespace tf2
{

/** \brief The data type which will be cross compatable with geometry_msgs
 * This is the tf2 datatype equivilant of a MessageStamped */
template<typename T>
class Stamped : public T
{
public:
  TimePoint stamp_;   ///< The timestamp associated with this data
  std::string frame_id_;   ///< The frame_id associated this data

  /** Default constructor */
  Stamped()
  : frame_id_("NO_ID_STAMPED_DEFAULT_CONSTRUCTION")
  {
  }

  /** Full constructor */
  Stamped(const T & input, const TimePoint & timestamp, const std::string & frame_id)
  : T(input), stamp_(timestamp), frame_id_(frame_id)
  {
  }

  /** Copy Constructor */
  Stamped(const Stamped<T> & s)
  : T(s),
    stamp_(s.stamp_),
    frame_id_(s.frame_id_)
  {
  }

  /** Set the data element */
  void setData(const T & input) {*static_cast<T *>(this) = input;}

  Stamped & operator=(const Stamped<T> & s)
  {
    T::operator=(s);
    this->stamp_ = s.stamp_;
    this->frame_id_ = s.frame_id_;
    return *this;
  }
};

/** \brief Comparison Operator for Stamped datatypes */
template<typename T>
bool operator==(const Stamped<T> & a, const Stamped<T> & b)
{
  return a.frame_id_ == b.frame_id_ && a.stamp_ == b.stamp_ &&
         static_cast<const T &>(a) == static_cast<const T &>(b);
}

/** \brief The data type which will be cross compatable with geometry_msgs
 * This is the tf2 datatype equivalent of a MessageWithCovarianceStamped */
template<typename T>
class WithCovarianceStamped : public T
{
public:
  TimePoint stamp_;   ///< The timestamp associated with this data
  std::string frame_id_;   ///< The frame_id associated this data
  std::array<std::array<double, 6>, 6> cov_mat_;  ///< The covariance matrix associated with this data // NOLINT

  /** Default constructor */
  WithCovarianceStamped()
  : frame_id_("NO_ID_STAMPED_DEFAULT_CONSTRUCTION"),
    cov_mat_{}
  {
  }

  /** Full constructor */
  WithCovarianceStamped(
    const T & input,
    const TimePoint & timestamp,
    const std::string & frame_id,
    const std::array<std::array<double, 6>, 6> & covariance_matrix
  )
  : T(input),
    stamp_(timestamp),
    frame_id_(frame_id),
    cov_mat_(covariance_matrix)
  {
  }

  /** Copy constructor */
  WithCovarianceStamped(const WithCovarianceStamped<T> & w)
  : T(w),
    stamp_(w.stamp_),
    frame_id_(w.frame_id_),
    cov_mat_(w.cov_mat_)
  {
  }

  /** Set the data element */
  void setData(const T & input) {*static_cast<T *>(this) = input;}

  WithCovarianceStamped & operator=(const WithCovarianceStamped<T> & w)
  {
    T::operator=(w);
    this->stamp_ = w.stamp_;
    this->frame_id_ = w.frame_id_;
    this->cov_mat_ = w.cov_mat_;
    return *this;
  }
};

/** \brief Comparison operator for WithCovarianceStamped datatypes */
template<typename T>
bool operator==(const WithCovarianceStamped<T> & a, const WithCovarianceStamped<T> & b)
{
  return a.frame_id_ == b.frame_id_ && a.stamp_ == b.stamp_ &&
         a.cov_mat_ == b.cov_mat_ && static_cast<const T &>(a) == static_cast<const T &>(b);
}

}  // namespace tf2

#endif  // TF2__TRANSFORM_DATATYPES_H_
