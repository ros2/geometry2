/*
 * Copyright (c) 2013, Open Source Robotics Foundation
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

#ifndef TF2_CONVERT_H
#define TF2_CONVERT_H

#include <array>
#include <string>

#include <builtin_interfaces/msg/time.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/impl/convert.h>
#include <tf2/visibility_control.h>

#include <rosidl_runtime_cpp/traits.hpp>

namespace tf2 {

/**\brief The templated function expected to be able to do a transform
 *
 * This is the method which tf2 will use to try to apply a transform for any given datatype.
 * \param data_in The data to be transformed.
 * \param data_out A reference to the output data.  Note this can point to data in and the method should be mutation safe.
 * \param transform The transform to apply to data_in to fill data_out.
 *
 * This method needs to be implemented by client library developers
 */
template <class T>
  void doTransform(const T& data_in, T& data_out, const geometry_msgs::msg::TransformStamped& transform);

/**\brief Get the timestamp from data
 * \param t The data input.
 * \return The timestamp associated with the data.
 */
template <class T>
  tf2::TimePoint getTimestamp(const T& t);

/**\brief Get the frame_id from data
 * \param t The data input.
 * \return The frame_id associated with the data.
 */
template <class T>
  std::string getFrameId(const T& t);

/**\brief Get the covariance matrix from data
 * \param[in] t The data input.
 * \return The covariance matrix associated with the data.
 */
template<class T>
  std::array<std::array<double, 6>, 6> getCovarianceMatrix(const T & t);

/* An implementation for Stamped<P> datatypes */
template <class P>
  tf2::TimePoint getTimestamp(const tf2::Stamped<P>& t)
  {
    return t.stamp_;
  }

/* An implementation for Stamped<P> datatypes */
template <class P>
  std::string getFrameId(const tf2::Stamped<P>& t)
  {
    return t.frame_id_;
  }

/**\brief Get the covariance matrix from data
 *
 * An implementation for WithCovarianceStamped<P> datatypes.
 *
 * \param[in] c The data input.
 * \return The covariance matrix associated with the data.
 */
template<class P>
  std::array<std::array<double, 6>, 6> getCovarianceMatrix(const tf2::WithCovarianceStamped<P> & t)
  {
    return t.cov_mat_;
  }

/** Function that converts from one type to a ROS message type. It has to be
 * implemented by each data type in tf2_* (except ROS messages) as it is
 * used in the "convert" function.
 * \param a an object of whatever type
 * \return the conversion as a ROS message
 */
template<typename A, typename B>
  B toMsg(const A& a);

/** Function that converts from a ROS message type to another type. It has to be
 * implemented by each data type in tf2_* (except ROS messages) as it is used
 * in the "convert" function.
 * \param a a ROS message to convert from
 * \param b the object to convert to
 */
template<typename A, typename B>
  void fromMsg(const A&, B& b);

/** Function that converts any type to any type (messages or not).
 * Matching toMsg and from Msg conversion functions need to exist.
 * If they don't exist or do not apply (for example, if your two
 * classes are ROS messages), just write a specialization of the function.
 * \param a an object to convert from
 * \param b the object to convert to
 */
template <class A, class B>
void convert(const A& a, B& b)
{
  impl::Converter<rosidl_generator_traits::is_message<A>::value,
                  rosidl_generator_traits::is_message<B>::value>::convert(a, b);
}

template <class A>
void convert(const A& a1, A& a2)
{
  if(&a1 != &a2)
    a2 = a1;
}

/**\brief Function that converts from a row-major representation of a 6x6
 * covariance matrix to a nested array representation.
 * \param row_major A row-major array of 36 covariance values.
 * \return A nested array representation of 6x6 covariance values.
 */
inline
std::array<std::array<double, 6>, 6> covarianceRowMajorToNested(const std::array<double, 36> & row_major)
{
  std::array<std::array<double, 6>, 6> nested_array = {};
  size_t l1 = 0, l2 = 0;
  for (const double & val : row_major) {
    nested_array[l2][l1] = val;

    l1++;

    if (l1 == nested_array[0].size()) {
      l1 = 0;
      l2++;
    }
  }
  return nested_array;
}

/**\brief Function that converts from a nested array representation of a 6x6
 * covariance matrix to a row-major representation.
 * \param nested_array A nested array representation of 6x6 covariance values.
 * \return A row-major array of 36 covariance values.
 */
inline
std::array<double, 36> covarianceNestedToRowMajor(const std::array<std::array<double, 6>, 6> & nested_array)
{
  std::array<double, 36> row_major = {};
  size_t counter = 0;
  for (const auto & arr : nested_array) {
    for (const double & val : arr) {
      row_major[counter] = val;
      counter++;
    }
  }
  return row_major;
}

}

#endif //TF2_CONVERT_H
