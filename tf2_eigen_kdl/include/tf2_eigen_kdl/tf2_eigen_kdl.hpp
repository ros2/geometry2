// Copyright 2020 Open Source Robotics Foundation, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of Open Source Robotics Foundation, Inc. nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*
 * Author: Adam Leeper, Stuart Glaser
 */

#ifndef TF2_EIGEN_KDL__TF2_EIGEN_KDL_HPP_
#define TF2_EIGEN_KDL__TF2_EIGEN_KDL_HPP_

// Version 3.4.0 of Eigen in Ubuntu 22.04 has a bug that causes -Wclass-memaccess warnings on
// aarch64.  Upstream Eigen has already fixed this in
// https://gitlab.com/libeigen/eigen/-/merge_requests/645 .  The Debian fix for this is in
// https://salsa.debian.org/science-team/eigen3/-/merge_requests/1 .
// However, it is not clear that that fix is going to make it into Ubuntu 22.04 before it
// freezes, so disable the warning here.
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wclass-memaccess"
#endif
#include <Eigen/Core>
#include <Eigen/Geometry>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

#include "kdl/frames.hpp"

#include "tf2/impl/convert.h"

#include "tf2_eigen_kdl/visibility_control.h"

namespace tf2
{

/// Converts a KDL rotation into an Eigen quaternion
TF2_EIGEN_KDL_PUBLIC
void quaternionKDLToEigen(const KDL::Rotation & k, Eigen::Quaterniond & e);

/// Converts an Eigen quaternion into a KDL rotation
TF2_EIGEN_KDL_PUBLIC
void quaternionEigenToKDL(const Eigen::Quaterniond & e, KDL::Rotation & k);

/// Converts a KDL frame into an Eigen Affine3d
TF2_EIGEN_KDL_PUBLIC
void transformKDLToEigen(const KDL::Frame & k, Eigen::Affine3d & e);

/// Converts a KDL frame into an Eigen Isometry3d
TF2_EIGEN_KDL_PUBLIC
void transformKDLToEigen(const KDL::Frame & k, Eigen::Isometry3d & e);

/// Converts an Eigen Affine3d into a KDL frame
TF2_EIGEN_KDL_PUBLIC
void transformEigenToKDL(const Eigen::Affine3d & e, KDL::Frame & k);

/// Converts an Eigen Isometry3d into a KDL frame
TF2_EIGEN_KDL_PUBLIC
void transformEigenToKDL(const Eigen::Isometry3d & e, KDL::Frame & k);

/// Converts a KDL twist into an Eigen matrix
TF2_EIGEN_KDL_PUBLIC
void twistKDLToEigen(const KDL::Twist & k, Eigen::Matrix<double, 6, 1> & e);

/// Converts an Eigen matrix into a KDL Twist
TF2_EIGEN_KDL_PUBLIC
void twistEigenToKDL(const Eigen::Matrix<double, 6, 1> & e, KDL::Twist & k);

/// Converts a KDL vector into an Eigen matrix
TF2_EIGEN_KDL_PUBLIC
void vectorKDLToEigen(const KDL::Vector & k, Eigen::Matrix<double, 3, 1> & e);

/// Converts an Eigen matrix into a KDL vector
TF2_EIGEN_KDL_PUBLIC
void vectorEigenToKDL(const Eigen::Matrix<double, 3, 1> & e, KDL::Vector & k);

/// Converts a KDL wrench into an Eigen matrix
TF2_EIGEN_KDL_PUBLIC
void wrenchKDLToEigen(const KDL::Wrench & k, Eigen::Matrix<double, 6, 1> & e);

/// Converts an Eigen matrix into a KDL wrench
TF2_EIGEN_KDL_PUBLIC
void wrenchEigenToKDL(const Eigen::Matrix<double, 6, 1> & e, KDL::Wrench & k);

namespace impl
{

template<>
template<>
inline void Converter<false, false>::convert(const KDL::Rotation & a, Eigen::Quaterniond & b)
{
  quaternionKDLToEigen(a, b);
}

template<>
template<>
inline void Converter<false, false>::convert(const Eigen::Quaterniond & a, KDL::Rotation & b)
{
  quaternionEigenToKDL(a, b);
}

template<>
template<>
inline void Converter<false, false>::convert(const KDL::Frame & a, Eigen::Affine3d & b)
{
  transformKDLToEigen(a, b);
}

template<>
template<>
inline void Converter<false, false>::convert(const KDL::Frame & a, Eigen::Isometry3d & b)
{
  transformKDLToEigen(a, b);
}

template<>
template<>
inline void Converter<false, false>::convert(const Eigen::Affine3d & a, KDL::Frame & b)
{
  transformEigenToKDL(a, b);
}

template<>
template<>
inline void Converter<false, false>::convert(const Eigen::Isometry3d & a, KDL::Frame & b)
{
  transformEigenToKDL(a, b);
}

template<>
template<>
inline void Converter<false, false>::convert(const KDL::Twist & a, Eigen::Matrix<double, 6, 1> & b)
{
  twistKDLToEigen(a, b);
}

template<>
template<>
inline void Converter<false, false>::convert(const Eigen::Matrix<double, 6, 1> & a, KDL::Twist & b)
{
  twistEigenToKDL(a, b);
}

template<>
template<>
inline void Converter<false, false>::convert(const KDL::Vector & a, Eigen::Matrix<double, 3, 1> & b)
{
  vectorKDLToEigen(a, b);
}

template<>
template<>
inline void Converter<false, false>::convert(const Eigen::Matrix<double, 3, 1> & a, KDL::Vector & b)
{
  vectorEigenToKDL(a, b);
}

template<>
template<>
inline void Converter<false, false>::convert(const KDL::Wrench & a, Eigen::Matrix<double, 6, 1> & b)
{
  wrenchKDLToEigen(a, b);
}

template<>
template<>
inline void Converter<false, false>::convert(const Eigen::Matrix<double, 6, 1> & a, KDL::Wrench & b)
{
  wrenchEigenToKDL(a, b);
}
}  // namespace impl

}  // namespace tf2

#endif  // TF2_EIGEN_KDL__TF2_EIGEN_KDL_HPP_
