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

#include <gtest/gtest.h>

#include <tf2_eigen_kdl/tf2_eigen_kdl.hpp>
#include <tf2/convert.hpp>

using Vector6d = Eigen::Matrix<double, 6, 1>;

TEST(TfEigenKdl, TestRotationQuaternion)
{
  const auto kdl_v = KDL::Rotation::RPY(1.5, 0.2, 0.3);
  Eigen::Quaterniond eigen_v = Eigen::Quaterniond::Identity();
  tf2::convert(kdl_v, eigen_v);
  KDL::Rotation kdl_v1;
  tf2::convert(eigen_v, kdl_v1);
  EXPECT_EQ(kdl_v, kdl_v1);
}

TEST(TfEigenKdl, TestQuaternionRotation)
{
  const Eigen::Quaterniond eigen_v = Eigen::Quaterniond(1, 2, 1.5, 3).normalized();
  KDL::Rotation kdl_v;
  tf2::convert(eigen_v, kdl_v);
  Eigen::Quaterniond eigen_v1;
  tf2::convert(kdl_v, eigen_v1);
  EXPECT_TRUE(eigen_v.isApprox(eigen_v1));
}

TEST(TfEigenKdl, TestFrameIsometry3d)
{
  const auto kdl_v = KDL::Frame(KDL::Rotation::RPY(1.2, 0.2, 0), KDL::Vector(1, 2, 3));
  Eigen::Isometry3d eigen_v = Eigen::Isometry3d::Identity();
  tf2::convert(kdl_v, eigen_v);
  KDL::Frame kdl_v1;
  tf2::convert(eigen_v, kdl_v1);
  EXPECT_EQ(kdl_v, kdl_v1);
}

TEST(TfEigenKdl, TestIsometry3dFrame)
{
  const Eigen::Isometry3d eigen_v(
    Eigen::Translation3d(1, 2, 3) * Eigen::AngleAxisd(1, Eigen::Vector3d::UnitX()));
  KDL::Frame kdl_v;
  tf2::convert(eigen_v, kdl_v);
  Eigen::Isometry3d eigen_v1;
  tf2::convert(kdl_v, eigen_v1);
  EXPECT_EQ(eigen_v.translation(), eigen_v1.translation());
  EXPECT_EQ(eigen_v.rotation(), eigen_v1.rotation());
}

TEST(TfEigenKdl, TestFrameAffine3d)
{
  const auto kdl_v = KDL::Frame(KDL::Rotation::RPY(1.2, 0.2, 0), KDL::Vector(1, 2, 3));
  Eigen::Affine3d eigen_v = Eigen::Affine3d::Identity();
  tf2::convert(kdl_v, eigen_v);
  KDL::Frame kdl_v1;
  tf2::convert(eigen_v, kdl_v1);
  EXPECT_EQ(kdl_v, kdl_v1);
}

TEST(TfEigenKdl, TestTwistMatrix)
{
  const auto kdl_v = KDL::Twist(KDL::Vector(1, 2, 3), KDL::Vector(4, 5, 6));
  Vector6d eigen_v;
  tf2::convert(kdl_v, eigen_v);
  KDL::Twist kdl_v1;
  tf2::convert(eigen_v, kdl_v1);
  EXPECT_EQ(kdl_v, kdl_v1);
}

TEST(TfEigenKdl, TestMatrixWrench)
{
  Vector6d eigen_v;
  eigen_v << 1, 2, 3, 3, 2, 1;
  KDL::Wrench kdl_v;
  tf2::convert(eigen_v, kdl_v);
  Vector6d eigen_v1;
  tf2::convert(kdl_v, eigen_v1);
  EXPECT_EQ(eigen_v, eigen_v1);
}


TEST(TfEigenKdl, TestVectorVector3d)
{
  const auto kdl_v = KDL::Vector(1, 2, 3);
  Eigen::Vector3d eigen_v;
  tf2::convert(kdl_v, eigen_v);
  KDL::Vector kdl_v1;
  tf2::convert(eigen_v, kdl_v1);
  EXPECT_EQ(kdl_v, kdl_v1);
}


TEST(TfEigenKdl, TestVector3dVector)
{
  Eigen::Vector3d eigen_v;
  eigen_v << 1, 2, 3;
  KDL::Vector kdl_v;
  tf2::convert(eigen_v, kdl_v);
  Eigen::Vector3d eigen_v1;
  tf2::convert(kdl_v, eigen_v1);
  EXPECT_EQ(eigen_v, eigen_v1);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
