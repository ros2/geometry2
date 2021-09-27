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

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "tf2/LinearMath/Quaternion.h"

std::vector<double> values;
unsigned int step = 0;

void seed_rand()
{
  values.clear();
  for (unsigned int i = 0; i < 2000; i++) {
    int pseudo_rand = static_cast<int>(std::floor(static_cast<double>(i * 3.141592653589793)));
    values.push_back(( pseudo_rand % 100) / 50.0 - 1.0);
  }
}

double get_rand()
{
  if (values.size() == 0) {throw std::runtime_error("you need to call seed_rand first");}
  if (step >= values.size()) {
    step = 0;
  } else {
    step++;
  }
  return values[step];
}

TEST(Quaternion, Slerp)
{
  uint64_t runs = 100;
  seed_rand();

  tf2::Quaternion q1, q2;
  q1.setEuler(0, 0, 0);

  for (uint64_t i = 0; i < runs; i++) {
    q2.setEuler(
      1.0 * get_rand(),
      1.0 * get_rand(),
      1.0 * get_rand());
    tf2::Quaternion q3 = slerp(q1, q2, 0.5);

    EXPECT_NEAR(q3.angle(q1), q2.angle(q3), 1e-5);
  }
}
