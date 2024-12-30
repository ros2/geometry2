// Copyright 2014 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TF2__IMPL__UTILS_H_
#define TF2__IMPL__UTILS_H_

#ifdef __GNUC__
#define TF2_IMPL_UTILS_H_PRAGMA(X) _Pragma(#X)
#define TF2_IMPL_UTILS_H_DEPRECATED(MSG) TF2_IMPL_UTILS_H_PRAGMA(GCC warning MSG)
#elif defined(_MSC_VER)
#define TF2_IMPL_UTILS_H_STRINGIZE_(MSG) #MSG
#define TF2_IMPL_UTILS_H_STRINGIZE(MSG) TF2_IMPL_UTILS_H_STRINGIZE_(MSG)
#define TF2_IMPL_UTILS_H_DEPRECATED(MSG) \
  __pragma(message(__FILE__ "(" TF2_IMPL_UTILS_H_STRINGIZE(__LINE__) ") : Warning: " MSG))
#else
#define TF2_IMPL_UTILS_H_DEPRECATED(MSG)
#endif

TF2_IMPL_UTILS_H_DEPRECATED(  // NOLINT
  "This header is obsolete, please include 'tf2/impl/utils.hpp' instead")

#include <tf2/impl/utils.hpp>

#endif  // TF2__IMPL__UTILS_H_
