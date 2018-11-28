// Copyright 2018 Robert Bosch GmbH
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

#ifndef TF2_FILTER__VISIBILITY_CONTROL_H_
#define TF2_FILTER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TF2_FILTER_EXPORT __attribute__ ((dllexport))
    #define TF2_FILTER_IMPORT __attribute__ ((dllimport))
  #else
    #define TF2_FILTER_EXPORT __declspec(dllexport)
    #define TF2_FILTER_IMPORT __declspec(dllimport)
  #endif
  #ifdef TF2_FILTER_BUILDING_LIBRARY
    #define TF2_FILTER_PUBLIC TF2_FILTER_EXPORT
  #else
    #define TF2_FILTER_PUBLIC TF2_FILTER_IMPORT
  #endif
  #define TF2_FILTER_PUBLIC_TYPE TF2_FILTER_PUBLIC
  #define TF2_FILTER_LOCAL
#else
  #define TF2_FILTER_EXPORT __attribute__ ((visibility("default")))
  #define TF2_FILTER_IMPORT
  #if __GNUC__ >= 4
    #define TF2_FILTER_PUBLIC __attribute__ ((visibility("default")))
    #define TF2_FILTER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TF2_FILTER_PUBLIC
    #define TF2_FILTER_LOCAL
  #endif
  #define TF2_FILTER_PUBLIC_TYPE
#endif

#endif  // TF2_FILTER__VISIBILITY_CONTROL_H_
