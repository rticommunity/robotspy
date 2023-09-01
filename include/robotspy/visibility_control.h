// Copyright 2017 Open Source Robotics Foundation, Inc.
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
#ifndef ROBOTSPY__VISIBILITY_CONTROL_H_
#define ROBOTSPY__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROBOTSPY_EXPORT __attribute__ ((dllexport))
    #define ROBOTSPY_IMPORT __attribute__ ((dllimport))
  #else
    #define ROBOTSPY_EXPORT __declspec(dllexport)
    #define ROBOTSPY_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROBOTSPY_BUILDING_LIBRARY
    #define ROBOTSPY_PUBLIC ROBOTSPY_EXPORT
  #else
    #define ROBOTSPY_PUBLIC ROBOTSPY_IMPORT
  #endif
  #define ROBOTSPY_PUBLIC_TYPE ROBOTSPY_PUBLIC
  #define ROBOTSPY_LOCAL
#else
  #define ROBOTSPY_EXPORT __attribute__ ((visibility("default")))
  #define ROBOTSPY_IMPORT
  #if __GNUC__ >= 4
    #define ROBOTSPY_PUBLIC __attribute__ ((visibility("default")))
    #define ROBOTSPY_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROBOTSPY_PUBLIC
    #define ROBOTSPY_LOCAL
  #endif
  #define ROBOTSPY_PUBLIC_TYPE
#endif

#endif  // ROBOTSPY__VISIBILITY_CONTROL_H_
