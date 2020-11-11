// Copyright 2018 Open Source Robotics Foundation, Inc.
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

// This header must be included by all rclcpp_generic headers which declare
// symbols which are defined in the rclcpp_generic library. When not building
// the rclcpp_generic library, i.e. when using the headers in other package's
// code, the contents of this header change the visibility of certain symbols
// which the rclcpp_generic library cannot have, but the consuming code must
// have in order to link.

#ifndef RCLCPP_GENERIC__VISIBILITY_CONTROL_HPP_
#define RCLCPP_GENERIC__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RCLCPP_GENERIC_EXPORT __attribute__ ((dllexport))
    #define RCLCPP_GENERIC_IMPORT __attribute__ ((dllimport))
  #else
    #define RCLCPP_GENERIC_EXPORT __declspec(dllexport)
    #define RCLCPP_GENERIC_IMPORT __declspec(dllimport)
  #endif
  #ifdef RCLCPP_GENERIC_BUILDING_LIBRARY
    #define RCLCPP_GENERIC_PUBLIC RCLCPP_GENERIC_EXPORT
  #else
    #define RCLCPP_GENERIC_PUBLIC RCLCPP_GENERIC_IMPORT
  #endif
  #define RCLCPP_GENERIC_PUBLIC_TYPE RCLCPP_GENERIC_PUBLIC
  #define RCLCPP_GENERIC_LOCAL
#else
  #define RCLCPP_GENERIC_EXPORT __attribute__ ((visibility("default")))
  #define RCLCPP_GENERIC_IMPORT
  #if __GNUC__ >= 4
    #define RCLCPP_GENERIC_PUBLIC __attribute__ ((visibility("default")))
    #define RCLCPP_GENERIC_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RCLCPP_GENERIC_PUBLIC
    #define RCLCPP_GENERIC_LOCAL
  #endif
  #define RCLCPP_GENERIC_PUBLIC_TYPE
#endif

#endif  // RCLCPP_GENERIC__VISIBILITY_CONTROL_HPP_
