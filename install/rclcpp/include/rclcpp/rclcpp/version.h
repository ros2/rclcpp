// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__VERSION_H_
#define RCLCPP__VERSION_H_

/// \def RCLCPP_VERSION_MAJOR
/// Defines RCLCPP major version number
#define RCLCPP_VERSION_MAJOR (21)

/// \def RCLCPP_VERSION_MINOR
/// Defines RCLCPP minor version number
#define RCLCPP_VERSION_MINOR (1)

/// \def RCLCPP_VERSION_PATCH
/// Defines RCLCPP version patch number
#define RCLCPP_VERSION_PATCH (0)

/// \def RCLCPP_VERSION_STR
/// Defines RCLCPP version string
#define RCLCPP_VERSION_STR "21.1.0"

/// \def RCLCPP_VERSION_GTE
/// Defines a macro to check whether the version of RCLCPP is greater than or equal to
/// the given version triple.
#define RCLCPP_VERSION_GTE(major, minor, patch) ( \
     (major < RCLCPP_VERSION_MAJOR) ? true \
     : ((major > RCLCPP_VERSION_MAJOR) ? false \
     : ((minor < RCLCPP_VERSION_MINOR) ? true \
     : ((minor > RCLCPP_VERSION_MINOR) ? false \
     : ((patch < RCLCPP_VERSION_PATCH) ? true \
     : ((patch > RCLCPP_VERSION_PATCH) ? false \
     : true))))))

#endif  // RCLCPP__VERSION_H_
