// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__DEPRECATION_CONTROL_HPP_
#define RCLCPP__DEPRECATION_CONTROL_HPP_

/*! \file deprecation_control.hpp
 * \brief Cross-platform macros for marking declarations as deprecated.
 *
 * The C++14 standard attribute [[deprecated]] is correctly accepted by
 * all major compilers when applied to functions, variables, type
 * aliases, and enumerators. However, when applied to a class declaration
 * that also carries a visibility/export macro (such as RCLCPP_PUBLIC),
 * the accepted attribute placement differs between GCC, Clang, and MSVC,
 * and no single placement of [[deprecated]] works on all three.
 *
 * These macros expand to the vendor-specific attribute syntax
 * (__attribute__((deprecated(msg))) on GCC/Clang,
 *  __declspec(deprecated(msg)) on MSVC), which can be placed immediately
 * after the class-key alongside other vendor attributes in a consistent,
 * portable way.
 *
 * Example:
 * \code
 *   class RCLCPP_PUBLIC
 *         RCLCPP_DEPRECATED_WITH_MSG("Use NewClass instead") OldClass
 *   {
 *     // ...
 *   };
 * \endcode
 *
 * For functions, variables, and other declarations where the standard
 * [[deprecated]] attribute works portably, prefer the standard attribute
 * directly.
 */

#if defined(_MSC_VER)
#  define RCLCPP_DEPRECATED            __declspec(deprecated)
#  define RCLCPP_DEPRECATED_WITH_MSG(msg) __declspec(deprecated(msg))
#elif defined(__GNUC__) || defined(__clang__)
#  define RCLCPP_DEPRECATED            __attribute__((deprecated))
#  define RCLCPP_DEPRECATED_WITH_MSG(msg) __attribute__((deprecated(msg)))
#else
#  define RCLCPP_DEPRECATED
#  define RCLCPP_DEPRECATED_WITH_MSG(msg)
#endif

#endif  // RCLCPP__DEPRECATION_CONTROL_HPP_