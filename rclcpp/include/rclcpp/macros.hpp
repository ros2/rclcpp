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

#ifndef RCLCPP_RCLCPP_MACROS_HPP_
#define RCLCPP_RCLCPP_MACROS_HPP_

/* Disables the copy constructor and operator= for the given class.
 *
 * Use in the private section of the class.
 */
#define RCLCPP_DISABLE_COPY(Class) \
  Class(const Class &) = delete; \
  Class & operator=(const Class &) = delete;

/* Defines aliases and static functions for using the Class with smart pointers.
 *
 * Use in the public section of the class.
 * Make sure to include <memory> in the header when using this.
 */
#define RCLCPP_SMART_PTR_DEFINITIONS(Class) \
  RCLCPP_SHARED_PTR_DEFINITIONS(Class) \
  RCLCPP_WEAK_PTR_DEFINITIONS(Class) \
  RCLCPP_UNIQUE_PTR_DEFINITIONS(Class)

/* Defines aliases and static functions for using the Class with smart pointers.
 *
 * Same as RCLCPP_SMART_PTR_DEFINITIONS expect it excludes the static
 * Class::make_unique() method definition which does not work on classes which
 * are not CopyConstructable.
 *
 * Use in the public section of the class.
 * Make sure to include <memory> in the header when using this.
 */
#define RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(Class) \
  RCLCPP_SHARED_PTR_DEFINITIONS(Class) \
  RCLCPP_WEAK_PTR_DEFINITIONS(Class) \
  __RCLCPP_UNIQUE_PTR_ALIAS(Class)

#define __RCLCPP_SHARED_PTR_ALIAS(Class) using SharedPtr = std::shared_ptr<Class>;

#define __RCLCPP_MAKE_SHARED_DEFINITION(Class) \
  template<typename ... Args> \
  static std::shared_ptr<Class> \
  make_shared(Args && ... args) \
  { \
    return std::make_shared<Class>(std::forward<Args>(args) ...); \
  }

/// Defines aliases and static functions for using the Class with shared_ptrs.
#define RCLCPP_SHARED_PTR_DEFINITIONS(Class) \
  __RCLCPP_SHARED_PTR_ALIAS(Class) \
  __RCLCPP_MAKE_SHARED_DEFINITION(Class)

#define __RCLCPP_WEAK_PTR_ALIAS(Class) using WeakPtr = std::weak_ptr<Class>;

/// Defines aliases and static functions for using the Class with weak_ptrs.
#define RCLCPP_WEAK_PTR_DEFINITIONS(Class) __RCLCPP_WEAK_PTR_ALIAS(Class)

#define __RCLCPP_UNIQUE_PTR_ALIAS(Class) using UniquePtr = std::unique_ptr<Class>;

#define __RCLCPP_MAKE_UNIQUE_DEFINITION(Class) \
  template<typename ... Args> \
  static std::unique_ptr<Class> \
  make_unique(Args && ... args) \
  { \
    return std::unique_ptr<Class>(new Class(std::forward<Args>(args) ...)); \
  }
/// Defines aliases and static functions for using the Class with unique_ptrs.
#define RCLCPP_UNIQUE_PTR_DEFINITIONS(Class) \
  __RCLCPP_UNIQUE_PTR_ALIAS(Class) \
  __RCLCPP_MAKE_UNIQUE_DEFINITION(Class)

#define RCLCPP_INFO(Args) std::cout << Args << std::endl;

#endif /* RCLCPP_RCLCPP_MACROS_HPP_ */
