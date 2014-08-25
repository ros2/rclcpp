/* Copyright 2014 Open Source Robotics Foundation, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef RCLCPP_RCLCPP_MACROS_HPP_
#define RCLCPP_RCLCPP_MACROS_HPP_

/* Disables the copy constructor and operator= for the given class.
 *
 * Use in the private section of the class.
 */
#define RCLCPP_DISABLE_COPY(Class) \
  Class(const Class&) = delete; \
  Class& operator=(const Class&) = delete;

/* Defines a make_shared static function on the class using perfect forwarding.
 *
 * Use in the public section of the class.
 * Make sure to include <memory> in the header when using this.
 */
#define RCLCPP_MAKE_SHARED_DEFINITIONS(Class) \
  typedef std::shared_ptr<Class> SharedPtr; \
  \
  template<typename...Args> \
  static std::shared_ptr<Class> \
  make_shared(Args &&...args) \
  { \
    return std::make_shared<Class>(std::forward<Args>(args)...); \
  }

#endif /* RCLCPP_RCLCPP_MACROS_HPP_ */
