// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__DYNAMIC_TYPESUPPORT__DYNAMIC_SERIALIZATION_SUPPORT_HPP_
#define RCLCPP__DYNAMIC_TYPESUPPORT__DYNAMIC_SERIALIZATION_SUPPORT_HPP_

#include <rcl/allocator.h>
#include <rosidl_dynamic_typesupport/api/serialization_support.h>
#include <rosidl_dynamic_typesupport/types.h>

#include <memory>
#include <string>

#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace dynamic_typesupport
{

/// Utility wrapper class for rosidl_dynamic_typesupport_serialization_support_t
/**
 * This class:
 * - Exposes getter methods for the struct
 * - Exposes the underlying serialization support API
 *
 * Ownership:
 * - This class, similarly to the rosidl_dynamic_typesupport_serialization_support_t, must outlive
 *   all downstream usages of the serialization support.
 */
class DynamicSerializationSupport : public std::enable_shared_from_this<DynamicSerializationSupport>
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(DynamicSerializationSupport)

  // CONSTRUCTION ==================================================================================
  RCLCPP_PUBLIC
  explicit DynamicSerializationSupport(rcl_allocator_t allocator = rcl_get_default_allocator());

  /// Get the rmw middleware implementation specific serialization support (configured by name)
  RCLCPP_PUBLIC
  DynamicSerializationSupport(
    const std::string & serialization_library_name,
    rcl_allocator_t allocator = rcl_get_default_allocator());

  /// Assume ownership of struct
  RCLCPP_PUBLIC
  explicit DynamicSerializationSupport(
    rosidl_dynamic_typesupport_serialization_support_t && rosidl_serialization_support);

  /// Move constructor
  RCLCPP_PUBLIC
  DynamicSerializationSupport(DynamicSerializationSupport && other) noexcept;

  /// Move assignment
  RCLCPP_PUBLIC
  DynamicSerializationSupport & operator=(DynamicSerializationSupport && other) noexcept;

  RCLCPP_PUBLIC
  virtual ~DynamicSerializationSupport();


  // GETTERS =======================================================================================
  RCLCPP_PUBLIC
  const std::string
  get_serialization_library_identifier() const;

  RCLCPP_PUBLIC
  rosidl_dynamic_typesupport_serialization_support_t &
  get_rosidl_serialization_support();

  RCLCPP_PUBLIC
  const rosidl_dynamic_typesupport_serialization_support_t &
  get_rosidl_serialization_support() const;

protected:
  RCLCPP_DISABLE_COPY(DynamicSerializationSupport)

private:
  rosidl_dynamic_typesupport_serialization_support_t rosidl_serialization_support_;
};

}  // namespace dynamic_typesupport
}  // namespace rclcpp

#endif  // RCLCPP__DYNAMIC_TYPESUPPORT__DYNAMIC_SERIALIZATION_SUPPORT_HPP_
