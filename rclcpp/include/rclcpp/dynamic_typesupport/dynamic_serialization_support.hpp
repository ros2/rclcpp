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

#include <memory>
#include <string>

#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"

#include <rosidl_dynamic_typesupport/types.h>


namespace rclcpp
{
namespace dynamic_typesupport
{

/// Utility wrapper class for rosidl_dynamic_typesupport_serialization_support_t *
/**
 * This class:
 * - Manages the lifetime of the raw pointer.
 * - Exposes getter methods to get the raw pointer and shared pointers
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
  /// Get the rmw middleware implementation specific serialization support (configured by name)
  RCLCPP_PUBLIC
  explicit DynamicSerializationSupport(const std::string & serialization_library_name = nullptr);

  /// Assume ownership of raw pointer
  RCLCPP_PUBLIC
  explicit DynamicSerializationSupport(
    rosidl_dynamic_typesupport_serialization_support_t * rosidl_serialization_support);

  /// Copy shared pointer
  RCLCPP_PUBLIC
  DynamicSerializationSupport(  // NOLINT(explicit)
    std::shared_ptr<rosidl_dynamic_typesupport_serialization_support_t> serialization_support);

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
  get_library_identifier() const;

  RCLCPP_PUBLIC
  rosidl_dynamic_typesupport_serialization_support_t *
  get_rosidl_serialization_support();

  RCLCPP_PUBLIC
  const rosidl_dynamic_typesupport_serialization_support_t *
  get_rosidl_serialization_support() const;

  RCLCPP_PUBLIC
  std::shared_ptr<rosidl_dynamic_typesupport_serialization_support_t>
  get_shared_rosidl_serialization_support();

  RCLCPP_PUBLIC
  std::shared_ptr<const rosidl_dynamic_typesupport_serialization_support_t>
  get_shared_rosidl_serialization_support() const;

protected:
  RCLCPP_DISABLE_COPY(DynamicSerializationSupport)

  std::shared_ptr<rosidl_dynamic_typesupport_serialization_support_t> rosidl_serialization_support_;

private:
  RCLCPP_PUBLIC
  DynamicSerializationSupport();
};


}  // namespace dynamic_typesupport
}  // namespace rclcpp

#endif  // RCLCPP__DYNAMIC_TYPESUPPORT__DYNAMIC_SERIALIZATION_SUPPORT_HPP_
