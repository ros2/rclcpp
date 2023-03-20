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

#ifndef RCLCPP__DYNAMIC_TYPESUPPORT__DYNAMIC_MESSAGE_TYPE_HPP_
#define RCLCPP__DYNAMIC_TYPESUPPORT__DYNAMIC_MESSAGE_TYPE_HPP_

#include <memory>
#include <string>

#include "rclcpp/dynamic_typesupport/dynamic_serialization_support.hpp"
#include "rclcpp/dynamic_typesupport/dynamic_type.hpp"
#include "rclcpp/dynamic_typesupport/dynamic_type_builder.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"

#include <rosidl_dynamic_typesupport/types.h>


namespace rclcpp
{
namespace dynamic_typesupport
{


/// Thin wrapper around DynamicType object for message pubsub type representation
/**
 * This class:
 * - Manages the lifetime of the raw pointer.
 * - Exposes getter methods to get the raw pointer and shared pointers
 * - Exposes the underlying serialization support API
 *
 * Ownership:
 * - This class borrows the rosidl_dynamic_typesupport_serialization_support_t stored in the passed
 *   DynamicSerializationSupport. So it cannot outlive the DynamicSerializationSupport.
 * - The DynamicSerializationSupport's rosidl_dynamic_typesupport_serialization_support_t pointer
 *   must point to the same location in memory as the stored raw pointer!
 */
class DynamicMessageType final : public DynamicType
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(DynamicMessageType)

  // CONSTRUCTION ==================================================================================
  // Most constructors require a passed in DynamicSerializationSupport::SharedPtr, to extend the
  // lifetime of the serialization support (if the constructor cannot otherwise get it from args).
  //
  // In cases where a dynamic type pointer is passed, the serialization support composed by
  // the type should be the exact same object managed by the DynamicSerializationSupport,
  // otherwise the lifetime management will not work properly.

  /// Construct a new DynamicType with the provided dynamic type builder
  RCLCPP_PUBLIC
  explicit DynamicMessageType(std::shared_ptr<DynamicTypeBuilder> dynamic_type_builder);

  /// Assume ownership of raw pointer
  RCLCPP_PUBLIC
  DynamicMessageType(
    DynamicSerializationSupport::SharedPtr serialization_support,
    rosidl_dynamic_typesupport_dynamic_type_t * rosidl_dynamic_type);

  /// Copy shared pointer
  RCLCPP_PUBLIC
  DynamicMessageType(
    DynamicSerializationSupport::SharedPtr serialization_support,
    std::shared_ptr<rosidl_dynamic_typesupport_dynamic_type_t> rosidl_dynamic_type);

  /// From description
  RCLCPP_PUBLIC
  DynamicMessageType(
    DynamicSerializationSupport::SharedPtr serialization_support,
    const rosidl_runtime_c__type_description__TypeDescription * description);

private:
  RCLCPP_PUBLIC
  DynamicMessageType();
};


}  // namespace dynamic_typesupport
}  // namespace rclcpp

#endif  // RCLCPP__DYNAMIC_TYPESUPPORT__DYNAMIC_MESSAGE_TYPE_HPP_
