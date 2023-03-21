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

#ifndef RCLCPP__DYNAMIC_TYPESUPPORT__DYNAMIC_MESSAGE_HPP_
#define RCLCPP__DYNAMIC_TYPESUPPORT__DYNAMIC_MESSAGE_HPP_


#include <memory>
#include <string>

#include "rclcpp/dynamic_typesupport/dynamic_data.hpp"
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

using DynamicMessageType = DynamicType;
/// Thin wrapper around DynamicData object for message pubsub
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

// class DynamicMessage final : public DynamicData
// {
// public:
//   RCLCPP_SMART_PTR_ALIASES_ONLY(DynamicMessage)
//
//   // CONSTRUCTION ==================================================================================
//   // Most constructors require a passed in DynamicSerializationSupport::SharedPtr, to extend the
//   // lifetime of the serialization support (if the constructor cannot otherwise get it from args).
//   //
//   // In cases where a dynamic data pointer is passed, the serialization support composed by
//   // the data should be the exact same object managed by the DynamicSerializationSupport,
//   // otherwise the lifetime management will not work properly.
//
//   /// Construct a new DynamicMessage with the provided dynamic type builder
//   RCLCPP_PUBLIC
//   explicit DynamicMessage(DynamicTypeBuilder::SharedPtr dynamic_type_builder);
//
//   /// Construct a new DynamicMessage with the provided dynamic type
//   RCLCPP_PUBLIC
//   explicit DynamicMessage(DynamicType::SharedPtr dynamic_type);
//
//   /// Assume ownership of raw pointer
//   RCLCPP_PUBLIC
//   DynamicMessage(
//     DynamicSerializationSupport::SharedPtr serialization_support,
//     rosidl_dynamic_typesupport_dynamic_data_t * rosidl_dynamic_data);
//
//   /// Copy shared pointer
//   RCLCPP_PUBLIC
//   DynamicMessage(
//     DynamicSerializationSupport::SharedPtr serialization_support,
//     std::shared_ptr<rosidl_dynamic_typesupport_dynamic_data_t> rosidl_dynamic_data);
//
//   /// Loaning constructor
//   /// Must only be called with raw ptr obtained from loaning!
//   // NOTE(methylDragon): I'd put this in protected, but I need this exposed to
//   //                     enable_shared_from_this...
//   RCLCPP_PUBLIC
//   DynamicMessage(
//     DynamicData::SharedPtr parent_data,
//     rosidl_dynamic_typesupport_dynamic_data_t * rosidl_loaned_data);
//
//   // NOTE(methylDragon): Deliberately no constructor from description to nudge users towards using
//   //                     construction from dynamic type/builder, which is more efficient
//
// private:
//   RCLCPP_PUBLIC
//   DynamicMessage();
// };


}  // namespace dynamic_typesupport
}  // namespace rclcpp

#endif  // RCLCPP__DYNAMIC_TYPESUPPORT__DYNAMIC_MESSAGE_HPP_
