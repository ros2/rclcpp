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

#include <rcl/allocator.h>
#include <rosidl_dynamic_typesupport/types.h>

#include <memory>
#include <string>

#include "rclcpp/dynamic_typesupport/dynamic_serialization_support.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace dynamic_typesupport
{

class DynamicMessage;
class DynamicMessageTypeBuilder;

/// Utility wrapper class for `rosidl_dynamic_typesupport_dynamic_type_t`
/**
 * This class:
 * - Exposes getter methods for the struct
 * - Exposes the underlying serialization support API
 *
 * Ownership:
 * - This class borrows the `rosidl_dynamic_typesupport_serialization_support_t` stored in the
 *   passed `DynamicSerializationSupport`.
 *   So it cannot outlive the `DynamicSerializationSupport`.
 * - The `DynamicSerializationSupport`'s `rosidl_dynamic_typesupport_serialization_support_t`
 *   pointer must point to the same location in memory as the stored raw pointer!
 *
 * This class is meant to map to the lower level `rosidl_dynamic_typesupport_dynamic_type_t`,
 * which can be constructed via `DynamicMessageTypeBuilder`, which maps to
 * `rosidl_dynamic_typesupport_dynamic_type_builder_t`.
 *
 *  The usual method of obtaining a `DynamicMessageType` is through construction of
 *  `rosidl_message_type_support_t` via `rcl_dynamic_message_type_support_handle_create()`, then
 *  taking ownership of its contents. But `DynamicMessageTypeBuilder` can also be used to obtain
 *  `DynamicMessageType` by constructing it bottom-up instead, since it exposes the lower_level
 *  rosidl methods.
 */
class DynamicMessageType : public std::enable_shared_from_this<DynamicMessageType>
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(DynamicMessageType)

  // CONSTRUCTION ==================================================================================
  // Most constructors require a passed in `DynamicSerializationSupport::SharedPtr`, to extend the
  // lifetime of the serialization support (if the constructor cannot otherwise get it from args).
  //
  // In cases where a dynamic type pointer is passed, the serialization support composed by
  // the type should be the exact same object managed by the `DynamicSerializationSupport`,
  // otherwise the lifetime management will not work properly.

  /// Construct a new `DynamicMessageType` with the provided dynamic type builder,
  /// using its allocator
  RCLCPP_PUBLIC
  explicit DynamicMessageType(std::shared_ptr<DynamicMessageTypeBuilder> dynamic_type_builder);

  /// Construct a new `DynamicMessageType` with the provided dynamic type builder and allocator
  RCLCPP_PUBLIC
  DynamicMessageType(
    std::shared_ptr<DynamicMessageTypeBuilder> dynamic_type_builder,
    rcl_allocator_t allocator);

  /// Assume ownership of struct
  RCLCPP_PUBLIC
  DynamicMessageType(
    DynamicSerializationSupport::SharedPtr serialization_support,
    rosidl_dynamic_typesupport_dynamic_type_t && rosidl_dynamic_type);

  /// From description
  RCLCPP_PUBLIC
  DynamicMessageType(
    DynamicSerializationSupport::SharedPtr serialization_support,
    const rosidl_runtime_c__type_description__TypeDescription & description,
    rcl_allocator_t allocator = rcl_get_default_allocator());

  /// Move constructor
  RCLCPP_PUBLIC
  DynamicMessageType(DynamicMessageType && other) noexcept;

  /// Move assignment
  RCLCPP_PUBLIC
  DynamicMessageType & operator=(DynamicMessageType && other) noexcept;

  RCLCPP_PUBLIC
  virtual ~DynamicMessageType();

  /// Swaps the serialization support if `serialization_support` is populated
  /**
   * The user can call this with another description to reconfigure the type without changing the
   * serialization support
   */
  RCLCPP_PUBLIC
  void
  init_from_description(
    const rosidl_runtime_c__type_description__TypeDescription & description,
    rcl_allocator_t allocator = rcl_get_default_allocator(),
    DynamicSerializationSupport::SharedPtr serialization_support = nullptr);

  // GETTERS =======================================================================================
  RCLCPP_PUBLIC
  const std::string
  get_serialization_library_identifier() const;

  RCLCPP_PUBLIC
  const std::string
  get_name() const;

  RCLCPP_PUBLIC
  size_t
  get_member_count() const;

  RCLCPP_PUBLIC
  rosidl_dynamic_typesupport_dynamic_type_t &
  get_rosidl_dynamic_type();

  RCLCPP_PUBLIC
  const rosidl_dynamic_typesupport_dynamic_type_t &
  get_rosidl_dynamic_type() const;

  RCLCPP_PUBLIC
  DynamicSerializationSupport::SharedPtr
  get_shared_dynamic_serialization_support();

  RCLCPP_PUBLIC
  DynamicSerializationSupport::ConstSharedPtr
  get_shared_dynamic_serialization_support() const;


  // METHODS =======================================================================================
  RCLCPP_PUBLIC
  DynamicMessageType
  clone(rcl_allocator_t allocator = rcl_get_default_allocator());

  RCLCPP_PUBLIC
  DynamicMessageType::SharedPtr
  clone_shared(rcl_allocator_t allocator = rcl_get_default_allocator());

  RCLCPP_PUBLIC
  bool
  equals(const DynamicMessageType & other) const;

  RCLCPP_PUBLIC
  DynamicMessage
  build_dynamic_message(rcl_allocator_t allocator = rcl_get_default_allocator());

  RCLCPP_PUBLIC
  std::shared_ptr<DynamicMessage>
  build_dynamic_message_shared(rcl_allocator_t allocator = rcl_get_default_allocator());

protected:
  // NOTE(methylDragon):
  // This is just here to extend the lifetime of the serialization support
  // It isn't actually used by the builder since the builder should compose its own support
  //
  // ... Though ideally it should be the exact same support as the one stored in the
  // `DynamicSerializationSupport`
  DynamicSerializationSupport::SharedPtr serialization_support_;

  rosidl_dynamic_typesupport_dynamic_type_t rosidl_dynamic_type_;

private:
  RCLCPP_DISABLE_COPY(DynamicMessageType)

  RCLCPP_PUBLIC
  DynamicMessageType();

  RCLCPP_PUBLIC
  bool
  match_serialization_support_(
    const DynamicSerializationSupport & serialization_support,
    const rosidl_dynamic_typesupport_dynamic_type_t & rosidl_dynamic_type);
};

}  // namespace dynamic_typesupport
}  // namespace rclcpp

#endif  // RCLCPP__DYNAMIC_TYPESUPPORT__DYNAMIC_MESSAGE_TYPE_HPP_
