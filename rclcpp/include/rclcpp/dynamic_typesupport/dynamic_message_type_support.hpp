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

#ifndef RCLCPP__DYNAMIC_TYPESUPPORT__DYNAMIC_MESSAGE_TYPE_SUPPORT_HPP_
#define RCLCPP__DYNAMIC_TYPESUPPORT__DYNAMIC_MESSAGE_TYPE_SUPPORT_HPP_

#include <memory>
#include <string>

#include "rclcpp/dynamic_typesupport/dynamic_message_type.hpp"
#include "rclcpp/dynamic_typesupport/dynamic_message.hpp"
#include "rclcpp/dynamic_typesupport/dynamic_serialization_support.hpp"

#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"

#include <rosidl_dynamic_typesupport/types.h>
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_runtime_c/type_description/type_description__struct.h>

namespace rclcpp
{
namespace dynamic_typesupport
{

/// Utility wrapper class for rosidl_message_type_support_t * containing managed
/// instances of the typesupport handle impl.
/**
 *
 * NOTE: This class is the recommended way to obtain the dynamic message type
 *       support struct, instead of rcl_get_dynamic_message_typesupport_handle,
 *       because this class will manage the lifetimes for you.
 *
 *       Do NOT call rcl_dynamic_message_typesupport_handle_fini!!
 *
 * This class:
 * - Manages the lifetime of the raw pointer.
 * - Exposes getter methods to get the raw pointer and shared pointers
 * - Stores shared pointers to wrapper classes that expose the underlying
 *   serialization support API
 *
 * Ownership:
 * - This class, similarly to the rosidl_dynamic_typesupport_serialization_support_t, must outlive
 *   all downstream usages of the serialization support.
 */
class DynamicMessageTypeSupport : public std::enable_shared_from_this<DynamicMessageTypeSupport>
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(DynamicMessageTypeSupport)

  // CONSTRUCTION ==================================================================================
  /// From description
  RCLCPP_PUBLIC
  DynamicMessageTypeSupport(
    rosidl_runtime_c__type_description__TypeDescription * description,
    const std::string & serialization_library_name = "");

  /// From description, for provided serialization support
  RCLCPP_PUBLIC
  DynamicMessageTypeSupport(
    DynamicSerializationSupport::SharedPtr serialization_support,
    rosidl_runtime_c__type_description__TypeDescription * description);

  /// Assume ownership of managed types
  RCLCPP_PUBLIC
  DynamicMessageTypeSupport(
    DynamicSerializationSupport::SharedPtr serialization_support,
    DynamicMessageType::SharedPtr dynamic_message_type,
    DynamicMessage::SharedPtr dynamic_message,
    rosidl_runtime_c__type_description__TypeDescription * description = nullptr);

  RCLCPP_PUBLIC
  virtual ~DynamicMessageTypeSupport();


  // GETTERS =======================================================================================
  RCLCPP_PUBLIC
  const std::string
  get_library_identifier() const;

  RCLCPP_PUBLIC
  rosidl_message_type_support_t *
  get_rosidl_message_type_support();

  RCLCPP_PUBLIC
  const rosidl_message_type_support_t *
  get_rosidl_message_type_support() const;

  RCLCPP_PUBLIC
  std::shared_ptr<rosidl_message_type_support_t>
  get_shared_rosidl_message_type_support();

  RCLCPP_PUBLIC
  std::shared_ptr<const rosidl_message_type_support_t>
  get_shared_rosidl_message_type_support() const;

  RCLCPP_PUBLIC
  rosidl_runtime_c__type_description__TypeDescription *
  get_rosidl_runtime_c_type_description();

  RCLCPP_PUBLIC
  const rosidl_runtime_c__type_description__TypeDescription *
  get_rosidl_runtime_c_type_description() const;

  RCLCPP_PUBLIC
  std::shared_ptr<rosidl_runtime_c__type_description__TypeDescription>
  get_shared_rosidl_runtime_c_type_description();

  RCLCPP_PUBLIC
  std::shared_ptr<const rosidl_runtime_c__type_description__TypeDescription>
  get_shared_rosidl_runtime_c_type_description() const;

  RCLCPP_PUBLIC
  DynamicSerializationSupport::SharedPtr
  get_shared_dynamic_serialization_support();

  RCLCPP_PUBLIC
  DynamicSerializationSupport::ConstSharedPtr
  get_shared_dynamic_serialization_support() const;

  RCLCPP_PUBLIC
  DynamicMessageType::SharedPtr
  get_shared_dynamic_message_type();

  RCLCPP_PUBLIC
  DynamicMessageType::ConstSharedPtr
  get_shared_dynamic_message_type() const;

  RCLCPP_PUBLIC
  DynamicMessage::SharedPtr
  get_shared_dynamic_message();

  RCLCPP_PUBLIC
  DynamicMessage::ConstSharedPtr
  get_shared_dynamic_message() const;


  // METHODS =======================================================================================
  RCLCPP_PUBLIC
  void
  print_description() const;

protected:
  RCLCPP_DISABLE_COPY(DynamicMessageTypeSupport)

  DynamicSerializationSupport::SharedPtr serialization_support_;
  DynamicMessageType::SharedPtr dynamic_message_type_;
  DynamicMessage::SharedPtr dynamic_message_;
  std::shared_ptr<rosidl_runtime_c__type_description__TypeDescription> description_;

  std::shared_ptr<rosidl_message_type_support_t> rosidl_message_type_support_;

private:
  RCLCPP_PUBLIC
  DynamicMessageTypeSupport();

  RCLCPP_PUBLIC
  void
  init_serialization_support_(const std::string & serialization_library_name);

  RCLCPP_PUBLIC
  void
  init_dynamic_message_type_(
    DynamicSerializationSupport::SharedPtr serialization_support,
    const rosidl_runtime_c__type_description__TypeDescription * description);

  RCLCPP_PUBLIC
  void
  init_dynamic_message_(DynamicType::SharedPtr dynamic_type);

  RCLCPP_PUBLIC
  void
  init_rosidl_message_type_support_(
    DynamicSerializationSupport::SharedPtr serialization_support,
    DynamicMessageType::SharedPtr dynamic_message_type,
    DynamicMessage::SharedPtr dynamic_message,
    rosidl_runtime_c__type_description__TypeDescription * description);
};


}  // namespace dynamic_typesupport
}  // namespace rclcpp

#endif  // RCLCPP__DYNAMIC_TYPESUPPORT__DYNAMIC_MESSAGE_TYPE_SUPPORT_HPP_
