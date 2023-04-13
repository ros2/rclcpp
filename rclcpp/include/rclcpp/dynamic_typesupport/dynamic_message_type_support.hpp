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

#include <rcl/allocator.h>

#include <rosidl_dynamic_typesupport/dynamic_message_type_support_struct.h>
#include <rosidl_dynamic_typesupport/types.h>
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_runtime_c/type_description/type_description__struct.h>

#include <memory>
#include <string>

#include "rclcpp/dynamic_typesupport/dynamic_message.hpp"
#include "rclcpp/dynamic_typesupport/dynamic_message_type.hpp"
#include "rclcpp/dynamic_typesupport/dynamic_serialization_support.hpp"

#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace dynamic_typesupport
{

/// Utility wrapper class for `rosidl_message_type_support_t` containing managed
/// STUBBED OUT
class DynamicMessageTypeSupport : public std::enable_shared_from_this<DynamicMessageTypeSupport>
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(DynamicMessageTypeSupport)

  RCLCPP_PUBLIC
  virtual ~DynamicMessageTypeSupport();

protected:
  DynamicSerializationSupport::SharedPtr serialization_support_;
  DynamicMessageType::SharedPtr dynamic_message_type_;
  DynamicMessage::SharedPtr dynamic_message_;

  rosidl_message_type_support_t rosidl_message_type_support_;

private:
  RCLCPP_DISABLE_COPY(DynamicMessageTypeSupport)

  RCLCPP_PUBLIC
  DynamicMessageTypeSupport();
};

}  // namespace dynamic_typesupport
}  // namespace rclcpp

#endif  // RCLCPP__DYNAMIC_TYPESUPPORT__DYNAMIC_MESSAGE_TYPE_SUPPORT_HPP_
