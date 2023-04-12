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

#include <rcl/allocator.h>
#include <rcl/types.h>
#include <rosidl_dynamic_typesupport/types.h>

#include <memory>
#include <string>

#include "rclcpp/dynamic_typesupport/dynamic_message_type.hpp"
#include "rclcpp/dynamic_typesupport/dynamic_serialization_support.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace dynamic_typesupport
{

/// Utility wrapper class for rosidl_dynamic_typesupport_dynamic_data_t
/// STUBBED OUT
class DynamicMessage : public std::enable_shared_from_this<DynamicMessage>
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(DynamicMessage)

  RCLCPP_PUBLIC
  virtual ~DynamicMessage();

protected:
  // NOTE(methylDragon):
  // This is just here to extend the lifetime of the serialization support
  // It isn't actually used by the builder since the builder should compose its own support
  //
  // ... Though ideally it should be the exact same support as the one stored in the
  // DynamicSerializationSupport
  DynamicSerializationSupport::SharedPtr serialization_support_;

  rosidl_dynamic_typesupport_dynamic_data_t rosidl_dynamic_data_;
  bool is_loaned_;

  // Used for returning the loaned value, and lifetime management
  DynamicMessage::SharedPtr parent_data_;

private:
  RCLCPP_DISABLE_COPY(DynamicMessage)

  RCLCPP_PUBLIC
  DynamicMessage();
};

}  // namespace dynamic_typesupport
}  // namespace rclcpp

#endif  // RCLCPP__DYNAMIC_TYPESUPPORT__DYNAMIC_MESSAGE_HPP_
