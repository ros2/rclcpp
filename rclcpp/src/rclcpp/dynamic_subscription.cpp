// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/dynamic_subscription.hpp"

#include <memory>
#include <string>

#include "rcl/subscription.h"

#include "rclcpp/exceptions.hpp"

namespace rclcpp
{

std::shared_ptr<void> DynamicSubscription::create_message()
{
  return create_serialized_message();
}

std::shared_ptr<rclcpp::SerializedMessage> DynamicSubscription::create_serialized_message()
{
  return std::make_shared<rclcpp::SerializedMessage>(0);
}

void DynamicSubscription::handle_message(std::shared_ptr<void> &, const rclcpp::MessageInfo &)
{
  throw rclcpp::exceptions::UnimplementedError(
          "handle_message is not implemented for DynamicSubscription");
}

void DynamicSubscription::handle_serialized_message(
  const std::shared_ptr<rclcpp::SerializedMessage> &, const rclcpp::MessageInfo &)
{
  throw rclcpp::exceptions::UnimplementedError(
          "handle_serialized_message is not implemented for DynamicSubscription");
}

void DynamicSubscription::handle_loaned_message(void *, const rclcpp::MessageInfo &)
{
  throw rclcpp::exceptions::UnimplementedError(
          "handle_loaned_message is not implemented for DynamicSubscription");
}

void DynamicSubscription::return_message(std::shared_ptr<void> & message)
{
  auto typed_message = std::static_pointer_cast<rclcpp::SerializedMessage>(message);
  return_serialized_message(typed_message);
}

void DynamicSubscription::return_serialized_message(
  std::shared_ptr<rclcpp::SerializedMessage> & message)
{
  message.reset();
}


// DYNAMIC TYPE ====================================================================================
// TODO(methylDragon): Re-order later

// Does not clone
rclcpp::dynamic_typesupport::DynamicMessageType::SharedPtr
DynamicSubscription::get_shared_dynamic_message_type()
{
  return dynamic_message_type_;
};

// Does not clone
rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr
DynamicSubscription::get_shared_dynamic_message()
{
  return dynamic_message_;
};

rclcpp::dynamic_typesupport::DynamicSerializationSupport::SharedPtr
DynamicSubscription::get_shared_dynamic_serialization_support()
{
  return serialization_support_;
};

rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr
DynamicSubscription::create_dynamic_message()
{
  return dynamic_message_->init_from_type_shared(*dynamic_message_type_);
};

void
DynamicSubscription::return_dynamic_message(
  rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr & message)
{
  message.reset();
};

void DynamicSubscription::handle_dynamic_message(
  const rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr & message,
  const rclcpp::MessageInfo &)
{
  callback_(message);
}

}  // namespace rclcpp
