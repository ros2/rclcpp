// Copyright 2018, Bosch Software Innovations GmbH.
// Copyright 2021, Apex.AI Inc.
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

#include "rclcpp/generic_subscription.hpp"

#include <memory>
#include <string>

#include "rcl/subscription.h"

#include "rclcpp/exceptions.hpp"

namespace rclcpp
{

std::shared_ptr<void> GenericSubscription::create_message()
{
  return create_serialized_message();
}

std::shared_ptr<rclcpp::SerializedMessage> GenericSubscription::create_serialized_message()
{
  return std::make_shared<rclcpp::SerializedMessage>(0);
}

void GenericSubscription::handle_message(
  std::shared_ptr<void> & message, const rclcpp::MessageInfo & message_info)
{
  (void) message_info;
  auto typed_message = std::static_pointer_cast<rclcpp::SerializedMessage>(message);
  callback_(typed_message);
}

void GenericSubscription::handle_loaned_message(
  void * message, const rclcpp::MessageInfo & message_info)
{
  (void) message;
  (void) message_info;
  throw rclcpp::exceptions::UnimplementedError(
          "handle_loaned_message is not implemented for GenericSubscription");
}

void GenericSubscription::return_message(std::shared_ptr<void> & message)
{
  auto typed_message = std::static_pointer_cast<rclcpp::SerializedMessage>(message);
  return_serialized_message(typed_message);
}

void GenericSubscription::return_serialized_message(
  std::shared_ptr<rclcpp::SerializedMessage> & message)
{
  message.reset();
}

}  // namespace rclcpp
