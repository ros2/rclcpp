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

#include "rclcpp/generic_publisher.hpp"

#include <memory>
#include <string>

namespace rclcpp
{

void GenericPublisher::publish(const rclcpp::SerializedMessage & message)
{
  TRACETOOLS_TRACEPOINT(
    rclcpp_publish,
    nullptr,
    static_cast<const void *>(&message.get_rcl_serialized_message()));
  auto return_code = rcl_publish_serialized_message(
    get_publisher_handle().get(), &message.get_rcl_serialized_message(), NULL);

  if (return_code != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(return_code, "failed to publish serialized message");
  }
}

void GenericPublisher::publish_as_loaned_msg(const rclcpp::SerializedMessage & message)
{
  auto loaned_message = borrow_loaned_message();
  deserialize_message(message.get_rcl_serialized_message(), loaned_message);
  publish_loaned_message(loaned_message);
}

void * GenericPublisher::borrow_loaned_message()
{
  void * loaned_message = nullptr;
  auto return_code = rcl_borrow_loaned_message(
    get_publisher_handle().get(), &type_support_, &loaned_message);

  if (return_code != RMW_RET_OK) {
    if (return_code == RCL_RET_UNSUPPORTED) {
      rclcpp::exceptions::throw_from_rcl_error(
        return_code,
        "current middleware cannot support loan messages");
    } else {
      rclcpp::exceptions::throw_from_rcl_error(return_code, "failed to borrow loaned msg");
    }
  }
  return loaned_message;
}

void GenericPublisher::deserialize_message(
  const rmw_serialized_message_t & serialized_message,
  void * deserialized_msg)
{
  auto return_code = rmw_deserialize(&serialized_message, &type_support_, deserialized_msg);
  if (return_code != RMW_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(return_code, "failed to deserialize msg");
  }
}

void GenericPublisher::publish_loaned_message(void * loaned_message)
{
  TRACETOOLS_TRACEPOINT(rclcpp_publish, nullptr, static_cast<const void *>(loaned_message));
  auto return_code = rcl_publish_loaned_message(
    get_publisher_handle().get(), loaned_message, NULL);

  if (return_code != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(return_code, "failed to publish loaned message");
  }
}

}  // namespace rclcpp
