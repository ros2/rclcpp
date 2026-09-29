// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXPERIMENTAL__BUFFERS__INTRA_PROCESS_BUFFER_DATA_HPP_
#define RCLCPP__EXPERIMENTAL__BUFFERS__INTRA_PROCESS_BUFFER_DATA_HPP_

#include <memory>
#include <utility>
#include <variant>

#include "rmw/types.h"

namespace rclcpp
{
namespace experimental
{
namespace buffers
{

template<
  typename MessageT,
  typename MessageDeleter = std::default_delete<MessageT>>
struct IntraProcessBufferData
{
  using MessageUniquePtr = std::unique_ptr<MessageT, MessageDeleter>;
  using MessageSharedPtr = std::shared_ptr<const MessageT>;
  using MessageVariant = std::variant<MessageUniquePtr, MessageSharedPtr>;

  MessageVariant message;
  rmw_message_info_t message_info;

  IntraProcessBufferData() = default;
  IntraProcessBufferData(IntraProcessBufferData &&) = default;
  IntraProcessBufferData & operator=(IntraProcessBufferData &&) = default;

  IntraProcessBufferData(MessageVariant message_in, rmw_message_info_t message_info_in)
  : message(std::move(message_in)), message_info(message_info_in)
  {}

  /// Deep-copies the unique_ptr alternative.
  /**
   * std::variant's implicit copy constructor is deleted whenever one of its
   * alternatives (here, MessageUniquePtr) is not copy-constructible, so this must be
   * done explicitly to keep IntraProcessBufferData itself copyable. A null
   * unique_ptr copies to another null unique_ptr; otherwise a fresh copy of the
   * pointee is allocated, since the original must remain owned by the buffer (e.g.
   * for future transient-local replays to other subscriptions).
   */
  IntraProcessBufferData(const IntraProcessBufferData & other)
  : message_info(other.message_info)
  {
    if (std::holds_alternative<MessageSharedPtr>(other.message)) {
      message = std::get<MessageSharedPtr>(other.message);
    } else {
      const auto & other_unique_msg = std::get<MessageUniquePtr>(other.message);
      if (other_unique_msg) {
        message = MessageUniquePtr(
          new MessageT(*other_unique_msg), other_unique_msg.get_deleter());
      } else {
        message = MessageUniquePtr(nullptr, other_unique_msg.get_deleter());
      }
    }
  }

  IntraProcessBufferData & operator=(const IntraProcessBufferData & other)
  {
    if (this != &other) {
      *this = IntraProcessBufferData(other);
    }
    return *this;
  }
};

}  // namespace buffers
}  // namespace experimental
}  // namespace rclcpp


#endif  // RCLCPP__EXPERIMENTAL__BUFFERS__INTRA_PROCESS_BUFFER_DATA_HPP_
