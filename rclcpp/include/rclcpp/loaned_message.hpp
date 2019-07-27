// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__LOANED_MESSAGE_HPP_
#define RCLCPP__LOANED_MESSAGE_HPP_

#include <memory>

#include "rclcpp/publisher.hpp"

#include "rcl/allocator.h"
#include "rcl/publisher.h"

namespace rclcpp
{

template<class MessageT, class AllocatorT = rcl_allocator_t>
class LoanedMessage
{
protected:
  const rclcpp::Publisher<MessageT> * pub_;

  void * message_memory_;
  MessageT * message_;

  AllocatorT allocator_;

  LoanedMessage(
    const rclcpp::Publisher<MessageT> * pub,
    AllocatorT allocator = rcl_get_default_allocator())
  : pub_(pub),
    message_memory_(nullptr),
    message_(nullptr),
    allocator_(allocator)
  {
    if (!pub_) {
      throw std::runtime_error("publisher pointer is null");
    }
    if (pub_->can_loan_messages()) {
      message_memory_ =
        rcl_allocate_loaned_message(pub_->get_publisher_handle(), nullptr, sizeof(MessageT));
    } else {
      message_memory_ = allocator_.allocate(sizeof(MessageT), allocator_.state);
    }
    if (!message_memory_) {
      throw std::runtime_error("unable to allocate memory for loaned message");
    }
    message_ = new (message_memory_) MessageT();
  }

public:
  LoanedMessage(const LoanedMessage<MessageT> & other) = delete;

  virtual ~LoanedMessage()
  {
    if (!pub_) {
      fprintf(stderr, "Can't destroy LoanedMessage. Publisher instance is null.");
      return;
    }
    if (pub_->can_loan_messages()) {
      auto ret =
        rcl_deallocate_loaned_message(pub_->get_publisher_handle(), message_memory_);
      if (ret != RCL_RET_OK) {
        fprintf(stderr, "Can't deallocate loaned message");
        return;
      }
    } else {
      allocator_.deallocate(message_memory_, allocator_.state);
    }
    message_memory_ = nullptr;
  }

  bool is_valid() const
  {
    return message_memory_ != nullptr;
  }

  MessageT & get() const
  {
    return *message_;
  }

  static
  std::unique_ptr<LoanedMessage<MessageT>>
  get_instance(const rclcpp::Publisher<MessageT> * pub)
  {
    return std::unique_ptr<rclcpp::LoanedMessage<MessageT>>(new LoanedMessage(pub));
  }
};

}  // namespace rclcpp

#endif  // RCLCPP__LOANED_MESSAGE_HPP_
