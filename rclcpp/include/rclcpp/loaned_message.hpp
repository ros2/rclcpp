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

template<typename MessageT, typename Alloc>
class Publisher;

template<typename MessageT, typename Alloc = std::allocator<void>>
class LoanedMessage
{
  using MessageAllocTraits = allocator::AllocRebind<MessageT, Alloc>;
  using MessageAlloc = typename MessageAllocTraits::allocator_type;

protected:
  const rclcpp::Publisher<MessageT, Alloc> * pub_;

  std::unique_ptr<MessageT> message_;

  const std::shared_ptr<MessageAlloc> message_allocator_;

  LoanedMessage(
    const rclcpp::Publisher<MessageT, Alloc> * pub,
    const std::shared_ptr<std::allocator<MessageT>> & allocator)
  : pub_(pub),
    message_(nullptr),
    message_allocator_(allocator)
  {
    void * message_memory = nullptr;
    if (pub_->can_loan_messages()) {
      message_memory =
        rcl_allocate_loaned_message(pub_->get_publisher_handle(), nullptr, sizeof(MessageT));
    } else {
      message_memory = message_allocator_->allocate(1);
    }
    if (!message_memory) {
      throw std::runtime_error("unable to allocate memory for loaned message");
    }
    message_.reset(new (message_memory) MessageT());
  }

  LoanedMessage(const LoanedMessage<MessageT> & other) = delete;

public:
  LoanedMessage(LoanedMessage<MessageT> && other)
  : pub_(std::move(other.pub_)),
    message_(std::move(other.message_)),
    message_allocator_(std::move(other.message_allocator_))
  {}

  virtual ~LoanedMessage()
  {
    if (!pub_) {
      fprintf(stderr, "Can't destroy LoanedMessage. Publisher instance is null.");
      return;
    }

    // release allocated memory from unique_ptr
    MessageT * message_memory = message_.release();

    if (pub_->can_loan_messages()) {
      // return allocated memory to the middleware
      auto ret =
        rcl_deallocate_loaned_message(pub_->get_publisher_handle(), message_memory);
      if (ret != RCL_RET_OK) {
        fprintf(stderr, "Can't deallocate loaned message");
        return;
      }
    } else {
      message_allocator_->deallocate(message_memory, 1);
    }
    message_memory = nullptr;
    message_ = nullptr;
  }

  bool is_valid() const
  {
    return message_ != nullptr;
  }

  MessageT & get() const
  {
    return *message_;
  }

  static
  LoanedMessage<MessageT>
  get_instance(const rclcpp::Publisher<MessageT, Alloc> * pub)
  {
    if (!pub) {
      throw std::runtime_error("publisher pointer is null");
    }
    return rclcpp::LoanedMessage<MessageT, Alloc>(pub, pub->get_allocator());
  }
};

}  // namespace rclcpp

#endif  // RCLCPP__LOANED_MESSAGE_HPP_
