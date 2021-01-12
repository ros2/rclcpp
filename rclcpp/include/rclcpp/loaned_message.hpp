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
#include <utility>

#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/publisher_base.hpp"

#include "rcl/allocator.h"
#include "rcl/publisher.h"

namespace rclcpp
{

template<typename MessageT, typename AllocatorT = std::allocator<void>>
class LoanedMessage
{
  using MessageAllocatorTraits = rclcpp::allocator::AllocRebind<MessageT, AllocatorT>;
  using MessageAllocator = typename MessageAllocatorTraits::allocator_type;

public:
  /// Constructor of the LoanedMessage class.
  /**
   * The constructor of this class allocates memory for a given message type
   * and associates this with a given publisher.
   *
   * Given the publisher instance, a case differentiation is being performaned
   * which decides whether the underlying middleware is able to allocate the appropriate
   * memory for this message type or not.
   * In the case that the middleware can not loan messages, the passed in allocator instance
   * is being used to allocate the message within the scope of this class.
   * Otherwise, the allocator is being ignored and the allocation is solely performaned
   * in the underlying middleware with its appropriate allocation strategy.
   * The need for this arises as the user code can be written explicitly targeting a middleware
   * capable of loaning messages.
   * However, this user code is ought to be usable even when dynamically linked against
   * a middleware which doesn't support message loaning in which case the allocator will be used.
   *
   * \param[in] pub rclcpp::Publisher instance to which the memory belongs
   * \param[in] allocator Allocator instance in case middleware can not allocate messages
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  LoanedMessage(
    const rclcpp::PublisherBase & pub,
    std::allocator<MessageT> allocator)
  : pub_(pub),
    message_(nullptr),
    message_allocator_(std::move(allocator))
  {
    if (pub_.can_loan_messages()) {
      void * message_ptr = nullptr;
      auto ret = rcl_borrow_loaned_message(
        pub_.get_publisher_handle().get(),
        rosidl_typesupport_cpp::get_message_type_support_handle<MessageT>(),
        &message_ptr);
      if (RCL_RET_OK != ret) {
        rclcpp::exceptions::throw_from_rcl_error(ret);
      }
      message_ = static_cast<MessageT *>(message_ptr);
    } else {
      RCLCPP_INFO_ONCE(
        rclcpp::get_logger("rclcpp"),
        "Currently used middleware can't loan messages. Local allocator will be used.");
      message_ = message_allocator_.allocate(1);
      new (message_) MessageT();
    }
  }

  /// Constructor of the LoanedMessage class.
  /**
   * The constructor of this class allocates memory for a given message type
   * and associates this with a given publisher.
   *
   * Given the publisher instance, a case differentiation is being performaned
   * which decides whether the underlying middleware is able to allocate the appropriate
   * memory for this message type or not.
   * In the case that the middleware can not loan messages, the passed in allocator instance
   * is being used to allocate the message within the scope of this class.
   * Otherwise, the allocator is being ignored and the allocation is solely performaned
   * in the underlying middleware with its appropriate allocation strategy.
   * The need for this arises as the user code can be written explicitly targeting a middleware
   * capable of loaning messages.
   * However, this user code is ought to be usable even when dynamically linked against
   * a middleware which doesn't support message loaning in which case the allocator will be used.
   *
   * \param[in] pub rclcpp::Publisher instance to which the memory belongs
   * \param[in] allocator Allocator instance in case middleware can not allocate messages
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  LoanedMessage(
    const rclcpp::PublisherBase * pub,
    std::shared_ptr<std::allocator<MessageT>> allocator)
  : LoanedMessage(*pub, *allocator)
  {}

  /// Move semantic for RVO
  LoanedMessage(LoanedMessage<MessageT> && other)
  : pub_(std::move(other.pub_)),
    message_(std::move(other.message_)),
    message_allocator_(std::move(other.message_allocator_))
  {}

  /// Destructor of the LoanedMessage class.
  /**
   * The destructor has the explicit task to return the allocated memory for its message
   * instance.
   * If the message was previously allocated via the middleware, the message is getting
   * returned to the middleware to cleanly destroy the allocation.
   * In the case that the local allocator instance was used, the same instance is then
   * being used to destroy the allocated memory.
   *
   * The contract here is that the memory for this message is valid as long as this instance
   * of the LoanedMessage class is alive.
   */
  virtual ~LoanedMessage()
  {
    auto error_logger = rclcpp::get_logger("LoanedMessage");

    if (message_ == nullptr) {
      return;
    }

    if (pub_.can_loan_messages()) {
      // return allocated memory to the middleware
      auto ret =
        rcl_return_loaned_message_from_publisher(pub_.get_publisher_handle().get(), message_);
      if (ret != RCL_RET_OK) {
        RCLCPP_ERROR(
          error_logger, "rcl_deallocate_loaned_message failed: %s", rcl_get_error_string().str);
        rcl_reset_error();
      }
    } else {
      // call destructor before deallocating
      message_->~MessageT();
      message_allocator_.deallocate(message_, 1);
    }
    message_ = nullptr;
  }

  /// Validate if the message was correctly allocated.
  /**
   * The allocated memory might not be always consistent and valid.
   * Reasons why this could fail is that an allocation step was failing,
   * e.g. just like malloc could fail or a maximum amount of previously allocated
   * messages is exceeded in which case the loaned messages have to be returned
   * to the middleware prior to be able to allocate a new one.
   */
  bool is_valid() const
  {
    return message_ != nullptr;
  }

  /// Access the ROS message instance.
  /**
   * A call to `get()` will return a mutable reference to the underlying ROS message instance.
   * This allows a user to modify the content of the message prior to publishing it.
   *
   * If this reference is copied, the memory for this copy is no longer managed
   * by the LoanedMessage instance and has to be cleanup individually.
   */
  MessageT & get() const
  {
    return *message_;
  }

  /// Release ownership of the ROS message instance.
  /**
   * A call to `release()` will unmanage the memory for the ROS message.
   * That means that the destructor of this class will not free the memory on scope exit.
   * If the message is loaned from the middleware but not be published, the user needs to call
   * `rcl_return_loaned_message_from_publisher` manually.
   * If the memory is from the local allocator, the memory is freed when the unique pointer
   * goes out instead.
   *
   * \return std::unique_ptr to the message instance.
   */
  std::unique_ptr<MessageT, std::function<void(MessageT *)>>
  release()
  {
    auto msg = message_;
    message_ = nullptr;

    if (pub_.can_loan_messages()) {
      return std::unique_ptr<MessageT, std::function<void(MessageT *)>>(msg, [](MessageT *) {});
    }

    return std::unique_ptr<MessageT, std::function<void(MessageT *)>>(
      msg,
      [allocator = message_allocator_](MessageT * msg_ptr) mutable {
        // call destructor before deallocating
        msg_ptr->~MessageT();
        allocator.deallocate(msg_ptr, 1);
      });
  }

protected:
  const rclcpp::PublisherBase & pub_;

  MessageT * message_;

  MessageAllocator message_allocator_;

  /// Deleted copy constructor to preserve memory integrity.
  LoanedMessage(const LoanedMessage<MessageT> & other) = delete;
};

}  // namespace rclcpp

#endif  // RCLCPP__LOANED_MESSAGE_HPP_
