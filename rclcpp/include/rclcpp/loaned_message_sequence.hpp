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

#ifndef RCLCPP__LOANED_MESSAGE_SEQUENCE_HPP_
#define RCLCPP__LOANED_MESSAGE_SEQUENCE_HPP_

#include <memory>
#include <utility>

#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/subscription_base.hpp"

#include "rmw/rmw.h"

namespace rclcpp
{

template<typename MessageT, typename AllocatorT = std::allocator<void>>
class LoanedMessageSequence
{
  using MessageAllocatorTraits = allocator::AllocRebind<MessageT, AllocatorT>;
  using MessageAllocator = typename MessageAllocatorTraits::allocator_type;

  LoanedMessageSequence(const LoanedMessageSequence & other) = delete;

public:
  /// Constructor for LoanedMessageSequence class
  /**
   * \param[in] sub Pointer to rclcpp::SubscriptionBase where the samples are associated to.
   */
  LoanedMessageSequence(const rclcpp::SubscriptionBase * sub)
  : sub_(sub),
    loaned_message_sequence_handle_(rmw_get_zero_initialized_loaned_message_sequence())
  {
    if (!sub_) {
      throw std::runtime_error("subscription pointer is null");
    }

    if (!sub_->can_loan_messages()) {
      // TODO(karsten1987): Have an alternative implementation on place in this class?
      RCL_WARN(rclcpp::get_logger("LoanedMessageSequence"), "Middleware can't loan messages");
    }
  }

  /// Move constructor of the LoanedMessageSequence class
  /**
   * \param[in] other LoanedMessageSequence to move.
   */
  LoanedMessageSequence(LoanedMessageSequence && other)
  : sub_(other.sub_),
    loaned_message_sequence_handle_(other.loaned_message_sequence_handle_)
  {}

  /// Destructor of the LoanedMessageSequence class
  virtual ~LoanedMessageSequence()
  {
    auto error_logger = rclcpp::get_logger("LoanedMessageSequence");
    if (!sub_) {
      RCLCPP_ERROR(
        error_logger, "Can't return loaned message sequence. Subscription instance is NULL");
      return;
    }

    auto ret = rmw_return_loaned_message_sequence(
      rcl_subscription_get_rmw_handle(
        sub_->get_subscription_handle().get()), &loaned_message_sequence_handle_);
    if (RCL_RET_OK != ret) {
      RCLCPP_ERROR(
        error_logger, "Can't return loaned message sequence. %s", rcl_get_error_string().str);
      rcl_reset_error();
    }
  }

  /// Get the size of the message sequence
  /**
   * \return size of the available messages within the sequence.
   */
  size_t size() const
  {
    return loaned_message_sequence_handle_.size;
  }

  /// Get the capacity of the message sequence
  /**
   * \return size of potential messages the sequence can hold.
   */
  size_t capacity() const
  {
    return loaned_message_sequence_handle_.capacity;
  }

  /// Access a ROS message within the sequence.
  /**
   * \param[in] pos Index indicating the position of the desired message.
   * \return Reference to the index message if available
   * \throw out of range exception if sequence is empty or index out of bounds.
   */
  MessageT & at(size_t pos) const
  {
    if (loaned_message_sequence_handle_.size == 0u) {
      throw std::out_of_range("loaned message sequence is empty");
    }
    if (pos >= loaned_message_sequence_handle_.size) {
      std::string err_msg = std::to_string(pos)
        + " is exceeding size of " + std::to_string(loaned_message_sequence_handle_.size);
      throw std::out_of_range(err_msg);
    }

    auto msg = reinterpret_cast<MessageT *>(
      rmw_loaned_message_sequence_at(
        rcl_subscription_get_rmw_handle(sub_->get_subscription_handle().get()),
        &loaned_message_sequence_handle_,
        pos));
    return *msg;
  }

  /// Access the RMW handle to the message queue.
  /**
   * \note The returned pointer to the RMW handle must not be destroyed.
   * The destructor of this class is returning the handle to the middleware.
   * \return pointer to the RMW loaned message sequence
   */
  rmw_loaned_message_sequence_t * get_loaned_message_sequence_handle()
  {
    return &loaned_message_sequence_handle_;
  }

private:
  const rclcpp::SubscriptionBase * sub_;
  rmw_loaned_message_sequence_t loaned_message_sequence_handle_;
};

}  // namespace rclcpp

#endif  // RCLCPP__LOANED_MESSAGE_SEQUENCE_HPP_
