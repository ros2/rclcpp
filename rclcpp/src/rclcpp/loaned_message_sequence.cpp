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

#include "rclcpp/loaned_message_sequence.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/subscription_base.hpp"

namespace rclcpp
{
LoanedMessageSequence::LoanedMessageSequence(const rclcpp::SubscriptionBase * sub)
: sub_(sub),
  loaned_message_sequence_handle_(rmw_get_zero_initialized_loaned_message_sequence())
{
  if (!sub_) {
    throw std::runtime_error("subscription pointer is null");
  }

  if (!sub_->can_loan_messages()) {
    // TODO(karsten1987): Have an alternative implementation on place in this class?
    RCLCPP_WARN(rclcpp::get_logger("LoanedMessageSequence"), "Middleware can't loan messages");
  }
}

LoanedMessageSequence::LoanedMessageSequence(LoanedMessageSequence && other)
: sub_(other.sub_),
  loaned_message_sequence_handle_(other.loaned_message_sequence_handle_)
{}

LoanedMessageSequence::~LoanedMessageSequence()
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

size_t LoanedMessageSequence::size() const
{
  return loaned_message_sequence_handle_.size;
}

size_t LoanedMessageSequence::capacity() const
{
  return loaned_message_sequence_handle_.capacity;
}

rmw_loaned_message_sequence_t * LoanedMessageSequence::get_loaned_message_sequence_handle()
{
  return &loaned_message_sequence_handle_;
}

const void * LoanedMessageSequence::at(size_t pos) const
{
  throw_on_out_of_range(pos);

  return rmw_loaned_message_sequence_at(
      rcl_subscription_get_rmw_handle(sub_->get_subscription_handle().get()),
      &loaned_message_sequence_handle_,
      pos);
}
}  // namespace rclcpp
