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
#include "rclcpp/subscription_base.hpp"

#include "rmw/rmw.h"

namespace rclcpp
{

class LoanedMessageSequence
{
  LoanedMessageSequence(const LoanedMessageSequence & other) = delete;

public:
  /// Destructor of the LoanedMessageSequence class
  virtual ~LoanedMessageSequence();

  /// Get the size of the message sequence
  /**
   * \return size of the available messages within the sequence.
   */
  size_t size() const;

  /// Get the capacity of the message sequence
  /**
   * \return size of potential messages the sequence can hold.
   */
  size_t capacity() const;

  /// Access the RMW handle to the message queue.
  /**
   * \note The returned pointer to the RMW handle must not be destroyed.
   * The destructor of this class is returning the handle to the middleware.
   * \return pointer to the RMW loaned message sequence
   */
  rmw_loaned_message_sequence_t * get_loaned_message_sequence_handle();

  /// Access a ROS message within the sequence.
  /**
   * \param[in] pos Index indicating the position of the desired message.
   * \return Reference to the index message if available
   * \throw out of range exception if sequence is empty or index out of bounds.
   */
  const void * at(size_t pos) const;

  /// Access a ROS message within the sequence.
  /**
   * \note The same functionality as \sa `at` however, the message pointer
   * is being casted to the right message type.
   *
   * \param[in] pos Index indicating the position of the desired message.
   * \return Reference to the index message if available
   * \throw out of range exception if sequence is empty or index out of bounds.
   */
  template<typename MessageT>
  std::shared_ptr<MessageT> at(size_t pos) const
  {
    return std::static_pointer_cast<MessageT>(at(pos));
  }

public:
  /// Constructor for LoanedMessageSequenceBase class
  /**
   * \param[in] sub Pointer to rclcpp::SubscriptionBase where the samples are associated to.
   */
  LoanedMessageSequence(const rclcpp::SubscriptionBase * sub);

  /// Move constructor of the LoanedMessageSequenceBase class
  /**
   * \param[in] other LoanedMessageSequenceBase to move.
   */
  LoanedMessageSequence(LoanedMessageSequence && other);

protected:
  inline void throw_on_out_of_range(size_t pos) const
  {
    if (loaned_message_sequence_handle_.size == 0u) {
      throw std::out_of_range("loaned message sequence is empty");
    }
    if (pos >= loaned_message_sequence_handle_.size) {
      std::string err_msg = std::to_string(pos)
        + " is exceeding size of " + std::to_string(loaned_message_sequence_handle_.size);
      throw std::out_of_range(err_msg);
    }
  }

  const rclcpp::SubscriptionBase * sub_;
  rmw_loaned_message_sequence_t loaned_message_sequence_handle_;
};
}  // namespace rclcpp

#endif  // RCLCPP__LOANED_MESSAGE_SEQUENCE_HPP_
