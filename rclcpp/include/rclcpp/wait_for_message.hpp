// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__WAIT_FOR_MESSAGE_HPP_
#define RCLCPP__WAIT_FOR_MESSAGE_HPP_

#include <memory>
#include <string>

#include "rclcpp/node.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/wait_set.hpp"

namespace rclcpp
{
/// Wait for the next incoming message.
/**
 * Given an already initialized subscription,
 * wait for the next incoming message to arrive before the specified timeout.
 *
 * \param[out] out is the message to be filled when a new message is arriving.
 * \param[in] subscription shared pointer to a previously initialized subscription.
 * \param[in] time_to_wait parameter specifying the timeout before returning.
 * \return true if a message was successfully received, false otherwise.
 */
template<class MsgT, class Rep = int64_t, class Period = std::milli>
bool wait_for_message(
  MsgT & out,
  std::shared_ptr<rclcpp::Subscription<MsgT>> subscription,
  std::chrono::duration<Rep, Period> time_to_wait = std::chrono::duration<Rep, Period>(-1))
{
  rclcpp::WaitSet wait_set;
  wait_set.add_subscription(subscription);
  auto ret = wait_set.wait(time_to_wait);
  if (ret.kind() != rclcpp::WaitResultKind::Ready) {
    return false;
  }

  rclcpp::MessageInfo info;
  if (!subscription->take(out, info)) {
    return false;
  }

  return true;
}

/// Wait for the next incoming message.
/**
 * Wait for the next incoming message to arrive on a specified topic before the specified timeout.
 *
 * \param[out] out is the message to be filled when a new message is arriving.
 * \param[in] node the node pointer to initialize the subscription on.
 * \param[in] topic the topic to wait for messages.
 * \param[in] time_to_wait parameter specifying the timeout before returning.
 * \return true if a message was successfully received, false otherwise.
 */
template<class MsgT, class Rep = int64_t, class Period = std::milli>
bool wait_for_message(
  MsgT & out,
  rclcpp::Node::SharedPtr node,
  const std::string & topic,
  std::chrono::duration<Rep, Period> time_to_wait = std::chrono::duration<Rep, Period>(-1))
{
  auto sub = node->create_subscription<MsgT>(topic, 1, [](const std::shared_ptr<MsgT>) {});
  return wait_for_message<MsgT, Rep, Period>(out, sub, time_to_wait);
}

}  // namespace rclcpp

#endif  // RCLCPP__WAIT_FOR_MESSAGE_HPP_
