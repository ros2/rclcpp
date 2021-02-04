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

#ifndef RCLCPP__EXPERIMENTAL__BUFFERS__EVENTS_QUEUE_HPP_
#define RCLCPP__EXPERIMENTAL__BUFFERS__EVENTS_QUEUE_HPP_

#include <queue>

#include "rclcpp/macros.hpp"

#include "rmw/listener_event_types.h"

namespace rclcpp
{
namespace experimental
{
namespace buffers
{

/**
 * @brief This abstract class is intended to be used as
 * a wrapper around a queue. The derived classes should chose
 * which container to use and the strategies for push and prune
 * events from the queue.
 */
class EventsQueue
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(EventsQueue)

  /**
   * @brief Destruct the object.
   */
  RCLCPP_PUBLIC
  virtual ~EventsQueue() = default;

  /**
   * @brief push event into the queue
   * @param event The event to push into the queue
   */
  RCLCPP_PUBLIC
  virtual
  void
  push(rmw_listener_event_t event) = 0;

  /**
   * @brief removes front element from the queue
   * @return iterator
   */
  RCLCPP_PUBLIC
  virtual
  void
  pop() = 0;

  /**
   * @brief gets the front event from the queue
   * @return the front event
   */
  RCLCPP_PUBLIC
  virtual
  rmw_listener_event_t
  front() = 0;

  /**
   * @brief Test whether queue is empty
   * @return true if the queue's size is 0, false otherwise.
   */
  RCLCPP_PUBLIC
  virtual
  bool
  empty() = 0;

  /**
   * @brief gets a queue with all events accumulated on it
   * @return queue with events
   */
  RCLCPP_PUBLIC
  virtual
  std::queue<rmw_listener_event_t>
  get_all_events() = 0;
};

}  // namespace buffers
}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__BUFFERS__EVENTS_QUEUE_HPP_
