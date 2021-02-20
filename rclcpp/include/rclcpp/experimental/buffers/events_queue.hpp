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
#include "rclcpp/visibility_control.hpp"

#include "rmw/listener_event_types.h"

namespace rclcpp
{
namespace experimental
{
namespace buffers
{

/**
 * @brief This abstract class can be used to implement different types of queues
 * where `rmw_listener_event_t` can be stored.
 * The derived classes should choose which underlying container to use and
 * the strategy for pushing and popping events.
 * For example a queue implementation may be bounded or unbounded and have
 * different pruning strategies.
 * Implementations may or may not check the validity of events and decide how to handle
 * the situation where an event is not valid anymore (e.g. a subscription history cache overruns)
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
  push(const rmw_listener_event_t & event) = 0;

  /**
   * @brief removes front element from the queue.
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
  front() const = 0;

  /**
   * @brief Test whether queue is empty
   * @return true if the queue's size is 0, false otherwise.
   */
  RCLCPP_PUBLIC
  virtual
  bool
  empty() const = 0;

  /**
   * @brief Returns the number of elements in the queue.
   * @return the number of elements in the queue.
   */
  RCLCPP_PUBLIC
  virtual
  size_t
  size() const = 0;

  /**
   * @brief Initializes the queue
   */
  RCLCPP_PUBLIC
  virtual
  void
  init() = 0;

  /**
   * @brief pops out all events stored in the object into an output queue.
   * @return queue with events
   */
  RCLCPP_PUBLIC
  virtual
  std::queue<rmw_listener_event_t>
  pop_all_events() = 0;
};

}  // namespace buffers
}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__BUFFERS__EVENTS_QUEUE_HPP_
