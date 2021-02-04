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

#ifndef RCLCPP__EXPERIMENTAL__BUFFERS__PERFORMANCE_EVENTS_QUEUE_HPP_
#define RCLCPP__EXPERIMENTAL__BUFFERS__PERFORMANCE_EVENTS_QUEUE_HPP_

#include <queue>

#include "rclcpp/executors/events_queue.hpp"

namespace rclcpp
{
namespace experimental
{
namespace buffers
{

/**
 * @brief This class provides a simple queue implementation
 * based on a std::queue. As the objective is having a CPU peformant
 * queue, it does not performs any checks about the size of
 * the queue, so the queue size could grow unbounded.
 * It does not implement any pruning mechanisms.
 */
class PerformanceEventsQueue : public EventsQueue
{
public:

  using EventQueue = std::queue<rmw_listener_event_t>;

  RCLCPP_PUBLIC
  ~PerformanceEventsQueue() = default;

  /**
   * @brief push event into the queue
   * @param event The event to push into the queue
   */
  RCLCPP_PUBLIC
  virtual
  void
  push(rmw_listener_event_t event)
  {
    event_queue_.push(event);
  }

  /**
   * @brief removes front element from the queue
   * @return iterator
   */
  RCLCPP_PUBLIC
  virtual
  void
  pop()
  {
    event_queue_.pop();
  }

  /**
   * @brief gets the front event from the queue
   * @return the front event
   */
  RCLCPP_PUBLIC
  virtual
  rmw_listener_event_t
  front()
  {
     return event_queue_.front();
  }

  /**
   * @brief Test whether queue is empty
   * @return true if the queue's size is 0, false otherwise.
   */
  RCLCPP_PUBLIC
  virtual
  bool
  empty()
  {
    return event_queue_.empty();
  }

  /**
   * @brief gets a queue with all events on it
   * @return queue with events
   */
  RCLCPP_PUBLIC
  virtual
  EventQueue
  get_all_events()
  {
    EventQueue local_queue;
    std::swap(event_queue_, local_queue);
    return local_queue;
  }

private:
  EventQueue event_queue_;
};

}  // namespace buffers
}  // namespace experimental
}  // namespace rclcpp


#endif  // RCLCPP__EXPERIMENTAL__BUFFERS__PERFORMANCE_EVENTS_QUEUE_HPP_
