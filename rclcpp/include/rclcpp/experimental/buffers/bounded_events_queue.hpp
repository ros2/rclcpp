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

#ifndef RCLCPP__EXPERIMENTAL__BUFFERS__BOUNDED_EVENTS_QUEUE_HPP_
#define RCLCPP__EXPERIMENTAL__BUFFERS__BOUNDED_EVENTS_QUEUE_HPP_

#include <queue>
#include <utility>

#include "rclcpp/executors/events_queue.hpp"

namespace rclcpp
{
namespace experimental
{
namespace buffers
{

/**
 * @brief This class provides a bounded queue implementation
 * based on a std::queue. Before pushing events into the queue
 * checks the queue size. In case of exceeding the size it performs
 * a prune of the queue.
 */
class BoundedEventsQueue : public EventsQueue
{
public:
  RCLCPP_PUBLIC
  explicit BoundedEventsQueue(size_t queue_size_limit)
  {
    queue_size_limit_ = queue_size_limit;
  }

  RCLCPP_PUBLIC
  ~BoundedEventsQueue() = default;

  /**
   * @brief push event into the queue
   * @param event The event to push into the queue
   */
  RCLCPP_PUBLIC
  virtual
  void
  push(const rmw_listener_event_t & event)
  {
    if (event_queue_.size() >= queue_size_limit_) {
      // Simple prune strategy: Remove all elements in the queue.
      this->init();
    }
    event_queue_.push(event);
  }

  /**
   * @brief removes front element from the queue
   * The element removed is the "oldest" element in the queue whose
   * value can be retrieved by calling member front().
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
  front() const
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
  empty() const
  {
    return event_queue_.empty();
  }

  /**
   * @brief Initializes the queue
   */
  RCLCPP_PUBLIC
  virtual
  void
  init()
  {
    // Make sure the queue is empty when we start
    std::queue<rmw_listener_event_t> local_queue;
    std::swap(event_queue_, local_queue);
  }


  /**
   * @brief gets a queue with all events accumulated on it since
   * the last call. The member queue is empty when the call returns.
   * @return std::queue with events
   */
  RCLCPP_PUBLIC
  virtual
  std::queue<rmw_listener_event_t>
  get_all_events()
  {
    std::queue<rmw_listener_event_t> local_queue;
    std::swap(event_queue_, local_queue);
    return local_queue;
  }

private:
  std::queue<rmw_listener_event_t> event_queue_;
  size_t queue_size_limit_;
};

}  // namespace buffers
}  // namespace experimental
}  // namespace rclcpp


#endif  // RCLCPP__EXPERIMENTAL__BUFFERS__BOUNDED_EVENTS_QUEUE_HPP_
