// Copyright 2023 Washington University in St. Louis.
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

#ifndef RCLCPP__EXPERIMENTAL__EXECUTORS__EVENTS_EXECUTOR__PRIORITY_EVENTS_QUEUE_HPP_
#define RCLCPP__EXPERIMENTAL__EXECUTORS__EVENTS_EXECUTOR__PRIORITY_EVENTS_QUEUE_HPP_

#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <type_traits>
#include <utility>
#include <vector>

#include "rclcpp/experimental/executors/events_executor/events_queue.hpp"

namespace rclcpp
{
namespace experimental
{
namespace executors
{

struct PriorityEvent
{
  int priority;
  rclcpp::experimental::executors::ExecutorEvent event;
};

struct ComparePriorities : public std::binary_function<PriorityEvent, PriorityEvent, bool>
{
  bool
  operator()(const PriorityEvent & __x, const PriorityEvent & __y) const
  {return __x.priority > __y.priority;}
};

/**
 * @brief This class implements an EventsQueue as a simple wrapper around a std::priority_queue.
 * It does not perform any checks about the size of queue, which can grow
 * unbounded without being pruned.
 */
class PriorityEventsQueue : public EventsQueue
{
public:
  RCLCPP_PUBLIC
  PriorityEventsQueue()
  {
    // Default callback to extract priority from event
    extract_priority_ = [](const ExecutorEvent & event) {
        (void)(event);
        return 0;
      };
  }

  RCLCPP_PUBLIC
  explicit PriorityEventsQueue(
    std::function<int(const ExecutorEvent &)> extract_priority)
  : extract_priority_(extract_priority) {
    static_assert(std::is_invocable_r_v<int, decltype(extract_priority), const ExecutorEvent &>,
      "extract_priority must be a callable with signature int(const ExecutorEvent &)");
  }

  RCLCPP_PUBLIC
  ~PriorityEventsQueue() override = default;

  /**
   * @brief enqueue event into the queue
   * Thread safe
   * @param event The event to enqueue into the queue
   */
  RCLCPP_PUBLIC
  void
  enqueue(const ExecutorEvent & event) override
  {
    int priority = extract_priority_(event);
    PriorityEvent single_event = {priority, event};
    single_event.event.num_events = 1;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      for (size_t ev = 0; ev < event.num_events; ev++) {
        event_queue_.push(single_event);
      }
    }
    events_queue_cv_.notify_one();
  }

  /**
   * @brief waits for an event until timeout, gets a single event
   * Thread safe
   * @return true if event, false if timeout
   */
  RCLCPP_PUBLIC
  bool
  dequeue(
    ExecutorEvent & event,
    std::chrono::nanoseconds timeout = std::chrono::nanoseconds::max()) override
  {
    std::unique_lock<std::mutex> lock(mutex_);

    // Initialize to true because it's only needed if we have a valid timeout
    bool has_data = true;
    if (timeout != std::chrono::nanoseconds::max()) {
      has_data =
        events_queue_cv_.wait_for(lock, timeout, [this]() {return !event_queue_.empty();});
    } else {
      events_queue_cv_.wait(lock, [this]() {return !event_queue_.empty();});
    }

    if (has_data) {
      event = event_queue_.top().event;
      event_queue_.pop();
      return true;
    }

    return false;
  }

  /**
   * @brief Test whether queue is empty
   * Thread safe
   * @return true if the queue's size is 0, false otherwise.
   */
  RCLCPP_PUBLIC
  bool
  empty() const override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return event_queue_.empty();
  }

  /**
   * @brief Returns the number of elements in the queue.
   * Thread safe
   * @return the number of elements in the queue.
   */
  RCLCPP_PUBLIC
  size_t
  size() const override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return event_queue_.size();
  }

private:
  // Callback to extract priority from event
  std::function<int(const ExecutorEvent &)> extract_priority_;
  // The underlying queue implementation
  std::priority_queue<PriorityEvent,
    std::vector<PriorityEvent>,
    ComparePriorities> event_queue_;
  // Mutex to protect read/write access to the queue
  mutable std::mutex mutex_;
  // Variable used to notify when an event is added to the queue
  std::condition_variable events_queue_cv_;
};

}  // namespace executors
}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__EXECUTORS__EVENTS_EXECUTOR__PRIORITY_EVENTS_QUEUE_HPP_
