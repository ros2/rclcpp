// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXECUTORS__TIMERS_MANAGER_HPP_
#define RCLCPP__EXECUTORS__TIMERS_MANAGER_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <list>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "rclcpp/context.hpp"
#include "rclcpp/timer.hpp"

namespace rclcpp
{
namespace executors
{

/**
 * @brief This class provides a way for storing and executing timer objects.
 * It APIs to suit the needs of different applications and execution models.
 * All public APIs provided by this class are thread-safe.
 *
 * Timers management
 * This class provides APIs to add and remove timers.
 * This class keeps ownership of the added timers through a shared pointer.
 * Timers are kept ordered in a binary-heap priority queue.
 * Calls to add/remove APIs will temporarily block the execution of the timers and
 * will require to reorder the internal priority queue of timers.
 * Because of this, they have a not-negligible impact on the performance.
 *
 * Timers execution
 * The most efficient implementation consists in letting a TimersManager object
 * to spawn a thread where timers are monitored and periodically executed.
 * Besides this, other APIs allow to either execute a single timer or all the
 * currently ready ones.
 * This class assumes that the execute_callback API of the stored timer is never
 * called by other entities, but can only be called from here.
 * If this assumption is not respected, the heap property will be invalidated,
 * so timers may be executed out of order.
 *
 */
class TimersManager
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(TimersManager)

  /**
   * @brief Construct a new TimersManager object
   */
  explicit TimersManager(std::shared_ptr<rclcpp::Context> context);

  /**
   * @brief Destruct the object making sure to stop thread and release memory.
   */
  ~TimersManager();

  /**
   * @brief Adds a new TimerBase to the storage.
   * This object will keep ownership of the timer.
   * @param timer the timer to be added
   */
  void add_timer(rclcpp::TimerBase::SharedPtr timer);

  /**
   * @brief Starts a thread that takes care of executing timers added to this object.
   */
  void start();

  /**
   * @brief Stops the timers thread.
   */
  void stop();

  /**
   * @brief Executes all the timers currently ready when the function is invoked
   * while keeping the heap correctly sorted.
   */
  void execute_ready_timers();

  /**
   * @brief Executes one ready timer if available
   *
   * @return true if there was a timer ready
   */
  bool execute_head_timer();

  /**
   * @brief Get the amount of time before the next timer expires.
   *
   * @return std::chrono::nanoseconds to wait,
   * the returned value could be negative if the timer is already expired
   * or MAX_TIME if the heap is empty.
   */
  std::chrono::nanoseconds get_head_timeout();

  /**
   * @brief Remove all the timers stored in the object.
   */
  void clear_all();

  /**
   * @brief Remove a single timer stored in the object.
   * @param timer the timer to remove.
   */
  void remove_timer(rclcpp::TimerBase::SharedPtr timer);

  // This is what the TimersManager uses to denote a duration forever.
  // We don't use std::chrono::nanoseconds::max because it will overflow.
  // See https://en.cppreference.com/w/cpp/thread/condition_variable/wait_for
  static constexpr std::chrono::nanoseconds MAX_TIME = std::chrono::hours(90);

private:
  RCLCPP_DISABLE_COPY(TimersManager)

  using TimerPtr = rclcpp::TimerBase::SharedPtr *;

  /**
   * @brief Implements a loop that keeps executing ready timers.
   * This function is executed in the timers thread.
   */
  void run_timers();

  /**
   * @brief Get the amount of time before the next timer expires.
   * This function is not thread safe, acquire a mutex before calling it.
   *
   * @return std::chrono::nanoseconds to wait,
   * the returned value could be negative if the timer is already expired
   * or MAX_TIME if the heap is empty.
   */
  std::chrono::nanoseconds get_head_timeout_unsafe()
  {
    if (heap_.empty()) {
      return MAX_TIME;
    }
    return (*heap_[0])->time_until_trigger();
  }

  /**
   * @brief Executes all the timers currently ready when the function is invoked
   * while keeping the heap correctly sorted.
   * This function is not thread safe, acquire a mutex before calling it.
   */
  void execute_ready_timers_unsafe();

  /**
   * @brief Helper function that checks whether a timer was already ready
   * at a specific timepoint
   * @param timer a pointer to the timer to check for
   * @param tp the timepoint to check for
   * @return true if timer was ready at tp
   */
  bool timer_was_ready_at_tp(
    TimerPtr timer,
    std::chrono::time_point<std::chrono::steady_clock> tp)
  {
    // A ready timer will return a negative duration when calling time_until_trigger
    auto time_ready = std::chrono::steady_clock::now() + (*timer)->time_until_trigger();
    return time_ready < tp;
  }

  /**
   * @brief Add a new timer to the heap and sort it.
   * @param x pointer to a timer owned by this object.
   */
  void add_timer_to_heap(TimerPtr x)
  {
    size_t i = heap_.size();  // Position where we are going to add timer
    heap_.push_back(x);

    size_t parent = (i - 1) / 2;
    while (i > 0 && ((*x)->time_until_trigger() < (*heap_[parent])->time_until_trigger())) {
      heap_[i] = heap_[parent];
      heap_[parent] = x;
      i = parent;
      parent = (i - 1) / 2;
    }
  }

  /**
   * @brief Restore a valid heap after the root value is replaced.
   * This function assumes that the value is increased, because if it was decreased
   * then there is nothing to do.
   */
  void restore_heap_root()
  {
    constexpr size_t start = 0;
    TimerPtr updated_timer = heap_[0];

    // Initialize to root and first left child
    size_t i = 0;
    size_t left_child = 1;

    // Swim up
    while (left_child < heap_.size()) {
      size_t right_child = left_child + 1;
      if (right_child < heap_.size() &&
        (*heap_[left_child])->time_until_trigger() >= (*heap_[right_child])->time_until_trigger())
      {
        left_child = right_child;
      }
      heap_[i] = heap_[left_child];
      i = left_child;
      left_child = 2 * i + 1;
    }

    // Swim down
    while (i > start) {
      size_t parent = (i - 1) / 2;
      if ((*updated_timer)->time_until_trigger() < (*heap_[parent])->time_until_trigger()) {
        heap_[i] = heap_[parent];
        i = parent;
        continue;
      }
      break;
    }

    heap_[i] = updated_timer;
  }

  // Thread used to run the timers monitoring and execution task
  std::thread timers_thread_;
  // Protects access to timers
  std::mutex timers_mutex_;
  // Notifies the timers thread whenever timers are added/removed
  std::condition_variable timers_cv_;
  // Flag used as predicate by timers_cv
  bool timers_updated_ {false};
  // Indicates whether the timers thread is currently running or requested to stop
  bool running_ {false};
  // Context of the parent executor
  std::shared_ptr<rclcpp::Context> context_;
  // Container to keep ownership of the timers
  std::list<rclcpp::TimerBase::SharedPtr> timers_storage_;
  // Vector of pointers to stored timers used to implement the priority queue
  std::vector<TimerPtr> heap_;
};

}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__TIMERS_MANAGER_HPP_
