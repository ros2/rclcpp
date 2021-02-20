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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
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
 * It keeps a list of weak pointers from added timers, and locks them only when
 * they need to be executed or modified.
 * Timers are kept ordered in a binary-heap priority queue.
 * Calls to add/remove APIs will temporarily block the execution of the timers and
 * will require to reorder the internal priority queue.
 * Because of this, they have a not-negligible impact on the performance.
 *
 * Timers execution
 * The most efficient implementation consists in letting a TimersManager object
 * to spawn a thread where timers are monitored and periodically executed.
 * Besides this, other APIs allow to either execute a single timer or all the
 * currently ready ones.
 * This class assumes that the `execute_callback()` API of the stored timers is never
 * called by other entities, but it can only be called from here.
 * If this assumption is not respected, the heap property may be invalidated,
 * so timers may be executed out of order, without this object noticing it.
 *
 */
class TimersManager
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(TimersManager)

  /**
   * @brief Construct a new TimersManager object
   * 
   * @param context custom context to be used.
   * Shared ownership of the context is held until destruction.
   */
  explicit TimersManager(std::shared_ptr<rclcpp::Context> context);

  /**
   * @brief Destruct the TimersManager object making sure to stop thread and release memory.
   */
  ~TimersManager();

  /**
   * @brief Adds a new timer to the storage, maintaining weak ownership of it.
   * Function is thread safe and it can be called regardless of the state of the timers thread.
   * 
   * @param timer the timer to add.
   */
  void add_timer(rclcpp::TimerBase::SharedPtr timer);

  /**
   * @brief Remove a single timer from the object storage.
   * Will do nothing if the timer was not being stored here.
   * Function is thread safe and it can be called regardless of the state of the timers thread.
   * 
   * @param timer the timer to remove.
   */
  void remove_timer(rclcpp::TimerBase::SharedPtr timer);

  /**
   * @brief Remove all the timers stored in the object.
   * Function is thread safe and it can be called regardless of the state of the timers thread.
   */
  void clear();

  /**
   * @brief Starts a thread that takes care of executing the timers stored in this object.
   * Function will throw an error if the timers thread was already running.
   */
  void start();

  /**
   * @brief Stops the timers thread.
   * Will do nothing if the timer thread was not running.
   */
  void stop();

  /**
   * @brief Executes all the timers currently ready when the function was invoked.
   * This function will lock all the stored timers throughout its duration.
   * Function is thread safe, but it will throw an error if the timers thread is running.
   */
  void execute_ready_timers();

  /**
   * @brief Executes head timer if ready at time point.
   * Function is thread safe, but it will throw an error if the timers thread is running.
   * 
   * @param tp the time point to check for, where `max()` denotes that no check will be performed.
   * @return true if head timer was ready at time point.
   */
  bool execute_head_timer(
    std::chrono::time_point<std::chrono::steady_clock> tp =
    std::chrono::time_point<std::chrono::steady_clock>::max());

  /**
   * @brief Get the amount of time before the next timer expires.
   * Function is thread safe, but it will throw an error if the timers thread is running.
   * 
   * @return std::chrono::nanoseconds to wait,
   * the returned value could be negative if the timer is already expired
   * or MAX_TIME if there are no timers stored in the object.
   */
  std::chrono::nanoseconds get_head_timeout();

  // This is what the TimersManager uses to denote a duration forever.
  // We don't use std::chrono::nanoseconds::max because it will overflow.
  // See https://en.cppreference.com/w/cpp/thread/condition_variable/wait_for
  static constexpr std::chrono::nanoseconds MAX_TIME = std::chrono::hours(90);

private:
  RCLCPP_DISABLE_COPY(TimersManager)

  using TimerPtr = rclcpp::TimerBase::SharedPtr;
  using WeakTimerPtr = rclcpp::TimerBase::WeakPtr;

  // Forward declaration
  class TimersHeap;

  /**
   * @brief This class allows to store weak pointers to timers in a heap-like data structure.
   * The root of the heap is the timer that expires first.
   * Since this class uses weak ownership, it is not guaranteed that it represents a valid heap
   * at any point in time as timers could go out of scope, thus invalidating it.
   * The "validate_and_lock" API allows to get ownership of the timers and also makes sure that
   * the heap property is respected.
   * This class is not thread safe and requires external mutexes to protect its usage.
   */
  class WeakTimersHeap
  {
public:
    /**
     * @brief Add a new timer to the heap. After the addition, the heap property is enforced.
     *
     * @param timer new timer to add.
     * @return true if timer has been added, false if it was already there.
     */
    bool add_timer(TimerPtr timer)
    {
      TimersHeap locked_heap = this->validate_and_lock();
      bool added = locked_heap.add_timer(std::move(timer));

      if (added) {
        // Re-create the weak heap with the new timer added
        this->store(locked_heap);
      }

      return added;
    }

    /**
     * @brief Remove a timer from the heap. After the removal, the heap property is enforced.
     *
     * @param timer timer to remove.
     * @return true if timer has been removed, false if it was not there.
     */
    bool remove_timer(TimerPtr timer)
    {
      TimersHeap locked_heap = this->validate_and_lock();
      bool removed = locked_heap.remove_timer(std::move(timer));

      if (removed) {
        // Re-create the weak heap with the timer removed
        this->store(locked_heap);
      }

      return removed;
    }

    /**
     * @brief This function restores the current object as a valid heap
     * and it returns a locked version of it.
     * It is the only public API to access the stored timers.
     *
     * @return TimersHeap owned timers corresponding to the current object
     */
    TimersHeap validate_and_lock()
    {
      TimersHeap locked_heap;
      bool any_timer_destroyed = false;

      auto it = weak_heap_.begin();

      while (it != weak_heap_.end()) {
        if (auto timer_shared_ptr = it->lock()) {
          // This timer is valid, add it to the locked heap
          // Note: we access private `owned_heap_` member field.
          locked_heap.owned_heap_.push_back(std::move(timer_shared_ptr));
          it++;
        } else {
          // This timer went out of scope, remove it
          it = weak_heap_.erase(it);
          any_timer_destroyed = true;
        }
      }

      // If a timer has gone out of scope, then the remaining elements may not represent
      // a valid heap anymore. We need to re heapify the timers heap.
      if (any_timer_destroyed) {
        locked_heap.heapify();
        // Re-create the weak heap now that elements have been heapified again
        this->store(locked_heap);
      }

      return locked_heap;
    }

    /**
     * @brief This function allows to recreate the heap of weak pointers
     * from an heap of owned pointers.
     * It is required to be called after a locked TimersHeap generated from this object
     * has been modified in any way (e.g. timers triggered, added, removed).
     *
     * @param heap timers heap to store as weak pointers
     */
    void store(const TimersHeap & heap)
    {
      weak_heap_.clear();
      // Note: we access private `owned_heap_` member field.
      for (auto t : heap.owned_heap_) {
        weak_heap_.push_back(t);
      }
    }

    /**
     * @brief Remove all timers from the heap.
     */
    void clear()
    {
      weak_heap_.clear();
    }

private:
    std::vector<WeakTimerPtr> weak_heap_;
  };

  /**
   * @brief This class is the equivalent of WeakTimersHeap but with ownership of the timers.
   * It can be generated by locking the weak version.
   * It provides operations to manipulate the heap.
   * This class is not thread safe and requires external mutexes to protect its usage.
   */
  class TimersHeap
  {
public:
    /**
     * @brief Try to add a new timer to the heap.
     * After the addition, the heap property is preserved.
     * @param timer new timer to add
     * @return true if timer has been added, false if it was already there
     */
    bool add_timer(TimerPtr timer)
    {
      // Nothing to do if the timer is already stored here
      auto it = std::find(owned_heap_.begin(), owned_heap_.end(), timer);
      if (it != owned_heap_.end()) {
        return false;
      }

      owned_heap_.push_back(std::move(timer));
      std::push_heap(owned_heap_.begin(), owned_heap_.end(), timer_greater);

      return true;
    }

    /**
     * @brief Try to remove a timer from the heap.
     * After the removal, the heap property is preserved.
     * @param timer timer to remove
     * @return true if timer has been removed, false if it was not there
     */
    bool remove_timer(TimerPtr timer)
    {
      // Nothing to do if the timer is not stored here
      auto it = std::find(owned_heap_.begin(), owned_heap_.end(), timer);
      if (it == owned_heap_.end()) {
        return false;
      }

      owned_heap_.erase(it);
      std::make_heap(owned_heap_.begin(), owned_heap_.end(), timer_greater);

      return true;
    }

    /**
     * @brief Returns a reference to the front element
     */
    TimerPtr & front()
    {
      return owned_heap_.front();
    }

    /**
     * @brief Returns a const reference to the front element
     */
    const TimerPtr & front() const
    {
      return owned_heap_.front();
    }

    /**
     * @brief Returns whether the heap is empty or not
     */
    bool empty() const
    {
      return owned_heap_.empty();
    }

    /**
    * @brief Restore a valid heap after the root value has been replaced (e.g. timer triggered).
    */
    void heapify_root()
    {
      // The following code is a more efficient version of doing
      // pop_heap();
      // pop_back();
      // push_back();
      // push_heap();
      // as it removes the need for the last push_heap

      // Push the modified element (i.e. the current root) at the bottom of the heap
      owned_heap_.push_back(owned_heap_[0]);
      // Exchange first and last-1 elements and reheapify
      std::pop_heap(owned_heap_.begin(), owned_heap_.end(), timer_greater);
      // Remove last element
      owned_heap_.pop_back();
    }

    /**
     * @brief Completely restores the structure to a valid heap
     */
    void heapify()
    {
      std::make_heap(owned_heap_.begin(), owned_heap_.end(), timer_greater);
    }

    /**
     * @brief Friend declaration to allow the `validate_and_lock()` function to access the
     * underlying heap container
     */
    friend TimersHeap WeakTimersHeap::validate_and_lock();

    /**
     * @brief Friend declaration to allow the `store()` function to access the
     * underlying heap container
     */
    friend void WeakTimersHeap::store(const TimersHeap & heap);

private:
    /**
     * @brief Comparison function between timers.
     * Returns true if `a` expires after `b`.
     */
    static bool timer_greater(TimerPtr a, TimerPtr b)
    {
      return a->time_until_trigger() > b->time_until_trigger();
    }

    std::vector<TimerPtr> owned_heap_;
  };

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
   * This function is not thread safe, acquire the timers_mutex_ before calling it.
   */
  std::chrono::nanoseconds get_head_timeout_unsafe(const TimersHeap & heap)
  {
    if (heap.empty()) {
      return MAX_TIME;
    }
    return (heap.front())->time_until_trigger();
  }

  /**
   * @brief Executes all the timers currently ready when the function is invoked
   * while keeping the heap correctly sorted.
   * This function is not thread safe, acquire the timers_mutex_ before calling it.
   */
  void execute_ready_timers_unsafe();

  /**
   * @brief Helper function that checks whether a timer was already ready
   * at a specific time point.

   * @param timer a pointer to the timer to check for
   * @param tp the time point to check for
   * @return true if timer was ready at tp
   */
  bool timer_was_ready_at_tp(
    TimerPtr timer,
    std::chrono::time_point<std::chrono::steady_clock> tp)
  {
    // A ready timer will return a negative duration when calling time_until_trigger
    auto time_ready = std::chrono::steady_clock::now() + timer->time_until_trigger();
    return time_ready < tp;
  }

  // Thread used to run the timers execution task
  std::thread timers_thread_;
  // Protects access to timers
  std::mutex timers_mutex_;
  // Notifies the timers thread whenever timers are added/removed
  std::condition_variable timers_cv_;
  // Flag used as predicate by timers_cv
  bool timers_updated_ {false};
  // Indicates whether the timers thread is currently running or requested to stop
  std::atomic<bool> running_ {false};
  // Context of the parent executor
  std::shared_ptr<rclcpp::Context> context_;
  // Timers heap with weak ownership
  WeakTimersHeap weak_timers_heap_;
};

}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__TIMERS_MANAGER_HPP_
