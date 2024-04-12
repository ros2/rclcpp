// Copyright 2023 iRobot Corporation.
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

#ifndef RCLCPP__EXPERIMENTAL__TIMERS_MANAGER_HPP_
#define RCLCPP__EXPERIMENTAL__TIMERS_MANAGER_HPP_

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>
#include <vector>
#include "rclcpp/context.hpp"
#include "rclcpp/timer.hpp"

namespace rclcpp
{
namespace experimental
{

/**
 * @brief This class provides a way for storing and executing timer objects.
 * It provides APIs to suit the needs of different applications and execution models.
 * All public APIs provided by this class are thread-safe.
 *
 * Timers management
 * This class provides APIs to add/remove timers to/from an internal storage.
 * It keeps a list of weak pointers from added timers, and locks them only when
 * they need to be executed or modified.
 * Timers are kept ordered in a binary-heap priority queue.
 * Calls to add/remove APIs will temporarily block the execution of the timers and
 * will require to reorder the internal priority queue.
 * Because of this, they have a not-negligible impact on the performance.
 *
 * Timers execution
 * The most efficient use of this class consists in letting a TimersManager object
 * to spawn a thread where timers are monitored and optionally executed.
 * This can be controlled via the `start` and `stop` methods.
 * Ready timers can either be executed or an on_ready_callback can be used to notify
 * other entities that they are ready and need to be executed.
 * Other APIs allow to directly execute a given timer.
 *
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
   * @param on_ready_callback The timers on ready callback. The presence of this function
   * indicates what to do when the TimersManager is running and a timer becomes ready.
   * The TimersManager is considered "running" when the `start` method has been called.
   * If it's callable, it will be invoked instead of the timer callback.
   * If it's not callable, then the TimersManager will
   * directly execute timers when they are ready.
   * All the methods that execute a given timer (e.g. `execute_head_timer`
   * or `execute_ready_timer`) without the TimersManager being `running`, i.e.
   * without actually explicitly waiting for the timer to become ready, will ignore this
   * callback.
   */
  RCLCPP_PUBLIC
  TimersManager(
    std::shared_ptr<rclcpp::Context> context,
    std::function<void(const rclcpp::TimerBase *,
    const std::shared_ptr<void> &)> on_ready_callback = nullptr);

  /**
   * @brief Destruct the TimersManager object making sure to stop thread and release memory.
   */
  RCLCPP_PUBLIC
  ~TimersManager();

  /**
   * @brief Adds a new timer to the storage, maintaining weak ownership of it.
   * Function is thread safe and it can be called regardless of the state of the timers thread.
   *
   * @param timer the timer to add.
   * @throws std::invalid_argument if timer is a nullptr.
   */
  RCLCPP_PUBLIC
  void add_timer(rclcpp::TimerBase::SharedPtr timer);

  /**
   * @brief Remove a single timer from the object storage.
   * Will do nothing if the timer was not being stored here.
   * Function is thread safe and it can be called regardless of the state of the timers thread.
   *
   * @param timer the timer to remove.
   */
  RCLCPP_PUBLIC
  void remove_timer(rclcpp::TimerBase::SharedPtr timer);

  /**
   * @brief Remove all the timers stored in the object.
   * Function is thread safe and it can be called regardless of the state of the timers thread.
   */
  RCLCPP_PUBLIC
  void clear();

  /**
   * @brief Starts a thread that takes care of executing the timers stored in this object.
   * Function will throw an error if the timers thread was already running.
   */
  RCLCPP_PUBLIC
  void start();

  /**
   * @brief Stops the timers thread.
   * Will do nothing if the timer thread was not running.
   */
  RCLCPP_PUBLIC
  void stop();

  /**
   * @brief Get the number of timers that are currently ready.
   * This function is thread safe.
   *
   * @return size_t number of ready timers.
   * @throws std::runtime_error if the timers thread was already running.
   */
  RCLCPP_PUBLIC
  size_t get_number_ready_timers();

  /**
   * @brief Executes head timer if ready.
   * This function is thread safe.
   * This function will try to execute the timer callback regardless of whether
   * the TimersManager on_ready_callback was passed during construction.
   *
   * @return true if head timer was ready.
   * @throws std::runtime_error if the timers thread was already running.
   */
  RCLCPP_PUBLIC
  bool execute_head_timer();

  /**
   * @brief Executes timer identified by its ID.
   * This function is thread safe.
   * This function will try to execute the timer callback regardless of whether
   * the TimersManager on_ready_callback was passed during construction.
   *
   * @param timer_id the timer ID of the timer to execute
   * @param data internal data of the timer
   */
  RCLCPP_PUBLIC
  void execute_ready_timer(const rclcpp::TimerBase * timer_id, const std::shared_ptr<void> & data);

  /**
   * @brief Get the amount of time before the next timer triggers.
   * This function is thread safe.
   *
   * @return std::optional<std::chrono::nanoseconds> to wait,
   * the returned value could be negative if the timer is already expired
   * or std::chrono::nanoseconds::max() if there are no timers stored in the object.
   * If the head timer was cancelled, then this will return a nullopt.
   * @throws std::runtime_error if the timers thread was already running.
   */
  RCLCPP_PUBLIC
  std::optional<std::chrono::nanoseconds> get_head_timeout();

private:
  RCLCPP_DISABLE_COPY(TimersManager)

  using TimerPtr = rclcpp::TimerBase::SharedPtr;
  using WeakTimerPtr = rclcpp::TimerBase::WeakPtr;

  // Forward declaration
  class TimersHeap;

  /**
   * @brief This class allows to store weak pointers to timers in a heap-like data structure.
   * The root of the heap is the timer that triggers first.
   * Since this class uses weak ownership, it is not guaranteed that it represents a valid heap
   * at any point in time as timers could go out of scope, thus invalidating it.
   * The "validate_and_lock" API allows to restore the heap property and also returns a locked version
   * of the timers heap.
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
     * @brief Retrieve the timer identified by the key
     * @param timer_id The ID of the timer to retrieve.
     * @return TimerPtr if there's a timer associated with the ID, nullptr otherwise
     */
    TimerPtr get_timer(const rclcpp::TimerBase * timer_id)
    {
      for (auto & weak_timer : weak_heap_) {
        auto timer = weak_timer.lock();
        if (timer.get() == timer_id) {
          return timer;
        }
      }
      return nullptr;
    }

    /**
     * @brief Returns a const reference to the front element.
     */
    const WeakTimerPtr & front() const
    {
      return weak_heap_.front();
    }

    /**
     * @brief Returns whether the heap is empty or not.
     */
    bool empty() const
    {
      return weak_heap_.empty();
    }

    /**
     * @brief This function restores the current object as a valid heap
     * and it returns a locked version of it.
     * Timers that went out of scope are removed from the container.
     * It is the only public API to access and manipulate the stored timers.
     *
     * @return TimersHeap owned timers corresponding to the current object
     */
    TimersHeap validate_and_lock()
    {
      TimersHeap locked_heap;
      bool any_timer_destroyed = false;

      for (auto weak_timer : weak_heap_) {
        auto timer = weak_timer.lock();
        if (timer) {
          // This timer is valid, so add it to the locked heap
          // Note: we access friend private `owned_heap_` member field.
          locked_heap.owned_heap_.push_back(std::move(timer));
        } else {
          // This timer went out of scope, so we don't add it to locked heap
          // and we mark the corresponding flag.
          // It's not needed to erase it from weak heap, as we are going to re-heapify.
          // Note: we can't exit from the loop here, as we need to find all valid timers.
          any_timer_destroyed = true;
        }
      }

      // If a timer has gone out of scope, then the remaining elements do not represent
      // a valid heap anymore. We need to re-heapify the timers heap.
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
      // Note: we access friend private `owned_heap_` member field.
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
     * @param timer new timer to add.
     * @return true if timer has been added, false if it was already there.
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
     * @param timer timer to remove.
     * @return true if timer has been removed, false if it was not there.
     */
    bool remove_timer(TimerPtr timer)
    {
      // Nothing to do if the timer is not stored here
      auto it = std::find(owned_heap_.begin(), owned_heap_.end(), timer);
      if (it == owned_heap_.end()) {
        return false;
      }

      owned_heap_.erase(it);
      this->heapify();

      return true;
    }

    /**
     * @brief Returns a reference to the front element.
     * @return reference to front element.
     */
    TimerPtr & front()
    {
      return owned_heap_.front();
    }

    /**
     * @brief Returns a const reference to the front element.
     * @return const reference to front element.
     */
    const TimerPtr & front() const
    {
      return owned_heap_.front();
    }

    /**
     * @brief Returns whether the heap is empty or not.
     * @return true if the heap is empty.
     */
    bool empty() const
    {
      return owned_heap_.empty();
    }

    /**
     * @brief Returns the size of the heap.
     * @return the number of valid timers in the heap.
     */
    size_t size() const
    {
      return owned_heap_.size();
    }

    /**
     * @brief Get the number of timers that are currently ready.
     * @return size_t number of ready timers.
     */
    size_t get_number_ready_timers() const
    {
      size_t ready_timers = 0;

      for (TimerPtr t : owned_heap_) {
        if (t->is_ready()) {
          ready_timers++;
        }
      }

      return ready_timers;
    }

    /**
    * @brief Restore a valid heap after the root value has been replaced (e.g. timer triggered).
    */
    void heapify_root()
    {
      // The following code is a more efficient version than doing
      // pop_heap, pop_back, push_back, push_heap
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
     * @brief Helper function to clear the "on_reset_callback" on all associated timers.
     */
    void clear_timers_on_reset_callbacks()
    {
      for (TimerPtr & t : owned_heap_) {
        t->clear_on_reset_callback();
      }
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
     * @return true if `a` triggers after `b`.
     */
    static bool timer_greater(TimerPtr a, TimerPtr b)
    {
      // TODO(alsora): this can cause an error if timers are using different clocks
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
   * @brief Get the amount of time before the next timer triggers.
   * This function is not thread safe, acquire a mutex before calling it.
   *
   * @return std::optional<std::chrono::nanoseconds> to wait,
   * the returned value could be negative if the timer is already expired
   * or std::chrono::nanoseconds::max() if the heap is empty.
   * If the head timer was cancelled, then this will return a nullopt.
   * This function is not thread safe, acquire the timers_mutex_ before calling it.
   */
  std::optional<std::chrono::nanoseconds> get_head_timeout_unsafe();

  /**
   * @brief Executes all the timers currently ready when the function is invoked
   * while keeping the heap correctly sorted.
   * This function is not thread safe, acquire the timers_mutex_ before calling it.
   */
  void execute_ready_timers_unsafe();

  // Callback to be called when timer is ready
  std::function<void(const rclcpp::TimerBase *,
    const std::shared_ptr<void> &)> on_ready_callback_ = nullptr;

  // Thread used to run the timers execution task
  std::thread timers_thread_;
  // Protects access to timers
  std::mutex timers_mutex_;
  // Protects access to stop()
  std::mutex stop_mutex_;
  // Notifies the timers thread whenever timers are added/removed
  std::condition_variable timers_cv_;
  // Flag used as predicate by timers_cv_ that denotes one or more timers being added/removed
  bool timers_updated_ {false};
  // Indicates whether the timers thread is currently running or not
  std::atomic<bool> running_ {false};
  // Parent context used to understand if ROS is still active
  std::shared_ptr<rclcpp::Context> context_;
  // Timers heap storage with weak ownership
  WeakTimersHeap weak_timers_heap_;
};

}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__TIMERS_MANAGER_HPP_
