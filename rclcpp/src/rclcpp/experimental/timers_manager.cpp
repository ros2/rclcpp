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

#include "rclcpp/experimental/timers_manager.hpp"

#include <inttypes.h>

#include <ctime>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "rcpputils/scope_exit.hpp"

using rclcpp::experimental::TimersManager;

TimersManager::TimersManager(
  std::shared_ptr<rclcpp::Context> context,
  std::function<void(const rclcpp::TimerBase *, const std::shared_ptr<void> &)> on_ready_callback)
: on_ready_callback_(on_ready_callback),
  context_(context)
{
}

TimersManager::~TimersManager()
{
  // Remove all timers
  this->clear();

  // Make sure timers thread is stopped before destroying this object
  this->stop();
}

void TimersManager::add_timer(rclcpp::TimerBase::SharedPtr timer)
{
  if (!timer) {
    throw std::invalid_argument("TimersManager::add_timer() trying to add nullptr timer");
  }

  bool added = false;
  {
    std::unique_lock<std::mutex> lock(timers_mutex_);
    added = weak_timers_heap_.add_timer(timer);
    timers_updated_ = timers_updated_ || added;
  }

  timer->set_on_reset_callback(
    [this](size_t arg) {
      {
        (void)arg;
        std::unique_lock<std::mutex> lock(timers_mutex_);
        timers_updated_ = true;
      }
      timers_cv_.notify_one();
    });

  if (added) {
    // Notify that a timer has been added
    timers_cv_.notify_one();
  }
}

void TimersManager::start()
{
  // Make sure that the thread is not already running
  if (running_.exchange(true)) {
    throw std::runtime_error("TimersManager::start() can't start timers thread as already running");
  }

  timers_thread_ = std::thread(&TimersManager::run_timers, this);
}

void TimersManager::stop()
{
  // Lock stop() function to prevent race condition in destructor
  std::unique_lock<std::mutex> lock(stop_mutex_);
  running_ = false;

  // Notify the timers manager thread to wake up
  {
    std::unique_lock<std::mutex> lock(timers_mutex_);
    timers_updated_ = true;
  }
  timers_cv_.notify_one();

  // Join timers thread if it's running
  if (timers_thread_.joinable()) {
    timers_thread_.join();
  }
}

std::optional<std::chrono::nanoseconds> TimersManager::get_head_timeout()
{
  // Do not allow to interfere with the thread running
  if (running_) {
    throw std::runtime_error(
            "get_head_timeout() can't be used while timers thread is running");
  }

  std::unique_lock<std::mutex> lock(timers_mutex_);
  return this->get_head_timeout_unsafe();
}

size_t TimersManager::get_number_ready_timers()
{
  // Do not allow to interfere with the thread running
  if (running_) {
    throw std::runtime_error(
            "get_number_ready_timers() can't be used while timers thread is running");
  }

  std::unique_lock<std::mutex> lock(timers_mutex_);
  TimersHeap locked_heap = weak_timers_heap_.validate_and_lock();
  return locked_heap.get_number_ready_timers();
}

bool TimersManager::execute_head_timer()
{
  // Do not allow to interfere with the thread running
  if (running_) {
    throw std::runtime_error(
            "execute_head_timer() can't be used while timers thread is running");
  }

  std::unique_lock<std::mutex> lock(timers_mutex_);

  TimersHeap timers_heap = weak_timers_heap_.validate_and_lock();

  // Nothing to do if we don't have any timer
  if (timers_heap.empty()) {
    return false;
  }

  TimerPtr head_timer = timers_heap.front();

  const bool timer_ready = head_timer->is_ready();
  if (timer_ready) {
    // NOTE: here we always execute the timer, regardless of whether the
    // on_ready_callback is set or not.
    auto data = head_timer->call();
    if (!data) {
      // someone canceled the timer between is_ready and call
      return false;
    }
    head_timer->execute_callback(data);
    timers_heap.heapify_root();
    weak_timers_heap_.store(timers_heap);
  }

  return timer_ready;
}

void TimersManager::execute_ready_timer(
  const rclcpp::TimerBase * timer_id,
  const std::shared_ptr<void> & data)
{
  TimerPtr ready_timer;
  {
    std::unique_lock<std::mutex> lock(timers_mutex_);
    ready_timer = weak_timers_heap_.get_timer(timer_id);
  }
  if (ready_timer) {
    ready_timer->execute_callback(data);
  }
}

std::optional<std::chrono::nanoseconds> TimersManager::get_head_timeout_unsafe()
{
  // If we don't have any weak pointer, then we just return maximum timeout
  if (weak_timers_heap_.empty()) {
    return std::chrono::nanoseconds::max();
  }
  // Weak heap is not empty, so try to lock the first element.
  // If it is still a valid pointer, it is guaranteed to be the correct head
  TimerPtr head_timer = weak_timers_heap_.front().lock();

  if (!head_timer) {
    // The first element has expired, we can't make other assumptions on the heap
    // and we need to entirely validate it.
    TimersHeap locked_heap = weak_timers_heap_.validate_and_lock();
    // NOTE: the following operations will not modify any element in the heap, so we
    // don't have to call `weak_timers_heap_.store(locked_heap)` at the end.

    if (locked_heap.empty()) {
      return std::chrono::nanoseconds::max();
    }
    head_timer = locked_heap.front();
  }
  if (head_timer->is_canceled()) {
    return std::nullopt;
  }
  return head_timer->time_until_trigger();
}

void TimersManager::execute_ready_timers_unsafe()
{
  // We start by locking the timers
  TimersHeap locked_heap = weak_timers_heap_.validate_and_lock();

  // Nothing to do if we don't have any timer
  if (locked_heap.empty()) {
    return;
  }

  // Keep executing timers until they are ready and they were already ready when we started.
  // The two checks prevent this function from blocking indefinitely if the
  // time required for executing the timers is longer than their period.

  TimerPtr head_timer = locked_heap.front();
  const size_t number_ready_timers = locked_heap.get_number_ready_timers();
  size_t executed_timers = 0;
  while (executed_timers < number_ready_timers && head_timer->is_ready()) {
    auto data = head_timer->call();
    if (data) {
      if (on_ready_callback_) {
        on_ready_callback_(head_timer.get(), data);
      } else {
        head_timer->execute_callback(data);
      }
    } else {
      // someone canceled the timer between is_ready and call
      // we don't do anything, as the timer is now 'processed'
    }

    executed_timers++;
    // Executing a timer will result in updating its time_until_trigger, so re-heapify
    locked_heap.heapify_root();
    // Get new head timer
    head_timer = locked_heap.front();
  }

  // After having performed work on the locked heap we reflect the changes to weak one.
  // Timers will be already sorted the next time we need them if none went out of scope.
  weak_timers_heap_.store(locked_heap);
}

void TimersManager::run_timers()
{
  // Make sure the running flag is set to false when we exit from this function
  // to allow restarting the timers thread.
  RCPPUTILS_SCOPE_EXIT(this->running_.store(false); );

  while (rclcpp::ok(context_) && running_) {
    // Lock mutex
    std::unique_lock<std::mutex> lock(timers_mutex_);

    std::optional<std::chrono::nanoseconds> time_to_sleep = get_head_timeout_unsafe();

    // If head timer was cancelled, try to reheap and get a new head.
    // This avoids an edge condition where head timer is cancelled, but other
    // valid timers remain in the heap.
    if (!time_to_sleep.has_value()) {
      // Re-heap to (possibly) move cancelled timer from head of heap. If
      // entire heap is cancelled, this will still result in a nullopt.
      TimersHeap locked_heap = weak_timers_heap_.validate_and_lock();
      locked_heap.heapify();
      weak_timers_heap_.store(locked_heap);
      time_to_sleep = get_head_timeout_unsafe();
    }

    // If no timers, or all timers cancelled, wait for an update.
    if (!time_to_sleep.has_value() || (time_to_sleep.value() == std::chrono::nanoseconds::max()) ) {
      // Wait until notification that timers have been updated
      timers_cv_.wait(lock, [this]() {return timers_updated_;});
    } else if (time_to_sleep.value() != std::chrono::nanoseconds::zero()) {
      // If time_to_sleep is zero, we immediately execute. Otherwise, wait
      // until timeout or notification that timers have been updated
      timers_cv_.wait_for(lock, time_to_sleep.value(), [this]() {return timers_updated_;});
    }

    if (timers_updated_) {
      // Re-heap in case ordering changed due to a cancelled timer
      // re-activating.
      TimersHeap locked_heap = weak_timers_heap_.validate_and_lock();
      locked_heap.heapify();
      weak_timers_heap_.store(locked_heap);

      // Reset timers updated flag
      timers_updated_ = false;
    }

    // Execute timers
    this->execute_ready_timers_unsafe();
  }
}

void TimersManager::clear()
{
  {
    // Lock mutex and then clear all data structures
    std::unique_lock<std::mutex> lock(timers_mutex_);

    TimersHeap locked_heap = weak_timers_heap_.validate_and_lock();
    locked_heap.clear_timers_on_reset_callbacks();

    weak_timers_heap_.clear();

    timers_updated_ = true;
  }

  // Notify timers thread such that it can re-compute its timeout
  timers_cv_.notify_one();
}

void TimersManager::remove_timer(TimerPtr timer)
{
  bool removed = false;
  {
    std::unique_lock<std::mutex> lock(timers_mutex_);
    removed = weak_timers_heap_.remove_timer(timer);

    timers_updated_ = timers_updated_ || removed;
  }

  if (removed) {
    // Notify timers thread such that it can re-compute its timeout
    timers_cv_.notify_one();
    timer->clear_on_reset_callback();
  }
}
