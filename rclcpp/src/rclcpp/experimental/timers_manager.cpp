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
  std::function<void(const rclcpp::TimerBase *)> on_ready_callback)
{
  context_ = context;
  on_ready_callback_ = on_ready_callback;
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

std::chrono::nanoseconds TimersManager::get_head_timeout()
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
    head_timer->call();
    head_timer->execute_callback();
    timers_heap.heapify_root();
    weak_timers_heap_.store(timers_heap);
  }

  return timer_ready;
}

void TimersManager::execute_ready_timer(const rclcpp::TimerBase * timer_id)
{
  TimerPtr ready_timer;
  {
    std::unique_lock<std::mutex> lock(timers_mutex_);
    ready_timer = weak_timers_heap_.get_timer(timer_id);
  }
  if (ready_timer) {
    ready_timer->execute_callback();
  }
}

std::chrono::nanoseconds TimersManager::get_head_timeout_unsafe()
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
    head_timer->call();
    if (on_ready_callback_) {
      on_ready_callback_(head_timer.get());
    } else {
      head_timer->execute_callback();
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

    std::chrono::nanoseconds time_to_sleep = get_head_timeout_unsafe();

    // No need to wait if a timer is already available
    if (time_to_sleep > std::chrono::nanoseconds::zero()) {
      if (time_to_sleep != std::chrono::nanoseconds::max()) {
        // Wait until timeout or notification that timers have been updated
        timers_cv_.wait_for(lock, time_to_sleep, [this]() {return timers_updated_;});
      } else {
        // Wait until notification that timers have been updated
        timers_cv_.wait(lock, [this]() {return timers_updated_;});
      }
    }

    // Reset timers updated flag
    timers_updated_ = false;

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
