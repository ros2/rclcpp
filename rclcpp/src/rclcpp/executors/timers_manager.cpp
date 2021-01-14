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

#include <memory>
#include <stdexcept>

#include "rclcpp/executors/timers_manager.hpp"

using rclcpp::executors::TimersManager;

constexpr std::chrono::nanoseconds TimersManager::MAX_TIME;

TimersManager::TimersManager(std::shared_ptr<rclcpp::Context> context)
{
  context_ = context;
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
    std::unique_lock<std::mutex> timers_lock(timers_mutex_);
    added = weak_timers_heap_.add_timer(timer);
    timers_updated_ = timers_updated_ || added;
  }

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
  pthread_setname_np(timers_thread_.native_handle(), "TimersManager");
}

void TimersManager::stop()
{
  // Nothing to do if the timers thread is not running
  // or if another thred already signaled to stop.
  if (!running_.exchange(false)) {
    return;
  }

  // Notify the timers manager thread to wake up
  {
    std::unique_lock<std::mutex> timers_lock(timers_mutex_);
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
            "TimersManager::get_head_timeout() can't be used while timers thread is running");
  }

  std::unique_lock<std::mutex> lock(timers_mutex_);
  TimersHeap timers_heap = weak_timers_heap_.validate_and_lock();
  return this->get_head_timeout_unsafe(timers_heap);
}

std::chrono::nanoseconds TimersManager::execute_ready_timers()
{
  // Do not allow to interfere with the thread running
  if (running_) {
    throw std::runtime_error(
            "TimersManager::execute_ready_timers() can't be used while timers thread is running");
  }

  std::unique_lock<std::mutex> lock(timers_mutex_);

  TimersHeap timers_heap = weak_timers_heap_.validate_and_lock();
  this->execute_ready_timers_unsafe(timers_heap);
  weak_timers_heap_.store(timers_heap);

  return this->get_head_timeout_unsafe(timers_heap);
}

bool TimersManager::execute_head_timer()
{
  // Do not allow to interfere with the thread running
  if (running_) {
    throw std::runtime_error(
            "TimersManager::execute_head_timer() can't be used while timers thread is running");
  }

  std::unique_lock<std::mutex> lock(timers_mutex_);

  TimersHeap timers_heap = weak_timers_heap_.validate_and_lock();
  // Nothing to do if we don't have any timer
  if (timers_heap.empty()) {
    return false;
  }

  TimerPtr head = timers_heap.front();
  if (head->is_ready()) {
    // Head timer is ready, execute
    head->execute_callback();
    timers_heap.heapify_root();
    weak_timers_heap_.store(timers_heap);
    return true;
  } else {
    // Head timer was not ready yet
    return false;
  }
}

void TimersManager::execute_ready_timers_unsafe(TimersHeap & heap)
{
  // Nothing to do if we don't have any timer
  if (heap.empty()) {
    return;
  }

  // Keep executing timers until they are ready and they were already ready when we started.
  // The second check prevents this function from blocking indefinitely if the
  // time required for executing the timers is longer than their period.

  TimerPtr head = heap.front();
  auto start_time = std::chrono::steady_clock::now();
  while (head->is_ready() && this->timer_was_ready_at_tp(head, start_time)) {
    // Execute head timer
    head->execute_callback();
    // Executing a timer will result in updating its time_until_trigger, so re-heapify
    heap.heapify_root();
    // Get new head timer
    head = heap.front();
  }
}

void TimersManager::run_timers()
{
  std::chrono::nanoseconds time_to_sleep;
  {
    std::unique_lock<std::mutex> lock(timers_mutex_);
    TimersHeap timers_heap = weak_timers_heap_.validate_and_lock();
    time_to_sleep = this->get_head_timeout_unsafe(timers_heap);
  }

  while (rclcpp::ok(context_) && running_) {
    // Lock mutex
    std::unique_lock<std::mutex> timers_lock(timers_mutex_);

    // Wait until timeout or notification that timers have been updated
    timers_cv_.wait_for(timers_lock, time_to_sleep, [this]() {return timers_updated_;});
    // Reset timers updated flag
    timers_updated_ = false;

    // Get ownership of timers
    TimersHeap timers_heap = weak_timers_heap_.validate_and_lock();
    // Execute timers
    this->execute_ready_timers_unsafe(timers_heap);
    // Store updated order of elements to efficiently re-use it next iteration
    weak_timers_heap_.store(timers_heap);
    // Get next timeout
    time_to_sleep = this->get_head_timeout_unsafe(timers_heap);
  }

  // Make sure the running flag is set to false when we exit from this function
  // to allow restarting the timers thread.
  running_ = false;
}

void TimersManager::clear()
{
  {
    // Lock mutex and then clear all data structures
    std::unique_lock<std::mutex> timers_lock(timers_mutex_);
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
    std::unique_lock<std::mutex> timers_lock(timers_mutex_);
    removed = weak_timers_heap_.remove_timer(timer);

    timers_updated_ = timers_updated_ || removed;
  }

  if (removed) {
    // Notify timers thread such that it can re-compute its timeout
    timers_cv_.notify_one();
  }
}
