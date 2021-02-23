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
  // or if another thread already signaled to stop.
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
  return this->get_head_timeout_unsafe();
}

void TimersManager::execute_ready_timers()
{
  // Do not allow to interfere with the thread running
  if (running_) {
    throw std::runtime_error(
            "TimersManager::execute_ready_timers() can't be used while timers thread is running");
  }

  std::unique_lock<std::mutex> lock(timers_mutex_);
  this->execute_ready_timers_unsafe();
}

bool TimersManager::execute_head_timer(
  std::chrono::time_point<std::chrono::steady_clock> tp)
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

  bool timer_ready = false;
  if (tp != std::chrono::time_point<std::chrono::steady_clock>::max()) {
    timer_ready = timer_was_ready_at_tp(head, tp);
  } else {
    timer_ready = head->is_ready();
  }

  if (timer_ready) {
    // Head timer is ready, execute
    head->execute_callback();
    timers_heap.heapify_root();
    weak_timers_heap_.store(timers_heap);
    return true;
  }

  // Head timer was not ready yet
  return false;
}

std::chrono::nanoseconds TimersManager::get_head_timeout_unsafe()
{
  // If we don't have any weak pointer, then we just return maximum timeout
  if (weak_timers_heap_.empty()) {
    return MAX_TIME;
  }

  // Weak heap is not empty, so try to lock the first element
  TimerPtr head_timer = weak_timers_heap_.front().lock();
  // If it is still a valid pointer, it is guaranteed to be the correct head
  if (head_timer != nullptr) {
    return head_timer->time_until_trigger();
  }

  // If the first elements has expired, we can't make other assumptions on the heap
  // and we need to entirely validate it.
  TimersHeap locked_heap = weak_timers_heap_.validate_and_lock();

  // NOTE: the following operations will not modify any element in the heap, so we
  // don't have to call `weak_timers_heap_.store(locked_heap)` at the end.

  if (locked_heap.empty()) {
    return MAX_TIME;
  }
  return locked_heap.front()->time_until_trigger();
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
  // The second check prevents this function from blocking indefinitely if the
  // time required for executing the timers is longer than their period.

  TimerPtr head = locked_heap.front();
  auto start_time = std::chrono::steady_clock::now();
  while (head->is_ready() && this->timer_was_ready_at_tp(head, start_time)) {
    // Execute head timer
    head->execute_callback();
    // Executing a timer will result in updating its time_until_trigger, so re-heapify
    locked_heap.heapify_root();
    // Get new head timer
    head = locked_heap.front();
  }

  // After having performed work on the locked heap we reflect the changes to weak one.
  // Timers will be already sorted the next time we need them if none went out of scope.
  weak_timers_heap_.store(locked_heap);
}

void TimersManager::run_timers()
{
  while (rclcpp::ok(context_) && running_) {
    // Lock mutex
    std::unique_lock<std::mutex> timers_lock(timers_mutex_);

    std::chrono::nanoseconds time_to_sleep = get_head_timeout_unsafe();
    // Wait until timeout or notification that timers have been updated
    timers_cv_.wait_for(timers_lock, time_to_sleep, [this]() {return timers_updated_;});
    // Reset timers updated flag
    timers_updated_ = false;

    // Execute timers
    this->execute_ready_timers_unsafe();
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
