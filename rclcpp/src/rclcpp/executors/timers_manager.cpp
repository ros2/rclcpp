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

  {
    std::unique_lock<std::mutex> timers_lock(timers_mutex_);

    // Nothing to do if the timer is already stored here
    for(const auto & weak_timer : weak_timers_) {
      bool matched = (weak_timer.lock() == timer);

      if (matched) {
        return;
      }
    }

    weak_timers_.push_back(timer);
    timers_updated_ = true;
  }

  // Notify that a timer has been added to the vector of weak timers
  timers_cv_.notify_one();
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

void TimersManager::own_timers(std::vector<TimerPtr> & timers)
{
  auto it = weak_timers_.begin();

  while (it != weak_timers_.end()) {
    if (auto timer_shared_ptr = it->lock()) {
      // The timer is valid, own it and add to heap
      timers.push_back(timer_shared_ptr);
    } else {
      // The timer went out of scope, remove it
      weak_timers_.erase(it);
    }
    it++;
  }

  heap_.make_heap(timers);
}

std::chrono::nanoseconds TimersManager::get_head_timeout()
{
  // Vector of pointers to timers used to implement the priority queue
  std::vector<TimerPtr> timers;
  this->own_timers(timers);

  if (timers.empty()) {
    return MAX_TIME;
  }

  return (timers.front())->time_until_trigger();
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

bool TimersManager::execute_head_timer()
{
  // Do not allow to interfere with the thread running
  if (running_) {
    throw std::runtime_error(
            "TimersManager::execute_head_timer() can't be used while timers thread is running");
  }

  std::unique_lock<std::mutex> lock(timers_mutex_); //needed?

  // Vector of pointers to timers used to implement the priority queue
  std::vector<TimerPtr> timers;

  this->own_timers(timers);

  // Nothing to do if we don't have any timer
  if (timers.empty()) {
    return false;
  }

  TimerPtr head = timers.front();
  if (head->is_ready()) {
    // Head timer is ready, execute and re-heapify
    head->execute_callback();
    return true;
  } else {
    // Head timer was not ready yet
    return false;
  }
}

std::chrono::nanoseconds TimersManager::execute_ready_timers_unsafe()
{
  // Vector of pointers to timers used to implement the priority queue
  std::vector<TimerPtr> timers;

  this->own_timers(timers);

  // Nothing to do if we don't have any timer
  if (timers.empty()) {
    return MAX_TIME;
  }

  // Keep executing timers until they are read and they were already ready when we started.
  // The second check prevents this function from blocking indefinitely if the
  // time required for executing the timers is longer than their period.

  auto start = std::chrono::steady_clock::now();

  TimerPtr head = timers.front();

  while (head->is_ready() && this->timer_was_ready_at_tp(head, start)) {
    // Execute head timer
    head->execute_callback();
    // Executing a timer will result in updating its time_until_trigger, so re-heapify
    heap_.heapify_root(timers);
    // Get new head timer
    head = timers.front();
  }

  return (timers.front())->time_until_trigger();
}

void TimersManager::run_timers()
{
  auto time_to_sleep = this->get_head_timeout(); // Aquires and releases ownership

  while (rclcpp::ok(context_) && running_) {
    // Lock mutex
    std::unique_lock<std::mutex> timers_lock(timers_mutex_);

    // Wait until timeout or notification that timers have been updated
    timers_cv_.wait_for(timers_lock, time_to_sleep, [this]() {return timers_updated_;});
    // Reset timers updated flag
    timers_updated_ = false;
    // Execute timers
    time_to_sleep = this->execute_ready_timers_unsafe();
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
    weak_timers_.clear();

    timers_updated_ = true;
  }

  // Notify timers thread such that it can re-compute its timeout
  timers_cv_.notify_one();
}

void TimersManager::remove_timer(TimerPtr timer)
{
  {
    std::unique_lock<std::mutex> timers_lock(timers_mutex_);

    bool matched = false;
    auto it = weak_timers_.begin();

    while (it != weak_timers_.end()) {
      matched = (it->lock() == timer);

      if (matched) {
        weak_timers_.erase(it);
        break;
      }

      it++;
    }

    if ((it == weak_timers_.end()) && !matched ) {
      throw std::runtime_error("Tried to remove timer not stored in the timers manager.");
    }

    timers_updated_ = true;
  }

  // Notify timers thread such that it can re-compute its timeout
  timers_cv_.notify_one();
}
