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

#include "rclcpp/executors/timers_manager.hpp"

#include <memory>

using rclcpp::executors::TimersManager;

constexpr std::chrono::nanoseconds TimersManager::MAX_TIME;

TimersManager::TimersManager(std::shared_ptr<rclcpp::Context> context)
{
  context_ = context;
}

TimersManager::~TimersManager()
{
  // Make sure timers thread is stopped before destroying this object
  this->stop();
  // Remove all timers
  this->clear_all();
}

void TimersManager::add_timer(rclcpp::TimerBase::SharedPtr timer)
{
  {
    std::unique_lock<std::mutex> lock(timers_mutex_);

    // Make sure that the provided timer is not already in the timers storage
    if (std::find(timers_storage_.begin(), timers_storage_.end(), timer) != timers_storage_.end()) {
      return;
    }

    // Store ownership of timer and add it to heap
    timers_storage_.emplace_back(timer);
    this->add_timer_to_heap(&(timers_storage_.back()));
  }

  // Notify that a timer has been added to the heap
  timers_cv_.notify_one();

  //verify();
}

void TimersManager::start()
{
  // Make sure that the thread is not already running
  if (running_) {
    throw std::runtime_error("TimersManager::start() can't start timers thread as already running");
  }

  running_ = true;
  timers_thread_ = std::thread(&TimersManager::run_timers, this);
  pthread_setname_np(timers_thread_.native_handle(), "TimersManager");
}

void TimersManager::stop()
{
  // Notify stop to timers thread
  running_ = false;
  timers_cv_.notify_one();

  // Join timers thread if it's running
  if (timers_thread_.joinable()) {
    timers_thread_.join();
  }
}

std::chrono::nanoseconds TimersManager::get_head_timeout()
{
  std::unique_lock<std::mutex> lock(timers_mutex_);
  return this->get_head_timeout_unsafe();
}

void TimersManager::execute_ready_timers()
{
  std::unique_lock<std::mutex> lock(timers_mutex_);
  this->execute_ready_timers_unsafe();
}

void TimersManager::execute_ready_timers_unsafe()
{
  if (heap_.empty()) {
    return;
  }

  // Keep executing timers until they are read and they were already ready when we started.
  // The second check prevents this function from blocking indefinitely if the
  // time required for executing the timers is longer than their period.

  auto start = std::chrono::steady_clock::now();
  TimerPtr head = heap_.front();
  while ((*head)->is_ready() && timer_was_ready_at_tp(start, head)) {
    (*head)->execute_callback();
    this->restore_heap_root();
    head = heap_.front();
    //verify();
  }
}

bool TimersManager::execute_head_timer()
{
  std::unique_lock<std::mutex> lock(timers_mutex_);

  if (heap_.empty()) {
    return false;
  }

  TimerPtr head = heap_.front();
  if ((*head)->is_ready()) {
    (*head)->execute_callback();
    this->restore_heap_root();
    return true;
  } else {
    return false;
  }
}

void TimersManager::run_timers()
{
  while (rclcpp::ok(context_) && running_) {
    std::unique_lock<std::mutex> lock(timers_mutex_);
    auto time_to_sleep = this->get_head_timeout_unsafe();
    timers_cv_.wait_for(lock, time_to_sleep);
    this->execute_ready_timers_unsafe();
  }

  running_ = false;
}

void TimersManager::clear_all()
{
  std::unique_lock<std::mutex> lock(timers_mutex_);
  heap_.clear();
  timers_storage_.clear();
}

void TimersManager::remove_timer(rclcpp::TimerBase::SharedPtr timer)
{
  std::unique_lock<std::mutex> lock(timers_mutex_);

  // Make sure that we are currently storing the provided timer before proceeding
  auto it = std::find(timers_storage_.begin(), timers_storage_.end(), timer);
  if (it == timers_storage_.end()) {
    return;
  }

  // Remove timer from the storage and rebuild heap
  timers_storage_.erase(it);
  this->rebuild_heap();
}

void TimersManager::rebuild_heap()
{
  heap_.clear();
  for (auto & t : timers_storage_) {
    this->add_timer_to_heap(&t);
  }
}
