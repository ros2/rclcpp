// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/executors/async_helpers.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace rclcpp
{
namespace executors
{

// TimeoutException implementation
TimeoutException::TimeoutException()
: AsyncOperationException("Async operation timed out"),
  timeout_(std::chrono::milliseconds::zero())
{
}

TimeoutException::TimeoutException(std::chrono::milliseconds timeout)
: AsyncOperationException(
    "Async operation timed out after " + std::to_string(timeout.count()) + "ms"),
  timeout_(timeout)
{
}

std::chrono::milliseconds TimeoutException::timeout() const noexcept
{
  return timeout_;
}

// CancelledException implementation
CancelledException::CancelledException()
: AsyncOperationException("Async operation was cancelled")
{
}

// ServiceNotReadyException implementation
ServiceNotReadyException::ServiceNotReadyException(const std::string & service_name)
: AsyncOperationException("Service '" + service_name + "' is not ready"),
  service_name_(service_name)
{
}

const std::string & ServiceNotReadyException::service_name() const noexcept
{
  return service_name_;
}

// CancellationTokenSource::Impl
struct CancellationTokenSource::Impl
{
  std::atomic<bool> cancelled{false};
  std::mutex callbacks_mutex;
  std::vector<std::weak_ptr<CancellationToken::CancellationCallback>> callbacks;
  std::thread cancel_timer_thread;
  std::atomic<bool> timer_running{false};

  ~Impl()
  {
    timer_running.store(false);
    if (cancel_timer_thread.joinable()) {
      cancel_timer_thread.join();
    }
  }

  void invoke_callbacks()
  {
    std::lock_guard<std::mutex> lock(callbacks_mutex);
    for (auto & weak_callback : callbacks) {
      if (auto callback = weak_callback.lock()) {
        (*callback)();
      }
    }
    callbacks.clear();
  }

  void cleanup_expired_callbacks()
  {
    callbacks.erase(
      std::remove_if(
        callbacks.begin(),
        callbacks.end(),
        [](const auto & weak_ptr) { return weak_ptr.expired(); }),
      callbacks.end());
  }
};

// CancellationTokenSource implementation
CancellationTokenSource::CancellationTokenSource()
: impl_(std::make_shared<Impl>())
{
}

CancellationTokenSource::~CancellationTokenSource()
{
}

CancellationToken CancellationTokenSource::get_token() const
{
  auto non_deleting_ptr = std::shared_ptr<const CancellationTokenSource>(
    this, [](const CancellationTokenSource *) {});
  return CancellationToken(
    std::const_pointer_cast<CancellationTokenSource>(non_deleting_ptr));
}

void CancellationTokenSource::cancel()
{
  if (!impl_->cancelled.exchange(true)) {
    impl_->invoke_callbacks();
  }
}

bool CancellationTokenSource::is_cancellation_requested() const noexcept
{
  return impl_->cancelled.load();
}

void CancellationTokenSource::cancel_after(std::chrono::milliseconds duration)
{
  if (impl_->timer_running.exchange(true)) {
    return;  // Timer already running
  }

  impl_->cancel_timer_thread = std::thread([this, duration]() {
      std::this_thread::sleep_for(duration);
      if (impl_->timer_running.load()) {
        cancel();
      }
    });
  impl_->cancel_timer_thread.detach();
}

std::shared_ptr<void> CancellationTokenSource::register_callback(
  CancellationToken::CancellationCallback callback)
{
  if (impl_->cancelled.load()) {
    // Already cancelled, invoke immediately
    callback();
    return std::shared_ptr<void>();
  }

  auto callback_ptr =
    std::make_shared<CancellationToken::CancellationCallback>(std::move(callback));

  {
    std::lock_guard<std::mutex> lock(impl_->callbacks_mutex);
    impl_->cleanup_expired_callbacks();
    impl_->callbacks.push_back(callback_ptr);
  }

  return callback_ptr;
}

// CancellationToken implementation
CancellationToken::CancellationToken()
: source_(nullptr)
{
}

CancellationToken::CancellationToken(std::shared_ptr<CancellationTokenSource> source)
: source_(source)
{
}

bool CancellationToken::is_cancellation_requested() const noexcept
{
  if (source_) {
    return source_->is_cancellation_requested();
  }
  return false;
}

void CancellationToken::throw_if_cancellation_requested() const
{
  if (is_cancellation_requested()) {
    throw CancelledException();
  }
}

std::shared_ptr<void> CancellationToken::register_callback(CancellationCallback callback)
{
  if (source_) {
    return source_->register_callback(std::move(callback));
  }
  return std::shared_ptr<void>();
}

// AsyncExecutorHelper implementation
AsyncExecutorHelper::AsyncExecutorHelper(rclcpp::Executor::SharedPtr executor)
: executor_(executor)
{
  // Start worker thread for pending tasks
  worker_thread_ = std::thread([this]() {
      while (running_.load() || !pending_tasks_.empty()) {
        std::function<void()> task;
        {
          std::unique_lock<std::mutex> lock(tasks_mutex_);
          tasks_cv_.wait(lock, [this]() {
              return !running_.load() || !pending_tasks_.empty();
            });

          if (!pending_tasks_.empty()) {
            task = std::move(pending_tasks_.back());
            pending_tasks_.pop_back();
          }
        }

        if (task) {
          task();
        }
      }
    });
}

AsyncExecutorHelper::~AsyncExecutorHelper()
{
  stop();
  tasks_cv_.notify_all();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

void AsyncExecutorHelper::start()
{
  if (running_.exchange(true)) {
    return;  // Already running
  }

  if (executor_) {
    spin_thread_ = std::thread([this]() {
        executor_->spin();
      });
  }
}

void AsyncExecutorHelper::stop()
{
  if (!running_.exchange(false)) {
    return;  // Already stopped
  }

  if (executor_) {
    executor_->cancel();
  }

  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

bool AsyncExecutorHelper::is_running() const
{
  return running_.load();
}

rclcpp::CallbackGroup::SharedPtr AsyncExecutorHelper::create_callback_group(
  rclcpp::Node::SharedPtr node,
  rclcpp::CallbackGroupType type)
{
  return node->create_callback_group(type, false);
}

}  // namespace executors
}  // namespace rclcpp
