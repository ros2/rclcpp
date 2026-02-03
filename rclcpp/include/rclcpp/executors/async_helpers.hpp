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

#ifndef RCLCPP__EXECUTORS__ASYNC_HELPERS_HPP_
#define RCLCPP__EXECUTORS__ASYNC_HELPERS_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <exception>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "rclcpp/callback_group.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace executors
{

// Forward declarations
class CancellationTokenSource;

/// Exception base class for async operations
class AsyncOperationException : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/// Exception thrown when an async operation times out
class TimeoutException : public AsyncOperationException
{
public:
  RCLCPP_PUBLIC
  TimeoutException();

  RCLCPP_PUBLIC
  explicit TimeoutException(std::chrono::milliseconds timeout);

  RCLCPP_PUBLIC
  std::chrono::milliseconds timeout() const noexcept;

private:
  std::chrono::milliseconds timeout_;
};

/// Exception thrown when an async operation is cancelled
class CancelledException : public AsyncOperationException
{
public:
  RCLCPP_PUBLIC
  CancelledException();
};

/// Exception thrown when a service is not ready
class ServiceNotReadyException : public AsyncOperationException
{
public:
  RCLCPP_PUBLIC
  explicit ServiceNotReadyException(const std::string & service_name);

  RCLCPP_PUBLIC
  const std::string & service_name() const noexcept;

private:
  std::string service_name_;
};

/// Token for cooperative cancellation of async operations
class CancellationToken
{
public:
  using CancellationCallback = std::function<void()>;

  RCLCPP_PUBLIC
  CancellationToken();

  RCLCPP_PUBLIC
  bool is_cancellation_requested() const noexcept;

  RCLCPP_PUBLIC
  void throw_if_cancellation_requested() const;

  /// Register a callback to be invoked when cancellation is requested
  /**
   * \param callback The callback to invoke on cancellation
   * \return A handle that can be used to unregister the callback (let it go out of scope)
   */
  RCLCPP_PUBLIC
  std::shared_ptr<void> register_callback(CancellationCallback callback);

private:
  friend class CancellationTokenSource;
  explicit CancellationToken(std::shared_ptr<CancellationTokenSource> source);
  std::shared_ptr<CancellationTokenSource> source_;
};

/// Source for creating cancellation tokens
class CancellationTokenSource
{
public:
  RCLCPP_PUBLIC
  CancellationTokenSource();

  RCLCPP_PUBLIC
  ~CancellationTokenSource();

  RCLCPP_PUBLIC
  CancellationToken get_token() const;

  RCLCPP_PUBLIC
  void cancel();

  RCLCPP_PUBLIC
  bool is_cancellation_requested() const noexcept;

  /// Request cancellation after a specified duration
  /**
   * \param duration Time to wait before requesting cancellation
   */
  RCLCPP_PUBLIC
  void cancel_after(std::chrono::milliseconds duration);

  /// Register callback for cancellation notification (internal use)
  RCLCPP_PUBLIC
  std::shared_ptr<void> register_callback(CancellationToken::CancellationCallback callback);

private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

/// Result wrapper for async operations with status information
template<typename T>
class AsyncResult
{
public:
  enum class Status
  {
    Success,
    Timeout,
    Cancelled,
    Error
  };

  /// Create a successful result
  static AsyncResult success(T value)
  {
    AsyncResult result;
    result.status_ = Status::Success;
    result.data_ = std::move(value);
    return result;
  }

  /// Create a timeout result
  static AsyncResult timeout()
  {
    AsyncResult result;
    result.status_ = Status::Timeout;
    result.data_ = std::make_exception_ptr(TimeoutException());
    return result;
  }

  /// Create a cancelled result
  static AsyncResult cancelled()
  {
    AsyncResult result;
    result.status_ = Status::Cancelled;
    result.data_ = std::make_exception_ptr(CancelledException());
    return result;
  }

  /// Create an error result
  static AsyncResult error(std::exception_ptr ex)
  {
    AsyncResult result;
    result.status_ = Status::Error;
    result.data_ = ex;
    return result;
  }

  Status status() const noexcept
  {
    return status_;
  }

  bool is_success() const noexcept
  {
    return status_ == Status::Success;
  }

  bool is_timeout() const noexcept
  {
    return status_ == Status::Timeout;
  }

  bool is_cancelled() const noexcept
  {
    return status_ == Status::Cancelled;
  }

  bool has_error() const noexcept
  {
    return status_ == Status::Error;
  }

  /// Get the value (throws if not successful)
  T & value()
  {
    if (!is_success()) {
      rethrow_if_error();
      if (is_timeout()) {
        throw TimeoutException();
      }
      if (is_cancelled()) {
        throw CancelledException();
      }
    }
    return std::get<T>(data_);
  }

  const T & value() const
  {
    if (!is_success()) {
      rethrow_if_error();
      if (is_timeout()) {
        throw TimeoutException();
      }
      if (is_cancelled()) {
        throw CancelledException();
      }
    }
    return std::get<T>(data_);
  }

  /// Get the value or a default if not successful
  T value_or(T default_value) const
  {
    if (is_success()) {
      return std::get<T>(data_);
    }
    return default_value;
  }

  /// Get the stored exception (nullptr if successful)
  std::exception_ptr get_exception() const noexcept
  {
    if (std::holds_alternative<std::exception_ptr>(data_)) {
      return std::get<std::exception_ptr>(data_);
    }
    return nullptr;
  }

  /// Rethrow the stored exception if present
  void rethrow_if_error() const
  {
    if (std::holds_alternative<std::exception_ptr>(data_)) {
      auto ex = std::get<std::exception_ptr>(data_);
      if (ex) {
        std::rethrow_exception(ex);
      }
    }
  }

private:
  AsyncResult() = default;
  Status status_ = Status::Error;
  std::variant<T, std::exception_ptr> data_;
};

/// Specialization for void type
template<>
class AsyncResult<void>
{
public:
  enum class Status
  {
    Success,
    Timeout,
    Cancelled,
    Error
  };

  static AsyncResult success()
  {
    AsyncResult result;
    result.status_ = Status::Success;
    return result;
  }

  static AsyncResult timeout()
  {
    AsyncResult result;
    result.status_ = Status::Timeout;
    result.error_ = std::make_exception_ptr(TimeoutException());
    return result;
  }

  static AsyncResult cancelled()
  {
    AsyncResult result;
    result.status_ = Status::Cancelled;
    result.error_ = std::make_exception_ptr(CancelledException());
    return result;
  }

  static AsyncResult error(std::exception_ptr ex)
  {
    AsyncResult result;
    result.status_ = Status::Error;
    result.error_ = ex;
    return result;
  }

  Status status() const noexcept { return status_; }
  bool is_success() const noexcept { return status_ == Status::Success; }
  bool is_timeout() const noexcept { return status_ == Status::Timeout; }
  bool is_cancelled() const noexcept { return status_ == Status::Cancelled; }
  bool has_error() const noexcept { return status_ == Status::Error; }

  void rethrow_if_error() const
  {
    if (error_) {
      std::rethrow_exception(error_);
    }
  }

private:
  AsyncResult() = default;
  Status status_ = Status::Error;
  std::exception_ptr error_;
};

/// Async wrapper for ROS2 service clients
/**
 * Provides enhanced async functionality for service clients including:
 * - Timeout support
 * - Cancellation support
 * - Future and callback-based interfaces
 */
template<typename ServiceT>
class AsyncServiceClient
{
public:
  using Request = typename ServiceT::Request;
  using Response = typename ServiceT::Response;
  using RequestSharedPtr = typename Request::SharedPtr;
  using ResponseSharedPtr = typename Response::SharedPtr;
  using SharedFutureResponse = std::shared_future<ResponseSharedPtr>;

  /// Create an AsyncServiceClient from a node
  /**
   * \param node The ROS node to create the client on
   * \param service_name Name of the service to connect to
   * \param qos Quality of service settings
   */
  RCLCPP_PUBLIC
  AsyncServiceClient(
    rclcpp::Node::SharedPtr node,
    const std::string & service_name,
    const rclcpp::QoS & qos = rclcpp::ServicesQoS())
  : node_(node)
  {
    callback_group_ = node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);

    rclcpp::SubscriptionOptions options;
    client_ = node->create_client<ServiceT>(
      service_name,
      qos.get_rmw_qos_profile(),
      callback_group_);
  }

  /// Create an AsyncServiceClient from an existing client
  /**
   * \param client Existing service client to wrap
   */
  RCLCPP_PUBLIC
  explicit AsyncServiceClient(typename rclcpp::Client<ServiceT>::SharedPtr client)
  : client_(client)
  {
  }

  /// Send a request asynchronously
  /**
   * \param request The request to send
   * \return Future that will be fulfilled with the response
   */
  std::future<ResponseSharedPtr>
  async_send_request(RequestSharedPtr request)
  {
    auto promise = std::make_shared<std::promise<ResponseSharedPtr>>();
    auto future = promise->get_future();

    client_->async_send_request(
      request,
      [promise](SharedFutureResponse response_future) {
        try {
          promise->set_value(response_future.get());
        } catch (...) {
          promise->set_exception(std::current_exception());
        }
      });

    return future;
  }

  /// Send a request with timeout
  /**
   * \param request The request to send
   * \param timeout Maximum time to wait for response
   * \return Future with AsyncResult containing response or timeout/error status
   */
  std::future<AsyncResult<ResponseSharedPtr>>
  async_send_request_with_timeout(
    RequestSharedPtr request,
    std::chrono::milliseconds timeout)
  {
    auto promise = std::make_shared<std::promise<AsyncResult<ResponseSharedPtr>>>();
    auto future = promise->get_future();

    auto timeout_flag = std::make_shared<std::atomic<bool>>(false);
    auto completed_flag = std::make_shared<std::atomic<bool>>(false);
    auto mutex = std::make_shared<std::mutex>();

    // Start timeout thread
    std::thread timeout_thread([promise, timeout, timeout_flag, completed_flag, mutex]() {
        std::this_thread::sleep_for(timeout);
        std::lock_guard<std::mutex> lock(*mutex);
        if (!completed_flag->exchange(true)) {
          timeout_flag->store(true);
          promise->set_value(AsyncResult<ResponseSharedPtr>::timeout());
        }
      });
    timeout_thread.detach();

    client_->async_send_request(
      request,
      [promise, timeout_flag, completed_flag, mutex](SharedFutureResponse response_future) {
        std::lock_guard<std::mutex> lock(*mutex);
        if (!completed_flag->exchange(true)) {
          try {
            promise->set_value(AsyncResult<ResponseSharedPtr>::success(response_future.get()));
          } catch (...) {
            promise->set_value(AsyncResult<ResponseSharedPtr>::error(std::current_exception()));
          }
        }
      });

    return future;
  }

  /// Send a request with cancellation support
  /**
   * \param request The request to send
   * \param token Cancellation token to monitor
   * \return Future with AsyncResult containing response or cancellation status
   */
  std::future<AsyncResult<ResponseSharedPtr>>
  async_send_request_with_cancellation(
    RequestSharedPtr request,
    CancellationToken token)
  {
    auto promise = std::make_shared<std::promise<AsyncResult<ResponseSharedPtr>>>();
    auto future = promise->get_future();

    auto completed_flag = std::make_shared<std::atomic<bool>>(false);
    auto mutex = std::make_shared<std::mutex>();

    // Register cancellation callback
    auto cancel_handle = token.register_callback([promise, completed_flag, mutex]() {
        std::lock_guard<std::mutex> lock(*mutex);
        if (!completed_flag->exchange(true)) {
          promise->set_value(AsyncResult<ResponseSharedPtr>::cancelled());
        }
      });

    client_->async_send_request(
      request,
      [promise, completed_flag, mutex, cancel_handle](SharedFutureResponse response_future) {
        std::lock_guard<std::mutex> lock(*mutex);
        if (!completed_flag->exchange(true)) {
          try {
            promise->set_value(AsyncResult<ResponseSharedPtr>::success(response_future.get()));
          } catch (...) {
            promise->set_value(AsyncResult<ResponseSharedPtr>::error(std::current_exception()));
          }
        }
      });

    return future;
  }

  /// Send a request with both timeout and cancellation support
  /**
   * \param request The request to send
   * \param timeout Maximum time to wait for response
   * \param token Cancellation token to monitor
   * \return Future with AsyncResult containing response or timeout/cancellation status
   */
  std::future<AsyncResult<ResponseSharedPtr>>
  async_send_request(
    RequestSharedPtr request,
    std::chrono::milliseconds timeout,
    CancellationToken token)
  {
    auto promise = std::make_shared<std::promise<AsyncResult<ResponseSharedPtr>>>();
    auto future = promise->get_future();

    auto completed_flag = std::make_shared<std::atomic<bool>>(false);
    auto mutex = std::make_shared<std::mutex>();

    // Register cancellation callback
    auto cancel_handle = token.register_callback([promise, completed_flag, mutex]() {
        std::lock_guard<std::mutex> lock(*mutex);
        if (!completed_flag->exchange(true)) {
          promise->set_value(AsyncResult<ResponseSharedPtr>::cancelled());
        }
      });

    // Start timeout thread
    std::thread timeout_thread([promise, timeout, completed_flag, mutex]() {
        std::this_thread::sleep_for(timeout);
        std::lock_guard<std::mutex> lock(*mutex);
        if (!completed_flag->exchange(true)) {
          promise->set_value(AsyncResult<ResponseSharedPtr>::timeout());
        }
      });
    timeout_thread.detach();

    client_->async_send_request(
      request,
      [promise, completed_flag, mutex, cancel_handle](SharedFutureResponse response_future) {
        std::lock_guard<std::mutex> lock(*mutex);
        if (!completed_flag->exchange(true)) {
          try {
            promise->set_value(AsyncResult<ResponseSharedPtr>::success(response_future.get()));
          } catch (...) {
            promise->set_value(AsyncResult<ResponseSharedPtr>::error(std::current_exception()));
          }
        }
      });

    return future;
  }

  /// Callback-based interface for sending requests
  using ResponseCallback = std::function<void(AsyncResult<ResponseSharedPtr>)>;

  void async_send_request(
    RequestSharedPtr request,
    ResponseCallback callback,
    std::chrono::milliseconds timeout = std::chrono::milliseconds::max())
  {
    auto completed_flag = std::make_shared<std::atomic<bool>>(false);
    auto mutex = std::make_shared<std::mutex>();

    if (timeout != std::chrono::milliseconds::max()) {
      std::thread timeout_thread([callback, timeout, completed_flag, mutex]() {
          std::this_thread::sleep_for(timeout);
          std::lock_guard<std::mutex> lock(*mutex);
          if (!completed_flag->exchange(true)) {
            callback(AsyncResult<ResponseSharedPtr>::timeout());
          }
        });
      timeout_thread.detach();
    }

    client_->async_send_request(
      request,
      [callback, completed_flag, mutex](SharedFutureResponse response_future) {
        std::lock_guard<std::mutex> lock(*mutex);
        if (!completed_flag->exchange(true)) {
          try {
            callback(AsyncResult<ResponseSharedPtr>::success(response_future.get()));
          } catch (...) {
            callback(AsyncResult<ResponseSharedPtr>::error(std::current_exception()));
          }
        }
      });
  }

  /// Check if the service is ready
  bool service_is_ready() const
  {
    return client_->service_is_ready();
  }

  /// Wait for the service to become available
  /**
   * \param timeout Maximum time to wait
   * \return Future that resolves to true if service is ready, false if timed out
   */
  std::future<bool> wait_for_service(std::chrono::milliseconds timeout)
  {
    return std::async(std::launch::async, [this, timeout]() {
        return client_->wait_for_service(timeout);
      });
  }

  /// Get the underlying client
  typename rclcpp::Client<ServiceT>::SharedPtr get_client() const
  {
    return client_;
  }

private:
  typename rclcpp::Client<ServiceT>::SharedPtr client_;
  rclcpp::Node::WeakPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
};

/// Utility functions for async operations
namespace async_utils
{

/// Add timeout to any future
/**
 * \param future The future to add timeout to
 * \param timeout Maximum time to wait
 * \return Future with AsyncResult containing value or timeout status
 */
template<typename T>
std::future<AsyncResult<T>> with_timeout(
  std::future<T> future,
  std::chrono::milliseconds timeout)
{
  return std::async(
    std::launch::async,
    [future = std::move(future), timeout]() mutable -> AsyncResult<T> {
      auto status = future.wait_for(timeout);
      if (status == std::future_status::timeout) {
        return AsyncResult<T>::timeout();
      }
      try {
        return AsyncResult<T>::success(future.get());
      } catch (...) {
        return AsyncResult<T>::error(std::current_exception());
      }
    });
}

/// Wait for all futures to complete
template<typename... Ts>
std::future<std::tuple<Ts...>> when_all(std::future<Ts>... futures)
{
  return std::async(
    std::launch::async,
    [... futures = std::move(futures)]() mutable {
      return std::make_tuple(futures.get()...);
    });
}

/// Wait for any future to complete (vector version)
/**
 * \param futures Vector of futures to wait on
 * \return Future with pair of (index of completed future, value)
 */
template<typename T>
std::future<std::pair<size_t, T>> when_any(std::vector<std::future<T>> futures)
{
  auto promise = std::make_shared<std::promise<std::pair<size_t, T>>>();
  auto result = promise->get_future();
  auto completed = std::make_shared<std::atomic<bool>>(false);

  for (size_t i = 0; i < futures.size(); ++i) {
    std::thread(
      [i, promise, completed](std::future<T> f) {
        try {
          T value = f.get();
          if (!completed->exchange(true)) {
            promise->set_value(std::make_pair(i, std::move(value)));
          }
        } catch (...) {
          // Only set exception if we're the first to complete
          if (!completed->exchange(true)) {
            promise->set_exception(std::current_exception());
          }
        }
      },
      std::move(futures[i])).detach();
  }

  return result;
}

/// Retry an operation with exponential backoff
/**
 * \param callable The operation to retry (must return T)
 * \param max_attempts Maximum number of attempts
 * \param initial_delay Initial delay between attempts
 * \param backoff_multiplier Multiplier for exponential backoff
 * \return Future with AsyncResult containing success or last error
 */
template<typename T, typename Callable>
std::future<AsyncResult<T>> with_retry(
  Callable callable,
  size_t max_attempts,
  std::chrono::milliseconds initial_delay,
  double backoff_multiplier = 2.0)
{
  return std::async(
    std::launch::async,
    [callable, max_attempts, initial_delay, backoff_multiplier]() -> AsyncResult<T> {
      std::chrono::milliseconds delay = initial_delay;
      std::exception_ptr last_error;

      for (size_t attempt = 0; attempt < max_attempts; ++attempt) {
        try {
          return AsyncResult<T>::success(callable());
        } catch (...) {
          last_error = std::current_exception();
          if (attempt + 1 < max_attempts) {
            std::this_thread::sleep_for(delay);
            delay = std::chrono::milliseconds(
              static_cast<long>(delay.count() * backoff_multiplier));
          }
        }
      }

      return AsyncResult<T>::error(last_error);
    });
}

/// Transform a future's value
/**
 * \param future The source future
 * \param transformer Function to transform the value
 * \return Future with transformed value
 */
template<typename T, typename U, typename Func>
std::future<U> then(std::future<T> future, Func transformer)
{
  return std::async(
    std::launch::async,
    [future = std::move(future), transformer]() mutable -> U {
      return transformer(future.get());
    });
}

}  // namespace async_utils

/// Helper class for managing async executor operations
class AsyncExecutorHelper
{
public:
  RCLCPP_PUBLIC
  explicit AsyncExecutorHelper(rclcpp::Executor::SharedPtr executor = nullptr);

  RCLCPP_PUBLIC
  ~AsyncExecutorHelper();

  /// Start background spinning (if no external executor provided)
  RCLCPP_PUBLIC
  void start();

  /// Stop background spinning
  RCLCPP_PUBLIC
  void stop();

  /// Check if the helper is running
  RCLCPP_PUBLIC
  bool is_running() const;

  /// Run an async operation
  template<typename Callable, typename... Args>
  auto run_async(Callable && callable, Args &&... args)
    -> std::future<std::invoke_result_t<Callable, Args...>>
  {
    using ReturnType = std::invoke_result_t<Callable, Args...>;
    auto task = std::make_shared<std::packaged_task<ReturnType()>>(
      std::bind(std::forward<Callable>(callable), std::forward<Args>(args)...));
    auto future = task->get_future();

    {
      std::lock_guard<std::mutex> lock(tasks_mutex_);
      pending_tasks_.push_back([task]() { (*task)(); });
    }
    tasks_cv_.notify_one();

    return future;
  }

  /// Schedule an operation to run after a delay
  template<typename Callable>
  void schedule_after(std::chrono::milliseconds delay, Callable && callable)
  {
    std::thread(
      [delay, callable = std::forward<Callable>(callable)]() {
        std::this_thread::sleep_for(delay);
        callable();
      }).detach();
  }

  /// Schedule periodic execution of an operation
  /**
   * \param period Time between executions
   * \param callable The operation to execute
   * \return Handle to stop periodic execution (let go out of scope to stop)
   */
  template<typename Callable>
  std::shared_ptr<void> schedule_periodic(
    std::chrono::milliseconds period,
    Callable && callable)
  {
    auto stop_flag = std::make_shared<std::atomic<bool>>(false);

    std::thread(
      [period, callable, stop_flag]() {
        while (!stop_flag->load()) {
          callable();
          std::this_thread::sleep_for(period);
        }
      }).detach();

    // Return a shared_ptr that sets stop_flag when destroyed
    return std::shared_ptr<void>(nullptr, [stop_flag](void *) {
        stop_flag->store(true);
      });
  }

  /// Create a callback group for async operations
  RCLCPP_PUBLIC
  rclcpp::CallbackGroup::SharedPtr create_callback_group(
    rclcpp::Node::SharedPtr node,
    rclcpp::CallbackGroupType type = rclcpp::CallbackGroupType::Reentrant);

  /// Factory method for creating async service clients
  template<typename ServiceT>
  AsyncServiceClient<ServiceT> create_async_service_client(
    rclcpp::Node::SharedPtr node,
    const std::string & service_name)
  {
    return AsyncServiceClient<ServiceT>(node, service_name);
  }

private:
  rclcpp::Executor::SharedPtr executor_;
  std::thread spin_thread_;
  std::atomic<bool> running_{false};

  std::mutex tasks_mutex_;
  std::condition_variable tasks_cv_;
  std::vector<std::function<void()>> pending_tasks_;
  std::thread worker_thread_;
};

}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__ASYNC_HELPERS_HPP_
