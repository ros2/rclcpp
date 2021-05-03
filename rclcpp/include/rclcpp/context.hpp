// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__CONTEXT_HPP_
#define RCLCPP__CONTEXT_HPP_

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rcl/context.h"
#include "rcl/guard_condition.h"
#include "rcl/wait.h"
#include "rclcpp/init_options.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/// Thrown when init is called on an already initialized context.
class ContextAlreadyInitialized : public std::runtime_error
{
public:
  ContextAlreadyInitialized()
  : std::runtime_error("context is already initialized") {}
};

/// Forward declare WeakContextsWrapper
class WeakContextsWrapper;

class OnShutdownCallbackHandle
{
  friend class Context;

public:
  using OnShutdownCallbackType = std::function<void ()>;

private:
  std::weak_ptr<OnShutdownCallbackType> callback;
};

/// Context which encapsulates shared state between nodes and other similar entities.
/**
 * A context also represents the lifecycle between init and shutdown of rclcpp.
 * It is often used in conjunction with rclcpp::init, or rclcpp::init_local,
 * and rclcpp::shutdown.
 */
class Context : public std::enable_shared_from_this<Context>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Context)

  /// Default constructor, after which the Context is still not "initialized".
  /**
   * Every context which is constructed is added to a global vector of contexts,
   * which is used by the signal handler to conditionally shutdown each context
   * on SIGINT.
   * See the shutdown_on_sigint option in the InitOptions class.
   */
  RCLCPP_PUBLIC
  Context();

  RCLCPP_PUBLIC
  virtual
  ~Context();

  /// Initialize the context, and the underlying elements like the rcl context.
  /**
   * This method must be called before passing this context to things like the
   * constructor of Node.
   * It must also be called before trying to shutdown the context.
   *
   * Note that this function does not setup any signal handlers, so if you want
   * it to be shutdown by the signal handler, then you need to either install
   * them manually with rclcpp::install_signal_handers() or use rclcpp::init().
   * In addition to installing the signal handlers, the shutdown_on_sigint
   * of the InitOptions needs to be `true` for this context to be shutdown by
   * the signal handler, otherwise it will be passed over.
   *
   * After calling this method, shutdown() can be called to invalidate the
   * context for derived entities, e.g. nodes, guard conditions, etc.
   * However, the underlying rcl context is not finalized until this Context's
   * destructor is called or this function is called again.
   * Allowing this class to go out of scope and get destructed or calling this
   * function a second time while derived entities are still using the context
   * is undefined behavior and should be avoided.
   * It's a good idea to not reuse context objects and instead create a new one
   * each time you need to shutdown and init again.
   * This allows derived entities to hold on to shard pointers to the first
   * context object until they are done.
   *
   * This function is thread-safe.
   *
   * \param[in] argc number of arguments
   * \param[in] argv argument array which may contain arguments intended for ROS
   * \param[in] init_options initialization options for rclcpp and underlying layers
   * \throw ContextAlreadyInitialized if called if init is called more than once
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   * \throws std::runtime_error if the global logging configure mutex is NULL
   * \throws exceptions::UnknownROSArgsError if there are unknown ROS arguments
   */
  RCLCPP_PUBLIC
  virtual
  void
  init(
    int argc,
    char const * const argv[],
    const rclcpp::InitOptions & init_options = rclcpp::InitOptions());

  /// Return true if the context is valid, otherwise false.
  /**
   * The context is valid if it has been initialized but not shutdown.
   *
   * This function is thread-safe.
   * This function is lock free so long as pointers and uint64_t atomics are
   * lock free.
   *
   * \return true if valid, otherwise false
   */
  RCLCPP_PUBLIC
  bool
  is_valid() const;

  /// Return the init options used during init.
  RCLCPP_PUBLIC
  const rclcpp::InitOptions &
  get_init_options() const;

  /// Return a copy of the init options used during init.
  RCLCPP_PUBLIC
  rclcpp::InitOptions
  get_init_options();

  /// Return actual domain id.
  RCLCPP_PUBLIC
  size_t
  get_domain_id() const;

  /// Return the shutdown reason, or empty string if not shutdown.
  /**
   * This function is thread-safe.
   */
  RCLCPP_PUBLIC
  std::string
  shutdown_reason() const;

  /// Shutdown the context, making it uninitialized and therefore invalid for derived entities.
  /**
   * Several things happen when the context is shutdown, in this order:
   *
   * - acquires a lock to prevent race conditions with init, on_shutdown, etc.
   * - if the context is not initialized, return false
   * - rcl_shutdown() is called on the internal rcl_context_t instance
   * - the shutdown reason is set
   * - each on_shutdown callback is called, in the order that they were added
   * - interrupt blocking sleep_for() calls, so they return early due to shutdown
   * - interrupt blocking executors and wait sets
   *
   * The underlying rcl context is not finalized by this function.
   *
   * This function is thread-safe.
   *
   * \param[in] reason the description of why shutdown happened
   * \return true if shutdown was successful, false if context was already shutdown
   * \throw various exceptions derived from rclcpp::exceptions::RCLError, if rcl_shutdown fails
   */
  RCLCPP_PUBLIC
  virtual
  bool
  shutdown(const std::string & reason);

  using OnShutdownCallback = OnShutdownCallbackHandle::OnShutdownCallbackType;

  /// Add a on_shutdown callback to be called when shutdown is called for this context.
  /**
   * These callbacks will be called in the order they are added as the second
   * to last step in shutdown().
   *
   * When shutdown occurs due to the signal handler, these callbacks are run
   * asynchronoulsy in the dedicated singal handling thread.
   *
   * Also, shutdown() may be called from the destructor of this function.
   * Therefore, it is not safe to throw exceptions from these callbacks.
   * Instead, log errors or use some other mechanism to indicate an error has
   * occurred.
   *
   * On shutdown callbacks may be registered before init and after shutdown,
   * and persist on repeated init's.
   *
   * \param[in] callback the on shutdown callback to be registered
   * \return the callback passed, for convenience when storing a passed lambda
   */
  RCLCPP_PUBLIC
  virtual
  OnShutdownCallback
  on_shutdown(OnShutdownCallback callback);

  /// Add a on_shutdown callback to be called when shutdown is called for this context.
  /**
   * These callbacks will be called in the order they are added as the second
   * to last step in shutdown().
   *
   * When shutdown occurs due to the signal handler, these callbacks are run
   * asynchronously in the dedicated singal handling thread.
   *
   * Also, shutdown() may be called from the destructor of this function.
   * Therefore, it is not safe to throw exceptions from these callbacks.
   * Instead, log errors or use some other mechanism to indicate an error has
   * occurred.
   *
   * On shutdown callbacks may be registered before init and after shutdown,
   * and persist on repeated init's.
   *
   * \param[in] callback the on_shutdown callback to be registered
   * \return the created callback handle
   */
  RCLCPP_PUBLIC
  virtual
  OnShutdownCallbackHandle
  add_on_shutdown_callback(OnShutdownCallback callback);

  /// Remove an registered on_shutdown callbacks.
  /**
   * \param[in] callback_handle the on_shutdown callback handle to be removed.
   * \return true if the callback is found and removed, otherwise false.
   */
  RCLCPP_PUBLIC
  virtual
  bool
  remove_on_shutdown_callback(const OnShutdownCallbackHandle & callback_handle);

  /// Return the shutdown callbacks.
  /**
   * Returned callbacks are a copy of the registered callbacks.
   */
  RCLCPP_PUBLIC
  std::vector<OnShutdownCallback>
  get_on_shutdown_callbacks() const;

  /// Return the internal rcl context.
  RCLCPP_PUBLIC
  std::shared_ptr<rcl_context_t>
  get_rcl_context();

  /// Sleep for a given period of time or until shutdown() is called.
  /**
   * This function can be interrupted early if:
   *
   *   - this context is shutdown()
   *   - this context is destructed (resulting in shutdown)
   *   - this context has shutdown_on_sigint=true and SIGINT occurs (resulting in shutdown)
   *   - interrupt_all_sleep_for() is called
   *
   * \param[in] nanoseconds A std::chrono::duration representing how long to sleep for.
   * \return true if the condition variable did not timeout, i.e. you were interrupted.
   */
  RCLCPP_PUBLIC
  bool
  sleep_for(const std::chrono::nanoseconds & nanoseconds);

  /// Interrupt any blocking sleep_for calls, causing them to return immediately and return true.
  RCLCPP_PUBLIC
  virtual
  void
  interrupt_all_sleep_for();

  /// Return a singleton instance for the SubContext type, constructing one if necessary.
  template<typename SubContext, typename ... Args>
  std::shared_ptr<SubContext>
  get_sub_context(Args && ... args)
  {
    std::lock_guard<std::recursive_mutex> lock(sub_contexts_mutex_);

    std::type_index type_i(typeid(SubContext));
    std::shared_ptr<SubContext> sub_context;
    auto it = sub_contexts_.find(type_i);
    if (it == sub_contexts_.end()) {
      // It doesn't exist yet, make it
      sub_context = std::shared_ptr<SubContext>(
        new SubContext(std::forward<Args>(args) ...),
        [](SubContext * sub_context_ptr) {
          delete sub_context_ptr;
        });
      sub_contexts_[type_i] = sub_context;
    } else {
      // It exists, get it out and cast it.
      sub_context = std::static_pointer_cast<SubContext>(it->second);
    }
    return sub_context;
  }

protected:
  // Called by constructor and destructor to clean up by finalizing the
  // shutdown rcl context and preparing for a new init cycle.
  RCLCPP_PUBLIC
  virtual
  void
  clean_up();

private:
  RCLCPP_DISABLE_COPY(Context)

  // This mutex is recursive so that the destructor can ensure atomicity
  // between is_initialized and shutdown.
  mutable std::recursive_mutex init_mutex_;
  std::shared_ptr<rcl_context_t> rcl_context_;
  rclcpp::InitOptions init_options_;
  std::string shutdown_reason_;

  // Keep shared ownership of the global logging mutex.
  std::shared_ptr<std::recursive_mutex> logging_mutex_;

  std::unordered_map<std::type_index, std::shared_ptr<void>> sub_contexts_;
  // This mutex is recursive so that the constructor of a sub context may
  // attempt to acquire another sub context.
  std::recursive_mutex sub_contexts_mutex_;

  std::unordered_set<std::shared_ptr<OnShutdownCallback>> on_shutdown_callbacks_;
  mutable std::mutex on_shutdown_callbacks_mutex_;

  /// Condition variable for timed sleep (see sleep_for).
  std::condition_variable interrupt_condition_variable_;
  /// Mutex for protecting the global condition variable.
  std::mutex interrupt_mutex_;

  /// Keep shared ownership of global vector of weak contexts
  std::shared_ptr<WeakContextsWrapper> weak_contexts_;
};

/// Return a copy of the list of context shared pointers.
/**
 * This function is thread-safe.
 */
RCLCPP_PUBLIC
std::vector<Context::SharedPtr>
get_contexts();

}  // namespace rclcpp

#endif  // RCLCPP__CONTEXT_HPP_
