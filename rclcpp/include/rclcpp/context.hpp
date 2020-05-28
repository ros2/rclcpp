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

  /// Return the shutdown reason, or empty string if not shutdown.
  /**
   * This function is thread-safe.
   */
  RCLCPP_PUBLIC
  std::string
  shutdown_reason();

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

  using OnShutdownCallback = std::function<void ()>;

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

  /// Return the shutdown callbacks as const.
  /**
   * Using the returned reference is not thread-safe with calls that modify
   * the list of "on shutdown" callbacks, i.e. on_shutdown().
   */
  RCLCPP_PUBLIC
  const std::vector<OnShutdownCallback> &
  get_on_shutdown_callbacks() const;

  /// Return the shutdown callbacks.
  /**
   * Using the returned reference is not thread-safe with calls that modify
   * the list of "on shutdown" callbacks, i.e. on_shutdown().
   */
  RCLCPP_PUBLIC
  std::vector<OnShutdownCallback> &
  get_on_shutdown_callbacks();

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

  /// Get a handle to the guard condition which is triggered when interrupted.
  /**
   * This guard condition is triggered any time interrupt_all_wait_sets() is
   * called, which may be called by the user, or shutdown().
   * And in turn, shutdown() may be called by the user, the destructor of this
   * context, or the signal handler if installed and shutdown_on_sigint is true
   * for this context.
   *
   * The first time that this function is called for a given wait set a new guard
   * condition will be created and returned; thereafter the same guard condition
   * will be returned for the same wait set.
   * This mechanism is designed to ensure that the same guard condition is not
   * reused across wait sets (e.g., when using multiple executors in the same
   * process).
   * This method will throw an exception if initialization of the guard
   * condition fails.
   *
   * The returned guard condition needs to be released with the
   * release_interrupt_guard_condition() method in order to reclaim resources.
   *
   * \param[in] wait_set Pointer to the rcl_wait_set_t that will be using the
   *   resulting guard condition.
   * \return Pointer to the guard condition.
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  RCLCPP_PUBLIC
  rcl_guard_condition_t *
  get_interrupt_guard_condition(rcl_wait_set_t * wait_set);

  /// Release the previously allocated guard condition which is triggered when interrupted.
  /**
   * If you previously called get_interrupt_guard_condition() for a given wait
   * set to get a interrupt guard condition, then you should call
   * release_interrupt_guard_condition() when you're done, to free that
   * condition.
   * Will throw an exception if get_interrupt_guard_condition() wasn't
   * previously called for the given wait set.
   *
   * After calling this, the pointer returned by get_interrupt_guard_condition()
   * for the given wait_set is invalid.
   *
   * \param[in] wait_set Pointer to the rcl_wait_set_t that was using the
   *   resulting guard condition.
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   * \throws std::runtime_error if a nonexistent wait set is trying to release sigint guard condition.
   */
  RCLCPP_PUBLIC
  void
  release_interrupt_guard_condition(rcl_wait_set_t * wait_set);

  /// Nothrow version of release_interrupt_guard_condition(), logs to RCLCPP_ERROR instead.
  RCLCPP_PUBLIC
  void
  release_interrupt_guard_condition(rcl_wait_set_t * wait_set, const std::nothrow_t &) noexcept;

  /// Interrupt any blocking executors, or wait sets associated with this context.
  RCLCPP_PUBLIC
  virtual
  void
  interrupt_all_wait_sets();

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
  std::recursive_mutex init_mutex_;
  std::shared_ptr<rcl_context_t> rcl_context_;
  rclcpp::InitOptions init_options_;
  std::string shutdown_reason_;

  // Keep shared ownership of the global logging mutex.
  std::shared_ptr<std::recursive_mutex> logging_mutex_;

  std::unordered_map<std::type_index, std::shared_ptr<void>> sub_contexts_;
  // This mutex is recursive so that the constructor of a sub context may
  // attempt to acquire another sub context.
  std::recursive_mutex sub_contexts_mutex_;

  std::vector<OnShutdownCallback> on_shutdown_callbacks_;
  std::mutex on_shutdown_callbacks_mutex_;

  /// Condition variable for timed sleep (see sleep_for).
  std::condition_variable interrupt_condition_variable_;
  /// Mutex for protecting the global condition variable.
  std::mutex interrupt_mutex_;

  /// Mutex to protect sigint_guard_cond_handles_.
  std::mutex interrupt_guard_cond_handles_mutex_;
  /// Guard conditions for interrupting of associated wait sets on interrupt_all_wait_sets().
  std::unordered_map<rcl_wait_set_t *, rcl_guard_condition_t> interrupt_guard_cond_handles_;

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
