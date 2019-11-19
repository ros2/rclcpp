// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/context.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rcl/init.h"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"
#include "rmw/impl/cpp/demangle.hpp"

/// Mutex to protect initialized contexts.
static std::mutex g_contexts_mutex;
/// Weak list of context to be shutdown by the signal handler.
static std::vector<std::weak_ptr<rclcpp::Context>> g_contexts;

using rclcpp::Context;

Context::Context()
: rcl_context_(nullptr), shutdown_reason_("") {}

Context::~Context()
{
  // acquire the init lock to prevent race conditions with init and shutdown
  // this will not prevent errors, but will maybe make them easier to reproduce
  std::lock_guard<std::recursive_mutex> lock(init_mutex_);
  try {
    this->shutdown("context destructor was called while still not shutdown");
    // at this point it is shutdown and cannot reinit
    // clean_up will finalize the rcl context
    this->clean_up();
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "unhandled exception in ~Context(): %s", exc.what());
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "unhandled exception in ~Context()");
  }
}

RCLCPP_LOCAL
void
__delete_context(rcl_context_t * context)
{
  if (context) {
    if (rcl_context_is_valid(context)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rclcpp"), "rcl context unexpectedly not shutdown during cleanup");
    } else {
      // if context pointer is not null and is shutdown, then it's ready for fini
      rcl_ret_t ret = rcl_context_fini(context);
      if (RCL_RET_OK != ret) {
        RCLCPP_ERROR(
          rclcpp::get_logger("rclcpp"),
          "failed to finalize context: %s", rcl_get_error_string().str);
        rcl_reset_error();
      }
    }
    delete context;
  }
}

void
Context::init(
  int argc,
  char const * const argv[],
  const rclcpp::InitOptions & init_options)
{
  std::lock_guard<std::recursive_mutex> init_lock(init_mutex_);
  if (this->is_valid()) {
    throw rclcpp::ContextAlreadyInitialized();
  }
  this->clean_up();
  rcl_context_.reset(new rcl_context_t, __delete_context);
  *rcl_context_.get() = rcl_get_zero_initialized_context();
  rcl_ret_t ret = rcl_init(argc, argv, init_options.get_rcl_init_options(), rcl_context_.get());
  if (RCL_RET_OK != ret) {
    rcl_context_.reset();
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to initialize rcl");
  }
  init_options_ = init_options;

  std::lock_guard<std::mutex> lock(g_contexts_mutex);
  g_contexts.push_back(this->shared_from_this());
}

bool
Context::is_valid() const
{
  // Take a local copy of the shared pointer to avoid it getting nulled under our feet.
  auto local_rcl_context = rcl_context_;
  if (!local_rcl_context) {
    return false;
  }
  return rcl_context_is_valid(local_rcl_context.get());
}

const rclcpp::InitOptions &
Context::get_init_options() const
{
  return init_options_;
}

rclcpp::InitOptions
Context::get_init_options()
{
  return init_options_;
}

std::string
Context::shutdown_reason()
{
  std::lock_guard<std::recursive_mutex> lock(init_mutex_);
  return shutdown_reason_;
}

bool
Context::shutdown(const std::string & reason)
{
  // prevent races
  std::lock_guard<std::recursive_mutex> init_lock(init_mutex_);
  // ensure validity
  if (!this->is_valid()) {
    // if it is not valid, then it cannot be shutdown
    return false;
  }
  // rcl shutdown
  rcl_ret_t ret = rcl_shutdown(rcl_context_.get());
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
  // set shutdown reason
  shutdown_reason_ = reason;
  // call each shutdown callback
  for (const auto & callback : on_shutdown_callbacks_) {
    callback();
  }
  // interrupt all blocking sleep_for() and all blocking executors or wait sets
  this->interrupt_all_sleep_for();
  this->interrupt_all_wait_sets();
  // remove self from the global contexts
  std::lock_guard<std::mutex> context_lock(g_contexts_mutex);
  for (auto it = g_contexts.begin(); it != g_contexts.end(); ) {
    auto shared_context = it->lock();
    if (shared_context.get() == this) {
      it = g_contexts.erase(it);
      break;
    } else {
      ++it;
    }
  }
  return true;
}

rclcpp::Context::OnShutdownCallback
Context::on_shutdown(OnShutdownCallback callback)
{
  on_shutdown_callbacks_.push_back(callback);
  return callback;
}

const std::vector<rclcpp::Context::OnShutdownCallback> &
Context::get_on_shutdown_callbacks() const
{
  return on_shutdown_callbacks_;
}

std::vector<rclcpp::Context::OnShutdownCallback> &
Context::get_on_shutdown_callbacks()
{
  return on_shutdown_callbacks_;
}

std::shared_ptr<rcl_context_t>
Context::get_rcl_context()
{
  return rcl_context_;
}

bool
Context::sleep_for(const std::chrono::nanoseconds & nanoseconds)
{
  std::chrono::nanoseconds time_left = nanoseconds;
  {
    std::unique_lock<std::mutex> lock(interrupt_mutex_);
    auto start = std::chrono::steady_clock::now();
    // this will release the lock while waiting
    interrupt_condition_variable_.wait_for(lock, nanoseconds);
    time_left -= std::chrono::steady_clock::now() - start;
  }
  if (time_left > std::chrono::nanoseconds::zero() && this->is_valid()) {
    return sleep_for(time_left);
  }
  // Return true if the timeout elapsed successfully, otherwise false.
  return this->is_valid();
}

void
Context::interrupt_all_sleep_for()
{
  interrupt_condition_variable_.notify_all();
}

rcl_guard_condition_t *
Context::get_interrupt_guard_condition(rcl_wait_set_t * wait_set)
{
  std::lock_guard<std::mutex> lock(interrupt_guard_cond_handles_mutex_);
  auto kv = interrupt_guard_cond_handles_.find(wait_set);
  if (kv != interrupt_guard_cond_handles_.end()) {
    return &kv->second;
  } else {
    rcl_guard_condition_t handle = rcl_get_zero_initialized_guard_condition();
    rcl_guard_condition_options_t options = rcl_guard_condition_get_default_options();
    auto ret = rcl_guard_condition_init(&handle, this->get_rcl_context().get(), options);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "Couldn't initialize guard condition");
    }
    interrupt_guard_cond_handles_.emplace(wait_set, handle);
    return &interrupt_guard_cond_handles_[wait_set];
  }
}

void
Context::release_interrupt_guard_condition(rcl_wait_set_t * wait_set)
{
  std::lock_guard<std::mutex> lock(interrupt_guard_cond_handles_mutex_);
  auto kv = interrupt_guard_cond_handles_.find(wait_set);
  if (kv != interrupt_guard_cond_handles_.end()) {
    rcl_ret_t ret = rcl_guard_condition_fini(&kv->second);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to destroy sigint guard condition");
    }
    interrupt_guard_cond_handles_.erase(kv);
  } else {
    throw std::runtime_error("Tried to release sigint guard condition for nonexistent wait set");
  }
}

void
Context::release_interrupt_guard_condition(
  rcl_wait_set_t * wait_set,
  const std::nothrow_t &) noexcept
{
  try {
    this->release_interrupt_guard_condition(wait_set);
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(
      rclcpp::get_logger("rclcpp"),
      "caught %s exception when releasing interrupt guard condition: %s",
      rmw::impl::cpp::demangle(exc).c_str(), exc.what());
  } catch (...) {
    RCLCPP_ERROR(
      rclcpp::get_logger("rclcpp"),
      "caught unknown exception when releasing interrupt guard condition");
  }
}

void
Context::interrupt_all_wait_sets()
{
  std::lock_guard<std::mutex> lock(interrupt_guard_cond_handles_mutex_);
  for (auto & kv : interrupt_guard_cond_handles_) {
    rcl_ret_t status = rcl_trigger_guard_condition(&(kv.second));
    if (status != RCL_RET_OK) {
      RCUTILS_LOG_ERROR_NAMED(
        "rclcpp",
        "failed to trigger guard condition in Context::interrupt_all_wait_sets(): %s",
        rcl_get_error_string().str);
    }
  }
}

void
Context::clean_up()
{
  shutdown_reason_ = "";
  rcl_context_.reset();
}

std::vector<Context::SharedPtr>
rclcpp::get_contexts()
{
  std::lock_guard<std::mutex> lock(g_contexts_mutex);
  std::vector<Context::SharedPtr> shared_contexts;
  for (auto it = g_contexts.begin(); it != g_contexts.end(); /* noop */) {
    auto context_ptr = it->lock();
    if (!context_ptr) {
      // remove invalid weak context pointers
      it = g_contexts.erase(it);
    } else {
      ++it;
      shared_contexts.push_back(context_ptr);
    }
  }
  return shared_contexts;
}
