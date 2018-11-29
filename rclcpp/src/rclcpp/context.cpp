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
Context::shutdown(const std::string & reason, bool notify_all)
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
  // notify all blocking calls, if asked
  if (notify_all) {
    rclcpp::notify_all();
  }
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
