// Copyright 2015-2020 Open Source Robotics Foundation, Inc.
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
#include <sstream>
#include <string>
#include <vector>
#include <unordered_set>
#include <utility>

#include "rcl/init.h"
#include "rcl/logging.h"

#include "rclcpp/detail/utilities.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"

#include "rcutils/error_handling.h"
#include "rcutils/macros.h"

#include "rcpputils/mutex.hpp"

#include "./logging_mutex.hpp"

using rclcpp::Context;

namespace rclcpp
{
/// Class to manage vector of weak pointers to all created contexts
class WeakContextsWrapper
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(WeakContextsWrapper)

  void
  add_context(const Context::SharedPtr & context)
  {
    std::lock_guard<std::mutex> guard(mutex_);
    weak_contexts_.push_back(context);
  }

  void
  remove_context(const Context * context)
  {
    std::lock_guard<std::mutex> guard(mutex_);
    weak_contexts_.erase(
      std::remove_if(
        weak_contexts_.begin(),
        weak_contexts_.end(),
        [context](const Context::WeakPtr weak_context) {
          auto locked_context = weak_context.lock();
          if (!locked_context) {
            // take advantage and removed expired contexts
            return true;
          }
          return locked_context.get() == context;
        }
      ),
      weak_contexts_.end());
  }

  std::vector<Context::SharedPtr>
  get_contexts()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Context::SharedPtr> shared_contexts;
    for (auto it = weak_contexts_.begin(); it != weak_contexts_.end(); /* noop */) {
      auto context_ptr = it->lock();
      if (!context_ptr) {
        // remove invalid weak context pointers
        it = weak_contexts_.erase(it);
      } else {
        ++it;
        shared_contexts.push_back(context_ptr);
      }
    }
    return shared_contexts;
  }

private:
  std::vector<std::weak_ptr<rclcpp::Context>> weak_contexts_;
  rcpputils::PIMutex mutex_;
};
}  // namespace rclcpp

using rclcpp::WeakContextsWrapper;

/// Global vector of weak pointers to all contexts
static
WeakContextsWrapper::SharedPtr
get_weak_contexts()
{
  static WeakContextsWrapper::SharedPtr weak_contexts = WeakContextsWrapper::make_shared();
  if (!weak_contexts) {
    throw std::runtime_error("weak contexts vector is not valid");
  }
  return weak_contexts;
}

/// Count of contexts that wanted to initialize the logging system.
static
size_t &
get_logging_reference_count()
{
  static size_t ref_count = 0;
  return ref_count;
}

extern "C"
{
static
void
rclcpp_logging_output_handler(
  const rcutils_log_location_t * location,
  int severity, const char * name, rcutils_time_point_value_t timestamp,
  const char * format, va_list * args)
{
  try {
    std::shared_ptr<std::recursive_mutex> logging_mutex;
    logging_mutex = get_global_logging_mutex();
    std::lock_guard<std::recursive_mutex> guard(*logging_mutex);
    return rcl_logging_multiple_output_handler(
      location, severity, name, timestamp, format, args);
  } catch (std::exception & ex) {
    RCUTILS_SAFE_FWRITE_TO_STDERR(ex.what());
    RCUTILS_SAFE_FWRITE_TO_STDERR("\n");
  } catch (...) {
    RCUTILS_SAFE_FWRITE_TO_STDERR("failed to take global rclcpp logging mutex\n");
  }
}
}  // extern "C"

Context::Context()
: rcl_context_(nullptr),
  shutdown_reason_(""),
  logging_mutex_(nullptr)
{}

Context::~Context()
{
  // acquire the init lock to prevent race conditions with init and shutdown
  // this will not prevent errors, but will maybe make them easier to reproduce
  std::lock_guard<std::recursive_mutex> lock(init_mutex_);
  try {
    // Cannot rely on virtual dispatch in a destructor, so explicitly use the
    // shutdown() provided by this base class.
    Context::shutdown("context destructor was called while still not shutdown");
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
  char const * const * argv,
  const rclcpp::InitOptions & init_options)
{
  std::lock_guard<std::recursive_mutex> init_lock(init_mutex_);
  if (this->is_valid()) {
    throw rclcpp::ContextAlreadyInitialized();
  }
  this->clean_up();
  rcl_context_t * context = new rcl_context_t;
  if (!context) {
    throw std::runtime_error("failed to allocate memory for rcl context");
  }
  *context = rcl_get_zero_initialized_context();
  rcl_ret_t ret = rcl_init(argc, argv, init_options.get_rcl_init_options(), context);
  if (RCL_RET_OK != ret) {
    delete context;
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to initialize rcl");
  }
  rcl_context_.reset(context, __delete_context);

  if (init_options.auto_initialize_logging()) {
    logging_mutex_ = get_global_logging_mutex();
    std::lock_guard<std::recursive_mutex> guard(*logging_mutex_);
    size_t & count = get_logging_reference_count();
    if (0u == count) {
      ret = rcl_logging_configure_with_output_handler(
        &rcl_context_->global_arguments,
        rcl_init_options_get_allocator(init_options_.get_rcl_init_options()),
        rclcpp_logging_output_handler);
      if (RCL_RET_OK != ret) {
        rcl_context_.reset();
        rclcpp::exceptions::throw_from_rcl_error(ret, "failed to configure logging");
      }
    } else {
      RCLCPP_WARN(
        rclcpp::get_logger("rclcpp"),
        "logging was initialized more than once");
    }
    ++count;
  }

  try {
    std::vector<std::string> unparsed_ros_arguments = detail::get_unparsed_ros_arguments(
      argc, argv, &(rcl_context_->global_arguments), rcl_get_default_allocator());
    if (!unparsed_ros_arguments.empty()) {
      throw exceptions::UnknownROSArgsError(std::move(unparsed_ros_arguments));
    }

    init_options_ = init_options;

    weak_contexts_ = get_weak_contexts();
    weak_contexts_->add_context(this->shared_from_this());
  } catch (const std::exception & e) {
    ret = rcl_shutdown(rcl_context_.get());
    rcl_context_.reset();
    if (RCL_RET_OK != ret) {
      std::ostringstream oss;
      oss << "While handling: " << e.what() << std::endl <<
        "    another exception was thrown";
      rclcpp::exceptions::throw_from_rcl_error(ret, oss.str());
    }
    throw;
  }
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

size_t
Context::get_domain_id() const
{
  size_t domain_id;
  rcl_ret_t ret = rcl_context_get_domain_id(rcl_context_.get(), &domain_id);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to get domain id from context");
  }
  return domain_id;
}

std::string
Context::shutdown_reason() const
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

  // call each pre-shutdown callback
  {
    std::lock_guard<std::mutex> lock{pre_shutdown_callbacks_mutex_};
    for (const auto & callback : pre_shutdown_callbacks_) {
      (*callback)();
    }
  }

  // rcl shutdown
  rcl_ret_t ret = rcl_shutdown(rcl_context_.get());
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
  // set shutdown reason
  shutdown_reason_ = reason;
  // call each shutdown callback
  {
    std::lock_guard<std::mutex> lock(on_shutdown_callbacks_mutex_);
    for (const auto & callback : on_shutdown_callbacks_) {
      (*callback)();
    }
  }

  // interrupt all blocking sleep_for() and all blocking executors or wait sets
  this->interrupt_all_sleep_for();
  // remove self from the global contexts
  weak_contexts_->remove_context(this);
  // shutdown logger
  if (logging_mutex_) {
    // logging was initialized by this context
    std::lock_guard<std::recursive_mutex> guard(*logging_mutex_);
    size_t & count = get_logging_reference_count();
    if (0u == --count) {
      rcl_ret_t rcl_ret = rcl_logging_fini();
      if (RCL_RET_OK != rcl_ret) {
        RCUTILS_SAFE_FWRITE_TO_STDERR(
          RCUTILS_STRINGIFY(__file__) ":"
          RCUTILS_STRINGIFY(__LINE__)
          " failed to fini logging");
        rcl_reset_error();
      }
    }
  }
  return true;
}

rclcpp::Context::OnShutdownCallback
Context::on_shutdown(OnShutdownCallback callback)
{
  add_on_shutdown_callback(callback);
  return callback;
}

rclcpp::OnShutdownCallbackHandle
Context::add_on_shutdown_callback(OnShutdownCallback callback)
{
  return add_shutdown_callback(ShutdownType::on_shutdown, callback);
}

bool
Context::remove_on_shutdown_callback(const OnShutdownCallbackHandle & callback_handle)
{
  return remove_shutdown_callback(ShutdownType::on_shutdown, callback_handle);
}

rclcpp::PreShutdownCallbackHandle
Context::add_pre_shutdown_callback(PreShutdownCallback callback)
{
  return add_shutdown_callback(ShutdownType::pre_shutdown, callback);
}

bool
Context::remove_pre_shutdown_callback(
  const PreShutdownCallbackHandle & callback_handle)
{
  return remove_shutdown_callback(ShutdownType::pre_shutdown, callback_handle);
}

rclcpp::ShutdownCallbackHandle
Context::add_shutdown_callback(
  ShutdownType shutdown_type,
  ShutdownCallback callback)
{
  auto callback_shared_ptr =
    std::make_shared<ShutdownCallbackHandle::ShutdownCallbackType>(callback);

  switch (shutdown_type) {
    case ShutdownType::pre_shutdown:
      {
        std::lock_guard<std::mutex> lock(pre_shutdown_callbacks_mutex_);
        pre_shutdown_callbacks_.emplace(callback_shared_ptr);
      }
      break;
    case ShutdownType::on_shutdown:
      {
        std::lock_guard<std::mutex> lock(on_shutdown_callbacks_mutex_);
        on_shutdown_callbacks_.emplace(callback_shared_ptr);
      }
      break;
  }

  ShutdownCallbackHandle callback_handle;
  callback_handle.callback = callback_shared_ptr;
  return callback_handle;
}

bool
Context::remove_shutdown_callback(
  ShutdownType shutdown_type,
  const ShutdownCallbackHandle & callback_handle)
{
  std::mutex * mutex_ptr = nullptr;
  std::unordered_set<
    std::shared_ptr<ShutdownCallbackHandle::ShutdownCallbackType>> * callback_list_ptr;

  switch (shutdown_type) {
    case ShutdownType::pre_shutdown:
      mutex_ptr = &pre_shutdown_callbacks_mutex_;
      callback_list_ptr = &pre_shutdown_callbacks_;
      break;
    case ShutdownType::on_shutdown:
      mutex_ptr = &on_shutdown_callbacks_mutex_;
      callback_list_ptr = &on_shutdown_callbacks_;
      break;
  }

  std::lock_guard<std::mutex> lock(*mutex_ptr);
  auto callback_shared_ptr = callback_handle.callback.lock();
  if (callback_shared_ptr == nullptr) {
    return false;
  }
  return callback_list_ptr->erase(callback_shared_ptr) == 1;
}

std::vector<rclcpp::Context::OnShutdownCallback>
Context::get_on_shutdown_callbacks() const
{
  return get_shutdown_callback(ShutdownType::on_shutdown);
}

std::vector<rclcpp::Context::PreShutdownCallback>
Context::get_pre_shutdown_callbacks() const
{
  return get_shutdown_callback(ShutdownType::pre_shutdown);
}

std::vector<rclcpp::Context::ShutdownCallback>
Context::get_shutdown_callback(ShutdownType shutdown_type) const
{
  std::mutex * mutex_ptr = nullptr;
  const std::unordered_set<
    std::shared_ptr<ShutdownCallbackHandle::ShutdownCallbackType>> * callback_list_ptr;

  switch (shutdown_type) {
    case ShutdownType::pre_shutdown:
      mutex_ptr = &pre_shutdown_callbacks_mutex_;
      callback_list_ptr = &pre_shutdown_callbacks_;
      break;
    case ShutdownType::on_shutdown:
      mutex_ptr = &on_shutdown_callbacks_mutex_;
      callback_list_ptr = &on_shutdown_callbacks_;
      break;
  }

  std::vector<rclcpp::Context::ShutdownCallback> callbacks;
  {
    std::lock_guard<std::mutex> lock(*mutex_ptr);
    for (auto & iter : *callback_list_ptr) {
      callbacks.emplace_back(*iter);
    }
  }

  return callbacks;
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
  do {
    {
      std::unique_lock<std::mutex> lock(interrupt_mutex_);
      auto start = std::chrono::steady_clock::now();
      // this will release the lock while waiting
      interrupt_condition_variable_.wait_for(lock, nanoseconds);
      time_left -= std::chrono::steady_clock::now() - start;
    }
  } while (time_left > std::chrono::nanoseconds::zero() && this->is_valid());
  // Return true if the timeout elapsed successfully, otherwise false.
  return this->is_valid();
}

void
Context::interrupt_all_sleep_for()
{
  interrupt_condition_variable_.notify_all();
}

void
Context::clean_up()
{
  shutdown_reason_ = "";
  rcl_context_.reset();
  sub_contexts_.clear();
}

std::vector<Context::SharedPtr>
rclcpp::get_contexts()
{
  WeakContextsWrapper::SharedPtr weak_contexts = get_weak_contexts();
  return weak_contexts->get_contexts();
}
