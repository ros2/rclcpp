// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include <mutex>
#include <shared_mutex>  // NOLINT
#include <unordered_map>

#include <cstdio>
#include <cstdlib>
#include <execinfo.h>
#include <cxxabi.h>

#include "rclcpp/clock.hpp"

#include "rclcpp/exceptions.hpp"

#include "rcutils/logging_macros.h"

namespace rclcpp
{

std::shared_timed_mutex g_clock_map_mutex;
std::unordered_map<rclcpp::Clock *, std::mutex> g_clock_mutex_map;

/** Print a demangled stack backtrace of the caller function to FILE* out. */
void print_backtrace(FILE *out = stderr, unsigned int max_frames = 63)
{
  fprintf(out, "stack trace:\n");

  // storage array for stack trace address data
  void* addrlist[max_frames+1];

  // retrieve current stack addresses
  int addrlen = backtrace(addrlist, sizeof(addrlist) / sizeof(void*));

  if (addrlen == 0) {
    fprintf(out, "  <empty, possibly corrupt>\n");
    return;
  }

  // resolve addresses into strings containing "filename(function+address)",
  // this array must be free()-ed
  char** symbollist = backtrace_symbols(addrlist, addrlen);

  // allocate string which will be filled with the demangled function name
  size_t funcnamesize = 256;
  char* funcname = (char*)malloc(funcnamesize);

  // iterate over the returned symbol lines. skip the first, it is the
  // address of this function.
  for (int i = 1; i < addrlen; i++) {
    char *begin_name = 0, *begin_offset = 0, *end_offset = 0;

    // find parentheses and +address offset surrounding the mangled name:
    // ./module(function+0x15c) [0x8048a6d]
    for (char *p = symbollist[i]; *p; ++p) {
      if (*p == '(') {
        begin_name = p;
      } else if (*p == '+') {
        begin_offset = p;
      } else if (*p == ')' && begin_offset) {
        end_offset = p;
        break;
      }
    }

    if (begin_name && begin_offset && end_offset && begin_name < begin_offset) {
      *begin_name++ = '\0';
      *begin_offset++ = '\0';
      *end_offset = '\0';

      // mangled name is now in [begin_name, begin_offset) and caller
      // offset in [begin_offset, end_offset). now apply
      // __cxa_demangle():

      int status;
      char* ret = abi::__cxa_demangle(begin_name, funcname, &funcnamesize, &status);
      if (status == 0) {
        funcname = ret; // use possibly realloc()-ed string
        fprintf(out, "  %s : %s+%s\n", symbollist[i], funcname, begin_offset);
      }
      else {
        // demangling failed. Output function name as a C function with
        // no arguments.
        fprintf(out, "  %s : %s()+%s\n", symbollist[i], begin_name, begin_offset);
      }
    }
    else {
      // couldn't parse the line? print the whole line.
      fprintf(out, "  %s\n", symbollist[i]);
    }
  }

  free(funcname);
  free(symbollist);
}

std::mutex & get_clock_mutex(rclcpp::Clock * clock)
{
  std::shared_lock<std::shared_timed_mutex> lk(g_clock_map_mutex);
  try {
    return g_clock_mutex_map.at(clock);
  } catch (const std::out_of_range &) {
    print_backtrace();
    throw;
  }
}

JumpHandler::JumpHandler(
  pre_callback_t pre_callback,
  post_callback_t post_callback,
  const rcl_jump_threshold_t & threshold)
: pre_callback(pre_callback),
  post_callback(post_callback),
  notice_threshold(threshold)
{}

Clock::Clock(rcl_clock_type_t clock_type)
{
  allocator_ = rcl_get_default_allocator();
  auto ret = rcl_clock_init(clock_type, &rcl_clock_, &allocator_);
  if (ret != RCL_RET_OK) {
    exceptions::throw_from_rcl_error(ret, "could not get current time stamp");
  }

  {
    std::lock_guard<std::shared_timed_mutex> lg(g_clock_map_mutex);
    // Add a new default-constructed mutex, keyed off of this pointer.
    fprintf(stderr, "Adding clock mutex for %p\n", static_cast<void *>(this));
    g_clock_mutex_map[this];
  }
}

Clock::~Clock()
{
  {
    std::lock_guard<std::shared_timed_mutex> lg(g_clock_map_mutex);
    // Remove the mutex corresponding to this clock object.
    g_clock_mutex_map.erase(this);
    fprintf(stderr, "Removed clock mutex for %p\n", static_cast<void *>(this));
  }

  auto ret = rcl_clock_fini(&rcl_clock_);
  if (ret != RCL_RET_OK) {
    RCUTILS_LOG_ERROR("Failed to fini rcl clock.");
  }
}

Time
Clock::now()
{
  Time now(0, 0, rcl_clock_.type);

  auto ret = rcl_clock_get_now(&rcl_clock_, &now.rcl_time_.nanoseconds);
  if (ret != RCL_RET_OK) {
    exceptions::throw_from_rcl_error(ret, "could not get current time stamp");
  }

  return now;
}

bool
Clock::ros_time_is_active()
{
  if (!rcl_clock_valid(&rcl_clock_)) {
    RCUTILS_LOG_ERROR("ROS time not valid!");
    return false;
  }

  bool is_enabled = false;
  auto ret = rcl_is_enabled_ros_time_override(&rcl_clock_, &is_enabled);
  if (ret != RCL_RET_OK) {
    exceptions::throw_from_rcl_error(
      ret, "Failed to check ros_time_override_status");
  }
  return is_enabled;
}

rcl_clock_t *
Clock::get_clock_handle() noexcept
{
  return &rcl_clock_;
}

rcl_clock_type_t
Clock::get_clock_type() const noexcept
{
  return rcl_clock_.type;
}

void
Clock::on_time_jump(
  const struct rcl_time_jump_t * time_jump,
  bool before_jump,
  void * user_data)
{
  const auto * handler = static_cast<JumpHandler *>(user_data);
  if (nullptr == handler) {
    return;
  }
  if (before_jump && handler->pre_callback) {
    handler->pre_callback();
  } else if (!before_jump && handler->post_callback) {
    handler->post_callback(*time_jump);
  }
}

JumpHandler::SharedPtr
Clock::create_jump_callback(
  JumpHandler::pre_callback_t pre_callback,
  JumpHandler::post_callback_t post_callback,
  const rcl_jump_threshold_t & threshold)
{
  // Allocate a new jump handler
  JumpHandler::UniquePtr handler(new JumpHandler(pre_callback, post_callback, threshold));
  if (nullptr == handler) {
    throw std::bad_alloc{};
  }

  {
    fprintf(stderr, "create_jump_callback() getting clock mutex for %p\n", static_cast<void *>(this));
    std::lock_guard<std::mutex> clock_guard(get_clock_mutex(this));
    // Try to add the jump callback to the clock
    rcl_ret_t ret = rcl_clock_add_jump_callback(
      &rcl_clock_, threshold, Clock::on_time_jump,
      handler.get());
    if (RCL_RET_OK != ret) {
      exceptions::throw_from_rcl_error(ret, "Failed to add time jump callback");
    }
  }

  // *INDENT-OFF*
  // create shared_ptr that removes the callback automatically when all copies are destructed
  // TODO(dorezyuk) UB, if the clock leaves scope before the JumpHandler
  return JumpHandler::SharedPtr(handler.release(), [this](JumpHandler * handler) noexcept {
    fprintf(stderr, "create_jump_callback release lambda getting clock mutex for %p\n", static_cast<void *>(this));
    std::lock_guard<std::mutex> clock_guard(get_clock_mutex(this));

    rcl_ret_t ret = rcl_clock_remove_jump_callback(&rcl_clock_, Clock::on_time_jump,
        handler);
    delete handler;
    handler = NULL;
    if (RCL_RET_OK != ret) {
      RCUTILS_LOG_ERROR("Failed to remove time jump callback");
    }
  });
  // *INDENT-ON*
}

}  // namespace rclcpp
