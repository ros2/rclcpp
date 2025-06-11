// Copyright 2025 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/detail/os_thread.hpp"

#if defined(_WIN32)
#include <windows.h>
#include <sstream>
#else  // posix and apple
#include <pthread.h>
#endif

#include "rclcpp/logging.hpp"

namespace rclcpp
{
namespace detail
{

// This includes the null terminator
#if defined(__APPLE__)
  #define MAXTHREADNAMESIZE 64
#else  // posix
  #define MAXTHREADNAMESIZE 16
#endif

void set_thread_name(const std::string & name)
{
  int rc;
#if defined(_WIN32)
  // This will only work on Windows 10 and above.
  auto result = SetThreadDescription(GetCurrentThread(), name.c_str());
  rc = FAILED(result) ? -1 : 0;
#elif defined(__APPLE__)
  // Apple's pthread_setname_np implementation does the truncation automatically
  rc = pthread_setname_np(name.c_str());
#else  // posix
  // Truncate name to maximum length supported by pthread_setname_np
  // leaving one character for the null terminator
  // otherwise pthread_setname_np will return an ERANGE error
  std::string truncated_name = name.substr(0, MAXTHREADNAMESIZE - 1);
  rc = pthread_setname_np(pthread_self(), truncated_name.c_str());
#endif
  if (rc != 0) {
        // Don't throw since this is not critical
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to set thread name: %s", name.c_str());
  }
}

std::string get_thread_name()
{
  std::string name;
#if defined(_WIN32)
  // This will only work on Windows 10 and above.
  PWSTR thread_description;
  auto result = GetThreadDescription(GetCurrentThread(), &thread_description);
  if (FAILED(result)) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to get thread name");
    return name;
  }
  std::wstringstream wss;
  wss << thread_description;
  name = wss.str();
#else  // posix and apple
  char thread_name[MAXTHREADNAMESIZE];  // This includes the null terminator
  int rc = pthread_getname_np(pthread_self(), thread_name, sizeof(thread_name));
  if (rc != 0) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to get thread name");
    return name;
  }
  name = std::string(thread_name);
#endif
  return name;
}

}  // namespace detail
}  // namespace rclcpp
