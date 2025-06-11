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
#else  // posix and apple
#include <pthread.h>
#endif

#include "rclcpp/logging.hpp"

namespace rclcpp
{
namespace detail
{

void set_thread_name(const std::string & name)
{
    int rc;
#if defined(_WIN32)
    // This will only work on Windows 10 and above.
    auto result = SetThreadDescription(GetCurrentThread(), name.c_str());
    rc = FAILED(result) ? -1 : 0;
#elif defined(__APPLE__)
    rc = pthread_setname_np(name.c_str());
#else  // posix
    // Truncate name to 16 characters as that's the maximum length supported by pthread_setname_np
    std::string truncated_name = name.substr(0, 15);
    rc = pthread_setname_np(pthread_self(), truncated_name.c_str());
#endif
    if (rc != 0) {
        // Don't throw since this is not critical
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to set thread name: %s", name.c_str());
    }
}
}
}
