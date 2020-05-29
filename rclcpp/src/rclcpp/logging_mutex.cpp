// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <mutex>

#include "rcutils/macros.h"

#include "./logging_mutex.hpp"

std::shared_ptr<std::recursive_mutex>
get_global_logging_mutex()
{
  static auto mutex = std::make_shared<std::recursive_mutex>();
  if (RCUTILS_UNLIKELY(!mutex)) {
    throw std::runtime_error("rclcpp global logging mutex is a nullptr");
  }
  return mutex;
}
