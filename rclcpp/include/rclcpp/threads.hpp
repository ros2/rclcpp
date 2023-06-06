// Copyright 2023 eSOL Co.,Ltd.
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

#ifndef RCLCPP__THREADS_HPP_
#define RCLCPP__THREADS_HPP_

#if defined(__linux__)
#include "rclcpp/threads/posix/thread.hpp"
#elif defined(_WIN32)
#include "rclcpp/threads/win32/thread.hpp"
#else
#include "rclcpp/threads/std/thread.hpp"
#endif

#endif  // RCLCPP__THREADS_HPP_
