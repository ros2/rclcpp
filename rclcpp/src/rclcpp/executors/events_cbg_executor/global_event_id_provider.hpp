// Copyright 2024 Cellumation GmbH.
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

#pragma once

#include <atomic>
#include <cstdint>

namespace rclcpp
{
namespace executors
{
namespace cbg_executor
{
class GlobalEventIdProvider
{
  static std::atomic<uint64_t> last_event_id;

public:
  using MonotonicId = uint64_t;

  // Returns the last id, returnd by getNextId
  static uint64_t get_last_id()
  {
    return last_event_id;
  }

  // increases the Id by one and returns the Id
  static uint64_t get_next_id()
  {
    return  ++last_event_id;
  }
};
}  // namespace cbg_executor
}  // namespace executors
}  // namespace rclcpp
