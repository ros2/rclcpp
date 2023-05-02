// Copyright 2023 PickNik, Inc.
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

#include "mutex_map.hpp"

namespace rclcpp_lifecycle
{
void MutexMap::add(const State * key)
{
  std::scoped_lock lock(map_access_mutex_);
  mutex_map_.emplace(key, std::make_unique<std::recursive_mutex>());
}

std::recursive_mutex & MutexMap::getMutex(const State * key)
{
  std::lock_guard<std::mutex> lock(map_access_mutex_);
  return *(mutex_map_.at(key));
}

void MutexMap::remove(const State * key)
{
  std::lock_guard<std::mutex> lock(map_access_mutex_);
  mutex_map_.erase(key);
}
}  // namespace rclcpp_lifecycle
