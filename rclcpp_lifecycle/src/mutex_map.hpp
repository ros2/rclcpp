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

#ifndef MUTEX_MAP_HPP_
#define MUTEX_MAP_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>

#include "rclcpp_lifecycle/state.hpp"

namespace rclcpp_lifecycle
{
/// @brief Associates instances of recursive_mutex with instances of State.
class MutexMap
{
public:
  MutexMap() = default;

  /// \brief Add a new mutex for an instance of State.
  /**
   * \param[in] key Raw pointer to the instance of State which will use the mutex.
   */
  void add(const State * key);

  /// \brief Retrieve the mutex for an instance of State.
  /**
   * \param key Raw pointer to an instance of State.
   * \return A reference to the mutex associated with the key.
   */
  std::recursive_mutex & getMutex(const State * key) const;

  /// \brief Remove the mutex for an instance of State.
  /**
   * \param key Raw pointer to an instance of State.
   */
  void remove(const State * key);

private:
  /// \brief Map that stores the mutexes
  /**
   * \details The mutexes are emplaced as unique_ptrs because mutexes are
   * not copyable or movable.
   */
  std::map<const State *, std::unique_ptr<std::recursive_mutex>> mutex_map_;

  /// @brief Controls access to mutex_map_.
  mutable std::shared_mutex map_access_mutex_;
};
}  // namespace rclcpp_lifecycle

#endif  // MUTEX_MAP_HPP_
