// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__WAIT_SET_POLICIES__DETAIL__CLIENT_ENTRY_HPP_
#define RCLCPP__WAIT_SET_POLICIES__DETAIL__CLIENT_ENTRY_HPP_

#include <memory>

#include "rclcpp/client.hpp"
#include "rclcpp/wait_set_policies/detail/entity_entry.hpp"

namespace rclcpp
{
namespace wait_set_policies
{
namespace detail
{

using ClientEntry =
  // EntityEntryTemplate<rclcpp::ClientBase, std::shared_ptr<rclcpp::ClientBase>>;
  EntityEntryTemplate<rclcpp::ClientBase>;
using WeakClientEntry =
  // EntityEntryTemplate<rclcpp::ClientBase, std::weak_ptr<rclcpp::ClientBase>>;
  EntityEntryTemplate<rclcpp::ClientBase>;

}  // namespace detail
}  // namespace wait_set_policies
}  // namespace rclcpp

#endif  // RCLCPP__WAIT_SET_POLICIES__DETAIL__CLIENT_ENTRY_HPP_
