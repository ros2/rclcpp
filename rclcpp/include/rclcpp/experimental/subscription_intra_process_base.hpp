// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXPERIMENTAL__SUBSCRIPTION_INTRA_PROCESS_BASE_HPP_
#define RCLCPP__EXPERIMENTAL__SUBSCRIPTION_INTRA_PROCESS_BASE_HPP_

#include <rmw/rmw.h>

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "rcl/error_handling.h"

#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/waitable.hpp"

namespace rclcpp
{
namespace experimental
{

class SubscriptionIntraProcessBase : public rclcpp::Waitable
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(SubscriptionIntraProcessBase)

  RCLCPP_PUBLIC
  SubscriptionIntraProcessBase(const std::string & topic_name, rmw_qos_profile_t qos_profile)
  : topic_name_(topic_name), qos_profile_(qos_profile)
  {}

  virtual ~SubscriptionIntraProcessBase() = default;

  RCLCPP_PUBLIC
  size_t
  get_number_of_ready_guard_conditions() {return 1;}

  RCLCPP_PUBLIC
  bool
  add_to_wait_set(rcl_wait_set_t * wait_set);

  virtual bool
  is_ready(rcl_wait_set_t * wait_set) = 0;

  virtual void
  execute() = 0;

  virtual bool
  use_take_shared_method() const = 0;

  RCLCPP_PUBLIC
  const char *
  get_topic_name() const;

  RCLCPP_PUBLIC
  rmw_qos_profile_t
  get_actual_qos() const;

protected:
  std::recursive_mutex reentrant_mutex_;
  rcl_guard_condition_t gc_;

private:
  virtual void
  trigger_guard_condition() = 0;

  std::string topic_name_;
  rmw_qos_profile_t qos_profile_;
};

}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__SUBSCRIPTION_INTRA_PROCESS_BASE_HPP_
