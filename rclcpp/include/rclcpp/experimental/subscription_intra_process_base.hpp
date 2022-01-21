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

#include <memory>
#include <mutex>
#include <string>

#include "rcl/wait.h"

#include "rclcpp/guard_condition.hpp"
#include "rclcpp/qos.hpp"
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
  SubscriptionIntraProcessBase(
    rclcpp::Context::SharedPtr context,
    const std::string & topic_name,
    const rclcpp::QoS & qos_profile)
  : gc_(context), topic_name_(topic_name), qos_profile_(qos_profile)
  {}

  virtual ~SubscriptionIntraProcessBase() = default;

  RCLCPP_PUBLIC
  size_t
  get_number_of_ready_guard_conditions() override {return 1;}

  RCLCPP_PUBLIC
  void
  add_to_wait_set(rcl_wait_set_t * wait_set) override;

  virtual bool
  use_take_shared_method() const = 0;

  RCLCPP_PUBLIC
  const char *
  get_topic_name() const;

  RCLCPP_PUBLIC
  QoS
  get_actual_qos() const;

protected:
  std::recursive_mutex reentrant_mutex_;
  rclcpp::GuardCondition gc_;

  virtual void
  trigger_guard_condition() = 0;

private:
  std::string topic_name_;
  QoS qos_profile_;
};

}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__SUBSCRIPTION_INTRA_PROCESS_BASE_HPP_
