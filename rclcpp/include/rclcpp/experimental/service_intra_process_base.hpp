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

#ifndef RCLCPP__EXPERIMENTAL__SERVICE_INTRA_PROCESS_BASE_HPP_
#define RCLCPP__EXPERIMENTAL__SERVICE_INTRA_PROCESS_BASE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>

#include "rcl/error_handling.h"

#include "rclcpp/context.hpp"
#include "rclcpp/experimental/client_intra_process_base.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/waitable.hpp"

namespace rclcpp
{
namespace experimental
{

class ServiceIntraProcessBase : public rclcpp::Waitable
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(ServiceIntraProcessBase)

  RCLCPP_PUBLIC
  ServiceIntraProcessBase(
    rclcpp::Context::SharedPtr context,
    const std::string & service_name,
    const rclcpp::QoS & qos_profile)
  : gc_(context), service_name_(service_name), qos_profile_(qos_profile)
  {}

  virtual ~ServiceIntraProcessBase() = default;

  RCLCPP_PUBLIC
  size_t
  get_number_of_ready_guard_conditions() {return 1;}

  RCLCPP_PUBLIC
  void
  add_to_wait_set(rcl_wait_set_t * wait_set);

  virtual bool
  is_ready(rcl_wait_set_t * wait_set) = 0;

  virtual
  std::shared_ptr<void>
  take_data() = 0;

  virtual void
  execute(std::shared_ptr<void> & data) = 0;

  RCLCPP_PUBLIC
  const char *
  get_service_name() const;

  RCLCPP_PUBLIC
  QoS
  get_actual_qos() const;

  RCLCPP_PUBLIC
  void
  add_intra_process_client(
    rclcpp::experimental::ClientIntraProcessBase::SharedPtr client,
    uint64_t client_id);

protected:
  std::recursive_mutex reentrant_mutex_;
  rclcpp::GuardCondition gc_;

  using ClientMap =
    std::unordered_map<uint64_t, rclcpp::experimental::ClientIntraProcessBase::WeakPtr>;

  ClientMap clients_;

private:
  std::string service_name_;
  QoS qos_profile_;
};

}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__SERVICE_INTRA_PROCESS_BASE_HPP_
