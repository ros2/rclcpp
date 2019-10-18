// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__NODE_INTERFACES__NODE_PARAMETERS_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_PARAMETERS_HPP_

#include <map>
#include <memory>
#include <list>
#include <string>
#include <vector>

#include "rcutils/macros.h"

#include "rcl_interfaces/msg/list_parameters_result.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_service.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace node_interfaces
{

// Internal RAII-style guard for mutation recursion
class ParameterMutationRecursionGuard
{
public:
  explicit ParameterMutationRecursionGuard(bool & allow_mod)
  : allow_modification_(allow_mod)
  {
    if (!allow_modification_) {
      throw rclcpp::exceptions::ParameterModifiedInCallbackException(
              "cannot set or declare a parameter, or change the callback from within set callback");
    }

    allow_modification_ = false;
  }

  ~ParameterMutationRecursionGuard()
  {
    allow_modification_ = true;
  }

private:
  bool & allow_modification_;
};

/// Implementation of the NodeParameters part of the Node API.
class NodeParameters : public NodeParametersInterface
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeParameters)

  RCLCPP_PUBLIC
  NodeParameters(
    const node_interfaces::NodeBaseInterface::SharedPtr node_base,
    const node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    const node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    const node_interfaces::NodeServicesInterface::SharedPtr node_services,
    const node_interfaces::NodeClockInterface::SharedPtr node_clock,
    const std::vector<Parameter> & parameter_overrides,
    bool start_parameter_services,
    bool start_parameter_event_publisher,
    const rclcpp::QoS & parameter_event_qos,
    const rclcpp::PublisherOptionsBase & parameter_event_publisher_options,
    bool allow_undeclared_parameters,
    bool automatically_declare_parameters_from_overrides);

  RCLCPP_PUBLIC
  virtual
  ~NodeParameters();

// This is overriding a deprecated method, so we need to ignore the deprecation warning here.
// Users of the method will still get a warning!
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#else
# pragma warning(push)
# pragma warning(disable: 4996)
#endif
  [[deprecated(RCLCPP_INTERNAL_NODE_PARAMETERS_INTERFACE_DEPRECATE_DECLARE)]]
  RCLCPP_PUBLIC
  const rclcpp::ParameterValue &
  declare_parameter(const std::string & name) override;
#ifndef _WIN32
# pragma GCC diagnostic pop
#else
# pragma warning(pop)
#endif

  RCLCPP_PUBLIC
  const rclcpp::ParameterValue &
  declare_parameter(
    const std::string & name,
    const rclcpp::ParameterValue & default_value,
    const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
    rcl_interfaces::msg::ParameterDescriptor{},
    bool ignore_override = false) override;

  RCLCPP_PUBLIC
  const rclcpp::ParameterValue &
  declare_parameter(
    const std::string & name,
    rclcpp::ParameterType type,
    const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
    rcl_interfaces::msg::ParameterDescriptor(),
    bool ignore_override = false) override;

  RCLCPP_PUBLIC
  void
  undeclare_parameter(const std::string & name) override;

  RCLCPP_PUBLIC
  bool
  has_parameter(const std::string & name) const override;

  RCLCPP_PUBLIC
  std::vector<rcl_interfaces::msg::SetParametersResult>
  set_parameters(
    const std::vector<rclcpp::Parameter> & parameters) override;

  RCLCPP_PUBLIC
  rcl_interfaces::msg::SetParametersResult
  set_parameters_atomically(
    const std::vector<rclcpp::Parameter> & parameters) override;

  RCLCPP_PUBLIC
  std::vector<rclcpp::Parameter>
  get_parameters(const std::vector<std::string> & names) const override;

  RCLCPP_PUBLIC
  rclcpp::Parameter
  get_parameter(const std::string & name) const override;

  RCLCPP_PUBLIC
  bool
  get_parameter(
    const std::string & name,
    rclcpp::Parameter & parameter) const override;

  RCLCPP_PUBLIC
  bool
  get_parameters_by_prefix(
    const std::string & prefix,
    std::map<std::string, rclcpp::Parameter> & parameters) const override;

  RCLCPP_PUBLIC
  std::vector<rcl_interfaces::msg::ParameterDescriptor>
  describe_parameters(const std::vector<std::string> & names) const override;

  RCLCPP_PUBLIC
  std::vector<uint8_t>
  get_parameter_types(const std::vector<std::string> & names) const override;

  RCLCPP_PUBLIC
  rcl_interfaces::msg::ListParametersResult
  list_parameters(const std::vector<std::string> & prefixes, uint64_t depth) const override;

  RCLCPP_PUBLIC
  RCUTILS_WARN_UNUSED
  OnSetParametersCallbackHandle::SharedPtr
  add_on_set_parameters_callback(OnParametersSetCallbackType callback) override;

  RCLCPP_PUBLIC
  void
  remove_on_set_parameters_callback(const OnSetParametersCallbackHandle * const handler) override;

  RCLCPP_PUBLIC
  const std::map<std::string, ParameterInfo> &
  get_parameter_overrides() const override;

  using CallbacksContainerType = std::list<OnSetParametersCallbackHandle::WeakPtr>;

private:
  RCLCPP_DISABLE_COPY(NodeParameters)

  mutable std::recursive_mutex mutex_;

  // There are times when we don't want to allow modifications to parameters
  // (particularly when a set_parameter callback tries to call set_parameter,
  // declare_parameter, etc).  In those cases, this will be set to false.
  bool parameter_modification_enabled_{true};

  OnParametersSetCallbackType on_parameters_set_callback_ = nullptr;

  CallbacksContainerType on_parameters_set_callback_container_;

  std::map<std::string, ParameterInfo> parameters_;

  std::map<std::string, ParameterInfo> parameter_overrides_;

  bool allow_undeclared_ = false;

  Publisher<rcl_interfaces::msg::ParameterEvent>::SharedPtr events_publisher_;

  std::shared_ptr<ParameterService> parameter_service_;

  std::string combined_name_;

  node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
  node_interfaces::NodeClockInterface::SharedPtr node_clock_;
};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_PARAMETERS_HPP_
