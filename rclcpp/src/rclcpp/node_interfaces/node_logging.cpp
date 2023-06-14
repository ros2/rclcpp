// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/node_impl.hpp"
#include "rclcpp/node_interfaces/node_logging.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"

using rclcpp::node_interfaces::NodeLogging;

NodeLogging::NodeLogging(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base)
: node_base_(node_base)
{
  logger_ = rclcpp::get_logger(NodeLogging::get_logger_name());
}

NodeLogging::~NodeLogging()
{
}

rclcpp::Logger
NodeLogging::get_logger() const
{
  return logger_;
}

const char *
NodeLogging::get_logger_name() const
{
  return rcl_node_get_logger_name(node_base_->get_rcl_node_handle());
}

void NodeLogging::create_logger_services(
  node_interfaces::NodeServicesInterface::SharedPtr node_services)
{
  rclcpp::ServicesQoS qos_profile;
  const std::string node_name = node_base_->get_name();
  auto callback_group = node_base_->get_default_callback_group();

  get_loggers_service_ = rclcpp::create_service<rcl_interfaces::srv::GetLoggerLevels>(
    node_base_, node_services,
    node_name + "/get_logger_levels",
    [](
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<rcl_interfaces::srv::GetLoggerLevels::Request> request,
      std::shared_ptr<rcl_interfaces::srv::GetLoggerLevels::Response> response)
    {
      for (auto & name : request->names) {
        rcl_interfaces::msg::LoggerLevel logger_level;
        logger_level.name = name;
        auto ret = rcutils_logging_get_logger_level(name.c_str());
        if (ret < 0) {
          logger_level.level = 0;
        } else {
          logger_level.level = static_cast<uint8_t>(ret);
        }
        response->levels.push_back(std::move(logger_level));
      }
    },
    qos_profile, callback_group);

  set_loggers_service_ = rclcpp::create_service<rcl_interfaces::srv::SetLoggerLevels>(
    node_base_, node_services,
    node_name + "/set_logger_levels",
    [](
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<rcl_interfaces::srv::SetLoggerLevels::Request> request,
      std::shared_ptr<rcl_interfaces::srv::SetLoggerLevels::Response> response)
    {
      rcl_interfaces::msg::SetLoggerLevelsResult result;
      for (auto & level : request->levels) {
        auto ret = rcutils_logging_set_logger_level(level.name.c_str(), level.level);
        if (ret != RCUTILS_RET_OK) {
          result.successful = false;
          result.reason = rcutils_get_error_string().str;
        } else {
          result.successful = true;
        }
        response->results.push_back(std::move(result));
      }
    },
    qos_profile, callback_group);
}
