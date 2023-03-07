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

#include "rclcpp/node_interfaces/node_logging.hpp"

using rclcpp::node_interfaces::NodeLogging;

NodeLogging::NodeLogging(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services,
  bool enable_log_service)
: node_base_(node_base),
  node_services_(node_services)
{
  logger_ = rclcpp::get_logger(NodeLogging::get_logger_name());
  if (enable_log_service) {
    add_log_services();
  }
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

void
NodeLogging::add_log_services(void)
{
  const rclcpp::QoS & qos_profile = rclcpp::ServicesQoS();
  const std::string node_name = node_base_->get_name();
  auto callback_group = node_base_->get_builtin_callback_group();

  get_loggers_service_ = rclcpp::create_service<rcl_interfaces::srv::GetLoggerLevels>(
    node_base_, node_services_,
    node_name + "/get_logger_levels",
    [](
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<rcl_interfaces::srv::GetLoggerLevels::Request> request,
      std::shared_ptr<rcl_interfaces::srv::GetLoggerLevels::Response> response)
    {
      int ret = 0;
      for (auto & n : request->names) {
        rcl_interfaces::msg::LoggerLevel level;
        level.name = n;
        ret = rcutils_logging_get_logger_level(n.c_str());
        if (ret < 0) {
          level.level = 0;
        } else {
          level.level = (uint8_t)ret;
        }
        response->levels.push_back(std::move(level));
      }
    },
    qos_profile, callback_group);

  set_loggers_service_ = rclcpp::create_service<rcl_interfaces::srv::SetLoggerLevels>(
    node_base_, node_services_,
    node_name + "/set_logger_levels",
    [](
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<rcl_interfaces::srv::SetLoggerLevels::Request> request,
      std::shared_ptr<rcl_interfaces::srv::SetLoggerLevels::Response> response)
    {
      int ret = 0;
      auto result = rcl_interfaces::msg::SetLoggerLevelsResult();
      for (auto & l : request->levels) {
        ret = rcutils_logging_set_logger_level(l.name.c_str(), l.level);
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
