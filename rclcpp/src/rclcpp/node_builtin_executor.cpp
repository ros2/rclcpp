// Copyright 2023 Sony Group Corporation.
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

#include "rclcpp/executor.hpp"
#include "rclcpp/node_builtin_executor.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node_interfaces/node_logging.hpp"

#include "rcl_interfaces/srv/get_logger_levels.hpp"
#include "rcl_interfaces/srv/set_logger_levels.hpp"

using rclcpp::NodeBuiltinExecutor;

class NodeBuiltinExecutor::NodeBuiltinExecutorImpl
{
public:
  explicit NodeBuiltinExecutorImpl(
    node_interfaces::NodeBaseInterface::SharedPtr node_base,
    node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    node_interfaces::NodeServicesInterface::SharedPtr node_services,
    const NodeOptions & node_options);

  ~NodeBuiltinExecutorImpl();

private:
  void
  add_logger_services();

  rclcpp::CallbackGroup::SharedPtr
  get_callback_group();

  node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
  node_interfaces::NodeServicesInterface::SharedPtr node_services_;

  rclcpp::Service<rcl_interfaces::srv::GetLoggerLevels>::SharedPtr get_loggers_service_;
  rclcpp::Service<rcl_interfaces::srv::SetLoggerLevels>::SharedPtr set_loggers_service_;

  rclcpp::CallbackGroup::SharedPtr builtin_callback_group_;

  rclcpp::Executor::SharedPtr executor_;
  std::promise<void> executor_promise_;
  std::thread thread_;
};

NodeBuiltinExecutor::NodeBuiltinExecutor(
  node_interfaces::NodeBaseInterface::SharedPtr node_base,
  node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
  node_interfaces::NodeServicesInterface::SharedPtr node_services,
  const NodeOptions & node_options
)
: impl_(new NodeBuiltinExecutorImpl(
      node_base,
      node_topics,
      node_services,
      node_options
    ), [](NodeBuiltinExecutorImpl * impl) {
      delete impl;
    })
{}

NodeBuiltinExecutor::NodeBuiltinExecutorImpl::NodeBuiltinExecutorImpl(
  node_interfaces::NodeBaseInterface::SharedPtr node_base,
  node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
  node_interfaces::NodeServicesInterface::SharedPtr node_services,
  const NodeOptions & node_options)
: node_base_(node_base),
  node_topics_(node_topics),
  node_services_(node_services)
{
  if (node_options.enable_logger_service()) {
    add_logger_services();
  }

  if (builtin_callback_group_) {
    ExecutorOptions executor_option;
    executor_option.context = node_options.context();

    executor_ = std::make_shared<executors::SingleThreadedExecutor>(executor_option);
    executor_->add_callback_group(
      builtin_callback_group_,
      node_base_);

    executor_promise_ = std::promise<void>{};
    thread_ = std::thread(
      [this]() {
        auto future = executor_promise_.get_future();
        executor_->spin_until_future_complete(future);
      }
    );
  }
}

NodeBuiltinExecutor::NodeBuiltinExecutorImpl::~NodeBuiltinExecutorImpl()
{
  if (thread_.joinable()) {
    executor_promise_.set_value();
    executor_->cancel();
    thread_.join();
  }
}

void
NodeBuiltinExecutor::NodeBuiltinExecutorImpl::add_logger_services()
{
  rclcpp::ServicesQoS qos_profile;
  const std::string node_name = node_base_->get_name();
  auto callback_group = get_callback_group();

  get_loggers_service_ = rclcpp::create_service<rcl_interfaces::srv::GetLoggerLevels>(
    node_base_, node_services_,
    node_name + "/get_logger_levels",
    [](
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<rcl_interfaces::srv::GetLoggerLevels::Request> request,
      std::shared_ptr<rcl_interfaces::srv::GetLoggerLevels::Response> response)
    {
      int ret = 0;
      for (auto & name : request->names) {
        rcl_interfaces::msg::LoggerLevel logger_level;
        logger_level.name = name;
        ret = rcutils_logging_get_logger_level(name.c_str());
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
    node_base_, node_services_,
    node_name + "/set_logger_levels",
    [](
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<rcl_interfaces::srv::SetLoggerLevels::Request> request,
      std::shared_ptr<rcl_interfaces::srv::SetLoggerLevels::Response> response)
    {
      int ret = 0;
      auto result = rcl_interfaces::msg::SetLoggerLevelsResult();
      for (auto & level : request->levels) {
        ret = rcutils_logging_set_logger_level(level.name.c_str(), level.level);
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

rclcpp::CallbackGroup::SharedPtr
NodeBuiltinExecutor::NodeBuiltinExecutorImpl::get_callback_group()
{
  if (!builtin_callback_group_) {
    builtin_callback_group_ = node_base_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
  }

  return builtin_callback_group_;
}
