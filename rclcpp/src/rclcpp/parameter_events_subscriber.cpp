// Copyright (c) 2019 Intel Corporation
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

#include <map>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/parameter_events_subscriber.hpp"

namespace rclcpp
{

ParameterEventsSubscriber::ParameterEventsSubscriber(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
  const rclcpp::QoS & qos)
: node_base_(node_base),
  node_topics_(node_topics),
  node_logging_(node_logging),
  qos_(qos),
  last_event_(std::make_shared<rcl_interfaces::msg::ParameterEvent>())
{}

void ParameterEventsSubscriber::add_namespace_event_subscriber(const std::string & node_namespace)
{
  if (std::find(node_namespaces_.begin(), node_namespaces_.end(),
    node_namespace) == node_namespaces_.end())
  {
    node_namespaces_.push_back(node_namespace);
    auto topic = join_path(node_namespace, "parameter_events");
    RCLCPP_INFO(node_logging_->get_logger(), "Subscribing to topic: %s", topic.c_str());

    auto event_sub = rclcpp::create_subscription<rcl_interfaces::msg::ParameterEvent>(
      node_topics_, topic, qos_,
      std::bind(&ParameterEventsSubscriber::event_callback, this, std::placeholders::_1));

    event_subscriptions_.push_back(event_sub);
  }
}

void ParameterEventsSubscriber::set_event_callback(
  std::function<void(const rcl_interfaces::msg::ParameterEvent::SharedPtr &)> callback,
  const std::string & node_namespace)
{
  std::string full_namespace;
  if (node_namespace == "") {
    full_namespace = node_base_->get_namespace();
  }

  full_namespace = resolve_path(full_namespace);
  add_namespace_event_subscriber(full_namespace);
  user_callback_ = callback;
}

void ParameterEventsSubscriber::register_param_callback(
  const std::string & parameter_name,
  std::function<void()> callback,
  const std::string & node_name)
{
  auto full_node_name = resolve_path(node_name);
  add_namespace_event_subscriber(split_path(full_node_name).first);
  parameter_node_map_[parameter_name] = full_node_name;
  parameter_callbacks_[parameter_name] = callback;
}

void ParameterEventsSubscriber::event_callback(
  const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  std::string node_name = event->node;
  RCLCPP_DEBUG(node_logging_->get_logger(), "Parameter event received for node: %s",
    node_name.c_str());

  last_event_ = event;

  for (std::map<std::string, std::string>::iterator it = parameter_node_map_.begin();
    it != parameter_node_map_.end(); ++it)
  {
    if (node_name == it->second) {
      rclcpp::ParameterEventsFilter filter(event, {it->first},
        {rclcpp::ParameterEventsFilter::EventType::NEW,
          rclcpp::ParameterEventsFilter::EventType::CHANGED});
      if (!filter.get_events().empty()) {
        parameter_callbacks_[it->first]();
      }
    }
  }

  if (user_callback_) {
    user_callback_(event);
  }
}

std::string ParameterEventsSubscriber::resolve_path(const std::string & path)
{
  std::string full_path;

  if (path == "") {
    full_path = node_base_->get_fully_qualified_name();
  } else {
    full_path = path;
    if (*full_path.begin() != '/') {
      full_path = join_path(node_base_->get_namespace(), full_path);
    }
  }

  return full_path;
}

std::pair<std::string, std::string> ParameterEventsSubscriber::split_path(const std::string & str)
{
  std::string path;
  std::size_t found = str.find_last_of("/\\");
  if (found == 0) {
    path = str.substr(0, found + 1);
  } else {
    path = str.substr(0, found);
  }
  std::string name = str.substr(found + 1);
  return {path, name};
}

std::string ParameterEventsSubscriber::join_path(std::string path, std::string name)
{
  std::string joined_path = path;
  if (*joined_path.rbegin() != '/' && *name.begin() != '/') {
    joined_path = joined_path + "/";
  }

  return joined_path + name;
}

}  // namespace rclcpp
