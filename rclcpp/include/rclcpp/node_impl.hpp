// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP_RCLCPP_NODE_IMPL_HPP_
#define RCLCPP_RCLCPP_NODE_IMPL_HPP_

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <rcl_interfaces/msg/intra_process_message.hpp>
#include <rmw/error_handling.h>
#include <rmw/rmw.h>
#include <rosidl_generator_cpp/message_type_support.hpp>
#include <rosidl_generator_cpp/service_type_support.hpp>

#include <rclcpp/contexts/default_context.hpp>
#include <rclcpp/intra_process_manager.hpp>
#include <rclcpp/parameter.hpp>

#ifndef RCLCPP_RCLCPP_NODE_HPP_
#include "node.hpp"
#endif

using namespace rclcpp;
using namespace rclcpp::node;

Node::Node(const std::string & node_name, bool use_intra_process_comms)
: Node(
    node_name,
    rclcpp::contexts::default_context::get_global_default_context(),
    use_intra_process_comms)
{}

Node::Node(
  const std::string & node_name,
  context::Context::SharedPtr context,
  bool use_intra_process_comms)
: name_(node_name), context_(context),
  number_of_subscriptions_(0), number_of_timers_(0), number_of_services_(0),
  use_intra_process_comms_(use_intra_process_comms)
{
  size_t domain_id = 0;
  char * ros_domain_id = nullptr;
  const char * env_var = "ROS_DOMAIN_ID";
#ifndef _WIN32
  ros_domain_id = getenv(env_var);
#else
  size_t ros_domain_id_size;
  _dupenv_s(&ros_domain_id, &ros_domain_id_size, env_var);
#endif
  if (ros_domain_id) {
    unsigned long number = strtoul(ros_domain_id, NULL, 0);
    if (number == (std::numeric_limits<unsigned long>::max)()) {
      throw std::runtime_error("failed to interpret ROS_DOMAIN_ID as integral number");
    }
    domain_id = static_cast<size_t>(number);
#ifdef _WIN32
    free(ros_domain_id);
#endif
  }

  auto node = rmw_create_node(name_.c_str(), domain_id);
  if (!node) {
    // *INDENT-OFF*
    throw std::runtime_error(
      std::string("could not create node: ") +
      rmw_get_error_string_safe());
    // *INDENT-ON*
  }
  // Initialize node handle shared_ptr with custom deleter.
  // *INDENT-OFF*
  node_handle_.reset(node, [](rmw_node_t * node) {
    auto ret = rmw_destroy_node(node);
    if (ret != RMW_RET_OK) {
      fprintf(
        stderr, "Error in destruction of rmw node handle: %s\n", rmw_get_error_string_safe());
    }
  });
  // *INDENT-ON*

  using rclcpp::callback_group::CallbackGroupType;
  default_callback_group_ = create_callback_group(
    CallbackGroupType::MutuallyExclusive);
  events_publisher_ = create_publisher<rcl_interfaces::msg::ParameterEvent>(
    "parameter_events", rmw_qos_profile_parameter_events);
}

rclcpp::callback_group::CallbackGroup::SharedPtr
Node::create_callback_group(
  rclcpp::callback_group::CallbackGroupType group_type)
{
  using rclcpp::callback_group::CallbackGroup;
  using rclcpp::callback_group::CallbackGroupType;
  auto group = CallbackGroup::SharedPtr(new CallbackGroup(group_type));
  callback_groups_.push_back(group);
  return group;
}

bool
Node::group_in_node(callback_group::CallbackGroup::SharedPtr group)
{
  bool group_belongs_to_this_node = false;
  for (auto & weak_group : this->callback_groups_) {
    auto cur_group = weak_group.lock();
    if (cur_group && (cur_group == group)) {
      group_belongs_to_this_node = true;
    }
  }
  return group_belongs_to_this_node;
}

template<typename MessageT, typename CallbackT, typename AllocWrapper>
typename rclcpp::subscription::Subscription<MessageT, AllocWrapper>::SharedPtr
Node::create_subscription(
  const std::string & topic_name,
  const rmw_qos_profile_t & qos_profile,
  CallbackT callback,
  rclcpp::callback_group::CallbackGroup::SharedPtr group,
  bool ignore_local_publications,
  AllocWrapper * allocator)
{
  rclcpp::subscription::AnySubscriptionCallback<MessageT, typename AllocWrapper::Deleter> any_subscription_callback;
  any_subscription_callback.set(callback);
  return this->create_subscription_internal<MessageT, AllocWrapper>(
    topic_name,
    qos_profile,
    any_subscription_callback,
    group,
    ignore_local_publications,
    allocator);
}

template<typename MessageT, typename AllocWrapper>
typename rclcpp::subscription::Subscription<MessageT, AllocWrapper>::SharedPtr
Node::create_subscription_with_unique_ptr_callback(
  const std::string & topic_name,
  const rmw_qos_profile_t & qos_profile,
  typename rclcpp::subscription::AnySubscriptionCallback<MessageT, typename AllocWrapper::Deleter>::UniquePtrCallback callback,
  rclcpp::callback_group::CallbackGroup::SharedPtr group,
  bool ignore_local_publications,
  AllocWrapper * allocator)
{
  rclcpp::subscription::AnySubscriptionCallback<MessageT, typename AllocWrapper::Deleter> any_subscription_callback;
  any_subscription_callback.unique_ptr_callback = callback;
  return this->create_subscription_internal<MessageT, AllocWrapper>(
    topic_name,
    qos_profile,
    any_subscription_callback,
    group,
    ignore_local_publications,
    allocator);
}

template<typename MessageT, typename AllocWrapper>
typename subscription::Subscription<MessageT, AllocWrapper>::SharedPtr
Node::create_subscription_internal(
  const std::string & topic_name,
  const rmw_qos_profile_t & qos_profile,
  rclcpp::subscription::AnySubscriptionCallback<MessageT, typename AllocWrapper::Deleter> callback,
  rclcpp::callback_group::CallbackGroup::SharedPtr group,
  bool ignore_local_publications,
  AllocWrapper * allocator)
{
  using rosidl_generator_cpp::get_message_type_support_handle;

  auto type_support_handle = get_message_type_support_handle<MessageT>();
  rmw_subscription_t * subscriber_handle = rmw_create_subscription(
    node_handle_.get(), type_support_handle,
    topic_name.c_str(), qos_profile, ignore_local_publications);
  if (!subscriber_handle) {
    // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
    throw std::runtime_error(
      std::string("could not create subscription: ") + rmw_get_error_string_safe());
    // *INDENT-ON*
  }

  using namespace rclcpp::subscription;

  auto sub = Subscription<MessageT, AllocWrapper>::make_shared(
    node_handle_,
    subscriber_handle,
    topic_name,
    ignore_local_publications,
    callback,
    allocator);

  auto sub_base_ptr = std::dynamic_pointer_cast<SubscriptionBase>(sub);
  // Setup intra process.
  if (use_intra_process_comms_) {
    rmw_subscription_t * intra_process_subscriber_handle = rmw_create_subscription(
      node_handle_.get(), ipm_ts_,
      (topic_name + "__intra").c_str(), qos_profile, false);
    if (!subscriber_handle) {
      // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
      throw std::runtime_error(
        std::string("could not create intra process subscription: ") + rmw_get_error_string_safe());
      // *INDENT-ON*
    }
    auto intra_process_manager =
      context_->get_sub_context<rclcpp::intra_process_manager::IntraProcessManager>();
    rclcpp::intra_process_manager::IntraProcessManager::WeakPtr weak_ipm = intra_process_manager;
    uint64_t intra_process_subscription_id =
      intra_process_manager->add_subscription(sub_base_ptr);
    // *INDENT-OFF*
    sub->setup_intra_process(
      intra_process_subscription_id,
      intra_process_subscriber_handle,
      [weak_ipm](
        uint64_t publisher_id,
        uint64_t message_sequence,
        uint64_t subscription_id,
        std::unique_ptr<MessageT, typename AllocWrapper::Deleter> & message)
      {
        auto ipm = weak_ipm.lock();
        if (!ipm) {
          // TODO(wjwwood): should this just return silently? Or maybe return with a logged warning?
          throw std::runtime_error(
            "intra process take called after destruction of intra process manager");
        }
        ipm->take_intra_process_message<MessageT, AllocWrapper>(publisher_id, message_sequence, subscription_id, message);
      },
      [weak_ipm](const rmw_gid_t * sender_gid) -> bool {
        auto ipm = weak_ipm.lock();
        if (!ipm) {
          throw std::runtime_error(
            "intra process publisher check called after destruction of intra process manager");
        }
        return ipm->matches_any_publishers(sender_gid);
      }
    );
    // *INDENT-ON*
  }
  // Assign to a group.
  if (group) {
    if (!group_in_node(group)) {
      // TODO: use custom exception
      throw std::runtime_error("Cannot create subscription, group not in node.");
    }
    group->add_subscription(sub_base_ptr);
  } else {
    default_callback_group_->add_subscription(sub_base_ptr);
  }
  number_of_subscriptions_++;
  return sub;
}

rclcpp::timer::WallTimer::SharedPtr
Node::create_wall_timer(
  std::chrono::nanoseconds period,
  rclcpp::timer::CallbackType callback,
  rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  auto timer = rclcpp::timer::WallTimer::make_shared(period, callback);
  if (group) {
    if (!group_in_node(group)) {
      // TODO: use custom exception
      throw std::runtime_error("Cannot create timer, group not in node.");
    }
    group->add_timer(timer);
  } else {
    default_callback_group_->add_timer(timer);
  }
  number_of_timers_++;
  return timer;
}

// TODO(wjwwood): reenable this once I figure out why the demo doesn't build with it.
// rclcpp::timer::WallTimer::SharedPtr
// Node::create_wall_timer(
//   std::chrono::duration<long double, std::nano> period,
//   rclcpp::timer::CallbackType callback,
//   rclcpp::callback_group::CallbackGroup::SharedPtr group)
// {
//   return create_wall_timer(
//     std::chrono::duration_cast<std::chrono::nanoseconds>(period),
//     callback,
//     group);
// }

template<typename ServiceT>
typename client::Client<ServiceT>::SharedPtr
Node::create_client(
  const std::string & service_name,
  rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  using rosidl_generator_cpp::get_service_type_support_handle;
  auto service_type_support_handle =
    get_service_type_support_handle<ServiceT>();

  rmw_client_t * client_handle = rmw_create_client(
    this->node_handle_.get(), service_type_support_handle, service_name.c_str());
  if (!client_handle) {
    // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
    throw std::runtime_error(
      std::string("could not create client: ") +
      rmw_get_error_string_safe());
    // *INDENT-ON*
  }

  using namespace rclcpp::client;

  auto cli = Client<ServiceT>::make_shared(
    node_handle_,
    client_handle,
    service_name);

  auto cli_base_ptr = std::dynamic_pointer_cast<ClientBase>(cli);
  if (group) {
    if (!group_in_node(group)) {
      // TODO(esteve): use custom exception
      throw std::runtime_error("Cannot create client, group not in node.");
    }
    group->add_client(cli_base_ptr);
  } else {
    default_callback_group_->add_client(cli_base_ptr);
  }
  number_of_clients_++;

  return cli;
}

template<typename ServiceT, typename FunctorT>
typename rclcpp::service::Service<ServiceT>::SharedPtr
Node::create_service(
  const std::string & service_name,
  FunctorT callback,
  rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  using rosidl_generator_cpp::get_service_type_support_handle;
  auto service_type_support_handle =
    get_service_type_support_handle<ServiceT>();

  rmw_service_t * service_handle = rmw_create_service(
    node_handle_.get(), service_type_support_handle, service_name.c_str());
  if (!service_handle) {
    // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
    throw std::runtime_error(
      std::string("could not create service: ") +
      rmw_get_error_string_safe());
    // *INDENT-ON*
  }

  auto serv = create_service_internal<ServiceT>(
    node_handle_, service_handle, service_name, callback);
  auto serv_base_ptr = std::dynamic_pointer_cast<service::ServiceBase>(serv);
  if (group) {
    if (!group_in_node(group)) {
      // TODO: use custom exception
      throw std::runtime_error("Cannot create service, group not in node.");
    }
    group->add_service(serv_base_ptr);
  } else {
    default_callback_group_->add_service(serv_base_ptr);
  }
  number_of_services_++;
  return serv;
}

std::vector<rcl_interfaces::msg::SetParametersResult>
Node::set_parameters(
  const std::vector<rclcpp::parameter::ParameterVariant> & parameters)
{
  std::vector<rcl_interfaces::msg::SetParametersResult> results;
  for (auto p : parameters) {
    auto result = set_parameters_atomically({{p}});
    results.push_back(result);
  }
  return results;
}

rcl_interfaces::msg::SetParametersResult
Node::set_parameters_atomically(
  const std::vector<rclcpp::parameter::ParameterVariant> & parameters)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::map<std::string, rclcpp::parameter::ParameterVariant> tmp_map;
  auto parameter_event = std::make_shared<rcl_interfaces::msg::ParameterEvent>();

  for (auto p : parameters) {
    if (parameters_.find(p.get_name()) == parameters_.end()) {
      if (p.get_type() != rclcpp::parameter::ParameterType::PARAMETER_NOT_SET) {
        parameter_event->new_parameters.push_back(p.to_parameter());
      }
    } else if (p.get_type() != rclcpp::parameter::ParameterType::PARAMETER_NOT_SET) {
      parameter_event->changed_parameters.push_back(p.to_parameter());
    } else {
      parameter_event->deleted_parameters.push_back(p.to_parameter());
    }
    tmp_map[p.get_name()] = p;
  }
  // std::map::insert will not overwrite elements, so we'll keep the new
  // ones and add only those that already exist in the Node's internal map
  tmp_map.insert(parameters_.begin(), parameters_.end());
  std::swap(tmp_map, parameters_);

  // TODO: handle parameter constraints
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  events_publisher_->publish(parameter_event);

  return result;
}

std::vector<rclcpp::parameter::ParameterVariant>
Node::get_parameters(
  const std::vector<std::string> & names) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<rclcpp::parameter::ParameterVariant> results;

  for (auto & name : names) {
    if (std::any_of(parameters_.cbegin(), parameters_.cend(),
      [&name](const std::pair<std::string, rclcpp::parameter::ParameterVariant> & kv) {
      return name == kv.first;
    }))
    {
      results.push_back(parameters_.at(name));
    }
  }
  return results;
}

std::vector<rcl_interfaces::msg::ParameterDescriptor>
Node::describe_parameters(
  const std::vector<std::string> & names) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<rcl_interfaces::msg::ParameterDescriptor> results;
  for (auto & kv : parameters_) {
    if (std::any_of(names.cbegin(), names.cend(), [&kv](const std::string & name) {
      return name == kv.first;
    }))
    {
      rcl_interfaces::msg::ParameterDescriptor parameter_descriptor;
      parameter_descriptor.name = kv.first;
      parameter_descriptor.type = kv.second.get_type();
      results.push_back(parameter_descriptor);
    }
  }
  return results;
}

std::vector<uint8_t>
Node::get_parameter_types(
  const std::vector<std::string> & names) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<uint8_t> results;
  for (auto & kv : parameters_) {
    if (std::any_of(names.cbegin(), names.cend(), [&kv](const std::string & name) {
      return name == kv.first;
    }))
    {
      results.push_back(kv.second.get_type());
    } else {
      results.push_back(rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET);
    }
  }
  return results;
}

rcl_interfaces::msg::ListParametersResult
Node::list_parameters(
  const std::vector<std::string> & prefixes, uint64_t depth) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  rcl_interfaces::msg::ListParametersResult result;

  // TODO(esteve): define parameter separator, use "." for now
  for (auto & kv : parameters_) {
    if (std::any_of(prefixes.cbegin(), prefixes.cend(), [&kv, &depth](const std::string & prefix) {
      if (kv.first == prefix) {
        return true;
      } else if (kv.first.find(prefix + ".") == 0) {
        size_t length = prefix.length();
        std::string substr = kv.first.substr(length);
        // Cast as unsigned integer to avoid warning
        return static_cast<uint64_t>(std::count(substr.begin(), substr.end(), '.')) < depth;
      }
      return false;
    }))
    {
      result.names.push_back(kv.first);
      size_t last_separator = kv.first.find_last_of('.');
      if (std::string::npos != last_separator) {
        std::string prefix = kv.first.substr(0, last_separator);
        if (std::find(result.prefixes.cbegin(), result.prefixes.cend(), prefix) ==
          result.prefixes.cend())
        {
          result.prefixes.push_back(prefix);
        }
      }
    }
  }
  return result;
}

std::map<std::string, std::string>
Node::get_topic_names_and_types() const
{
  rmw_topic_names_and_types_t topic_names_and_types;
  topic_names_and_types.topic_count = 0;
  topic_names_and_types.topic_names = nullptr;
  topic_names_and_types.type_names = nullptr;

  auto ret = rmw_get_topic_names_and_types(node_handle_.get(), &topic_names_and_types);
  if (ret != RMW_RET_OK) {
    // *INDENT-OFF*
    throw std::runtime_error(
      std::string("could not get topic names and types: ") + rmw_get_error_string_safe());
    // *INDENT-ON*
  }

  std::map<std::string, std::string> topics;
  for (size_t i = 0; i < topic_names_and_types.topic_count; ++i) {
    topics[topic_names_and_types.topic_names[i]] = topic_names_and_types.type_names[i];
  }

  ret = rmw_destroy_topic_names_and_types(&topic_names_and_types);
  if (ret != RMW_RET_OK) {
    // *INDENT-OFF*
    throw std::runtime_error(
      std::string("could not destroy topic names and types: ") + rmw_get_error_string_safe());
    // *INDENT-ON*
  }

  return topics;
}

size_t
Node::count_publishers(const std::string & topic_name) const
{
  size_t count;
  auto ret = rmw_count_publishers(node_handle_.get(), topic_name.c_str(), &count);
  if (ret != RMW_RET_OK) {
    // *INDENT-OFF*
    throw std::runtime_error(
      std::string("could not count publishers: ") + rmw_get_error_string_safe());
    // *INDENT-ON*
  }
  return count;
}

size_t
Node::count_subscribers(const std::string & topic_name) const
{
  size_t count;
  auto ret = rmw_count_subscribers(node_handle_.get(), topic_name.c_str(), &count);
  if (ret != RMW_RET_OK) {
    // *INDENT-OFF*
    throw std::runtime_error(
      std::string("could not count subscribers: ") + rmw_get_error_string_safe());
    // *INDENT-ON*
  }
  return count;
}

#endif /* RCLCPP_RCLCPP_NODE_IMPL_HPP_ */
