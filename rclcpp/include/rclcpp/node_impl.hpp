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

#ifndef RCLCPP__NODE_IMPL_HPP_
#define RCLCPP__NODE_IMPL_HPP_

#include <rmw/error_handling.h>
#include <rmw/rmw.h>

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rcl_interfaces/msg/intra_process_message.hpp"

#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/intra_process_manager.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/visibility_control.hpp"

#ifndef RCLCPP__NODE_HPP_
#include "node.hpp"
#endif

namespace rclcpp
{
namespace node
{

template<typename MessageT, typename Alloc>
typename rclcpp::publisher::Publisher<MessageT, Alloc>::SharedPtr
Node::create_publisher(
  const std::string & topic_name, size_t qos_history_depth,
  std::shared_ptr<Alloc> allocator)
{
  if (!allocator) {
    allocator = std::make_shared<Alloc>();
  }
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.depth = qos_history_depth;
  return this->create_publisher<MessageT, Alloc>(topic_name, qos, allocator);
}

template<typename MessageT, typename Alloc>
typename publisher::Publisher<MessageT, Alloc>::SharedPtr
Node::create_publisher(
  const std::string & topic_name, const rmw_qos_profile_t & qos_profile,
  std::shared_ptr<Alloc> allocator)
{
  if (!allocator) {
    allocator = std::make_shared<Alloc>();
  }
  using rosidl_generator_cpp::get_message_type_support_handle;
  auto type_support_handle = get_message_type_support_handle<MessageT>();
  rmw_publisher_t * publisher_handle = rmw_create_publisher(
    node_handle_.get(), type_support_handle, topic_name.c_str(), qos_profile);
  if (!publisher_handle) {
    // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
    throw std::runtime_error(
      std::string("could not create publisher: ") +
      rmw_get_error_string_safe());
    // *INDENT-ON*
  }

  auto publisher = publisher::Publisher<MessageT, Alloc>::make_shared(
    node_handle_, publisher_handle, topic_name, qos_profile.depth, allocator);

  if (use_intra_process_comms_) {
    rmw_publisher_t * intra_process_publisher_handle = rmw_create_publisher(
      node_handle_.get(), rclcpp::type_support::get_intra_process_message_msg_type_support(),
      (topic_name + "__intra").c_str(), qos_profile);
    if (!intra_process_publisher_handle) {
      // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
      throw std::runtime_error(
        std::string("could not create intra process publisher: ") +
        rmw_get_error_string_safe());
      // *INDENT-ON*
    }

    auto intra_process_manager =
      context_->get_sub_context<rclcpp::intra_process_manager::IntraProcessManager>();
    uint64_t intra_process_publisher_id =
      intra_process_manager->add_publisher<MessageT, Alloc>(publisher);
    rclcpp::intra_process_manager::IntraProcessManager::WeakPtr weak_ipm = intra_process_manager;
    // *INDENT-OFF*
    auto shared_publish_callback =
      [weak_ipm](uint64_t publisher_id, void * msg, const std::type_info & type_info) -> uint64_t
    {
      auto ipm = weak_ipm.lock();
      if (!ipm) {
        // TODO(wjwwood): should this just return silently? Or maybe return with a logged warning?
        throw std::runtime_error(
          "intra process publish called after destruction of intra process manager");
      }
      if (!msg) {
        throw std::runtime_error("cannot publisher msg which is a null pointer");
      }
      auto & message_type_info = typeid(MessageT);
      if (message_type_info != type_info) {
        throw std::runtime_error(
          std::string("published type '") + type_info.name() +
          "' is incompatible from the publisher type '" + message_type_info.name() + "'");
      }
      MessageT * typed_message_ptr = static_cast<MessageT *>(msg);
      using MessageDeleter = typename publisher::Publisher<MessageT, Alloc>::MessageDeleter;
      std::unique_ptr<MessageT, MessageDeleter> unique_msg(typed_message_ptr);
      uint64_t message_seq =
        ipm->store_intra_process_message<MessageT, Alloc>(publisher_id, unique_msg);
      return message_seq;
    };
    // *INDENT-ON*
    publisher->setup_intra_process(
      intra_process_publisher_id,
      shared_publish_callback,
      intra_process_publisher_handle);
  }
  return publisher;
}

template<typename MessageT, typename CallbackT, typename Alloc>
typename rclcpp::subscription::Subscription<MessageT, Alloc>::SharedPtr
Node::create_subscription(
  const std::string & topic_name,
  CallbackT && callback,
  const rmw_qos_profile_t & qos_profile,
  rclcpp::callback_group::CallbackGroup::SharedPtr group,
  bool ignore_local_publications,
  typename rclcpp::message_memory_strategy::MessageMemoryStrategy<MessageT, Alloc>::SharedPtr
  msg_mem_strat,
  typename std::shared_ptr<Alloc> allocator)
{
  if (!allocator) {
    allocator = std::make_shared<Alloc>();
  }

  rclcpp::subscription::AnySubscriptionCallback<MessageT,
  Alloc> any_subscription_callback(allocator);
  any_subscription_callback.set(std::forward<CallbackT>(callback));

  using rosidl_generator_cpp::get_message_type_support_handle;

  if (!msg_mem_strat) {
    msg_mem_strat =
      rclcpp::message_memory_strategy::MessageMemoryStrategy<MessageT, Alloc>::create_default();
  }

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

  using rclcpp::subscription::Subscription;
  using rclcpp::subscription::SubscriptionBase;

  auto sub = Subscription<MessageT, Alloc>::make_shared(
    node_handle_,
    subscriber_handle,
    topic_name,
    ignore_local_publications,
    any_subscription_callback,
    msg_mem_strat);
  auto sub_base_ptr = std::dynamic_pointer_cast<SubscriptionBase>(sub);
  // Setup intra process.
  if (use_intra_process_comms_) {
    rmw_subscription_t * intra_process_subscriber_handle = rmw_create_subscription(
      node_handle_.get(), rclcpp::type_support::get_intra_process_message_msg_type_support(),
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
        typename Subscription<MessageT, Alloc>::MessageUniquePtr & message)
      {
        auto ipm = weak_ipm.lock();
        if (!ipm) {
          // TODO(wjwwood): should this just return silently? Or maybe return with a logged warning?
          throw std::runtime_error(
            "intra process take called after destruction of intra process manager");
        }
        ipm->take_intra_process_message<MessageT, Alloc>(
          publisher_id, message_sequence, subscription_id, message);
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
      // TODO(jacquelinekay): use custom exception
      throw std::runtime_error("Cannot create subscription, group not in node.");
    }
    group->add_subscription(sub_base_ptr);
  } else {
    default_callback_group_->add_subscription(sub_base_ptr);
  }
  number_of_subscriptions_++;
  return sub;
}

template<typename MessageT, typename CallbackT, typename Alloc>
typename rclcpp::subscription::Subscription<MessageT, Alloc>::SharedPtr
Node::create_subscription(
  const std::string & topic_name,
  size_t qos_history_depth,
  CallbackT && callback,
  rclcpp::callback_group::CallbackGroup::SharedPtr group,
  bool ignore_local_publications,
  typename rclcpp::message_memory_strategy::MessageMemoryStrategy<MessageT, Alloc>::SharedPtr
  msg_mem_strat,
  std::shared_ptr<Alloc> allocator)
{
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.depth = qos_history_depth;
  return this->create_subscription<MessageT, CallbackT, Alloc>(
    topic_name,
    std::forward<CallbackT>(callback),
    qos,
    group,
    ignore_local_publications,
    msg_mem_strat,
    allocator);
}

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

  using rclcpp::client::Client;
  using rclcpp::client::ClientBase;

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

template<typename ServiceT, typename CallbackT>
typename rclcpp::service::Service<ServiceT>::SharedPtr
Node::create_service(
  const std::string & service_name,
  CallbackT && callback,
  rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  using rosidl_generator_cpp::get_service_type_support_handle;
  auto service_type_support_handle =
    get_service_type_support_handle<ServiceT>();

  rclcpp::service::AnyServiceCallback<ServiceT> any_service_callback;
  any_service_callback.set(std::forward<CallbackT>(callback));

  rmw_service_t * service_handle = rmw_create_service(
    node_handle_.get(), service_type_support_handle, service_name.c_str());
  if (!service_handle) {
    // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
    throw std::runtime_error(
      std::string("could not create service: ") +
      rmw_get_error_string_safe());
    // *INDENT-ON*
  }

  auto serv = service::Service<ServiceT>::make_shared(
    node_handle_, service_handle, service_name, any_service_callback);
  auto serv_base_ptr = std::dynamic_pointer_cast<service::ServiceBase>(serv);
  if (group) {
    if (!group_in_node(group)) {
      // TODO(jacquelinekay): use custom exception
      throw std::runtime_error("Cannot create service, group not in node.");
    }
    group->add_service(serv_base_ptr);
  } else {
    default_callback_group_->add_service(serv_base_ptr);
  }
  number_of_services_++;
  return serv;
}

}  // namespace node
}  // namespace rclcpp

#endif  // RCLCPP__NODE_IMPL_HPP_
