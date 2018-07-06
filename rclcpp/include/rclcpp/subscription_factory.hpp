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

#ifndef RCLCPP__SUBSCRIPTION_FACTORY_HPP_
#define RCLCPP__SUBSCRIPTION_FACTORY_HPP_

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "rcl/subscription.h"

#include "rosidl_typesupport_cpp/message_type_support.hpp"

#include "rclcpp/subscription.hpp"
#include "rclcpp/subscription_traits.hpp"
#include "rclcpp/intra_process_manager.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/// Factory with functions used to create a Subscription<MessageT>.
/**
 * This factory class is used to encapsulate the template generated functions
 * which are used during the creation of a Message type specific subscription
 * within a non-templated class.
 *
 * It is created using the create_subscription_factory function, which is
 * usually called from a templated "create_subscription" method of the Node
 * class, and is passed to the non-templated "create_subscription" method of
 * the NodeTopics class where it is used to create and setup the Subscription.
 */
struct SubscriptionFactory
{
  // Creates a Subscription<MessageT> object and returns it as a SubscriptionBase.
  using SubscriptionFactoryFunction = std::function<
    rclcpp::SubscriptionBase::SharedPtr(
      rclcpp::node_interfaces::NodeBaseInterface * node_base,
      const std::string & topic_name,
      rcl_subscription_options_t & subscription_options)>;

  SubscriptionFactoryFunction create_typed_subscription;

  // Function that takes a MessageT from the intra process manager
  using SetupIntraProcessFunction = std::function<
    void (
      rclcpp::intra_process_manager::IntraProcessManager::SharedPtr ipm,
      rclcpp::SubscriptionBase::SharedPtr subscription,
      const rcl_subscription_options_t & subscription_options)>;

  SetupIntraProcessFunction setup_intra_process;
};

/// Return a SubscriptionFactory with functions for creating a SubscriptionT<MessageT, Alloc>.
template<
  typename MessageT,
  typename CallbackT,
  typename Alloc,
  typename CallbackMessageT,
  typename SubscriptionT>
SubscriptionFactory
create_subscription_factory(
  CallbackT && callback,
  typename rclcpp::message_memory_strategy::MessageMemoryStrategy<
    CallbackMessageT, Alloc>::SharedPtr
  msg_mem_strat,
  std::shared_ptr<Alloc> allocator)
{
  SubscriptionFactory factory;

  using rclcpp::AnySubscriptionCallback;
  AnySubscriptionCallback<CallbackMessageT, Alloc> any_subscription_callback(allocator);
  any_subscription_callback.set(std::forward<CallbackT>(callback));

  auto message_alloc =
    std::make_shared<typename Subscription<CallbackMessageT, Alloc>::MessageAlloc>();

  // factory function that creates a MessageT specific SubscriptionT
  factory.create_typed_subscription =
    [allocator, msg_mem_strat, any_subscription_callback, message_alloc](
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    const std::string & topic_name,
    rcl_subscription_options_t & subscription_options
    ) -> rclcpp::SubscriptionBase::SharedPtr
    {
      subscription_options.allocator =
        rclcpp::allocator::get_rcl_allocator<CallbackMessageT>(*message_alloc.get());

      using rclcpp::Subscription;
      using rclcpp::SubscriptionBase;

      auto sub = Subscription<CallbackMessageT, Alloc>::make_shared(
        node_base->get_shared_rcl_node_handle(),
        *rosidl_typesupport_cpp::get_message_type_support_handle<MessageT>(),
        topic_name,
        subscription_options,
        any_subscription_callback,
        msg_mem_strat);
      auto sub_base_ptr = std::dynamic_pointer_cast<SubscriptionBase>(sub);
      return sub_base_ptr;
    };

  // function that will setup intra process communications for the subscription
  factory.setup_intra_process =
    [message_alloc](
    rclcpp::intra_process_manager::IntraProcessManager::SharedPtr ipm,
    rclcpp::SubscriptionBase::SharedPtr subscription,
    const rcl_subscription_options_t & subscription_options)
    {
      rclcpp::intra_process_manager::IntraProcessManager::WeakPtr weak_ipm = ipm;
      uint64_t intra_process_subscription_id = ipm->add_subscription(subscription);

      auto intra_process_options = rcl_subscription_get_default_options();
      intra_process_options.allocator = rclcpp::allocator::get_rcl_allocator<CallbackMessageT>(
        *message_alloc.get());
      intra_process_options.qos = subscription_options.qos;
      intra_process_options.ignore_local_publications = false;

      // function that will be called to take a MessageT from the intra process manager
      auto take_intra_process_message_func =
        [weak_ipm](
        uint64_t publisher_id,
        uint64_t message_sequence,
        uint64_t subscription_id,
        typename rclcpp::Subscription<CallbackMessageT, Alloc>::MessageUniquePtr & message)
        {
          auto ipm = weak_ipm.lock();
          if (!ipm) {
            // TODO(wjwwood): should this just return silently? Or return with a logged warning?
            throw std::runtime_error(
                    "intra process take called after destruction of intra process manager");
          }
          ipm->take_intra_process_message<CallbackMessageT, Alloc>(
            publisher_id, message_sequence, subscription_id, message);
        };

      // function that is called to see if the publisher id matches any local publishers
      auto matches_any_publisher_func =
        [weak_ipm](const rmw_gid_t * sender_gid) -> bool
        {
          auto ipm = weak_ipm.lock();
          if (!ipm) {
            throw std::runtime_error(
                    "intra process publisher check called "
                    "after destruction of intra process manager");
          }
          return ipm->matches_any_publishers(sender_gid);
        };

      auto typed_sub_ptr = std::dynamic_pointer_cast<SubscriptionT>(subscription);
      typed_sub_ptr->setup_intra_process(
        intra_process_subscription_id,
        take_intra_process_message_func,
        matches_any_publisher_func,
        intra_process_options
      );
    };
  // end definition of factory function to setup intra process

  // return the factory now that it is populated
  return factory;
}

}  // namespace rclcpp

#endif  // RCLCPP__SUBSCRIPTION_FACTORY_HPP_
