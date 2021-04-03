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

#include "rclcpp/any_subscription_callback.hpp"
#include "rclcpp/intra_process_buffer_type.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/subscription_traits.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/topic_statistics/subscription_topic_statistics.hpp"

namespace rclcpp
{

/// Factory containing a function used to create a Subscription<MessageT>.
/**
 * This factory class is used to encapsulate the template generated function
 * which is used during the creation of a Message type specific subscription
 * within a non-templated class.
 *
 * It is created using the create_subscription_factory function, which is
 * usually called from a templated "create_subscription" method of the Node
 * class, and is passed to the non-templated "create_subscription" method of
 * the NodeTopics class where it is used to create and setup the Subscription.
 *
 * It also handles the two step construction of Subscriptions, first calling
 * the constructor and then the post_init_setup() method.
 */
struct SubscriptionFactory
{
  // Creates a Subscription<MessageT> object and returns it as a SubscriptionBase.
  using SubscriptionFactoryFunction = std::function<
    rclcpp::SubscriptionBase::SharedPtr(
      rclcpp::node_interfaces::NodeBaseInterface * node_base,
      const std::string & topic_name,
      const rclcpp::QoS & qos)>;

  const SubscriptionFactoryFunction create_typed_subscription;
};

/// Return a SubscriptionFactory setup to create a SubscriptionT<MessageT, AllocatorT>.
/**
 * \param[in] callback The user-defined callback function to receive a message
 * \param[in] options Additional options for the creation of the Subscription.
 * \param[in] msg_mem_strat The message memory strategy to use for allocating messages.
 * \param[in] subscription_topic_stats Optional stats callback for topic_statistics
 */
template<
  typename MessageT,
  typename CallbackT,
  typename AllocatorT,
  typename CallbackMessageT =
  typename rclcpp::subscription_traits::has_message_type<CallbackT>::type,
  typename SubscriptionT = rclcpp::Subscription<CallbackMessageT, AllocatorT>,
  typename MessageMemoryStrategyT = rclcpp::message_memory_strategy::MessageMemoryStrategy<
    CallbackMessageT,
    AllocatorT
  >>
SubscriptionFactory
create_subscription_factory(
  CallbackT && callback,
  const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options,
  typename MessageMemoryStrategyT::SharedPtr msg_mem_strat,
  std::shared_ptr<rclcpp::topic_statistics::SubscriptionTopicStatistics<CallbackMessageT>>
  subscription_topic_stats = nullptr
)
{
  auto allocator = options.get_allocator();

  using rclcpp::AnySubscriptionCallback;
  AnySubscriptionCallback<CallbackMessageT, AllocatorT> any_subscription_callback(*allocator);
  any_subscription_callback.set(std::forward<CallbackT>(callback));

  SubscriptionFactory factory {
    // factory function that creates a MessageT specific SubscriptionT
    [options, msg_mem_strat, any_subscription_callback, subscription_topic_stats](
      rclcpp::node_interfaces::NodeBaseInterface * node_base,
      const std::string & topic_name,
      const rclcpp::QoS & qos
    ) -> rclcpp::SubscriptionBase::SharedPtr
    {
      using rclcpp::Subscription;
      using rclcpp::SubscriptionBase;

      auto sub = Subscription<CallbackMessageT, AllocatorT>::make_shared(
        node_base,
        *rosidl_typesupport_cpp::get_message_type_support_handle<MessageT>(),
        topic_name,
        qos,
        any_subscription_callback,
        options,
        msg_mem_strat,
        subscription_topic_stats);
      // This is used for setting up things like intra process comms which
      // require this->shared_from_this() which cannot be called from
      // the constructor.
      sub->post_init_setup(node_base, qos, options);
      auto sub_base_ptr = std::dynamic_pointer_cast<SubscriptionBase>(sub);
      return sub_base_ptr;
    }
  };

  // return the factory now that it is populated
  return factory;
}

}  // namespace rclcpp

#endif  // RCLCPP__SUBSCRIPTION_FACTORY_HPP_
