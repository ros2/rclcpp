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

#ifndef RCLCPP__PUBLISHER_FACTORY_HPP_
#define RCLCPP__PUBLISHER_FACTORY_HPP_

#include <functional>
#include <memory>
#include <string>

#include "rcl/publisher.h"

#include "rosidl_typesupport_cpp/message_type_support.hpp"

#include "rclcpp/publisher.hpp"
#include "rclcpp/intra_process_manager.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/// Factory with functions used to create a MessageT specific PublisherT.
/**
 * This factory class is used to encapsulate the template generated functions
 * which are used during the creation of a Message type specific publisher
 * within a non-templated class.
 *
 * It is created using the create_publisher_factory function, which is usually
 * called from a templated "create_publisher" method on the Node class, and
 * is passed to the non-templated "create_publisher" method on the NodeTopics
 * class where it is used to create and setup the Publisher.
 */
struct PublisherFactory
{
  // Creates a PublisherT<MessageT, ...> publisher object and returns it as a PublisherBase.
  using PublisherFactoryFunction = std::function<
    rclcpp::PublisherBase::SharedPtr(
      rclcpp::node_interfaces::NodeBaseInterface * node_base,
      const std::string & topic_name,
      const rcl_publisher_options_t & publisher_options)>;

  PublisherFactoryFunction create_typed_publisher;
};

/// Return a PublisherFactory with functions setup for creating a PublisherT<MessageT, Alloc>.
template<typename MessageT, typename Alloc, typename PublisherT>
PublisherFactory
create_publisher_factory(
  const PublisherEventCallbacks & event_callbacks,
  std::shared_ptr<Alloc> allocator)
{
  PublisherFactory factory;

  // factory function that creates a MessageT specific PublisherT
  factory.create_typed_publisher =
    [event_callbacks, allocator](
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    const std::string & topic_name,
    const rcl_publisher_options_t & publisher_options
    ) -> std::shared_ptr<PublisherT>
    {
      auto options_copy = publisher_options;
      auto message_alloc = std::make_shared<typename PublisherT::MessageAlloc>(*allocator.get());
      options_copy.allocator = allocator::get_rcl_allocator<MessageT>(*message_alloc.get());

      return std::make_shared<PublisherT>(
        node_base,
        topic_name,
        options_copy,
        event_callbacks,
        message_alloc);
    };

  // return the factory now that it is populated
  return factory;
}

}  // namespace rclcpp

#endif  // RCLCPP__PUBLISHER_FACTORY_HPP_
