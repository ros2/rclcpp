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
      rcl_publisher_options_t & publisher_options)>;

  PublisherFactoryFunction create_typed_publisher;

  // Adds the PublisherBase to the intraprocess manager with the correctly
  // templated call to IntraProcessManager::store_intra_process_message.
  using AddPublisherToIntraProcessManagerFunction = std::function<
    uint64_t(
      rclcpp::intra_process_manager::IntraProcessManager * ipm,
      rclcpp::PublisherBase::SharedPtr publisher)>;

  AddPublisherToIntraProcessManagerFunction add_publisher_to_intra_process_manager;

  // Creates the callback function which is called on each
  // PublisherT::publish() and which handles the intra process transmission of
  // the message being published.
  using SharedPublishCallbackFactoryFunction = std::function<
    rclcpp::PublisherBase::StoreMessageCallbackT(
      rclcpp::intra_process_manager::IntraProcessManager::SharedPtr ipm)>;

  SharedPublishCallbackFactoryFunction create_shared_publish_callback;
};

/// Return a PublisherFactory with functions setup for creating a PublisherT<MessageT, Alloc>.
template<typename MessageT, typename Alloc, typename PublisherT>
PublisherFactory
create_publisher_factory(std::shared_ptr<Alloc> allocator)
{
  PublisherFactory factory;

  // factory function that creates a MessageT specific PublisherT
  factory.create_typed_publisher =
    [allocator](
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    const std::string & topic_name,
    rcl_publisher_options_t & publisher_options) -> std::shared_ptr<PublisherT>
    {
      auto message_alloc = std::make_shared<typename PublisherT::MessageAlloc>(*allocator.get());
      publisher_options.allocator = allocator::get_rcl_allocator<MessageT>(*message_alloc.get());

      return std::make_shared<PublisherT>(node_base, topic_name, publisher_options, message_alloc);
    };

  // function to add a publisher to the intra process manager
  factory.add_publisher_to_intra_process_manager =
    [](
    rclcpp::intra_process_manager::IntraProcessManager * ipm,
    rclcpp::PublisherBase::SharedPtr publisher) -> uint64_t
    {
      return ipm->add_publisher<MessageT, Alloc>(std::dynamic_pointer_cast<PublisherT>(publisher));
    };

  // function to create a shared publish callback std::function
  using StoreMessageCallbackT = rclcpp::PublisherBase::StoreMessageCallbackT;
  factory.create_shared_publish_callback =
    [](rclcpp::intra_process_manager::IntraProcessManager::SharedPtr ipm) -> StoreMessageCallbackT
    {
      rclcpp::intra_process_manager::IntraProcessManager::WeakPtr weak_ipm = ipm;

      // this function is called on each call to publish() and handles storing
      // of the published message in the intra process manager
      auto shared_publish_callback =
        [weak_ipm](uint64_t publisher_id, void * msg, const std::type_info & type_info) -> uint64_t
        {
          auto ipm = weak_ipm.lock();
          if (!ipm) {
            // TODO(wjwwood): should this just return silently? Or maybe return with a warning?
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
          using MessageDeleter = typename Publisher<MessageT, Alloc>::MessageDeleter;
          std::unique_ptr<MessageT, MessageDeleter> unique_msg(typed_message_ptr);
          uint64_t message_seq =
            ipm->store_intra_process_message<MessageT, Alloc>(publisher_id, unique_msg);
          return message_seq;
        };

      return shared_publish_callback;
    };

  // return the factory now that it is populated
  return factory;
}

}  // namespace rclcpp

#endif  // RCLCPP__PUBLISHER_FACTORY_HPP_
