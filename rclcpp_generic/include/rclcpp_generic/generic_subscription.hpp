// Copyright 2018, Bosch Software Innovations GmbH.
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

#ifndef RCLCPP_GENERIC__GENERIC_SUBSCRIPTION_HPP_
#define RCLCPP_GENERIC__GENERIC_SUBSCRIPTION_HPP_

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/macros.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rcpputils/shared_library.hpp"

namespace rclcpp_generic
{

/**
 * This class is an implementation of an rclcpp::SubscriptionBase for serialized messages whose topic
 * is not known at compile time (hence templating does not work).
 *
 * It does not support intra-process handling.
 */
class GenericSubscription : public rclcpp::SubscriptionBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(GenericSubscription)

  /**
   * Factory method.
   *
   * The returned pointer will never be empty, but this function can throw various exceptions, for
   * instance when the message's package can not be found on the AMENT_PREFIX_PATH.
   *
   * \param topics_interface NodeTopicsInterface pointer used in parts of the setup.
   * \param topic_name Topic name
   * \param topic_type Topic type
   * \param qos QoS settings
   * \param callback Callback for new messages of serialized form
   * \param group Callback group
   */
  static std::shared_ptr<GenericSubscription> create(
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
    const std::string & topic_name,
    const std::string & topic_type,
    const rclcpp::QoS & qos,
    std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback,
    rclcpp::CallbackGroup::SharedPtr group = nullptr);

  // Same as create_serialized_message() as the subscription is to serialized_messages only
  std::shared_ptr<void> create_message() override;

  std::shared_ptr<rclcpp::SerializedMessage> create_serialized_message() override;

  void handle_message(
    std::shared_ptr<void> & message, const rclcpp::MessageInfo & message_info) override;

  void handle_loaned_message(
    void * loaned_message, const rclcpp::MessageInfo & message_info) override;

  // Same as return_serialized_message() as the subscription is to serialized_messages only
  void return_message(std::shared_ptr<void> & message) override;

  void return_serialized_message(std::shared_ptr<rclcpp::SerializedMessage> & message) override;

private:
  RCLCPP_DISABLE_COPY(GenericSubscription)

  /**
   * Constructor. In order to properly subscribe to a topic, this subscription needs to be added to
   * the node_topic_interface of the node passed into this constructor.
   *
   * \param node_base Pointer to parent node's NodeBaseInterface
   * \param ts_lib Type support library, needs to correspond to topic_type
   * \param topic_name Topic name
   * \param topic_type Topic type
   * \param qos QoS settings
   * \param callback Callback for new messages of serialized form
   */
  GenericSubscription(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    const std::shared_ptr<rcpputils::SharedLibrary> ts_lib,
    const std::string & topic_name,
    const std::string & topic_type,
    const rclcpp::QoS & qos,
    std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback);

  std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback_;
  // The type support library should stay loaded, so it is stored in the GenericSubscription
  std::shared_ptr<rcpputils::SharedLibrary> ts_lib_;
};

}  // namespace rclcpp_generic

#endif  // RCLCPP_GENERIC__GENERIC_SUBSCRIPTION_HPP_
