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

#ifndef RCLCPP_GENERIC__GENERIC_PUBLISHER_HPP_
#define RCLCPP_GENERIC__GENERIC_PUBLISHER_HPP_

#include <memory>
#include <string>

#include "rclcpp/publisher_base.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/callback_group.hpp"
#include "rcpputils/shared_library.hpp"

namespace rclcpp_generic
{

/**
 * This class is an implementation of an rclcpp::PublisherBase for serialized messages whose topic
 * is not known at compile time (hence templating does not work).
 *
 * It does not support intra-process handling.
 */
class GenericPublisher : public rclcpp::PublisherBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(GenericPublisher)

  /**
   * Factory method.
   *
   * The returned pointer will never be empty, but this function can throw various exceptions, for
   * instance when the message's package can not be found on the AMENT_PREFIX_PATH.
   *
   * \param topics_interface NodeTopicsInterface pointer used in parts of the setup
   * \param topic_name Topic name
   * \param topic_type Topic type
   * \param qos QoS settings
   * \param group Callback group
   */
  static std::shared_ptr<GenericPublisher> create(
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
    const std::string & topic_name,
    const std::string & topic_type,
    const rclcpp::QoS & qos,
    rclcpp::CallbackGroup::SharedPtr group = nullptr);

  virtual ~GenericPublisher() = default;

  void publish(std::shared_ptr<rmw_serialized_message_t> message);

private:
  /**
   * Constructor. In order to properly publish to a topic, this publisher needs to be added to
   * the node_topic_interface of the node passed into this constructor.
   *
   * \param node_base Pointer to parent node's NodeBaseInterface
   * \param ts_lib Type support library, needs to correspond to topic_type
   * \param topic_name Topic name
   * \param topic_type Topic type
   * \param qos QoS settings
   * \param callback Callback for new messages of serialized form
   */
  GenericPublisher(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    std::shared_ptr<rcpputils::SharedLibrary> ts_lib,
    const std::string & topic_name,
    const std::string & topic_type,
    const rclcpp::QoS & qos);

  // The type support library should stay loaded, so it is stored in the GenericPublisher
  std::shared_ptr<rcpputils::SharedLibrary> ts_lib_;
};

}  // namespace rclcpp_generic

#endif  // RCLCPP_GENERIC__GENERIC_PUBLISHER_HPP_
