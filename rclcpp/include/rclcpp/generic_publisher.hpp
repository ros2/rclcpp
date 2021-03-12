// Copyright 2018, Bosch Software Innovations GmbH.
// Copyright 2021, Apex.AI Inc.
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

#ifndef RCLCPP__GENERIC_PUBLISHER_HPP_
#define RCLCPP__GENERIC_PUBLISHER_HPP_

#include <memory>
#include <string>

#include "rcpputils/shared_library.hpp"

#include "rclcpp/callback_group.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/publisher_base.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/qos.hpp"

namespace rclcpp
{

/// Publisher for serialized messages whose type is not known at compile time.
/**
 * Since the type is not known at compile time, this is not a template, and the dynamic library
 * containing type support information has to be identified and loaded based on the type name.
 *
 * This works for packages installed from the ROS repositories as well as locally built packages,
 * as long as you ensure that the `AMENT_PREFIX_PATH` environment variable has been populated with
 * the package's install location, usually by sourcing the appropriate install script. That is
 * required because it will look up the package install location for the given type, and load the
 * type support library from there.
 *
 * It does not support intra-process handling.
 */
class GenericPublisher : public rclcpp::PublisherBase
{
public:
  // cppcheck-suppress unknownMacro
  RCLCPP_SMART_PTR_DEFINITIONS(GenericPublisher)

  /// Constructor.
  /**
   * In order to properly publish to a topic, this publisher needs to be added to
   * the node_topic_interface of the node passed into this constructor.
   *
   * \sa rclcpp::Node::create_generic_publisher() or rclcpp::create_generic_publisher() for
   * creating an instance of this class and adding it to the node_topic_interface.
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

  virtual ~GenericPublisher() = default;

  /// Publish a rclcpp::SerializedMessage.
  void publish(const rclcpp::SerializedMessage & message);

private:
  // The type support library should stay loaded, so it is stored in the GenericPublisher
  std::shared_ptr<rcpputils::SharedLibrary> ts_lib_;
};

}  // namespace rclcpp

#endif  // RCLCPP__GENERIC_PUBLISHER_HPP_
