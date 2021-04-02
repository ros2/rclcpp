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
#include "rclcpp/qos.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/typesupport_helpers.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/// %Publisher for serialized messages whose type is not known at compile time.
/**
 * Since the type is not known at compile time, this is not a template, and the dynamic library
 * containing type support information has to be identified and loaded based on the type name.
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
   * \param qos %QoS settings
   * \param callback Callback for new messages of serialized form
   * \param options %Publisher options.
   * Not all publisher options are currently respected, the only relevant options for this
   * publisher are `event_callbacks`, `use_default_callbacks`, and `%callback_group`.
   */
  template<typename AllocatorT = std::allocator<void>>
  GenericPublisher(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    std::shared_ptr<rcpputils::SharedLibrary> ts_lib,
    const std::string & topic_name,
    const std::string & topic_type,
    const rclcpp::QoS & qos,
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options)
  : rclcpp::PublisherBase(
      node_base,
      topic_name,
      *rclcpp::get_typesupport_handle(topic_type, "rosidl_typesupport_cpp", *ts_lib),
      options.template to_rcl_publisher_options<rclcpp::SerializedMessage>(qos)),
    ts_lib_(ts_lib)
  {
    // This is unfortunately duplicated with the code in publisher.hpp.
    // TODO(nnmm): Deduplicate by moving this into PublisherBase.
    if (options.event_callbacks.deadline_callback) {
      this->add_event_handler(
        options.event_callbacks.deadline_callback,
        RCL_PUBLISHER_OFFERED_DEADLINE_MISSED);
    }
    if (options.event_callbacks.liveliness_callback) {
      this->add_event_handler(
        options.event_callbacks.liveliness_callback,
        RCL_PUBLISHER_LIVELINESS_LOST);
    }
    if (options.event_callbacks.incompatible_qos_callback) {
      this->add_event_handler(
        options.event_callbacks.incompatible_qos_callback,
        RCL_PUBLISHER_OFFERED_INCOMPATIBLE_QOS);
    } else if (options.use_default_callbacks) {
      // Register default callback when not specified
      try {
        this->add_event_handler(
          [this](QOSOfferedIncompatibleQoSInfo & info) {
            this->default_incompatible_qos_callback(info);
          },
          RCL_PUBLISHER_OFFERED_INCOMPATIBLE_QOS);
      } catch (UnsupportedEventTypeException & /*exc*/) {
        // pass
      }
    }
  }

  RCLCPP_PUBLIC
  virtual ~GenericPublisher() = default;

  /// Publish a rclcpp::SerializedMessage.
  RCLCPP_PUBLIC
  void publish(const rclcpp::SerializedMessage & message);

private:
  // The type support library should stay loaded, so it is stored in the GenericPublisher
  std::shared_ptr<rcpputils::SharedLibrary> ts_lib_;
};

}  // namespace rclcpp

#endif  // RCLCPP__GENERIC_PUBLISHER_HPP_
