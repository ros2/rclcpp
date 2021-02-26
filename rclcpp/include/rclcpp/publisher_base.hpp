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

#ifndef RCLCPP__PUBLISHER_BASE_HPP_
#define RCLCPP__PUBLISHER_BASE_HPP_

#include <rmw/error_handling.h>
#include <rmw/rmw.h>

#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rcl/publisher.h"

#include "rclcpp/macros.hpp"
#include "rclcpp/network_flow_endpoint.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

// Forward declaration is used for friend statement.
namespace node_interfaces
{
class NodeBaseInterface;
class NodeTopicsInterface;
}  // namespace node_interfaces

namespace experimental
{
/**
 * IntraProcessManager is forward declared here, avoiding a circular inclusion between
 * `intra_process_manager.hpp` and `publisher_base.hpp`.
 */
class IntraProcessManager;
}  // namespace experimental

class PublisherBase : public std::enable_shared_from_this<PublisherBase>
{
  friend ::rclcpp::node_interfaces::NodeTopicsInterface;

public:
  RCLCPP_SMART_PTR_DEFINITIONS(PublisherBase)

  /// Default constructor.
  /**
   * Typically, a publisher is not created through this method, but instead is created through a
   * call to `Node::create_publisher`.
   * \param[in] node_base A pointer to the NodeBaseInterface for the parent node.
   * \param[in] topic The topic that this publisher publishes on.
   * \param[in] type_support The type support structure for the type to be published.
   * \param[in] publisher_options QoS settings for this publisher.
   */
  RCLCPP_PUBLIC
  PublisherBase(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    const std::string & topic,
    const rosidl_message_type_support_t & type_support,
    const rcl_publisher_options_t & publisher_options);

  RCLCPP_PUBLIC
  virtual ~PublisherBase();

  /// Get the topic that this publisher publishes on.
  /** \return The topic name. */
  RCLCPP_PUBLIC
  const char *
  get_topic_name() const;

  /// Get the queue size for this publisher.
  /** \return The queue size. */
  RCLCPP_PUBLIC
  size_t
  get_queue_size() const;

  /// Get the global identifier for this publisher (used in rmw and by DDS).
  /** \return The gid. */
  RCLCPP_PUBLIC
  const rmw_gid_t &
  get_gid() const;

  /// Get the rcl publisher handle.
  /** \return The rcl publisher handle. */
  RCLCPP_PUBLIC
  std::shared_ptr<rcl_publisher_t>
  get_publisher_handle();

  /// Get the rcl publisher handle.
  /** \return The rcl publisher handle. */
  RCLCPP_PUBLIC
  std::shared_ptr<const rcl_publisher_t>
  get_publisher_handle() const;

  /// Get all the QoS event handlers associated with this publisher.
  /** \return The vector of QoS event handlers. */
  RCLCPP_PUBLIC
  const std::vector<std::shared_ptr<rclcpp::QOSEventHandlerBase>> &
  get_event_handlers() const;

  /// Get subscription count
  /** \return The number of subscriptions. */
  RCLCPP_PUBLIC
  size_t
  get_subscription_count() const;

  /// Get intraprocess subscription count
  /** \return The number of intraprocess subscriptions. */
  RCLCPP_PUBLIC
  size_t
  get_intra_process_subscription_count() const;

  /// Manually assert that this Publisher is alive (for RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC).
  /**
   * If the rmw Liveliness policy is set to RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC, the creator
   * of this publisher may manually call `assert_liveliness` at some point in time to signal to the
   * rest of the system that this Node is still alive.
   *
   * \return `true` if the liveliness was asserted successfully, otherwise `false`
   */
  RCLCPP_PUBLIC
  RCUTILS_WARN_UNUSED
  bool
  assert_liveliness() const;

  /// Get the actual QoS settings, after the defaults have been determined.
  /**
   * The actual configuration applied when using RMW_QOS_POLICY_*_SYSTEM_DEFAULT
   * can only be resolved after the creation of the publisher, and it
   * depends on the underlying rmw implementation.
   * If the underlying setting in use can't be represented in ROS terms,
   * it will be set to RMW_QOS_POLICY_*_UNKNOWN.
   * May throw runtime_error when an unexpected error occurs.
   *
   * \return The actual qos settings.
   */
  RCLCPP_PUBLIC
  rclcpp::QoS
  get_actual_qos() const;

  /// Check if publisher instance can loan messages.
  /**
   * Depending on the middleware and the message type, this will return true if the middleware
   * can allocate a ROS message instance.
   */
  RCLCPP_PUBLIC
  bool
  can_loan_messages() const;

  /// Compare this publisher to a gid.
  /**
   * Note that this function calls the next function.
   * \param[in] gid Reference to a gid.
   * \return True if the publisher's gid matches the input.
   */
  RCLCPP_PUBLIC
  bool
  operator==(const rmw_gid_t & gid) const;

  /// Compare this publisher to a pointer gid.
  /**
   * A wrapper for comparing this publisher's gid to the input using rmw_compare_gids_equal.
   * \param[in] gid A pointer to a gid.
   * \return True if this publisher's gid matches the input.
   */
  RCLCPP_PUBLIC
  bool
  operator==(const rmw_gid_t * gid) const;

  using IntraProcessManagerSharedPtr =
    std::shared_ptr<rclcpp::experimental::IntraProcessManager>;

  /// Implementation utility function used to setup intra process publishing after creation.
  RCLCPP_PUBLIC
  void
  setup_intra_process(
    uint64_t intra_process_publisher_id,
    IntraProcessManagerSharedPtr ipm);

  /// Get network flow endpoints
  /**
   * Describes network flow endpoints that this publisher is sending messages out on
   * \return vector of NetworkFlowEndpoint
   */
  RCLCPP_PUBLIC
  std::vector<rclcpp::NetworkFlowEndpoint>
  get_network_flow_endpoints() const;

protected:
  template<typename EventCallbackT>
  void
  add_event_handler(
    const EventCallbackT & callback,
    const rcl_publisher_event_type_t event_type)
  {
    auto handler = std::make_shared<QOSEventHandler<EventCallbackT,
        std::shared_ptr<rcl_publisher_t>>>(
      callback,
      rcl_publisher_event_init,
      publisher_handle_,
      event_type);
    event_handlers_.emplace_back(handler);
  }

  RCLCPP_PUBLIC
  void default_incompatible_qos_callback(QOSOfferedIncompatibleQoSInfo & info) const;

  std::shared_ptr<rcl_node_t> rcl_node_handle_;

  std::shared_ptr<rcl_publisher_t> publisher_handle_;

  std::vector<std::shared_ptr<rclcpp::QOSEventHandlerBase>> event_handlers_;

  using IntraProcessManagerWeakPtr =
    std::weak_ptr<rclcpp::experimental::IntraProcessManager>;
  bool intra_process_is_enabled_;
  IntraProcessManagerWeakPtr weak_ipm_;
  uint64_t intra_process_publisher_id_;

  rmw_gid_t rmw_gid_;
};

}  // namespace rclcpp

#endif  // RCLCPP__PUBLISHER_BASE_HPP_
