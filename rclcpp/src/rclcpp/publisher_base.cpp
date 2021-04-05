// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/publisher_base.hpp"

#include <rmw/error_handling.h>
#include <rmw/rmw.h>

#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "rcutils/logging_macros.h"
#include "rmw/impl/cpp/demangle.hpp"

#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/allocator/allocator_deleter.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/experimental/intra_process_manager.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/network_flow_endpoint.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos_event.hpp"

using rclcpp::PublisherBase;

PublisherBase::PublisherBase(
  rclcpp::node_interfaces::NodeBaseInterface * node_base,
  const std::string & topic,
  const rosidl_message_type_support_t & type_support,
  const rcl_publisher_options_t & publisher_options)
: rcl_node_handle_(node_base->get_shared_rcl_node_handle()),
  intra_process_is_enabled_(false), intra_process_publisher_id_(0)
{
  auto custom_deleter = [node_handle = this->rcl_node_handle_](rcl_publisher_t * rcl_pub)
    {
      if (rcl_publisher_fini(rcl_pub, node_handle.get()) != RCL_RET_OK) {
        RCLCPP_ERROR(
          rclcpp::get_node_logger(node_handle.get()).get_child("rclcpp"),
          "Error in destruction of rcl publisher handle: %s",
          rcl_get_error_string().str);
        rcl_reset_error();
      }
      delete rcl_pub;
    };

  publisher_handle_ = std::shared_ptr<rcl_publisher_t>(
    new rcl_publisher_t, custom_deleter);
  *publisher_handle_.get() = rcl_get_zero_initialized_publisher();

  rcl_ret_t ret = rcl_publisher_init(
    publisher_handle_.get(),
    rcl_node_handle_.get(),
    &type_support,
    topic.c_str(),
    &publisher_options);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_TOPIC_NAME_INVALID) {
      auto rcl_node_handle = rcl_node_handle_.get();
      // this will throw on any validation problem
      rcl_reset_error();
      expand_topic_or_service_name(
        topic,
        rcl_node_get_name(rcl_node_handle),
        rcl_node_get_namespace(rcl_node_handle));
    }

    rclcpp::exceptions::throw_from_rcl_error(ret, "could not create publisher");
  }
  // Life time of this object is tied to the publisher handle.
  rmw_publisher_t * publisher_rmw_handle = rcl_publisher_get_rmw_handle(publisher_handle_.get());
  if (!publisher_rmw_handle) {
    auto msg = std::string("failed to get rmw handle: ") + rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(msg);
  }
  if (rmw_get_gid_for_publisher(publisher_rmw_handle, &rmw_gid_) != RMW_RET_OK) {
    auto msg = std::string("failed to get publisher gid: ") + rmw_get_error_string().str;
    rmw_reset_error();
    throw std::runtime_error(msg);
  }
}

PublisherBase::~PublisherBase()
{
  // must fini the events before fini-ing the publisher
  event_handlers_.clear();

  auto ipm = weak_ipm_.lock();

  if (!intra_process_is_enabled_) {
    return;
  }
  if (!ipm) {
    // TODO(ivanpauno): should this raise an error?
    RCLCPP_WARN(
      rclcpp::get_logger("rclcpp"),
      "Intra process manager died before a publisher.");
    return;
  }
  ipm->remove_publisher(intra_process_publisher_id_);
}

const char *
PublisherBase::get_topic_name() const
{
  return rcl_publisher_get_topic_name(publisher_handle_.get());
}

size_t
PublisherBase::get_queue_size() const
{
  const rcl_publisher_options_t * publisher_options = rcl_publisher_get_options(
    publisher_handle_.get());
  if (!publisher_options) {
    auto msg = std::string("failed to get publisher options: ") + rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(msg);
  }
  return publisher_options->qos.depth;
}

const rmw_gid_t &
PublisherBase::get_gid() const
{
  return rmw_gid_;
}

std::shared_ptr<rcl_publisher_t>
PublisherBase::get_publisher_handle()
{
  return publisher_handle_;
}

std::shared_ptr<const rcl_publisher_t>
PublisherBase::get_publisher_handle() const
{
  return publisher_handle_;
}

const std::vector<std::shared_ptr<rclcpp::QOSEventHandlerBase>> &
PublisherBase::get_event_handlers() const
{
  return event_handlers_;
}

size_t
PublisherBase::get_subscription_count() const
{
  size_t inter_process_subscription_count = 0;

  rcl_ret_t status = rcl_publisher_get_subscription_count(
    publisher_handle_.get(),
    &inter_process_subscription_count);

  if (RCL_RET_PUBLISHER_INVALID == status) {
    rcl_reset_error();  /* next call will reset error message if not context */
    if (rcl_publisher_is_valid_except_context(publisher_handle_.get())) {
      rcl_context_t * context = rcl_publisher_get_context(publisher_handle_.get());
      if (nullptr != context && !rcl_context_is_valid(context)) {
        /* publisher is invalid due to context being shutdown */
        return 0;
      }
    }
  }
  if (RCL_RET_OK != status) {
    rclcpp::exceptions::throw_from_rcl_error(status, "failed to get get subscription count");
  }
  return inter_process_subscription_count;
}

size_t
PublisherBase::get_intra_process_subscription_count() const
{
  auto ipm = weak_ipm_.lock();
  if (!intra_process_is_enabled_) {
    return 0;
  }
  if (!ipm) {
    // TODO(ivanpauno): should this just return silently? Or maybe return with a warning?
    //                  Same as wjwwood comment in publisher_factory create_shared_publish_callback.
    throw std::runtime_error(
            "intra process subscriber count called after "
            "destruction of intra process manager");
  }
  return ipm->get_subscription_count(intra_process_publisher_id_);
}

rclcpp::QoS
PublisherBase::get_actual_qos() const
{
  const rmw_qos_profile_t * qos = rcl_publisher_get_actual_qos(publisher_handle_.get());
  if (!qos) {
    auto msg = std::string("failed to get qos settings: ") + rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(msg);
  }

  return rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(*qos), *qos);
}

bool
PublisherBase::assert_liveliness() const
{
  return RCL_RET_OK == rcl_publisher_assert_liveliness(publisher_handle_.get());
}

bool
PublisherBase::can_loan_messages() const
{
  return rcl_publisher_can_loan_messages(publisher_handle_.get());
}

bool
PublisherBase::operator==(const rmw_gid_t & gid) const
{
  return *this == &gid;
}

bool
PublisherBase::operator==(const rmw_gid_t * gid) const
{
  bool result = false;
  auto ret = rmw_compare_gids_equal(gid, &this->get_gid(), &result);
  if (ret != RMW_RET_OK) {
    auto msg = std::string("failed to compare gids: ") + rmw_get_error_string().str;
    rmw_reset_error();
    throw std::runtime_error(msg);
  }
  return result;
}

void
PublisherBase::setup_intra_process(
  uint64_t intra_process_publisher_id,
  IntraProcessManagerSharedPtr ipm)
{
  intra_process_publisher_id_ = intra_process_publisher_id;
  weak_ipm_ = ipm;
  intra_process_is_enabled_ = true;
}

void
PublisherBase::default_incompatible_qos_callback(
  rclcpp::QOSOfferedIncompatibleQoSInfo & event) const
{
  std::string policy_name = qos_policy_name_from_kind(event.last_policy_kind);
  RCLCPP_WARN(
    rclcpp::get_logger(rcl_node_get_logger_name(rcl_node_handle_.get())),
    "New subscription discovered on topic '%s', requesting incompatible QoS. "
    "No messages will be sent to it. "
    "Last incompatible policy: %s",
    get_topic_name(),
    policy_name.c_str());
}

std::vector<rclcpp::NetworkFlowEndpoint> PublisherBase::get_network_flow_endpoints() const
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcl_network_flow_endpoint_array_t network_flow_endpoint_array =
    rcl_get_zero_initialized_network_flow_endpoint_array();
  rcl_ret_t ret = rcl_publisher_get_network_flow_endpoints(
    publisher_handle_.get(), &allocator, &network_flow_endpoint_array);
  if (RCL_RET_OK != ret) {
    auto error_msg = std::string("error obtaining network flows of publisher: ") +
      rcl_get_error_string().str;
    rcl_reset_error();
    if (RCL_RET_OK !=
      rcl_network_flow_endpoint_array_fini(&network_flow_endpoint_array))
    {
      error_msg += std::string(", also error cleaning up network flow array: ") +
        rcl_get_error_string().str;
      rcl_reset_error();
    }
    rclcpp::exceptions::throw_from_rcl_error(ret, error_msg);
  }

  std::vector<rclcpp::NetworkFlowEndpoint> network_flow_endpoint_vector;
  for (size_t i = 0; i < network_flow_endpoint_array.size; ++i) {
    network_flow_endpoint_vector.push_back(
      rclcpp::NetworkFlowEndpoint(
        network_flow_endpoint_array.network_flow_endpoint[i]));
  }

  ret = rcl_network_flow_endpoint_array_fini(&network_flow_endpoint_array);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "error cleaning up network flow array");
  }

  return network_flow_endpoint_vector;
}
