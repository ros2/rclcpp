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

#include "rclcpp/subscription_base.hpp"

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/experimental/intra_process_manager.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/qos_event.hpp"

#include "rmw/error_handling.h"
#include "rmw/rmw.h"

using rclcpp::SubscriptionBase;

SubscriptionBase::SubscriptionBase(
  rclcpp::node_interfaces::NodeBaseInterface * node_base,
  const rosidl_message_type_support_t & type_support_handle,
  const std::string & topic_name,
  const rcl_subscription_options_t & subscription_options,
  bool is_serialized)
: node_base_(node_base),
  node_handle_(node_base_->get_shared_rcl_node_handle()),
  use_intra_process_(false),
  intra_process_subscription_id_(0),
  type_support_(type_support_handle),
  is_serialized_(is_serialized)
{
  auto custom_deletor = [node_handle = this->node_handle_](rcl_subscription_t * rcl_subs)
    {
      if (rcl_subscription_fini(rcl_subs, node_handle.get()) != RCL_RET_OK) {
        RCLCPP_ERROR(
          rclcpp::get_node_logger(node_handle.get()).get_child("rclcpp"),
          "Error in destruction of rcl subscription handle: %s",
          rcl_get_error_string().str);
        rcl_reset_error();
      }
      delete rcl_subs;
    };

  subscription_handle_ = std::shared_ptr<rcl_subscription_t>(
    new rcl_subscription_t, custom_deletor);
  *subscription_handle_.get() = rcl_get_zero_initialized_subscription();

  rcl_ret_t ret = rcl_subscription_init(
    subscription_handle_.get(),
    node_handle_.get(),
    &type_support_handle,
    topic_name.c_str(),
    &subscription_options);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_TOPIC_NAME_INVALID) {
      auto rcl_node_handle = node_handle_.get();
      // this will throw on any validation problem
      rcl_reset_error();
      expand_topic_or_service_name(
        topic_name,
        rcl_node_get_name(rcl_node_handle),
        rcl_node_get_namespace(rcl_node_handle));
    }

    rclcpp::exceptions::throw_from_rcl_error(ret, "could not create subscription");
  }
}

SubscriptionBase::~SubscriptionBase()
{
  if (!use_intra_process_) {
    return;
  }
  auto ipm = weak_ipm_.lock();
  if (!ipm) {
    // TODO(ivanpauno): should this raise an error?
    RCLCPP_WARN(
      rclcpp::get_logger("rclcpp"),
      "Intra process manager died before than a subscription.");
    return;
  }
  ipm->remove_subscription(intra_process_subscription_id_);
}

const char *
SubscriptionBase::get_topic_name() const
{
  return rcl_subscription_get_topic_name(subscription_handle_.get());
}

std::shared_ptr<rcl_subscription_t>
SubscriptionBase::get_subscription_handle()
{
  return subscription_handle_;
}

std::shared_ptr<const rcl_subscription_t>
SubscriptionBase::get_subscription_handle() const
{
  return subscription_handle_;
}

const std::vector<std::shared_ptr<rclcpp::QOSEventHandlerBase>> &
SubscriptionBase::get_event_handlers() const
{
  return event_handlers_;
}

rclcpp::QoS
SubscriptionBase::get_actual_qos() const
{
  const rmw_qos_profile_t * qos = rcl_subscription_get_actual_qos(subscription_handle_.get());
  if (!qos) {
    auto msg = std::string("failed to get qos settings: ") + rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(msg);
  }

  return rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(*qos), *qos);
}

bool
SubscriptionBase::take_type_erased(void * message_out, rclcpp::MessageInfo & message_info_out)
{
  rcl_ret_t ret = rcl_take(
    this->get_subscription_handle().get(),
    message_out,
    &message_info_out.get_rmw_message_info(),
    nullptr  // rmw_subscription_allocation_t is unused here
  );
  if (RCL_RET_SUBSCRIPTION_TAKE_FAILED == ret) {
    return false;
  } else if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
  if (
    matches_any_intra_process_publishers(&message_info_out.get_rmw_message_info().publisher_gid))
  {
    // In this case, the message will be delivered via intra-process and
    // we should ignore this copy of the message.
    return false;
  }
  return true;
}

bool
SubscriptionBase::take_serialized(
  rclcpp::SerializedMessage & message_out,
  rclcpp::MessageInfo & message_info_out)
{
  rcl_ret_t ret = rcl_take_serialized_message(
    this->get_subscription_handle().get(),
    &message_out.get_rcl_serialized_message(),
    &message_info_out.get_rmw_message_info(),
    nullptr);
  if (RCL_RET_SUBSCRIPTION_TAKE_FAILED == ret) {
    return false;
  } else if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
  return true;
}

const rosidl_message_type_support_t &
SubscriptionBase::get_message_type_support_handle() const
{
  return type_support_;
}

bool
SubscriptionBase::is_serialized() const
{
  return is_serialized_;
}

size_t
SubscriptionBase::get_publisher_count() const
{
  size_t inter_process_publisher_count = 0;

  rmw_ret_t status = rcl_subscription_get_publisher_count(
    subscription_handle_.get(),
    &inter_process_publisher_count);

  if (RCL_RET_OK != status) {
    rclcpp::exceptions::throw_from_rcl_error(status, "failed to get get publisher count");
  }
  return inter_process_publisher_count;
}

void
SubscriptionBase::setup_intra_process(
  uint64_t intra_process_subscription_id,
  IntraProcessManagerWeakPtr weak_ipm)
{
  intra_process_subscription_id_ = intra_process_subscription_id;
  weak_ipm_ = weak_ipm;
  use_intra_process_ = true;
}

bool
SubscriptionBase::can_loan_messages() const
{
  return rcl_subscription_can_loan_messages(subscription_handle_.get());
}

rclcpp::Waitable::SharedPtr
SubscriptionBase::get_intra_process_waitable() const
{
  // If not using intra process, shortcut to nullptr.
  if (!use_intra_process_) {
    return nullptr;
  }
  // Get the intra process manager.
  auto ipm = weak_ipm_.lock();
  if (!ipm) {
    throw std::runtime_error(
            "SubscriptionBase::get_intra_process_waitable() called "
            "after destruction of intra process manager");
  }

  // Use the id to retrieve the subscription intra-process from the intra-process manager.
  return ipm->get_subscription_intra_process(intra_process_subscription_id_);
}

void
SubscriptionBase::default_incompatible_qos_callback(
  rclcpp::QOSRequestedIncompatibleQoSInfo & event) const
{
  std::string policy_name = qos_policy_name_from_kind(event.last_policy_kind);
  RCLCPP_WARN(
    rclcpp::get_logger(rcl_node_get_logger_name(node_handle_.get())),
    "New publisher discovered on topic '%s', offering incompatible QoS. "
    "No messages will be sent to it. "
    "Last incompatible policy: %s",
    get_topic_name(),
    policy_name.c_str());
}

bool
SubscriptionBase::matches_any_intra_process_publishers(const rmw_gid_t * sender_gid) const
{
  if (!use_intra_process_) {
    return false;
  }
  auto ipm = weak_ipm_.lock();
  if (!ipm) {
    throw std::runtime_error(
            "intra process publisher check called "
            "after destruction of intra process manager");
  }
  return ipm->matches_any_publishers(sender_gid);
}

bool
SubscriptionBase::exchange_in_use_by_wait_set_state(
  void * pointer_to_subscription_part,
  bool in_use_state)
{
  if (nullptr == pointer_to_subscription_part) {
    throw std::invalid_argument("pointer_to_subscription_part is unexpectedly nullptr");
  }
  if (this == pointer_to_subscription_part) {
    return subscription_in_use_by_wait_set_.exchange(in_use_state);
  }
  if (get_intra_process_waitable().get() == pointer_to_subscription_part) {
    return intra_process_subscription_waitable_in_use_by_wait_set_.exchange(in_use_state);
  }
  for (const auto & qos_event : event_handlers_) {
    if (qos_event.get() == pointer_to_subscription_part) {
      return qos_events_in_use_by_wait_set_[qos_event.get()].exchange(in_use_state);
    }
  }
  throw std::runtime_error("given pointer_to_subscription_part does not match any part");
}

std::vector<rclcpp::NetworkFlowEndpoint> SubscriptionBase::get_network_flow_endpoints() const
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcl_network_flow_endpoint_array_t network_flow_endpoint_array =
    rcl_get_zero_initialized_network_flow_endpoint_array();
  rcl_ret_t ret = rcl_subscription_get_network_flow_endpoints(
    subscription_handle_.get(), &allocator, &network_flow_endpoint_array);
  if (RCL_RET_OK != ret) {
    auto error_msg = std::string("Error obtaining network flows of subscription: ") +
      rcl_get_error_string().str;
    rcl_reset_error();
    if (RCL_RET_OK !=
      rcl_network_flow_endpoint_array_fini(&network_flow_endpoint_array))
    {
      error_msg += std::string(". Also error cleaning up network flow array: ") +
        rcl_get_error_string().str;
      rcl_reset_error();
    }
    rclcpp::exceptions::throw_from_rcl_error(ret, error_msg);
  }

  std::vector<rclcpp::NetworkFlowEndpoint> network_flow_endpoint_vector;
  for (size_t i = 0; i < network_flow_endpoint_array.size; ++i) {
    network_flow_endpoint_vector.push_back(
      rclcpp::NetworkFlowEndpoint(
        network_flow_endpoint_array.
        network_flow_endpoint[i]));
  }

  ret = rcl_network_flow_endpoint_array_fini(&network_flow_endpoint_array);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "error cleaning up network flow array");
  }

  return network_flow_endpoint_vector;
}
