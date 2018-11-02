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

#include "rclcpp/publisher.hpp"

#include <rmw/error_handling.h>
#include <rmw/rmw.h>

#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include "rcl_interfaces/msg/intra_process_message.hpp"
#include "rcutils/logging_macros.h"
#include "rmw/impl/cpp/demangle.hpp"

#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/allocator/allocator_deleter.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"

using rclcpp::PublisherBase;

PublisherBase::PublisherBase(
  rclcpp::node_interfaces::NodeBaseInterface * node_base,
  const std::string & topic,
  const rosidl_message_type_support_t & type_support,
  const rcl_publisher_options_t & publisher_options)
: rcl_node_handle_(node_base->get_shared_rcl_node_handle()),
  intra_process_publisher_id_(0), store_intra_process_message_(nullptr)
{
  rcl_ret_t ret = rcl_publisher_init(
    &publisher_handle_,
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
  rmw_publisher_t * publisher_rmw_handle = rcl_publisher_get_rmw_handle(&publisher_handle_);
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
  if (rcl_publisher_fini(&intra_process_publisher_handle_, rcl_node_handle_.get()) != RCL_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED(
      "rclcpp",
      "Error in destruction of intra process rcl publisher handle: %s",
      rcl_get_error_string().str);
    rcl_reset_error();
  }

  if (rcl_publisher_fini(&publisher_handle_, rcl_node_handle_.get()) != RCL_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED(
      "rclcpp",
      "Error in destruction of rcl publisher handle: %s",
      rcl_get_error_string().str);
    rcl_reset_error();
  }
}

const char *
PublisherBase::get_topic_name() const
{
  return rcl_publisher_get_topic_name(&publisher_handle_);
}

size_t
PublisherBase::get_queue_size() const
{
  const rcl_publisher_options_t * publisher_options = rcl_publisher_get_options(&publisher_handle_);
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

const rmw_gid_t &
PublisherBase::get_intra_process_gid() const
{
  return intra_process_rmw_gid_;
}

rcl_publisher_t *
PublisherBase::get_publisher_handle()
{
  return &publisher_handle_;
}

const rcl_publisher_t *
PublisherBase::get_publisher_handle() const
{
  return &publisher_handle_;
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
  if (!result) {
    ret = rmw_compare_gids_equal(gid, &this->get_intra_process_gid(), &result);
    if (ret != RMW_RET_OK) {
      auto msg = std::string("failed to compare gids: ") + rmw_get_error_string().str;
      rmw_reset_error();
      throw std::runtime_error(msg);
    }
  }
  return result;
}

void
PublisherBase::setup_intra_process(
  uint64_t intra_process_publisher_id,
  StoreMessageCallbackT callback,
  const rcl_publisher_options_t & intra_process_options)
{
  const char * topic_name = this->get_topic_name();
  if (!topic_name) {
    throw std::runtime_error("failed to get topic name");
  }

  auto intra_process_topic_name = std::string(topic_name) + "/_intra";

  rcl_ret_t ret = rcl_publisher_init(
    &intra_process_publisher_handle_,
    rcl_node_handle_.get(),
    rclcpp::type_support::get_intra_process_message_msg_type_support(),
    intra_process_topic_name.c_str(),
    &intra_process_options);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_TOPIC_NAME_INVALID) {
      auto rcl_node_handle = rcl_node_handle_.get();
      // this will throw on any validation problem
      rcl_reset_error();
      expand_topic_or_service_name(
        intra_process_topic_name,
        rcl_node_get_name(rcl_node_handle),
        rcl_node_get_namespace(rcl_node_handle));
    }

    rclcpp::exceptions::throw_from_rcl_error(ret, "could not create intra process publisher");
  }

  intra_process_publisher_id_ = intra_process_publisher_id;
  store_intra_process_message_ = callback;
  // Life time of this object is tied to the publisher handle.
  rmw_publisher_t * publisher_rmw_handle = rcl_publisher_get_rmw_handle(
    &intra_process_publisher_handle_);
  if (publisher_rmw_handle == nullptr) {
    auto msg = std::string("Failed to get rmw publisher handle") + rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(msg);
  }
  auto rmw_ret = rmw_get_gid_for_publisher(
    publisher_rmw_handle, &intra_process_rmw_gid_);
  if (rmw_ret != RMW_RET_OK) {
    auto msg =
      std::string("failed to create intra process publisher gid: ") + rmw_get_error_string().str;
    rmw_reset_error();
    throw std::runtime_error(msg);
  }
}
