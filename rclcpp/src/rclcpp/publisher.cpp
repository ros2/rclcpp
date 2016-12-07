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
#include "rmw/impl/cpp/demangle.hpp"

#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/allocator/allocator_deleter.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"

using rclcpp::publisher::PublisherBase;

PublisherBase::PublisherBase(
  rclcpp::node_interfaces::NodeBaseInterface * node_base,
  const std::string & topic,
  const rosidl_message_type_support_t & type_support,
  const rcl_publisher_options_t & publisher_options)
: rcl_node_handle_(node_base->get_shared_rcl_node_handle()),
  intra_process_publisher_id_(0), store_intra_process_message_(nullptr)
{
  if (rcl_publisher_init(
      &publisher_handle_, rcl_node_handle_.get(), &type_support,
      topic.c_str(), &publisher_options) != RCL_RET_OK)
  {
    throw std::runtime_error(
            std::string("could not create publisher: ") +
            rcl_get_error_string_safe());
  }
  // Life time of this object is tied to the publisher handle.
  rmw_publisher_t * publisher_rmw_handle = rcl_publisher_get_rmw_handle(&publisher_handle_);
  if (!publisher_rmw_handle) {
    throw std::runtime_error(
            std::string("failed to get rmw handle: ") + rcl_get_error_string_safe());
  }
  if (rmw_get_gid_for_publisher(publisher_rmw_handle, &rmw_gid_) != RMW_RET_OK) {
    // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
    throw std::runtime_error(
      std::string("failed to get publisher gid: ") + rmw_get_error_string_safe());
    // *INDENT-ON*
  }
}

PublisherBase::~PublisherBase()
{
  if (rcl_publisher_fini(&intra_process_publisher_handle_, rcl_node_handle_.get()) != RCL_RET_OK) {
    fprintf(
      stderr,
      "Error in destruction of intra process rcl publisher handle: %s\n",
      rcl_get_error_string_safe());
  }

  if (rcl_publisher_fini(&publisher_handle_, rcl_node_handle_.get()) != RCL_RET_OK) {
    fprintf(
      stderr,
      "Error in destruction of rcl publisher handle: %s\n",
      rcl_get_error_string_safe());
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
    throw std::runtime_error(
            std::string("failed to get publisher options: ") + rcl_get_error_string_safe());
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
    throw std::runtime_error(
            std::string("failed to compare gids: ") + rmw_get_error_string_safe());
  }
  if (!result) {
    ret = rmw_compare_gids_equal(gid, &this->get_intra_process_gid(), &result);
    if (ret != RMW_RET_OK) {
      throw std::runtime_error(
              std::string("failed to compare gids: ") + rmw_get_error_string_safe());
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
  auto intra_process_topic_name = std::string(this->get_topic_name()) + "__intra";
  if (rcl_publisher_init(
      &intra_process_publisher_handle_, rcl_node_handle_.get(),
      rclcpp::type_support::get_intra_process_message_msg_type_support(),
      intra_process_topic_name.c_str(), &intra_process_options) != RCL_RET_OK)
  {
    throw std::runtime_error(
            std::string("could not create intra process publisher: ") +
            rcl_get_error_string_safe());
  }

  intra_process_publisher_id_ = intra_process_publisher_id;
  store_intra_process_message_ = callback;
  // Life time of this object is tied to the publisher handle.
  rmw_publisher_t * publisher_rmw_handle = rcl_publisher_get_rmw_handle(
    &intra_process_publisher_handle_);
  if (publisher_rmw_handle == nullptr) {
    throw std::runtime_error(std::string(
              "Failed to get rmw publisher handle") + rcl_get_error_string_safe());
  }
  auto ret = rmw_get_gid_for_publisher(
    publisher_rmw_handle, &intra_process_rmw_gid_);
  if (ret != RMW_RET_OK) {
    // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
    throw std::runtime_error(
      std::string("failed to create intra process publisher gid: ") +
      rmw_get_error_string_safe());
    // *INDENT-ON*
  }
}
