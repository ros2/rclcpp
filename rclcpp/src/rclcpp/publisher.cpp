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

using rclcpp::publisher::PublisherBase;

PublisherBase::PublisherBase(
  std::shared_ptr<rcl_node_t> node_handle,
  rcl_publisher_t * publisher_handle,
  std::string topic,
  size_t queue_size)
: node_handle_(node_handle), publisher_handle_(publisher_handle),
  intra_process_publisher_handle_(nullptr),
  topic_(topic), queue_size_(queue_size),
  intra_process_publisher_id_(0), store_intra_process_message_(nullptr)
{
  // Life time of this object is tied to the publisher handle.
  if (rmw_get_gid_for_publisher(
    rcl_publisher_get_rmw_handle(publisher_handle_), &rmw_gid_) != RMW_RET_OK)
  {
    // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
    throw std::runtime_error(
      std::string("failed to get publisher gid: ") + rmw_get_error_string_safe());
    // *INDENT-ON*
  }
}

PublisherBase::~PublisherBase()
{
  if (intra_process_publisher_handle_) {
    if (rcl_publisher_fini(intra_process_publisher_handle_, node_handle_.get()) != RCL_RET_OK) {
      fprintf(
        stderr,
        "Error in destruction of intra process rcl publisher handle: %s\n",
        rcl_get_error_string_safe());

    }
    // TODO custom deallocator
    delete intra_process_publisher_handle_;
  }
  if (publisher_handle_) {
    if (rcl_publisher_fini(publisher_handle_, node_handle_.get()) != RCL_RET_OK) {
      fprintf(
        stderr,
        "Error in destruction of rcl publisher handle: %s\n",
        rcl_get_error_string_safe());

    }
    // TODO custom deallocator
    delete publisher_handle_;  }
}

const std::string &
PublisherBase::get_topic_name() const
{
  return topic_;
}

size_t
PublisherBase::get_queue_size() const
{
  return queue_size_;
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
  rcl_publisher_t * intra_process_publisher_handle)
{
  intra_process_publisher_id_ = intra_process_publisher_id;
  store_intra_process_message_ = callback;
  intra_process_publisher_handle_ = intra_process_publisher_handle;
  // Life time of this object is tied to the publisher handle.
  auto ret = rmw_get_gid_for_publisher(
    rcl_publisher_get_rmw_handle(intra_process_publisher_handle_), &intra_process_rmw_gid_);
  if (ret != RMW_RET_OK) {
    // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
    throw std::runtime_error(
      std::string("failed to create intra process publisher gid: ") +
      rmw_get_error_string_safe());
    // *INDENT-ON*
  }
}
