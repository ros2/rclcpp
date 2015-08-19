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

#ifndef RCLCPP_RCLCPP_PUBLISHER_HPP_
#define RCLCPP_RCLCPP_PUBLISHER_HPP_

#include <rclcpp/macros.hpp>

#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <rcl_interfaces/msg/intra_process_message.hpp>
#include <rmw/impl/cpp/demangle.hpp>
#include <rmw/error_handling.h>
#include <rmw/rmw.h>

namespace rclcpp
{

// Forward declaration for friend statement
namespace node
{
class Node;
} // namespace node

namespace publisher
{

class Publisher
{
  friend rclcpp::node::Node;
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Publisher);

  Publisher(
    std::shared_ptr<rmw_node_t> node_handle,
    rmw_publisher_t * publisher_handle,
    std::string topic,
    size_t queue_size)
  : node_handle_(node_handle), publisher_handle_(publisher_handle),
    intra_process_publisher_handle_(nullptr),
    topic_(topic), queue_size_(queue_size),
    intra_process_publisher_id_(0), store_intra_process_message_(nullptr)
  {}

  virtual ~Publisher()
  {
    if (intra_process_publisher_handle_) {
      if (rmw_destroy_publisher(node_handle_.get(), intra_process_publisher_handle_)) {
        fprintf(
          stderr,
          "Error in destruction of intra process rmw publisher handle: %s\n",
          rmw_get_error_string_safe());
      }
    }
    if (publisher_handle_) {
      if (rmw_destroy_publisher(node_handle_.get(), publisher_handle_) != RMW_RET_OK) {
        fprintf(
          stderr,
          "Error in destruction of rmw publisher handle: %s\n",
          rmw_get_error_string_safe());
      }
    }
  }

  template<typename MessageT>
  void
  publish(std::shared_ptr<MessageT> & msg)
  {
    rmw_ret_t status;
    if (!store_intra_process_message_) {
      // TODO(wjwwood): for now, make intra process and inter process mutually exclusive.
      // Later we'll have them together, when we have a way to filter more efficiently.
      status = rmw_publish(publisher_handle_, msg.get());
      if (status != RMW_RET_OK) {
        // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
        throw std::runtime_error(
          std::string("failed to publish message: ") + rmw_get_error_string_safe());
        // *INDENT-ON*
      }
    }
    if (store_intra_process_message_) {
      uint64_t message_seq = store_intra_process_message_(intra_process_publisher_id_, msg);
      rcl_interfaces::msg::IntraProcessMessage ipm;
      ipm.publisher_id = intra_process_publisher_id_;
      ipm.message_sequence = message_seq;
      status = rmw_publish(intra_process_publisher_handle_, &ipm);
      if (status != RMW_RET_OK) {
        // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
        throw std::runtime_error(
          std::string("failed to publish intra process message: ") + rmw_get_error_string_safe());
        // *INDENT-ON*
      }
    }
  }

  std::string
  get_topic_name() const
  {
    return topic_;
  }

  size_t
  get_queue_size() const
  {
    return queue_size_;
  }

  typedef std::function<uint64_t (uint64_t, std::shared_ptr<void>)> StoreSharedMessageCallbackT;
protected:
  void
  setup_intra_process(
    uint64_t intra_process_publisher_id,
    StoreSharedMessageCallbackT callback,
    rmw_publisher_t * intra_process_publisher_handle)
  {
    intra_process_publisher_id_ = intra_process_publisher_id;
    store_intra_process_message_ = callback;
    intra_process_publisher_handle_ = intra_process_publisher_handle;
  }

private:
  std::shared_ptr<rmw_node_t> node_handle_;

  rmw_publisher_t * publisher_handle_;
  rmw_publisher_t * intra_process_publisher_handle_;

  std::string topic_;
  size_t queue_size_;

  uint64_t intra_process_publisher_id_;
  StoreSharedMessageCallbackT store_intra_process_message_;

};

} /* namespace publisher */
} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_PUBLISHER_HPP_ */
