// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include <rclcpp_action/client.hpp>

#include <string>

#include <rcl_action/client.h>


using rclcpp_action::ClientBase;

namespace rclcpp_action
{
class ClientBaseImpl
{
public:
  ClientBaseImpl(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    const std::string & action_name,
    const rosidl_action_type_support_t * type_support,
    const rcl_action_client_options_t & client_options)
  {
    std::weak_ptr<rcl_node_t> weak_node_handle(node_handle_);
    client_handle_ = std::shared_ptr<rcl_action_client_t>(
      new rcl_client_t, [weak_node_handle](rcl_action_client_t * client)
      {
        auto handle = weak_node_handle.lock();
        if (handle) {
          if (RCL_RET_OK != rcl_action_client_fini(client, handle.get())) {
            RCLCPP_ERROR(
              rclcpp::get_logger(rcl_node_get_logger_name(handle.get())).get_child("rclcpp"),
              "Error in destruction of rcl client handle: %s", rcl_get_error_string().str);
            rcl_reset_error();
          }
        } else {
          RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"),
            "Error in destruction of rcl client handle: "
            "the Node Handle was destructed too early. You will leak memory");
        }
        delete client;
      });
    *client_handle_ = rcl_get_zero_initialized_client();
    rcl_ret_t ret = rcl_action_client_init(
      client_handle_.get(), node_handle_.get(), type_support,
      action_name.c_str(), &action_client_options);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(
        ret, "could not create action client");
    }

  }

  void
  handle_goal_response(
    std::shared_ptr<rmw_request_id_t> response_header,
    std::shared_ptr<void> response)
  {
    std::unique_lock<std::mutex> lock(goals_mutex_);
    const int64_t sequence_number = response_header->sequence_number;
    if (pending_goal_responses_.count(sequence_number) == 0) {
      RCUTILS_LOG_ERROR_NAMED("rclcpp", "unknown goal response, ignoring...");
      return;
    }
    pending_goal_responses_[sequence_number](response);
    pending_goal_responses_.erase(sequence_number);
  }

  void
  send_goal_request(std::shared_ptr<void> request, ResponseCallback callback)
  {
    std::unique_lock<std::mutex> lock(goals_mutex_);
    int64_t sequence_number;
    rcl_ret_t ret = rcl_action_send_goal_request(
      client_handle_.get(), request.get(), &sequence_number);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send goal request");
    }
    if (pending_goal_responses_.count(sequence_number) != 0) {
      
    }
    pending_goal_responses_[sequence_number] = callback;
  }
  
  void
  send_result_request(std::shared_ptr<void> request, ResponseCallback callback)
  {
    std::unique_lock<std::mutex> lock(results_mutex_);
    int64_t sequence_number;
    rcl_ret_t ret = rcl_action_send_result_request(
      client_handle_.get(), request.get(), &sequence_number);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send goal request");
    }
    if (pending_result_responses_.count(sequence_number) != 0) {
      
    }
    pending_result_responses_[sequence_number] = callback;
  }

  void
  handle_cancel_response(
    std::shared_ptr<rmw_request_id_t> response_header,
    std::shared_ptr<void> response)
  {
    std::unique_lock<std::mutex> lock(cancellations_mutex_);
    const int64_t sequence_number = response_header->sequence_number;
    if (pending_cancel_responses_.count(sequence_number) == 0) {
      RCUTILS_LOG_ERROR_NAMED("rclcpp", "unknown cancel response, ignoring...");
      return;
    }
    pending_cancel_responses_[sequence_number](response);
    pending_cancel_responses_.erase(sequence_number);
  }

  void
  send_cancel_request(std::shared_ptr<void> request, ResponseCallback callback)
  {
    std::unique_lock<std::mutex> lock(cancellations_mutex_);
    int64_t sequence_number;
    rcl_ret_t ret = rcl_action_send_cancel_request(
      client_handle_.get(), request.get(), &sequence_number);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send cancel request");
    }
    if (pending_cancel_responses_.count(sequence_number) != 0) {
      
    }
    pending_cancel_responses_[sequence_number] = callback;
  }

private:
  std::map<int64_t, ResponseCallback> pending_goal_responses_;
  std::map<int64_t, ResponseCallback> pending_result_responses_;
  std::map<int64_t, ResponseCallback> pending_cancel_responses_;
  std::shared_ptr<rcl_action_client_t> client_handle_;
  std::shared_ptr<rcl_node_t> node_handle_;
  std::mutex client_mutex_;
};
} 

ClientBase::ClientBase(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  const std::string & action_name,
  const rosidl_action_type_support_t * type_support,
  const rcl_action_client_options_t & client_options)
    : pimpl_(new ClientBaseImpl(
      node_base, action_name,
      type_support, client_options))
{
  
}

bool
ClientBase::wait_for_action_nanoseconds(std::chrono::nanoseconds timeout)
{
  
}


ClientBase::~ClientBase()
{
  
}
