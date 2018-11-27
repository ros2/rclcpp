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
    ret = rcl_action_client_wait_set_get_num_entities(
        client_handle_.get()
        &num_subscriptions,
        &num_guard_conditions,
        &num_timers,
        &num_clients,
        &num_services);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(
        ret, "could not retrieve action client details");
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
    assert(pending_goal_responses_.count(sequence_number) != 0);
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
    assert(pending_result_responses_.count(sequence_number) != 0);
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

  std::shared_ptr<rcl_action_client_t>
  get_action_client()
  {
    return client_handle_;
  }

  // rcl interface entities
  site_t num_subscriptions_,
    num_guard_conditions_,
    num_timers_,
    num_clients_,
    num_services_;

  // rcl interface flags
  bool is_feedback_ready_,
    is_status_ready_,
    is_goal_response_ready_,
    is_cancel_response_ready_,
    is_result_response_ready_;
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

size_t
ClientBase::get_number_of_ready_subscriptions() override
{
  return pimpl_->num_subscriptions_;
}

size_t
ClientBase::get_number_of_ready_guard_conditions() override
{
  return pimpl_->num_guard_conditions_;
}

size_t
ClientBase::get_number_of_ready_timers() override
{
  return pimpl_->num_timers_;
}

size_t
ClientBase::get_number_of_ready_clients() override
{
  return pimpl_->num_clients_;
}

size_t
ClientBase::get_number_of_ready_services() override
{
  return pimpl_->num_services_;
}

bool
ClientBase::add_to_wait_set(rcl_wait_set_t * wait_set) override
{
  rcl_ret_t ret = rcl_action_wait_set_add_action_client(
    wait_set, pimpl_->get_action_client().get());
  return (RCL_RET_OK != ret);
}

bool
ClientBase::is_ready(rcl_wait_set_t * wait_set) override
{
  rcl_ret_t ret = rcl_action_client_wait_set_get_entities_ready(
    wait_set, pimpl_->get_action_client().get(),
    &is_feedback_ready_,
    &is_status_ready_,
    &is_goal_response_ready_,
    &is_cancel_response_ready_,
    &is_result_response_ready_);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(
      ret, "failed to check for ready entities");
  }
  return is_feedback_ready_ ||
         is_status_ready_ ||
         is_goal_response_ready_ ||
         is_cancel_response_ready_ ||
         is_result_response_ready_;
}

void
ClientBase::execute() override
{
  std::shared_ptr<void> client_handle = pimpl_->get_client_handle();
  if (pimpl_->is_feedback_ready)
  {
    std::shared_ptr<void> feedback_message = this->create_feedback_message();
    rcl_ret_t ret = rcl_action_take_feedback(client_handle.get(), feedback_message.get());
    if (RCL_RET_OK == ret) {
    } else {
      this->handle_feedback(feedback_message);
      is_feedback_ready = false;
    }
  }
  if (is_status_ready_)
  {
    std::shared_ptr<void> status_message = this->create_status_message();
    rcl_ret_t ret = rcl_action_take_status(client_handle.get(), status_message.get());
    if (RCL_RET_OK == ret) {
      is_status_ready = false;
      this->handle_status(status_message);
    }
  }
  if (is_goal_response_ready_)
  {
    std::shared_ptr<void> goal_response = this->create_goal_response();
    std::shared_ptr<rmw_request_id_t> response_header = this->create_response_header();
    rcl_ret_t ret = rcl_action_take_goal_response(
      client_handle.get(), response_header.get(), goal_response.get());

  }
  if (is_cancel_response_ready_)
  {
    std::shared_ptr<void> cancel_response = this->create_cancel_response();
    std::shared_ptr<rmw_request_id_t> response_header = this->create_response_header();
    rcl_ret_t ret = rcl_action_take_cancel_response(
      client_handle.get(), response_header.get(), cancel_response.get());
    if (RCL_RET_OK == ret) {
      is_cancel_response_ready = false;
      this->handle_cancel_response(response_header, cancel_response);
    } else {
      RCUTILS_LOG_ERROR_NAMED(
          "rclcpp",
          "take goal response failed for client of action '%s': %s",
          pimpl_->get_action_name(), rcl_get_error_string().str);
      rcl_reset_error();
    }
  }
  if (is_result_response_ready_)
  {
    std::shared_ptr<void> result_response = this->create_result_response();
    std::shared_ptr<rmw_request_id_t> response_header = this->create_response_header();
    rcl_ret_t ret = rcl_action_take_result_response(
      client_handle.get(), response_header.get(), result_response.get());
    if (RCL_RET_OK == ret) {
      is_result_response_ready = false;
      this->handle_result_response(response_header, result_response);
    } else {

    }
  }
}

