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

#include <rcl_action/action_client.h>
#include <rcl_action/wait.h>

#include <algorithm>
#include <map>
#include <memory>
#include <random>
#include <string>

#include "rclcpp_action/client.hpp"

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
  : node_handle(node_base->get_shared_rcl_node_handle()),
    logger(rclcpp::get_logger(rcl_node_get_logger_name(node_handle.get())).get_child("rclcpp")),
    random_bytes_generator(std::random_device{} ())
  {
    std::weak_ptr<rcl_node_t> weak_node_handle(node_handle);
    client_handle = std::shared_ptr<rcl_action_client_t>(
      new rcl_action_client_t, [weak_node_handle](rcl_action_client_t * client)
      {
        auto handle = weak_node_handle.lock();
        if (handle) {
          if (RCL_RET_OK != rcl_action_client_fini(client, handle.get())) {
            RCLCPP_ERROR(
              rclcpp::get_logger(rcl_node_get_logger_name(handle.get())).get_child("rclcpp"),
              "Error in destruction of rcl action client handle: %s", rcl_get_error_string().str);
            rcl_reset_error();
          }
        } else {
          RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"),
            "Error in destruction of rcl action client handle: "
            "the Node Handle was destructed too early. You will leak memory");
        }
        delete client;
      });
    *client_handle = rcl_action_get_zero_initialized_client();
    rcl_ret_t ret = rcl_action_client_init(
      client_handle.get(), node_handle.get(), type_support,
      action_name.c_str(), &client_options);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(
        ret, "could not initialize rcl action client");
    }

    ret = rcl_action_client_wait_set_get_num_entities(
      client_handle.get(),
      &num_subscriptions,
      &num_guard_conditions,
      &num_timers,
      &num_clients,
      &num_services);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(
        ret, "could not retrieve rcl action client details");
    }
  }

  size_t num_subscriptions{0u};
  size_t num_guard_conditions{0u};
  size_t num_timers{0u};
  size_t num_clients{0u};
  size_t num_services{0u};

  bool is_feedback_ready{false};
  bool is_status_ready{false};
  bool is_goal_response_ready{false};
  bool is_cancel_response_ready{false};
  bool is_result_response_ready{false};

  std::shared_ptr<rcl_action_client_t> client_handle{nullptr};
  std::shared_ptr<rcl_node_t> node_handle{nullptr};
  rclcpp::Logger logger;

  using ResponseCallback = std::function<void (std::shared_ptr<void> response)>;

  std::map<int64_t, ResponseCallback> pending_goal_responses;
  std::mutex goal_requests_mutex;

  std::map<int64_t, ResponseCallback> pending_result_responses;
  std::mutex result_requests_mutex;

  std::map<int64_t, ResponseCallback> pending_cancel_responses;
  std::mutex cancel_requests_mutex;

  std::independent_bits_engine<
    std::default_random_engine, 8, uint8_t> random_bytes_generator;
};

ClientBase::ClientBase(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  const std::string & action_name,
  const rosidl_action_type_support_t * type_support,
  const rcl_action_client_options_t & client_options)
: pimpl_(new ClientBaseImpl(node_base, action_name, type_support, client_options))
{
}

ClientBase::~ClientBase()
{
}

rclcpp::Logger
ClientBase::get_logger()
{
  return pimpl_->logger;
}

size_t
ClientBase::get_number_of_ready_subscriptions()
{
  return pimpl_->num_subscriptions;
}

size_t
ClientBase::get_number_of_ready_guard_conditions()
{
  return pimpl_->num_guard_conditions;
}

size_t
ClientBase::get_number_of_ready_timers()
{
  return pimpl_->num_timers;
}

size_t
ClientBase::get_number_of_ready_clients()
{
  return pimpl_->num_clients;
}

size_t
ClientBase::get_number_of_ready_services()
{
  return pimpl_->num_services;
}

bool
ClientBase::add_to_wait_set(rcl_wait_set_t * wait_set)
{
  rcl_ret_t ret = rcl_action_wait_set_add_action_client(
    wait_set, pimpl_->client_handle.get(), nullptr, nullptr);
  return RCL_RET_OK == ret;
}

bool
ClientBase::is_ready(rcl_wait_set_t * wait_set)
{
  rcl_ret_t ret = rcl_action_client_wait_set_get_entities_ready(
    wait_set, pimpl_->client_handle.get(),
    &pimpl_->is_feedback_ready,
    &pimpl_->is_status_ready,
    &pimpl_->is_goal_response_ready,
    &pimpl_->is_cancel_response_ready,
    &pimpl_->is_result_response_ready);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(
      ret, "failed to check for any ready entities");
  }
  return
    pimpl_->is_feedback_ready ||
    pimpl_->is_status_ready ||
    pimpl_->is_goal_response_ready ||
    pimpl_->is_cancel_response_ready ||
    pimpl_->is_result_response_ready;
}

void
ClientBase::handle_goal_response(
  const rmw_request_id_t & response_header,
  std::shared_ptr<void> response)
{
  std::lock_guard<std::mutex> guard(pimpl_->goal_requests_mutex);
  const int64_t & sequence_number = response_header.sequence_number;
  if (pimpl_->pending_goal_responses.count(sequence_number) == 0) {
    RCLCPP_ERROR(pimpl_->logger, "unknown goal response, ignoring...");
    return;
  }
  pimpl_->pending_goal_responses[sequence_number](response);
  pimpl_->pending_goal_responses.erase(sequence_number);
}

void
ClientBase::send_goal_request(std::shared_ptr<void> request, ResponseCallback callback)
{
  std::unique_lock<std::mutex> guard(pimpl_->goal_requests_mutex);
  int64_t sequence_number;
  rcl_ret_t ret = rcl_action_send_goal_request(
    pimpl_->client_handle.get(), request.get(), &sequence_number);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send goal request");
  }
  assert(pimpl_->pending_goal_responses.count(sequence_number) == 0);
  pimpl_->pending_goal_responses[sequence_number] = callback;
}

void
ClientBase::handle_result_response(
  const rmw_request_id_t & response_header,
  std::shared_ptr<void> response)
{
  std::lock_guard<std::mutex> guard(pimpl_->result_requests_mutex);
  const int64_t & sequence_number = response_header.sequence_number;
  if (pimpl_->pending_result_responses.count(sequence_number) == 0) {
    RCLCPP_ERROR(pimpl_->logger, "unknown result response, ignoring...");
    return;
  }
  pimpl_->pending_result_responses[sequence_number](response);
  pimpl_->pending_result_responses.erase(sequence_number);
}

void
ClientBase::send_result_request(std::shared_ptr<void> request, ResponseCallback callback)
{
  std::lock_guard<std::mutex> guard(pimpl_->result_requests_mutex);
  int64_t sequence_number;
  rcl_ret_t ret = rcl_action_send_result_request(
    pimpl_->client_handle.get(), request.get(), &sequence_number);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send result request");
  }
  assert(pimpl_->pending_result_responses.count(sequence_number) == 0);
  pimpl_->pending_result_responses[sequence_number] = callback;
}

void
ClientBase::handle_cancel_response(
  const rmw_request_id_t & response_header,
  std::shared_ptr<void> response)
{
  std::lock_guard<std::mutex> guard(pimpl_->cancel_requests_mutex);
  const int64_t & sequence_number = response_header.sequence_number;
  if (pimpl_->pending_cancel_responses.count(sequence_number) == 0) {
    RCLCPP_ERROR(pimpl_->logger, "unknown cancel response, ignoring...");
    return;
  }
  pimpl_->pending_cancel_responses[sequence_number](response);
  pimpl_->pending_cancel_responses.erase(sequence_number);
}

void
ClientBase::send_cancel_request(std::shared_ptr<void> request, ResponseCallback callback)
{
  std::lock_guard<std::mutex> guard(pimpl_->cancel_requests_mutex);
  int64_t sequence_number;
  rcl_ret_t ret = rcl_action_send_cancel_request(
    pimpl_->client_handle.get(), request.get(), &sequence_number);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send cancel request");
  }
  assert(pimpl_->pending_cancel_responses.count(sequence_number) == 0);
  pimpl_->pending_cancel_responses[sequence_number] = callback;
}

GoalID
ClientBase::generate_goal_id()
{
  GoalID goal_id;
  // TODO(hidmic): Do something better than this for UUID generation.
  // std::generate(
  //   goal_id.uuid.begin(), goal_id.uuid.end(),
  //   std::ref(pimpl_->random_bytes_generator));
  std::generate(
    goal_id.begin(), goal_id.end(),
    std::ref(pimpl_->random_bytes_generator));
  return goal_id;
}

void
ClientBase::execute()
{
  if (pimpl_->is_feedback_ready) {
    std::shared_ptr<void> feedback_message = this->create_feedback_message();
    rcl_ret_t ret = rcl_action_take_feedback(
      pimpl_->client_handle.get(), feedback_message.get());
    pimpl_->is_feedback_ready = false;
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "error taking feedback");
    } else {
      this->handle_feedback_message(feedback_message);
    }
  }

  if (pimpl_->is_status_ready) {
    std::shared_ptr<void> status_message = this->create_status_message();
    rcl_ret_t ret = rcl_action_take_status(
      pimpl_->client_handle.get(), status_message.get());
    pimpl_->is_status_ready = false;
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "error taking status");
    } else {
      this->handle_status_message(status_message);
    }
  }

  if (pimpl_->is_goal_response_ready) {
    rmw_request_id_t response_header;
    std::shared_ptr<void> goal_response = this->create_goal_response();
    rcl_ret_t ret = rcl_action_take_goal_response(
      pimpl_->client_handle.get(), &response_header, goal_response.get());
    pimpl_->is_goal_response_ready = false;
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "error taking goal response");
    } else {
      this->handle_goal_response(response_header, goal_response);
    }
  }

  if (pimpl_->is_result_response_ready) {
    rmw_request_id_t response_header;
    std::shared_ptr<void> result_response = this->create_result_response();
    rcl_ret_t ret = rcl_action_take_result_response(
      pimpl_->client_handle.get(), &response_header, result_response.get());
    pimpl_->is_result_response_ready = false;
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "error taking result response");
    } else {
      this->handle_result_response(response_header, result_response);
    }
  }

  if (pimpl_->is_cancel_response_ready) {
    rmw_request_id_t response_header;
    std::shared_ptr<void> cancel_response = this->create_cancel_response();
    rcl_ret_t ret = rcl_action_take_cancel_response(
      pimpl_->client_handle.get(), &response_header, cancel_response.get());
    pimpl_->is_cancel_response_ready = false;
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "error taking cancel response");
    } else {
      this->handle_cancel_response(response_header, cancel_response);
    }
  }
}

}  // namespace rclcpp_action
