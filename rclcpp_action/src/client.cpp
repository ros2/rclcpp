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
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>

#include <algorithm>
#include <map>
#include <memory>
#include <random>
#include <string>

#include "rclcpp_action/client.hpp"
#include "rclcpp_action/exceptions.hpp"

namespace rclcpp_action
{

class ClientBaseImpl
{
public:
  ClientBaseImpl(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    const std::string & action_name,
    const rosidl_action_type_support_t * type_support,
    const rcl_action_client_options_t & client_options)
  : node_graph_(node_graph),
    node_handle(node_base->get_shared_rcl_node_handle()),
    logger(node_logging->get_logger().get_child("rclcpp_acton")),
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
              rclcpp::get_logger(rcl_node_get_logger_name(handle.get())).get_child("rclcpp_action"),
              "Error in destruction of rcl action client handle: %s", rcl_get_error_string().str);
            rcl_reset_error();
          }
        } else {
          RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp_action"),
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

  rclcpp::Context::SharedPtr context_;
  rclcpp::node_interfaces::NodeGraphInterface::WeakPtr node_graph_;
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
    std::default_random_engine, 8, unsigned int> random_bytes_generator;
};

ClientBase::ClientBase(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
  const std::string & action_name,
  const rosidl_action_type_support_t * type_support,
  const rcl_action_client_options_t & client_options)
: pimpl_(new ClientBaseImpl(
      node_base, node_graph, node_logging, action_name, type_support, client_options))
{
}

ClientBase::~ClientBase()
{
}

bool
ClientBase::action_server_is_ready() const
{
  bool is_ready;
  rcl_ret_t ret = rcl_action_server_is_available(
    this->pimpl_->node_handle.get(),
    this->pimpl_->client_handle.get(),
    &is_ready);
  if (RCL_RET_NODE_INVALID == ret) {
    const rcl_node_t * node_handle = this->pimpl_->node_handle.get();
    if (node_handle && !rcl_context_is_valid(node_handle->context)) {
      // context is shutdown, do a soft failure
      return false;
    }
  }
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "rcl_action_server_is_available failed");
  }
  return is_ready;
}

bool
ClientBase::wait_for_action_server_nanoseconds(std::chrono::nanoseconds timeout)
{
  auto start = std::chrono::steady_clock::now();
  // make an event to reuse, rather than create a new one each time
  auto node_ptr = pimpl_->node_graph_.lock();
  if (!node_ptr) {
    throw rclcpp::exceptions::InvalidNodeError();
  }
  auto event = node_ptr->get_graph_event();
  // check to see if the server is ready immediately
  if (this->action_server_is_ready()) {
    return true;
  }
  if (timeout == std::chrono::nanoseconds(0)) {
    // check was non-blocking, return immediately
    return false;
  }
  // update the time even on the first loop to account for time spent in the first call
  // to this->server_is_ready()
  std::chrono::nanoseconds time_to_wait =
    timeout > std::chrono::nanoseconds(0) ?
    timeout - (std::chrono::steady_clock::now() - start) :
    std::chrono::nanoseconds::max();
  if (time_to_wait < std::chrono::nanoseconds(0)) {
    // Do not allow the time_to_wait to become negative when timeout was originally positive.
    // Setting time_to_wait to 0 will allow one non-blocking wait because of the do-while.
    time_to_wait = std::chrono::nanoseconds(0);
  }
  do {
    if (!rclcpp::ok(this->pimpl_->context_)) {
      return false;
    }
    // Limit each wait to 100ms to workaround an issue specific to the Connext RMW implementation.
    // A race condition means that graph changes for services becoming available may trigger the
    // wait set to wake up, but then not be reported as ready immediately after the wake up
    // (see https://github.com/ros2/rmw_connext/issues/201)
    // If no other graph events occur, the wait set will not be triggered again until the timeout
    // has been reached, despite the service being available, so we artificially limit the wait
    // time to limit the delay.
    node_ptr->wait_for_graph_change(
      event, std::min(time_to_wait, std::chrono::nanoseconds(RCL_MS_TO_NS(100))));
    // Because of the aforementioned race condition, we check if the service is ready even if the
    // graph event wasn't triggered.
    event->check_and_clear();
    if (this->action_server_is_ready()) {
      return true;
    }
    // server is not ready, loop if there is time left
    if (timeout > std::chrono::nanoseconds(0)) {
      time_to_wait = timeout - (std::chrono::steady_clock::now() - start);
    }
    // if timeout is negative, time_to_wait will never reach zero
  } while (time_to_wait > std::chrono::nanoseconds(0));
  return false;  // timeout exceeded while waiting for the server to be ready
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

GoalUUID
ClientBase::generate_goal_id()
{
  GoalUUID goal_id;
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
  } else if (pimpl_->is_status_ready) {
    std::shared_ptr<void> status_message = this->create_status_message();
    rcl_ret_t ret = rcl_action_take_status(
      pimpl_->client_handle.get(), status_message.get());
    pimpl_->is_status_ready = false;
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "error taking status");
    } else {
      this->handle_status_message(status_message);
    }
  } else if (pimpl_->is_goal_response_ready) {
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
  } else if (pimpl_->is_result_response_ready) {
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
  } else if (pimpl_->is_cancel_response_ready) {
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
  } else {
    throw std::runtime_error("Executing action client but nothing is ready");
  }
}

}  // namespace rclcpp_action
