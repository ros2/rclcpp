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

#include <rcl_action/action_server.h>
#include <rcl_action/wait.h>

#include <action_msgs/msg/goal_status_array.hpp>
#include <action_msgs/srv/cancel_goal.hpp>
#include <rclcpp/exceptions.hpp>
#include <rclcpp/scope_exit.hpp>
#include <rclcpp_action/server.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

using rclcpp_action::ServerBase;
using rclcpp_action::GoalUUID;

namespace rclcpp_action
{
class ServerBaseImpl
{
public:
  ServerBaseImpl(
    rclcpp::Clock::SharedPtr clock,
    rclcpp::Logger logger
  )
  : clock_(clock), logger_(logger)
  {
  }

  // Lock everything except user callbacks
  std::recursive_mutex reentrant_mutex_;

  std::shared_ptr<rcl_action_server_t> action_server_;

  rclcpp::Clock::SharedPtr clock_;

  size_t num_subscriptions_ = 0;
  size_t num_timers_ = 0;
  size_t num_clients_ = 0;
  size_t num_services_ = 0;
  size_t num_guard_conditions_ = 0;

  bool goal_request_ready_ = false;
  bool cancel_request_ready_ = false;
  bool result_request_ready_ = false;
  bool goal_expired_ = false;

  // Results to be kept until the goal expires after reaching a terminal state
  std::unordered_map<GoalUUID, std::shared_ptr<void>> goal_results_;
  // Requests for results are kept until a result becomes available
  std::unordered_map<GoalUUID, std::vector<rmw_request_id_t>> result_requests_;
  // rcl goal handles are kept so api to send result doesn't try to access freed memory
  std::unordered_map<GoalUUID, std::shared_ptr<rcl_action_goal_handle_t>> goal_handles_;

  rclcpp::Logger logger_;
};
}  // namespace rclcpp_action

ServerBase::ServerBase(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
  const std::string & name,
  const rosidl_action_type_support_t * type_support,
  const rcl_action_server_options_t & options
)
: pimpl_(new ServerBaseImpl(
      node_clock->get_clock(), node_logging->get_logger().get_child("rclcpp_action")))
{
  auto deleter = [node_base](rcl_action_server_t * ptr)
    {
      if (nullptr != ptr) {
        rcl_node_t * rcl_node = node_base->get_rcl_node_handle();
        rcl_ret_t ret = rcl_action_server_fini(ptr, rcl_node);
        (void)ret;
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp_action"),
          "failed to fini rcl_action_server_t in deleter");
      }
      delete ptr;
    };

  pimpl_->action_server_.reset(new rcl_action_server_t, deleter);
  *(pimpl_->action_server_) = rcl_action_get_zero_initialized_server();

  rcl_node_t * rcl_node = node_base->get_rcl_node_handle();
  rcl_clock_t * rcl_clock = pimpl_->clock_->get_clock_handle();

  rcl_ret_t ret = rcl_action_server_init(
    pimpl_->action_server_.get(), rcl_node, rcl_clock, type_support, name.c_str(), &options);

  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  ret = rcl_action_server_wait_set_get_num_entities(
    pimpl_->action_server_.get(),
    &pimpl_->num_subscriptions_,
    &pimpl_->num_guard_conditions_,
    &pimpl_->num_timers_,
    &pimpl_->num_clients_,
    &pimpl_->num_services_);

  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

ServerBase::~ServerBase()
{
}

size_t
ServerBase::get_number_of_ready_subscriptions()
{
  return pimpl_->num_subscriptions_;
}

size_t
ServerBase::get_number_of_ready_timers()
{
  return pimpl_->num_timers_;
}

size_t
ServerBase::get_number_of_ready_clients()
{
  return pimpl_->num_clients_;
}

size_t
ServerBase::get_number_of_ready_services()
{
  return pimpl_->num_services_;
}

size_t
ServerBase::get_number_of_ready_guard_conditions()
{
  return pimpl_->num_guard_conditions_;
}

bool
ServerBase::add_to_wait_set(rcl_wait_set_t * wait_set)
{
  std::lock_guard<std::recursive_mutex> lock(pimpl_->reentrant_mutex_);
  rcl_ret_t ret = rcl_action_wait_set_add_action_server(
    wait_set, pimpl_->action_server_.get(), NULL);
  return RCL_RET_OK == ret;
}

bool
ServerBase::is_ready(rcl_wait_set_t * wait_set)
{
  std::lock_guard<std::recursive_mutex> lock(pimpl_->reentrant_mutex_);
  rcl_ret_t ret = rcl_action_server_wait_set_get_entities_ready(
    wait_set,
    pimpl_->action_server_.get(),
    &pimpl_->goal_request_ready_,
    &pimpl_->cancel_request_ready_,
    &pimpl_->result_request_ready_,
    &pimpl_->goal_expired_);

  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  return pimpl_->goal_request_ready_ ||
         pimpl_->cancel_request_ready_ ||
         pimpl_->result_request_ready_ ||
         pimpl_->goal_expired_;
}

void
ServerBase::execute()
{
  if (pimpl_->goal_request_ready_) {
    execute_goal_request_received();
  } else if (pimpl_->cancel_request_ready_) {
    execute_cancel_request_received();
  } else if (pimpl_->result_request_ready_) {
    execute_result_request_received();
  } else if (pimpl_->goal_expired_) {
    execute_check_expired_goals();
  } else {
    throw std::runtime_error("Executing action server but nothing is ready");
  }
}

void
ServerBase::execute_goal_request_received()
{
  rcl_ret_t ret;
  rcl_action_goal_info_t goal_info = rcl_action_get_zero_initialized_goal_info();
  rmw_request_id_t request_header;

  std::lock_guard<std::recursive_mutex> lock(pimpl_->reentrant_mutex_);

  std::shared_ptr<void> message = create_goal_request();
  ret = rcl_action_take_goal_request(
    pimpl_->action_server_.get(),
    &request_header,
    message.get());

  pimpl_->goal_request_ready_ = false;

  if (RCL_RET_ACTION_SERVER_TAKE_FAILED == ret) {
    // Ignore take failure because connext fails if it receives a sample without valid data.
    // This happens when a client shuts down and connext receives a sample saying the client is
    // no longer alive.
    return;
  } else if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  GoalUUID uuid = get_goal_id_from_goal_request(message.get());
  convert(uuid, &goal_info);

  // Call user's callback, getting the user's response and a ros message to send back
  auto response_pair = call_handle_goal_callback(uuid, message);

  ret = rcl_action_send_goal_response(
    pimpl_->action_server_.get(),
    &request_header,
    response_pair.second.get());

  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  const auto status = response_pair.first;

  // if goal is accepted, create a goal handle, and store it
  if (GoalResponse::ACCEPT_AND_EXECUTE == status || GoalResponse::ACCEPT_AND_DEFER == status) {
    RCLCPP_DEBUG(pimpl_->logger_, "Accepted goal %s", to_string(uuid).c_str());
    // rcl_action will set time stamp
    auto deleter = [](rcl_action_goal_handle_t * ptr)
      {
        if (nullptr != ptr) {
          rcl_ret_t fail_ret = rcl_action_goal_handle_fini(ptr);
          (void)fail_ret;
          RCLCPP_DEBUG(rclcpp::get_logger("rclcpp_action"),
            "failed to fini rcl_action_goal_handle_t in deleter");
          delete ptr;
        }
      };
    rcl_action_goal_handle_t * rcl_handle;
    rcl_handle = rcl_action_accept_new_goal(pimpl_->action_server_.get(), &goal_info);
    if (!rcl_handle) {
      throw std::runtime_error("Failed to accept new goal\n");
    }

    std::shared_ptr<rcl_action_goal_handle_t> handle(new rcl_action_goal_handle_t, deleter);
    // Copy out goal handle since action server storage disappears when it is fini'd
    *handle = *rcl_handle;

    pimpl_->goal_handles_[uuid] = handle;

    if (GoalResponse::ACCEPT_AND_EXECUTE == status) {
      // Change status to executing
      ret = rcl_action_update_goal_state(handle.get(), GOAL_EVENT_EXECUTE);
      if (RCL_RET_OK != ret) {
        rclcpp::exceptions::throw_from_rcl_error(ret);
      }
    }
    // publish status since a goal's state has changed (was accepted or has begun execution)
    publish_status();

    // Tell user to start executing action
    call_goal_accepted_callback(handle, uuid, message);
  }
}

void
ServerBase::execute_cancel_request_received()
{
  rcl_ret_t ret;
  rmw_request_id_t request_header;

  // Initialize cancel request
  auto request = std::make_shared<action_msgs::srv::CancelGoal::Request>();

  std::lock_guard<std::recursive_mutex> lock(pimpl_->reentrant_mutex_);
  ret = rcl_action_take_cancel_request(
    pimpl_->action_server_.get(),
    &request_header,
    request.get());

  pimpl_->cancel_request_ready_ = false;

  if (RCL_RET_ACTION_SERVER_TAKE_FAILED == ret) {
    // Ignore take failure because connext fails if it receives a sample without valid data.
    // This happens when a client shuts down and connext receives a sample saying the client is
    // no longer alive.
    return;
  } else if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  // Convert c++ message to C message
  rcl_action_cancel_request_t cancel_request = rcl_action_get_zero_initialized_cancel_request();
  convert(request->goal_info.goal_id.uuid, &cancel_request.goal_info);
  cancel_request.goal_info.stamp.sec = request->goal_info.stamp.sec;
  cancel_request.goal_info.stamp.nanosec = request->goal_info.stamp.nanosec;

  // Get a list of goal info that should be attempted to be cancelled
  rcl_action_cancel_response_t cancel_response = rcl_action_get_zero_initialized_cancel_response();

  ret = rcl_action_process_cancel_request(
    pimpl_->action_server_.get(),
    &cancel_request,
    &cancel_response);

  RCLCPP_SCOPE_EXIT({
    ret = rcl_action_cancel_response_fini(&cancel_response);
    if (RCL_RET_OK != ret) {
      RCLCPP_ERROR(pimpl_->logger_, "Failed to fini cancel response");
    }
  });

  auto response = std::make_shared<action_msgs::srv::CancelGoal::Response>();

  auto & goals = cancel_response.msg.goals_canceling;
  // For each canceled goal, call cancel callback
  for (size_t i = 0; i < goals.size; ++i) {
    const rcl_action_goal_info_t & goal_info = goals.data[i];
    GoalUUID uuid;
    convert(goal_info, &uuid);
    auto response_code = call_handle_cancel_callback(uuid);
    if (CancelResponse::ACCEPT == response_code) {
      action_msgs::msg::GoalInfo cpp_info;
      cpp_info.goal_id.uuid = uuid;
      cpp_info.stamp.sec = goal_info.stamp.sec;
      cpp_info.stamp.nanosec = goal_info.stamp.nanosec;
      response->goals_canceling.push_back(cpp_info);
    }
  }

  if (!response->goals_canceling.empty()) {
    // at least one goal state changed, publish a new status message
    publish_status();
  }

  ret = rcl_action_send_cancel_response(
    pimpl_->action_server_.get(), &request_header, response.get());
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

void
ServerBase::execute_result_request_received()
{
  rcl_ret_t ret;
  // Get the result request message
  rmw_request_id_t request_header;
  std::shared_ptr<void> result_request = create_result_request();
  std::lock_guard<std::recursive_mutex> lock(pimpl_->reentrant_mutex_);
  ret = rcl_action_take_result_request(
    pimpl_->action_server_.get(), &request_header, result_request.get());

  pimpl_->result_request_ready_ = false;

  if (RCL_RET_ACTION_SERVER_TAKE_FAILED == ret) {
    // Ignore take failure because connext fails if it receives a sample without valid data.
    // This happens when a client shuts down and connext receives a sample saying the client is
    // no longer alive.
    return;
  } else if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  std::shared_ptr<void> result_response;

  // check if the goal exists
  GoalUUID uuid = get_goal_id_from_result_request(result_request.get());
  rcl_action_goal_info_t goal_info;
  convert(uuid, &goal_info);
  bool goal_exists;
  goal_exists = rcl_action_server_goal_exists(pimpl_->action_server_.get(), &goal_info);
  if (!goal_exists) {
    // Goal does not exists
    result_response = create_result_response(action_msgs::msg::GoalStatus::STATUS_UNKNOWN);
  } else {
    // Goal exists, check if a result is already available
    auto iter = pimpl_->goal_results_.find(uuid);
    if (iter != pimpl_->goal_results_.end()) {
      result_response = iter->second;
    }
  }

  if (result_response) {
    // Send the result now
    ret = rcl_action_send_result_response(
      pimpl_->action_server_.get(), &request_header, result_response.get());
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret);
    }
  } else {
    // Store the request so it can be responded to later
    pimpl_->result_requests_[uuid].push_back(request_header);
  }
}

void
ServerBase::execute_check_expired_goals()
{
  // Allocate expecting only one goal to expire at a time
  rcl_action_goal_info_t expired_goals[1];
  size_t num_expired = 1;

  // Loop in case more than 1 goal expired
  while (num_expired > 0u) {
    std::lock_guard<std::recursive_mutex> lock(pimpl_->reentrant_mutex_);
    rcl_ret_t ret;
    ret = rcl_action_expire_goals(pimpl_->action_server_.get(), expired_goals, 1, &num_expired);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret);
    } else if (num_expired) {
      // A goal expired!
      GoalUUID uuid;
      convert(expired_goals[0], &uuid);
      RCLCPP_DEBUG(pimpl_->logger_, "Expired goal %s", to_string(uuid).c_str());
      pimpl_->goal_results_.erase(uuid);
      pimpl_->result_requests_.erase(uuid);
      pimpl_->goal_handles_.erase(uuid);
    }
  }
}

void
ServerBase::publish_status()
{
  rcl_ret_t ret;

  // Get all goal handles known to C action server
  rcl_action_goal_handle_t ** goal_handles = NULL;
  size_t num_goals = 0;
  ret = rcl_action_server_get_goal_handles(
    pimpl_->action_server_.get(), &goal_handles, &num_goals);

  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  auto status_msg = std::make_shared<action_msgs::msg::GoalStatusArray>();
  status_msg->status_list.reserve(num_goals);
  // Populate a c++ status message with the goals and their statuses
  rcl_action_goal_status_array_t c_status_array =
    rcl_action_get_zero_initialized_goal_status_array();
  ret = rcl_action_get_goal_status_array(pimpl_->action_server_.get(), &c_status_array);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  RCLCPP_SCOPE_EXIT({
    ret = rcl_action_goal_status_array_fini(&c_status_array);
    if (RCL_RET_OK != ret) {
      RCLCPP_ERROR(pimpl_->logger_, "Failed to fini status array message");
    }
  });

  for (size_t i = 0; i < c_status_array.msg.status_list.size; ++i) {
    auto & c_status_msg = c_status_array.msg.status_list.data[i];

    action_msgs::msg::GoalStatus msg;
    msg.status = c_status_msg.status;
    // Convert C goal info to C++ goal info
    convert(c_status_msg.goal_info, &msg.goal_info.goal_id.uuid);
    msg.goal_info.stamp.sec = c_status_msg.goal_info.stamp.sec;
    msg.goal_info.stamp.nanosec = c_status_msg.goal_info.stamp.nanosec;

    status_msg->status_list.push_back(msg);
  }

  // Publish the message through the status publisher
  ret = rcl_action_publish_status(pimpl_->action_server_.get(), status_msg.get());

  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

void
ServerBase::publish_result(const GoalUUID & uuid, std::shared_ptr<void> result_msg)
{
  // Check that the goal exists
  rcl_action_goal_info_t goal_info;
  convert(uuid, &goal_info);
  std::lock_guard<std::recursive_mutex> lock(pimpl_->reentrant_mutex_);
  bool goal_exists;
  goal_exists = rcl_action_server_goal_exists(pimpl_->action_server_.get(), &goal_info);

  if (!goal_exists) {
    throw std::runtime_error("Asked to publish result for goal that does not exist");
  }

  pimpl_->goal_results_[uuid] = result_msg;

  // if there are clients who already asked for the result, send it to them
  auto iter = pimpl_->result_requests_.find(uuid);
  if (iter != pimpl_->result_requests_.end()) {
    for (auto & request_header : iter->second) {
      rcl_ret_t ret = rcl_action_send_result_response(
        pimpl_->action_server_.get(), &request_header, result_msg.get());
      if (RCL_RET_OK != ret) {
        rclcpp::exceptions::throw_from_rcl_error(ret);
      }
    }
  }
}

void
ServerBase::notify_goal_terminal_state()
{
  std::lock_guard<std::recursive_mutex> lock(pimpl_->reentrant_mutex_);
  rcl_ret_t ret = rcl_action_notify_goal_done(pimpl_->action_server_.get());
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

void
ServerBase::publish_feedback(std::shared_ptr<void> feedback_msg)
{
  std::lock_guard<std::recursive_mutex> lock(pimpl_->reentrant_mutex_);
  rcl_ret_t ret = rcl_action_publish_feedback(pimpl_->action_server_.get(), feedback_msg.get());
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to publish feedback");
  }
}
