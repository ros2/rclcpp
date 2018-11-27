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

#include <rclcpp/exceptions.hpp>
#include <rclcpp_action/server.hpp>

#include <string>

using rclcpp_action::ServerBase;

namespace rclcpp_action
{
class ServerBaseImpl
{
public:
  rcl_action_server_t action_server_;
  rclcpp::Clock::SharedPtr clock_;

  size_t num_subscriptions_ = 0;
  size_t num_timers_ = 0;
  size_t num_clients_ = 0;
  size_t num_services_ = 0;
  size_t num_guard_conditions_ = 0;

  bool goal_request_ready_ = false;
  bool cancel_request_ready_ = false;
  bool result_request_ready_ = false;
};
}

ServerBase::ServerBase(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
  const std::string & name,
  const rosidl_action_type_support_t * type_support
  )
  : pimpl_(new ServerBaseImpl)
{
  rcl_ret_t ret;
  // Create action server
  pimpl_->clock_ = node_clock->get_clock();
  pimpl_->action_server_ = rcl_action_get_zero_initialized_server();

  // TODO(sloretz) pass options into API
  const rcl_action_server_options_t server_options = rcl_action_server_get_default_options();

  rcl_node_t * rcl_node = node_base->get_rcl_node_handle();
  rcl_clock_t * rcl_clock = pimpl_->clock_->get_clock_handle();

  ret = rcl_action_server_init(
    &pimpl_->action_server_, rcl_node, rcl_clock, type_support, name.c_str(), &server_options);

  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  ret = rcl_action_server_wait_set_get_num_entities(
    &pimpl_->action_server_,
    &pimpl_->num_subscriptions_,
    &pimpl_->num_guard_conditions_,
    &pimpl_->num_timers_,
    &pimpl_->num_clients_,
    &pimpl_->num_services_);

  if (RCL_RET_OK != ret) {
    if (RCL_RET_OK != rcl_action_server_fini(&pimpl_->action_server_, rcl_node)) {
      // Ignoring error during finalization
    }
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
  rcl_ret_t ret = rcl_action_wait_set_add_action_server(wait_set, &pimpl_->action_server_, NULL);
  return RCL_RET_OK == ret;
}

bool
ServerBase::is_ready(rcl_wait_set_t * wait_set)
{
  rcl_ret_t ret = rcl_action_server_wait_set_get_entities_ready(
    wait_set,
    &pimpl_->action_server_,
    &pimpl_->goal_request_ready_,
    &pimpl_->cancel_request_ready_,
    &pimpl_->result_request_ready_);

  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  bool result = pimpl_->goal_request_ready_
    || pimpl_->cancel_request_ready_
    || pimpl_->result_request_ready_;

  if (result) {
  }
  return result;
}

void
ServerBase::execute()
{
  rcl_ret_t ret;

  if (pimpl_->goal_request_ready_) {
    rcl_action_goal_info_t info;
    rmw_request_id_t request_header;

    // TODO this needs to be available here
    std::shared_ptr<void> message = create_goal_request();
    ret = rcl_action_take_goal_request(
      &pimpl_->action_server_,
      &request_header,
      message.get());

    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret);
    }

    pimpl_->goal_request_ready_ = false;
    std::array<uint8_t, 16> uuid = get_goal_id_from_goal_request(message.get());
    for (size_t i = 0; i < 16; ++i) {
      info.uuid[i] = uuid[i];
    }

    auto response_pair = base_handle_goal_(info, message);

    // TODO(sloretz) if goal was accepted then does anything need to be stored here?

    ret = rcl_action_send_goal_response(
      &pimpl_->action_server_,
      &request_header,
      response_pair.second.get());

    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret);
    }

    // TODO(sloretz) if goal was accepted then tell Server<> to begin execution
    // maybe a base_handle_execute_ that sends a goal id so Server<> can look up the goal handle?
  } else if (pimpl_->cancel_request_ready_) {
    // TODO(sloretz) figure out which goals where canceled and notify Server<>
  } else if (pimpl_->result_request_ready_) {
    // TODO(sloretz) store the result request so it can be responded to later
  } else {
    throw std::runtime_error("Executing action server but nothing is ready");
  }
}
