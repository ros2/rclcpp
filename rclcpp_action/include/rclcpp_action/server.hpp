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

#ifndef RCLCPP_ACTION__SERVER_HPP_
#define RCLCPP_ACTION__SERVER_HPP_

#include <functional>
#include <memory>

#include <rclcpp/create_publisher.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_interfaces/node_services_interface.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>
#include "rclcpp_action/visibility_control.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include <rosidl_generator_c/action_type_support_struct.h>
#include <rosidl_typesupport_cpp/action_type_support.hpp>

#include "rcl/service.h"

namespace rclcpp_action
{
// Forward declaration
class ServerBaseImpl;

class ServerBase
{
public:
  // TODO(slorez) NodeLoggingInterface when it can be gotten off a node
  RCLCPP_ACTION_PUBLIC
  ServerBase(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    const std::string & name,
    const rosidl_action_type_support_t * type_support);

  RCLCPP_ACTION_PUBLIC
  virtual ~ServerBase();

private:
  std::unique_ptr<ServerBaseImpl> pimpl_;
};

template <typename SERVICE, typename CALLBACK>
rclcpp::AnyServiceCallback<SERVICE>
make_service_callback(CALLBACK callback)
{
  rclcpp::AnyServiceCallback<SERVICE> any_callback;
  any_callback.set(std::forward<CALLBACK>(callback));
  return any_callback;
}

template <typename ACTION>
class Server : public ServerBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(Server)

  using Callback = std::function<void(std::shared_ptr<ServerGoalHandle<ACTION>>)>;

  RCLCPP_ACTION_PUBLIC
  Server(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_srv,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topic,
    const std::string & name,
    Callback handle_goal,
    Callback handle_cancel) :
      ServerBase(
          node_base,
          name,
          rosidl_typesupport_cpp::get_action_type_support_handle<ACTION>()),
      handle_goal_(handle_goal),
      handle_cancel_(handle_cancel)
  {
    auto goal_request_callback = make_service_callback<typename ACTION::GoalRequestService>(
      [this](
        const std::shared_ptr<typename ACTION::GoalRequestService::Request> req,
        std::shared_ptr<typename ACTION::GoalRequestService::Response> resp)
      {
        this->handle_goal(req, resp);
      });

    auto cancel_goal_callback = make_service_callback<typename ACTION::CancelGoalService>(
      [this](
        const std::shared_ptr<typename ACTION::CancelGoalService::Request> req,
        std::shared_ptr<typename ACTION::CancelGoalService::Response> resp)
      {
        this->handle_cancel(req, resp);
      });

    auto get_result_callback = make_service_callback<typename ACTION::GoalResultService>(
      [this](
        const std::shared_ptr<typename ACTION::GoalResultService::Request> req,
        std::shared_ptr<typename ACTION::GoalResultService::Response> resp)
      {
        this->handle_result(req, resp);
      });

    rcl_service_options_t options = rcl_service_get_default_options();

    // TODO(sloretz) Use rcl_action API to generate service and topic names

    send_goal_server_ = std::make_shared<rclcpp::Service<typename ACTION::GoalRequestService>>(
      node_base->get_shared_rcl_node_handle(),
      name + "/_action/send_goal",
      goal_request_callback,
      options);
    node_srv->add_service(send_goal_server_, node_base->get_default_callback_group());

    cancel_goal_server_ = std::make_shared<rclcpp::Service<typename ACTION::CancelGoalService>>(
      node_base->get_shared_rcl_node_handle(),
      name + "/_action/cancel_goal",
      cancel_goal_callback,
      options);
    node_srv->add_service(cancel_goal_server_, node_base->get_default_callback_group());

    get_result_server_ = std::make_shared<rclcpp::Service<typename ACTION::GoalResultService>>(
      node_base->get_shared_rcl_node_handle(),
      name + "/_action/get_result",
      get_result_callback,
      options);
    node_srv->add_service(get_result_server_, node_base->get_default_callback_group());

    rmw_qos_profile_t qos = rmw_qos_profile_default;
    feedback_publisher_ = rclcpp::create_publisher<
        typename ACTION::Feedback, std::allocator<void>,
        rclcpp::Publisher<typename ACTION::Feedback>>
    (
      node_topic.get(),
      name + "/_action/feedback",
      qos,
      true,
      nullptr);
    status_publisher_ = rclcpp::create_publisher<
        typename ACTION::GoalStatusMessage, std::allocator<void>,
        rclcpp::Publisher<typename ACTION::GoalStatusMessage>>
    (
      node_topic.get(),
      name + "/_action/status",
      qos,
      true,
      nullptr);
  }

  virtual ~Server()
  {
  }

protected:

  void
  handle_goal(
    const std::shared_ptr<typename ACTION::GoalRequestService::Request> & req,
    std::shared_ptr<typename ACTION::GoalRequestService::Response> & resp)
  {
    // TODO(sloretz) create goal handle and pass that to user callback
    (void)req;
    (void)resp;
  }

  void
  handle_cancel(
    const std::shared_ptr<typename ACTION::CancelGoalService::Request> & req,
    std::shared_ptr<typename ACTION::CancelGoalService::Response> & resp)
  {
    // TODO(sloretz) look up goal handle and pass that to user callback
    (void)req;
    (void)resp;
  }

  void
  handle_result(
    const std::shared_ptr<typename ACTION::GoalResultService::Request> & req,
    std::shared_ptr<typename ACTION::GoalResultService::Response> & resp)
  {
    // TODO(sloretz) Start a background thread because rclcpp api currently isn't asynchronous
    // Store the result in future objects that are requested from the goal handle
    (void)req;
    (void)resp;
  }

private:
  Callback handle_goal_;
  Callback handle_cancel_;

  std::shared_ptr<rclcpp::Service<typename ACTION::GoalRequestService>> send_goal_server_;
  std::shared_ptr<rclcpp::Service<typename ACTION::CancelGoalService>> cancel_goal_server_;
  std::shared_ptr<rclcpp::Service<typename ACTION::GoalResultService>> get_result_server_;

  std::shared_ptr<rclcpp::Publisher<typename ACTION::Feedback>> feedback_publisher_;
  std::shared_ptr<rclcpp::Publisher<typename ACTION::GoalStatusMessage>> status_publisher_;
};
}  // namespace rclcpp_action
#endif  // RCLCPP_ACTION__SERVER_HPP_
