// Copyright 2020 Open Source Robotics Foundation, Inc.
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

/**
 * Service client test was adopted from:
 * https://github.com/ros2/demos/blob/master/lifecycle/src/lifecycle_service_client.cpp
 */

#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/transition_description.hpp"
#include "lifecycle_msgs/srv/cancel_transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "rcl_lifecycle/rcl_lifecycle.h"

#include "rclcpp/node_interfaces/node_graph.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "rcpputils/scope_exit.hpp"

#include "./mocking_utils/patch.hpp"

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;

constexpr char const * ping_pong_node_name = "ping_pong_node";

constexpr char const * node_get_state_topic = "/ping_pong_node/get_state";
constexpr char const * node_change_state_topic = "/ping_pong_node/change_state";
constexpr char const * node_cancel_transition_topic =
  "/ping_pong_node/cancel_transition";

const lifecycle_msgs::msg::State unknown_state = lifecycle_msgs::msg::State();

class PingPongAsyncLCNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief `PingPong` refers to transition callback FAILURE->SUCCESS->... returns
   *        It has 2 notable exceptions:
   *        - on_shutdown_async ping pongs ERROR -> SUCCESS (FAILURE is undefined for this transition)
   *        - on_error_async always returns SUCCESS
   *        It also contains a function to switch to a detached thread wrapper of each callback
  */
  explicit PingPongAsyncLCNode(std::string node_name)
  : rclcpp_lifecycle::LifecycleNode(std::move(node_name))
  {
    register_async_on_configure(
      std::bind(
        &PingPongAsyncLCNode::on_configure_async,
        this, std::placeholders::_1,
        std::placeholders::_2));
    register_async_on_activate(
      std::bind(
        &PingPongAsyncLCNode::on_activate_async,
        this, std::placeholders::_1,
        std::placeholders::_2));
    register_async_on_deactivate(
      std::bind(
        &PingPongAsyncLCNode::on_deactivate_async,
        this, std::placeholders::_1,
        std::placeholders::_2));
    register_async_on_cleanup(
      std::bind(
        &PingPongAsyncLCNode::on_cleanup_async,
        this, std::placeholders::_1,
        std::placeholders::_2));
    register_async_on_shutdown(
      std::bind(
        &PingPongAsyncLCNode::on_shutdown_async,
        this, std::placeholders::_1,
        std::placeholders::_2));
    register_async_on_error(
      std::bind(
        &PingPongAsyncLCNode::on_error_async,
        this, std::placeholders::_1,
        std::placeholders::_2));
  }

  void switch_to_detached_thread_callbacks()
  {
    register_async_on_configure(
      std::bind(
        &PingPongAsyncLCNode::on_configure_async_dt,
        this, std::placeholders::_1,
        std::placeholders::_2));
    register_async_on_activate(
      std::bind(
        &PingPongAsyncLCNode::on_activate_async_dt,
        this, std::placeholders::_1,
        std::placeholders::_2));
    register_async_on_deactivate(
      std::bind(
        &PingPongAsyncLCNode::on_deactivate_async_dt,
        this, std::placeholders::_1,
        std::placeholders::_2));
    register_async_on_cleanup(
      std::bind(
        &PingPongAsyncLCNode::on_cleanup_async_dt,
        this, std::placeholders::_1,
        std::placeholders::_2));
    register_async_on_shutdown(
      std::bind(
        &PingPongAsyncLCNode::on_shutdown_async_dt,
        this, std::placeholders::_1,
        std::placeholders::_2));
    register_async_on_error(
      std::bind(
        &PingPongAsyncLCNode::on_error_async_dt,
        this, std::placeholders::_1,
        std::placeholders::_2));
  }

  size_t number_of_callbacks = 0;

protected:
  // Simulates fail -> success -> fail -> success -> ...
  bool ret_success{false};
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  ping_pong_ret_fail_success()
  {
    auto ret = ret_success ?
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS :
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    ret_success = !ret_success;
    return ret;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  ping_pong_ret_error_success()
  {
    auto ret = ret_success ?
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS :
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    ret_success = !ret_success;
    return ret;
  }

  // Async callbacks
  void
  on_configure_async(
    const rclcpp_lifecycle::State &,
    std::shared_ptr<rclcpp_lifecycle::ChangeStateHandler> change_state_hdl)
  {
    EXPECT_EQ(State::TRANSITION_STATE_CONFIGURING, get_current_state().id());
    EXPECT_TRUE(change_state_hdl != nullptr);
    ++number_of_callbacks;
    change_state_hdl->send_callback_resp(ping_pong_ret_fail_success());
  }

  void
  on_activate_async(
    const rclcpp_lifecycle::State &,
    std::shared_ptr<rclcpp_lifecycle::ChangeStateHandler> change_state_hdl)
  {
    EXPECT_EQ(State::TRANSITION_STATE_ACTIVATING, get_current_state().id());
    EXPECT_TRUE(change_state_hdl != nullptr);
    ++number_of_callbacks;
    change_state_hdl->send_callback_resp(ping_pong_ret_fail_success());
  }

  void
  on_deactivate_async(
    const rclcpp_lifecycle::State &,
    std::shared_ptr<rclcpp_lifecycle::ChangeStateHandler> change_state_hdl)
  {
    EXPECT_EQ(State::TRANSITION_STATE_DEACTIVATING, get_current_state().id());
    EXPECT_TRUE(change_state_hdl != nullptr);
    ++number_of_callbacks;
    change_state_hdl->send_callback_resp(ping_pong_ret_fail_success());
  }

  void
  on_cleanup_async(
    const rclcpp_lifecycle::State &,
    std::shared_ptr<rclcpp_lifecycle::ChangeStateHandler> change_state_hdl)
  {
    EXPECT_EQ(State::TRANSITION_STATE_CLEANINGUP, get_current_state().id());
    EXPECT_TRUE(change_state_hdl != nullptr);
    ++number_of_callbacks;
    change_state_hdl->send_callback_resp(ping_pong_ret_fail_success());
  }

  void on_shutdown_async(
    const rclcpp_lifecycle::State &,
    std::shared_ptr<rclcpp_lifecycle::ChangeStateHandler> change_state_hdl)
  {
    EXPECT_EQ(State::TRANSITION_STATE_SHUTTINGDOWN, get_current_state().id());
    EXPECT_TRUE(change_state_hdl != nullptr);
    ++number_of_callbacks;
    change_state_hdl->send_callback_resp(ping_pong_ret_error_success());
  }

  void on_error_async(
    const rclcpp_lifecycle::State &,
    std::shared_ptr<rclcpp_lifecycle::ChangeStateHandler> change_state_hdl)
  {
    EXPECT_EQ(State::TRANSITION_STATE_ERRORPROCESSING, get_current_state().id());
    EXPECT_TRUE(change_state_hdl != nullptr);
    ++number_of_callbacks;
    change_state_hdl->send_callback_resp(
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
  }

  // Detached thread callbacks
  void
  on_configure_async_dt(
    const rclcpp_lifecycle::State & s,
    std::shared_ptr<rclcpp_lifecycle::ChangeStateHandler> change_state_hdl)
  {
    std::thread t(&PingPongAsyncLCNode::on_configure_async, this,
      s, change_state_hdl);
    t.detach();
  }

  void
  on_activate_async_dt(
    const rclcpp_lifecycle::State & s,
    std::shared_ptr<rclcpp_lifecycle::ChangeStateHandler> change_state_hdl)
  {
    std::thread t(&PingPongAsyncLCNode::on_activate_async, this,
      s, change_state_hdl);
    t.detach();
  }

  void
  on_deactivate_async_dt(
    const rclcpp_lifecycle::State & s,
    std::shared_ptr<rclcpp_lifecycle::ChangeStateHandler> change_state_hdl)
  {
    std::thread t(&PingPongAsyncLCNode::on_deactivate_async, this,
      s, change_state_hdl);
    t.detach();
  }

  void
  on_cleanup_async_dt(
    const rclcpp_lifecycle::State & s,
    std::shared_ptr<rclcpp_lifecycle::ChangeStateHandler> change_state_hdl)
  {
    std::thread t(&PingPongAsyncLCNode::on_cleanup_async, this,
      s, change_state_hdl);
    t.detach();
  }

  void on_shutdown_async_dt(
    const rclcpp_lifecycle::State & s,
    std::shared_ptr<rclcpp_lifecycle::ChangeStateHandler> change_state_hdl)
  {
    std::thread t(&PingPongAsyncLCNode::on_shutdown_async, this,
      s, change_state_hdl);
    t.detach();
  }

  void on_error_async_dt(
    const rclcpp_lifecycle::State & s,
    std::shared_ptr<rclcpp_lifecycle::ChangeStateHandler> change_state_hdl)
  {
    std::thread t(&PingPongAsyncLCNode::on_error_async, this,
      s, change_state_hdl);
    t.detach();
  }
};


class LifecycleServiceClient : public rclcpp::Node
{
public:
  explicit LifecycleServiceClient(std::string node_name)
  : Node(node_name)
  {
    client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(
      node_get_state_topic);
    client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
      node_change_state_topic);
    client_cancel_transition_ = this->create_client<lifecycle_msgs::srv::CancelTransition>(
      node_cancel_transition_topic);
  }

  lifecycle_msgs::msg::State
  get_state(std::chrono::seconds time_out = 1s)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    if (!client_get_state_->wait_for_service(time_out)) {
      return unknown_state;
    }

    auto future_result = client_get_state_->async_send_request(request);
    auto future_status = future_result.wait_for(time_out);

    if (future_status != std::future_status::ready) {
      return unknown_state;
    }

    auto result = future_result.get();
    if (result) {
      return result->current_state;
    } else {
      return unknown_state;
    }
  }

  bool
  change_state(std::uint8_t transition, std::chrono::seconds time_out = 1s)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    if (!client_change_state_->wait_for_service(time_out)) {
      return false;
    }

    auto future_result = client_change_state_->async_send_request(request);
    auto future_status = future_result.wait_for(time_out);

    if (future_status != std::future_status::ready) {
      return false;
    }

    return future_result.get()->success;
  }

  bool cancel_transition(std::chrono::seconds time_out = 1s)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::CancelTransition::Request>();

    if (!client_cancel_transition_->wait_for_service(time_out)) {
      return false;
    }

    auto future_result = client_cancel_transition_->async_send_request(request);
    auto future_status = future_result.wait_for(time_out);

    if (future_status != std::future_status::ready) {
      return false;
    }

    return future_result.get()->success;
  }

private:
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::CancelTransition>>
  client_cancel_transition_;
};


class TestLifecycleAsyncTransitions : public ::testing::Test
{
protected:
  PingPongAsyncLCNode * lifecycle_node() {return lifecycle_node_.get();}
  LifecycleServiceClient * lifecycle_client() {return lifecycle_client_.get();}

private:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    lifecycle_node_ = std::make_shared<PingPongAsyncLCNode>(ping_pong_node_name);
    lifecycle_client_ = std::make_shared<LifecycleServiceClient>("client");
    spinner_ = std::thread(&TestLifecycleAsyncTransitions::spin, this);
  }

  void TearDown() override
  {
    {
      std::lock_guard<std::mutex> guard(shutdown_mutex_);
      rclcpp::shutdown();
    }
    spinner_.join();
  }

  void spin()
  {
    while (true) {
      {
        std::lock_guard<std::mutex> guard(shutdown_mutex_);
        if (!rclcpp::ok()) {
          break;
        }
        rclcpp::spin_some(lifecycle_node_->get_node_base_interface());
        rclcpp::spin_some(lifecycle_client_);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  std::shared_ptr<PingPongAsyncLCNode> lifecycle_node_;
  std::shared_ptr<LifecycleServiceClient> lifecycle_client_;
  std::mutex shutdown_mutex_;
  std::thread spinner_;
};


TEST_F(TestLifecycleAsyncTransitions, lifecycle_async_transitions_w_immediate_ret) {
  EXPECT_EQ(
    lifecycle_client()->get_state().id,
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  // Configure 2x
  EXPECT_FALSE(
    lifecycle_client()->change_state(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
  EXPECT_EQ(
    lifecycle_client()->get_state().id,
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_TRUE(
    lifecycle_client()->change_state(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
  EXPECT_EQ(
    lifecycle_client()->get_state().id,
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // Activate 2x
  EXPECT_FALSE(
    lifecycle_client()->change_state(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));
  EXPECT_EQ(
    lifecycle_client()->get_state().id,
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_TRUE(
    lifecycle_client()->change_state(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));
  EXPECT_EQ(
    lifecycle_client()->get_state().id,
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // Deactivate 2x
  EXPECT_FALSE(
    lifecycle_client()->change_state(
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE));
  EXPECT_EQ(
    lifecycle_client()->get_state().id,
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_TRUE(
    lifecycle_client()->change_state(
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE));
  EXPECT_EQ(
    lifecycle_client()->get_state().id,
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // Cleanup 2x
  EXPECT_FALSE(
    lifecycle_client()->change_state(
      lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP));
  EXPECT_EQ(
    lifecycle_client()->get_state().id,
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_TRUE(
    lifecycle_client()->change_state(
      lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP));
  EXPECT_EQ(
    lifecycle_client()->get_state().id,
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  EXPECT_EQ(
    lifecycle_client()->get_state().id,
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  // Shutdown(Error) -> ErrorProcessing(Success)
  EXPECT_TRUE(
    lifecycle_client()->change_state(
      lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN));
  EXPECT_EQ(
    lifecycle_client()->get_state().id,
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  // Shutdown(Success)
  EXPECT_TRUE(
    lifecycle_client()->change_state(
      lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN));
  EXPECT_EQ(
    lifecycle_client()->get_state().id,
    lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);

  // 2 * change_state calls + 1 ErrorProcessing
  EXPECT_EQ(lifecycle_node()->number_of_callbacks, 11);
}

TEST_F(TestLifecycleAsyncTransitions, lifecycle_async_transitions_w_detached_thread) {
  lifecycle_node()->switch_to_detached_thread_callbacks();

  EXPECT_EQ(
    lifecycle_client()->get_state().id,
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  // Configure 2x
  EXPECT_FALSE(
    lifecycle_client()->change_state(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
  EXPECT_EQ(
    lifecycle_client()->get_state().id,
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_TRUE(
    lifecycle_client()->change_state(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
  EXPECT_EQ(
    lifecycle_client()->get_state().id,
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // Activate 2x
  EXPECT_FALSE(
    lifecycle_client()->change_state(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));
  EXPECT_EQ(
    lifecycle_client()->get_state().id,
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_TRUE(
    lifecycle_client()->change_state(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));
  EXPECT_EQ(
    lifecycle_client()->get_state().id,
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // Deactivate 2x
  EXPECT_FALSE(
    lifecycle_client()->change_state(
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE));
  EXPECT_EQ(
    lifecycle_client()->get_state().id,
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_TRUE(
    lifecycle_client()->change_state(
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE));
  EXPECT_EQ(
    lifecycle_client()->get_state().id,
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // Cleanup 2x
  EXPECT_FALSE(
    lifecycle_client()->change_state(
      lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP));
  EXPECT_EQ(
    lifecycle_client()->get_state().id,
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_TRUE(
    lifecycle_client()->change_state(
      lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP));
  EXPECT_EQ(
    lifecycle_client()->get_state().id,
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  EXPECT_EQ(
    lifecycle_client()->get_state().id,
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  // Shutdown(Error) -> ErrorProcessing(Success)
  EXPECT_TRUE(
    lifecycle_client()->change_state(
      lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN));
  EXPECT_EQ(
    lifecycle_client()->get_state().id,
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  // Shutdown(Success)
  EXPECT_TRUE(
    lifecycle_client()->change_state(
      lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN));
  EXPECT_EQ(
    lifecycle_client()->get_state().id,
    lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);

  // 2 * change_state calls + 1 ErrorProcessing
  EXPECT_EQ(lifecycle_node()->number_of_callbacks, 11);
}
