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

#include <gtest/gtest.h>

#include <rcl/allocator.h>
#include <rcl/time.h>
#include <rcl/types.h>

#include <rcl_action/names.h>
#include <rcl_action/default_qos.h>
#include <rcl_action/wait.h>

#include <rclcpp/clock.hpp>
#include <rclcpp/exceptions.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/time.hpp>

#include <test_msgs/action/fibonacci.hpp>

#include <future>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <thread>
#include <chrono>

#include "rclcpp_action/exceptions.hpp"
#include "rclcpp_action/create_client.hpp"
#include "rclcpp_action/client.hpp"
#include "rclcpp_action/qos.hpp"
#include "rclcpp_action/types.hpp"

#include "./mocking_utils/patch.hpp"

using namespace std::chrono_literals;

const auto WAIT_FOR_SERVER_TIMEOUT = 10s;

class TestClient : public ::testing::Test
{
protected:
  using ActionType = test_msgs::action::Fibonacci;
  using ActionGoal = ActionType::Goal;
  using ActionGoalHandle = rclcpp_action::ClientGoalHandle<ActionType>;
  using ActionGoalRequestService = ActionType::Impl::SendGoalService;
  using ActionGoalRequest = ActionGoalRequestService::Request;
  using ActionGoalResponse = ActionGoalRequestService::Response;
  using ActionGoalResultService = ActionType::Impl::GetResultService;
  using ActionGoalResultRequest = ActionGoalResultService::Request;
  using ActionGoalResultResponse = ActionGoalResultService::Response;
  using ActionCancelGoalService = ActionType::Impl::CancelGoalService;
  using ActionCancelGoalRequest = ActionType::Impl::CancelGoalService::Request;
  using ActionCancelGoalResponse = ActionType::Impl::CancelGoalService::Response;
  using ActionStatusMessage = ActionType::Impl::GoalStatusMessage;
  using ActionFeedbackMessage = ActionType::Impl::FeedbackMessage;
  using ActionFeedback = ActionType::Feedback;

  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUpServer()
  {
    rcl_allocator_t allocator = rcl_get_default_allocator();

    server_node = std::make_shared<rclcpp::Node>(server_node_name, namespace_name);

    char * goal_service_name = nullptr;
    rcl_ret_t ret = rcl_action_get_goal_service_name(
      action_name, allocator, &goal_service_name);
    ASSERT_EQ(RCL_RET_OK, ret);
    goal_service = server_node->create_service<ActionGoalRequestService>(
      goal_service_name,
      [this](
        const ActionGoalRequest::SharedPtr request,
        ActionGoalResponse::SharedPtr response)
      {
        response->stamp = clock.now();
        response->accepted = (request->goal.order >= 0);
        if (response->accepted) {
          goals[request->goal_id.uuid] = {request, response};
        }
      });
    ASSERT_TRUE(goal_service != nullptr);
    allocator.deallocate(goal_service_name, allocator.state);

    char * result_service_name = nullptr;
    ret = rcl_action_get_result_service_name(
      action_name, allocator, &result_service_name);
    ASSERT_EQ(RCL_RET_OK, ret);
    result_service = server_node->create_service<ActionGoalResultService>(
      result_service_name,
      [this](
        const ActionGoalResultRequest::SharedPtr request,
        ActionGoalResultResponse::SharedPtr response)
      {
        if (goals.count(request->goal_id.uuid) == 1) {
          auto goal_request = goals[request->goal_id.uuid].first;
          auto goal_response = goals[request->goal_id.uuid].second;
          ActionStatusMessage status_message;
          rclcpp_action::GoalStatus goal_status;
          goal_status.goal_info.goal_id.uuid = goal_request->goal_id.uuid;
          goal_status.goal_info.stamp = goal_response->stamp;
          goal_status.status = rclcpp_action::GoalStatus::STATUS_EXECUTING;
          status_message.status_list.push_back(goal_status);
          status_publisher->publish(status_message);
          client_executor.spin_once();
          ActionFeedbackMessage feedback_message;
          feedback_message.goal_id.uuid = goal_request->goal_id.uuid;
          feedback_message.feedback.sequence.push_back(0);
          feedback_publisher->publish(feedback_message);
          client_executor.spin_once();
          if (goal_request->goal.order > 0) {
            feedback_message.feedback.sequence.push_back(1);
            feedback_publisher->publish(feedback_message);
            client_executor.spin_once();
            for (int i = 1; i < goal_request->goal.order; ++i) {
              feedback_message.feedback.sequence.push_back(
                feedback_message.feedback.sequence[i] +
                feedback_message.feedback.sequence[i - 1]);
              feedback_publisher->publish(feedback_message);
              client_executor.spin_once();
            }
          }
          goal_status.status = rclcpp_action::GoalStatus::STATUS_SUCCEEDED;
          status_message.status_list[0] = goal_status;
          status_publisher->publish(status_message);
          client_executor.spin_once();
          response->result.sequence = feedback_message.feedback.sequence;
          response->status = rclcpp_action::GoalStatus::STATUS_SUCCEEDED;
          goals.erase(request->goal_id.uuid);
        } else {
          response->status = rclcpp_action::GoalStatus::STATUS_UNKNOWN;
        }
      });
    ASSERT_TRUE(result_service != nullptr);
    allocator.deallocate(result_service_name, allocator.state);

    char * cancel_service_name = nullptr;
    ret = rcl_action_get_cancel_service_name(
      action_name, allocator, &cancel_service_name);
    ASSERT_EQ(RCL_RET_OK, ret);
    cancel_service = server_node->create_service<ActionCancelGoalService>(
      cancel_service_name,
      [this](
        const ActionCancelGoalRequest::SharedPtr request,
        ActionCancelGoalResponse::SharedPtr response)
      {
        rclcpp_action::GoalUUID zero_uuid;
        std::fill(zero_uuid.begin(), zero_uuid.end(), 0u);
        const rclcpp::Time cancel_stamp = request->goal_info.stamp;
        bool cancel_all = (
          request->goal_info.goal_id.uuid == zero_uuid &&
          cancel_stamp == zero_stamp);
        ActionStatusMessage status_message;
        auto it = goals.begin();
        while (it != goals.end()) {
          auto goal_request = it->second.first;
          auto goal_response = it->second.second;
          const rclcpp::Time goal_stamp = goal_response->stamp;
          bool cancel_this = (
            request->goal_info.goal_id.uuid == goal_request->goal_id.uuid ||
            cancel_stamp > goal_stamp);
          if (cancel_all || cancel_this) {
            rclcpp_action::GoalStatus goal_status;
            goal_status.goal_info.goal_id.uuid = goal_request->goal_id.uuid;
            goal_status.goal_info.stamp = goal_response->stamp;
            goal_status.status = rclcpp_action::GoalStatus::STATUS_CANCELED;
            status_message.status_list.push_back(goal_status);
            response->goals_canceling.push_back(goal_status.goal_info);
            it = goals.erase(it);
          } else {
            ++it;
          }
        }
        status_publisher->publish(status_message);
        client_executor.spin_once();
      });
    ASSERT_TRUE(cancel_service != nullptr);
    allocator.deallocate(cancel_service_name, allocator.state);

    char * feedback_topic_name = nullptr;
    ret = rcl_action_get_feedback_topic_name(
      action_name, allocator, &feedback_topic_name);
    ASSERT_EQ(RCL_RET_OK, ret);
    feedback_publisher =
      server_node->create_publisher<ActionFeedbackMessage>(feedback_topic_name, 10);
    ASSERT_TRUE(feedback_publisher != nullptr);
    allocator.deallocate(feedback_topic_name, allocator.state);

    char * status_topic_name = nullptr;
    ret = rcl_action_get_status_topic_name(
      action_name, allocator, &status_topic_name);
    ASSERT_EQ(RCL_RET_OK, ret);
    status_publisher = server_node->create_publisher<ActionStatusMessage>(
      status_topic_name, rclcpp_action::DefaultActionStatusQoS());
    ASSERT_TRUE(status_publisher != nullptr);
    allocator.deallocate(status_topic_name, allocator.state);
    server_executor.add_node(server_node);
  }

  void SetUp() override
  {
    client_node = std::make_shared<rclcpp::Node>(client_node_name, namespace_name);
    client_executor.add_node(client_node);

    ASSERT_EQ(RCL_RET_OK, rcl_enable_ros_time_override(clock.get_clock_handle()));
    ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(clock.get_clock_handle(), RCL_S_TO_NS(1)));
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void TearDownServer()
  {
    status_publisher.reset();
    feedback_publisher.reset();
    cancel_service.reset();
    result_service.reset();
    goal_service.reset();
    server_node.reset();
  }

  void TearDown() override
  {
    client_node.reset();
  }

  template<typename FutureT>
  void dual_spin_until_future_complete(std::shared_future<FutureT> & future)
  {
    std::future_status status;
    do {
      server_executor.spin_some();
      client_executor.spin_some();
      status = future.wait_for(std::chrono::seconds(0));
    } while (std::future_status::ready != status);
  }

  rclcpp::Clock clock{RCL_ROS_TIME};
  const rclcpp::Time zero_stamp{0, 0, RCL_ROS_TIME};

  rclcpp::Node::SharedPtr server_node;
  rclcpp::executors::SingleThreadedExecutor server_executor;
  rclcpp::Node::SharedPtr client_node;
  rclcpp::executors::SingleThreadedExecutor client_executor;
  const char * const server_node_name{"fibonacci_action_test_server"};
  const char * const client_node_name{"fibonacci_action_test_client"};
  const char * const namespace_name{"/rclcpp_action/test/client"};
  const char * const action_name{"fibonacci_test"};

  std::map<
    rclcpp_action::GoalUUID,
    std::pair<
      typename ActionGoalRequest::SharedPtr,
      typename ActionGoalResponse::SharedPtr>> goals;
  typename rclcpp::Service<ActionGoalRequestService>::SharedPtr goal_service;
  typename rclcpp::Service<ActionGoalResultService>::SharedPtr result_service;
  typename rclcpp::Service<ActionCancelGoalService>::SharedPtr cancel_service;
  typename rclcpp::Publisher<ActionFeedbackMessage>::SharedPtr feedback_publisher;
  typename rclcpp::Publisher<ActionStatusMessage>::SharedPtr status_publisher;
};

class TestClientAgainstServer : public TestClient
{
protected:
  void SetUp() override
  {
    SetUpServer();
    TestClient::SetUp();
  }

  void TearDown() override
  {
    TestClient::TearDown();
    TearDownServer();
  }
};


TEST_F(TestClient, construction_and_destruction)
{
  ASSERT_NO_THROW(rclcpp_action::create_client<ActionType>(client_node, action_name).reset());
}

TEST_F(TestClient, construction_and_destruction_after_node)
{
  ASSERT_NO_THROW(
  {
    auto action_client = rclcpp_action::create_client<ActionType>(client_node, action_name);
    client_node.reset();
  });
}

TEST_F(TestClient, construction_and_destruction_callback_group)
{
  auto group = client_node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  ASSERT_NO_THROW(
    rclcpp_action::create_client<ActionType>(
      client_node->get_node_base_interface(),
      client_node->get_node_graph_interface(),
      client_node->get_node_logging_interface(),
      client_node->get_node_waitables_interface(),
      action_name,
      group
    ).reset());
}

TEST_F(TestClient, construction_and_destruction_rcl_errors)
{
  {
    auto mock = mocking_utils::inject_on_return(
      "lib:rclcpp_action", rcl_action_client_fini, RCL_RET_ERROR);
    // It just logs an error message and continues
    EXPECT_NO_THROW(
      rclcpp_action::create_client<ActionType>(client_node, action_name).reset());
  }
  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp_action", rcl_action_client_init, RCL_RET_ERROR);
    EXPECT_THROW(
      rclcpp_action::create_client<ActionType>(client_node, action_name).reset(),
      rclcpp::exceptions::RCLError);
  }
  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp_action", rcl_action_client_wait_set_get_num_entities, RCL_RET_ERROR);
    EXPECT_THROW(
      rclcpp_action::create_client<ActionType>(client_node, action_name),
      rclcpp::exceptions::RCLError);
  }
}

TEST_F(TestClient, wait_for_action_server)
{
  auto action_client = rclcpp_action::create_client<ActionType>(client_node, action_name);
  EXPECT_FALSE(action_client->wait_for_action_server(0ms));
  EXPECT_FALSE(action_client->wait_for_action_server(10ms));
  SetUpServer();
  ASSERT_TRUE(action_client->wait_for_action_server(WAIT_FOR_SERVER_TIMEOUT));
  TearDownServer();

  client_node.reset();  // Drop node before action client
  EXPECT_THROW(action_client->wait_for_action_server(0ms), rclcpp::exceptions::InvalidNodeError);
}

TEST_F(TestClient, wait_for_action_server_rcl_errors)
{
  auto action_client = rclcpp_action::create_client<ActionType>(client_node, action_name);
  SetUpServer();
  ASSERT_TRUE(action_client->wait_for_action_server(WAIT_FOR_SERVER_TIMEOUT));
  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp_action", rcl_action_server_is_available, RCL_RET_NODE_INVALID);
    EXPECT_THROW(action_client->action_server_is_ready(), rclcpp::exceptions::RCLError);

    auto mock_context_is_valid = mocking_utils::patch_and_return(
      "lib:rclcpp_action", rcl_context_is_valid, false);
    EXPECT_FALSE(action_client->action_server_is_ready());
  }
  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp_action", rcl_action_server_is_available, RCL_RET_ERROR);
    EXPECT_THROW(action_client->action_server_is_ready(), rclcpp::exceptions::RCLError);
  }
  TearDownServer();
}

TEST_F(TestClient, is_ready) {
  auto action_client = rclcpp_action::create_client<ActionType>(client_node, action_name);
  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto rcl_context = client_node->get_node_base_interface()->get_context()->get_rcl_context().get();
  ASSERT_EQ(
    RCL_RET_OK,
    rcl_wait_set_init(&wait_set, 10, 10, 10, 10, 10, 10, rcl_context, allocator));
  ASSERT_TRUE(action_client->add_to_wait_set(&wait_set));
  EXPECT_TRUE(action_client->is_ready(&wait_set));

  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp_action", rcl_action_client_wait_set_get_entities_ready, RCL_RET_ERROR);
    EXPECT_THROW(action_client->is_ready(&wait_set), rclcpp::exceptions::RCLError);
  }
  client_node.reset();  // Drop node before action client
}

TEST_F(TestClientAgainstServer, async_send_goal_no_callbacks)
{
  auto action_client = rclcpp_action::create_client<ActionType>(client_node, action_name);
  ASSERT_TRUE(action_client->wait_for_action_server(WAIT_FOR_SERVER_TIMEOUT));

  ActionGoal bad_goal;
  bad_goal.order = -5;
  auto future_goal_handle = action_client->async_send_goal(bad_goal);
  dual_spin_until_future_complete(future_goal_handle);
  EXPECT_EQ(nullptr, future_goal_handle.get().get());

  ActionGoal good_goal;
  good_goal.order = 5;
  future_goal_handle = action_client->async_send_goal(good_goal);
  dual_spin_until_future_complete(future_goal_handle);
  auto goal_handle = future_goal_handle.get();
  EXPECT_EQ(rclcpp_action::GoalStatus::STATUS_ACCEPTED, goal_handle->get_status());
  EXPECT_FALSE(goal_handle->is_feedback_aware());
  EXPECT_FALSE(goal_handle->is_result_aware());
}

TEST_F(TestClientAgainstServer, bad_goal_handles)
{
  auto action_client0 = rclcpp_action::create_client<ActionType>(client_node, action_name);
  ASSERT_TRUE(action_client0->wait_for_action_server(WAIT_FOR_SERVER_TIMEOUT));

  ActionGoal goal;
  goal.order = 0;
  auto future_goal_handle = action_client0->async_send_goal(goal);
  dual_spin_until_future_complete(future_goal_handle);
  auto goal_handle = future_goal_handle.get();

  auto action_client1 = rclcpp_action::create_client<ActionType>(client_node, action_name);
  using rclcpp_action::exceptions::UnknownGoalHandleError;
  EXPECT_THROW(action_client1->async_get_result(goal_handle), UnknownGoalHandleError);
  EXPECT_THROW(action_client1->async_cancel_goal(goal_handle), UnknownGoalHandleError);
}

TEST_F(TestClientAgainstServer, async_send_goal_no_callbacks_wait_for_result)
{
  auto action_client = rclcpp_action::create_client<ActionType>(client_node, action_name);
  ASSERT_TRUE(action_client->wait_for_action_server(WAIT_FOR_SERVER_TIMEOUT));

  ActionGoal goal;
  goal.order = 5;
  auto future_goal_handle = action_client->async_send_goal(goal);
  dual_spin_until_future_complete(future_goal_handle);
  auto goal_handle = future_goal_handle.get();
  EXPECT_EQ(rclcpp_action::GoalStatus::STATUS_ACCEPTED, goal_handle->get_status());
  EXPECT_FALSE(goal_handle->is_feedback_aware());
  EXPECT_FALSE(goal_handle->is_result_aware());
  auto future_result = action_client->async_get_result(goal_handle);
  EXPECT_TRUE(goal_handle->is_result_aware());
  dual_spin_until_future_complete(future_result);
  auto wrapped_result = future_result.get();
  ASSERT_EQ(6ul, wrapped_result.result->sequence.size());
  EXPECT_EQ(0, wrapped_result.result->sequence[0]);
  EXPECT_EQ(1, wrapped_result.result->sequence[1]);
  EXPECT_EQ(5, wrapped_result.result->sequence[5]);
}

TEST_F(TestClientAgainstServer, async_send_goal_no_callbacks_then_invalidate)
{
  auto action_client = rclcpp_action::create_client<ActionType>(client_node, action_name);
  ASSERT_TRUE(action_client->wait_for_action_server(WAIT_FOR_SERVER_TIMEOUT));

  ActionGoal goal;
  goal.order = 5;
  auto future_goal_handle = action_client->async_send_goal(goal);
  dual_spin_until_future_complete(future_goal_handle);
  auto goal_handle = future_goal_handle.get();
  ASSERT_NE(nullptr, goal_handle);
  EXPECT_EQ(rclcpp_action::GoalStatus::STATUS_ACCEPTED, goal_handle->get_status());
  auto future_result = action_client->async_get_result(goal_handle);
  EXPECT_TRUE(goal_handle->is_result_aware());

  action_client.reset();  // Ensure goal handle is invalidated once client goes out of scope

  EXPECT_EQ(rclcpp_action::GoalStatus::STATUS_UNKNOWN, goal_handle->get_status());
  using rclcpp_action::exceptions::UnawareGoalHandleError;
  EXPECT_THROW(future_result.get(), UnawareGoalHandleError);
}

TEST_F(TestClientAgainstServer, async_send_goal_with_goal_response_callback_wait_for_result)
{
  auto action_client = rclcpp_action::create_client<ActionType>(client_node, action_name);
  ASSERT_TRUE(action_client->wait_for_action_server(WAIT_FOR_SERVER_TIMEOUT));

  bool goal_response_received = false;
  auto send_goal_ops = rclcpp_action::Client<ActionType>::SendGoalOptions();
  send_goal_ops.goal_response_callback =
    [&goal_response_received]
      (std::shared_future<typename ActionGoalHandle::SharedPtr> future) mutable
    {
      auto goal_handle = future.get();
      if (goal_handle) {
        goal_response_received = true;
      }
    };

  {
    ActionGoal bad_goal;
    bad_goal.order = -1;
    auto future_goal_handle = action_client->async_send_goal(bad_goal, send_goal_ops);
    dual_spin_until_future_complete(future_goal_handle);
    auto goal_handle = future_goal_handle.get();
    EXPECT_FALSE(goal_response_received);
    EXPECT_EQ(nullptr, goal_handle);
  }

  {
    ActionGoal goal;
    goal.order = 4;
    auto future_goal_handle = action_client->async_send_goal(goal, send_goal_ops);
    dual_spin_until_future_complete(future_goal_handle);
    auto goal_handle = future_goal_handle.get();
    EXPECT_TRUE(goal_response_received);
    EXPECT_EQ(rclcpp_action::GoalStatus::STATUS_ACCEPTED, goal_handle->get_status());
    EXPECT_FALSE(goal_handle->is_feedback_aware());
    EXPECT_FALSE(goal_handle->is_result_aware());
    auto future_result = action_client->async_get_result(goal_handle);
    EXPECT_TRUE(goal_handle->is_result_aware());
    dual_spin_until_future_complete(future_result);
    auto wrapped_result = future_result.get();
    ASSERT_EQ(5u, wrapped_result.result->sequence.size());
    EXPECT_EQ(3, wrapped_result.result->sequence.back());
  }
}

TEST_F(TestClientAgainstServer, async_send_goal_with_feedback_callback_wait_for_result)
{
  auto action_client = rclcpp_action::create_client<ActionType>(client_node, action_name);
  ASSERT_TRUE(action_client->wait_for_action_server(WAIT_FOR_SERVER_TIMEOUT));

  ActionGoal goal;
  goal.order = 4;
  int feedback_count = 0;
  auto send_goal_ops = rclcpp_action::Client<ActionType>::SendGoalOptions();
  send_goal_ops.feedback_callback =
    [&feedback_count](
    typename ActionGoalHandle::SharedPtr goal_handle,
    const std::shared_ptr<const ActionFeedback> feedback)
    {
      (void)goal_handle;
      (void)feedback;
      feedback_count++;
    };
  auto future_goal_handle = action_client->async_send_goal(goal, send_goal_ops);
  dual_spin_until_future_complete(future_goal_handle);
  auto goal_handle = future_goal_handle.get();
  EXPECT_EQ(rclcpp_action::GoalStatus::STATUS_ACCEPTED, goal_handle->get_status());
  EXPECT_TRUE(goal_handle->is_feedback_aware());
  EXPECT_FALSE(goal_handle->is_result_aware());
  auto future_result = action_client->async_get_result(goal_handle);
  EXPECT_TRUE(goal_handle->is_result_aware());
  dual_spin_until_future_complete(future_result);
  auto wrapped_result = future_result.get();

  ASSERT_EQ(5u, wrapped_result.result->sequence.size());
  EXPECT_EQ(3, wrapped_result.result->sequence.back());
  EXPECT_EQ(5, feedback_count);
}

TEST_F(TestClientAgainstServer, async_send_goal_with_result_callback_wait_for_result)
{
  auto action_client = rclcpp_action::create_client<ActionType>(client_node, action_name);
  ASSERT_TRUE(action_client->wait_for_action_server(WAIT_FOR_SERVER_TIMEOUT));

  ActionGoal goal;
  goal.order = 4;
  bool result_callback_received = false;
  auto send_goal_ops = rclcpp_action::Client<ActionType>::SendGoalOptions();
  send_goal_ops.result_callback =
    [&result_callback_received](
    const typename ActionGoalHandle::WrappedResult & result)
    {
      if (
        rclcpp_action::ResultCode::SUCCEEDED == result.code &&
        result.result->sequence.size() == 5u)
      {
        result_callback_received = true;
      }
    };
  auto future_goal_handle = action_client->async_send_goal(goal, send_goal_ops);
  dual_spin_until_future_complete(future_goal_handle);
  auto goal_handle = future_goal_handle.get();
  EXPECT_EQ(rclcpp_action::GoalStatus::STATUS_ACCEPTED, goal_handle->get_status());
  EXPECT_FALSE(goal_handle->is_feedback_aware());
  EXPECT_TRUE(goal_handle->is_result_aware());
  auto future_result = action_client->async_get_result(goal_handle);
  dual_spin_until_future_complete(future_result);
  auto wrapped_result = future_result.get();

  EXPECT_TRUE(result_callback_received);
  ASSERT_EQ(5u, wrapped_result.result->sequence.size());
  EXPECT_EQ(3, wrapped_result.result->sequence.back());
}

TEST_F(TestClientAgainstServer, async_get_result_with_callback)
{
  auto action_client = rclcpp_action::create_client<ActionType>(client_node, action_name);
  ASSERT_TRUE(action_client->wait_for_action_server(WAIT_FOR_SERVER_TIMEOUT));

  ActionGoal goal;
  goal.order = 4;
  auto future_goal_handle = action_client->async_send_goal(goal);
  dual_spin_until_future_complete(future_goal_handle);
  auto goal_handle = future_goal_handle.get();
  EXPECT_NE(goal_handle, nullptr);
  EXPECT_EQ(rclcpp_action::GoalStatus::STATUS_ACCEPTED, goal_handle->get_status());
  EXPECT_FALSE(goal_handle->is_feedback_aware());
  EXPECT_FALSE(goal_handle->is_result_aware());
  bool result_callback_received = false;
  auto future_result = action_client->async_get_result(
    goal_handle,
    [&result_callback_received](
      const typename ActionGoalHandle::WrappedResult & result)
    {
      if (
        rclcpp_action::ResultCode::SUCCEEDED == result.code &&
        result.result->sequence.size() == 5u)
      {
        result_callback_received = true;
      }
    });
  dual_spin_until_future_complete(future_result);
  auto wrapped_result = future_result.get();

  EXPECT_TRUE(result_callback_received);
  ASSERT_EQ(5u, wrapped_result.result->sequence.size());
  EXPECT_EQ(3, wrapped_result.result->sequence.back());
}

TEST_F(TestClientAgainstServer, async_cancel_one_goal)
{
  auto action_client = rclcpp_action::create_client<ActionType>(client_node, action_name);
  ASSERT_TRUE(action_client->wait_for_action_server(WAIT_FOR_SERVER_TIMEOUT));

  ActionGoal goal;
  goal.order = 5;
  auto future_goal_handle = action_client->async_send_goal(goal);
  dual_spin_until_future_complete(future_goal_handle);
  auto goal_handle = future_goal_handle.get();
  EXPECT_EQ(rclcpp_action::GoalStatus::STATUS_ACCEPTED, goal_handle->get_status());

  auto future_cancel = action_client->async_cancel_goal(goal_handle);
  dual_spin_until_future_complete(future_cancel);
  ActionCancelGoalResponse::SharedPtr cancel_response = future_cancel.get();
  EXPECT_EQ(ActionCancelGoalResponse::ERROR_NONE, cancel_response->return_code);
}

TEST_F(TestClientAgainstServer, async_cancel_one_goal_with_callback)
{
  auto action_client = rclcpp_action::create_client<ActionType>(client_node, action_name);
  ASSERT_TRUE(action_client->wait_for_action_server(WAIT_FOR_SERVER_TIMEOUT));

  ActionGoal goal;
  goal.order = 5;
  auto future_goal_handle = action_client->async_send_goal(goal);
  dual_spin_until_future_complete(future_goal_handle);
  auto goal_handle = future_goal_handle.get();
  EXPECT_EQ(rclcpp_action::GoalStatus::STATUS_ACCEPTED, goal_handle->get_status());

  bool cancel_response_received = false;
  auto future_cancel = action_client->async_cancel_goal(
    goal_handle,
    [&cancel_response_received, goal_handle](
      ActionCancelGoalResponse::SharedPtr response)
    {
      if (
        ActionCancelGoalResponse::ERROR_NONE == response->return_code &&
        1ul == response->goals_canceling.size() &&
        goal_handle->get_goal_id() == response->goals_canceling[0].goal_id.uuid)
      {
        cancel_response_received = true;
      }
    });
  dual_spin_until_future_complete(future_cancel);
  auto cancel_response = future_cancel.get();
  EXPECT_EQ(ActionCancelGoalResponse::ERROR_NONE, cancel_response->return_code);
  ASSERT_EQ(1ul, cancel_response->goals_canceling.size());
  EXPECT_EQ(goal_handle->get_goal_id(), cancel_response->goals_canceling[0].goal_id.uuid);
  EXPECT_TRUE(cancel_response_received);
}

TEST_F(TestClientAgainstServer, async_cancel_all_goals)
{
  auto action_client = rclcpp_action::create_client<ActionType>(client_node, action_name);
  ASSERT_TRUE(action_client->wait_for_action_server(WAIT_FOR_SERVER_TIMEOUT));

  ActionGoal goal;
  goal.order = 6;
  auto future_goal_handle0 = action_client->async_send_goal(goal);
  dual_spin_until_future_complete(future_goal_handle0);
  auto goal_handle0 = future_goal_handle0.get();

  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(clock.get_clock_handle(), RCL_S_TO_NS(2)));

  goal.order = 8;
  auto future_goal_handle1 = action_client->async_send_goal(goal);
  dual_spin_until_future_complete(future_goal_handle1);
  auto goal_handle1 = future_goal_handle1.get();

  if (goal_handle1->get_goal_id() < goal_handle0->get_goal_id()) {
    goal_handle0.swap(goal_handle1);
  }

  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(clock.get_clock_handle(), RCL_S_TO_NS(3)));

  auto future_cancel_all = action_client->async_cancel_all_goals();
  dual_spin_until_future_complete(future_cancel_all);
  auto cancel_response = future_cancel_all.get();

  EXPECT_EQ(ActionCancelGoalResponse::ERROR_NONE, cancel_response->return_code);
  ASSERT_EQ(2ul, cancel_response->goals_canceling.size());
  EXPECT_EQ(goal_handle0->get_goal_id(), cancel_response->goals_canceling[0].goal_id.uuid);
  EXPECT_EQ(goal_handle1->get_goal_id(), cancel_response->goals_canceling[1].goal_id.uuid);
  EXPECT_EQ(rclcpp_action::GoalStatus::STATUS_CANCELED, goal_handle0->get_status());
  EXPECT_EQ(rclcpp_action::GoalStatus::STATUS_CANCELED, goal_handle1->get_status());
}

TEST_F(TestClientAgainstServer, async_cancel_all_goals_with_callback)
{
  auto action_client = rclcpp_action::create_client<ActionType>(client_node, action_name);
  ASSERT_TRUE(action_client->wait_for_action_server(WAIT_FOR_SERVER_TIMEOUT));

  ActionGoal goal;
  goal.order = 6;
  auto future_goal_handle0 = action_client->async_send_goal(goal);
  dual_spin_until_future_complete(future_goal_handle0);
  auto goal_handle0 = future_goal_handle0.get();

  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(clock.get_clock_handle(), RCL_S_TO_NS(2)));

  goal.order = 8;
  auto future_goal_handle1 = action_client->async_send_goal(goal);
  dual_spin_until_future_complete(future_goal_handle1);
  auto goal_handle1 = future_goal_handle1.get();

  if (goal_handle1->get_goal_id() < goal_handle0->get_goal_id()) {
    goal_handle0.swap(goal_handle1);
  }

  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(clock.get_clock_handle(), RCL_S_TO_NS(3)));

  bool cancel_callback_received = false;
  auto future_cancel_all = action_client->async_cancel_all_goals(
    [&cancel_callback_received, goal_handle0, goal_handle1](
      ActionCancelGoalResponse::SharedPtr response)
    {
      if (
        response &&
        2ul == response->goals_canceling.size() &&
        goal_handle0->get_goal_id() == response->goals_canceling[0].goal_id.uuid &&
        goal_handle1->get_goal_id() == response->goals_canceling[1].goal_id.uuid)
      {
        cancel_callback_received = true;
      }
    });
  dual_spin_until_future_complete(future_cancel_all);
  auto cancel_response = future_cancel_all.get();

  EXPECT_EQ(ActionCancelGoalResponse::ERROR_NONE, cancel_response->return_code);
  EXPECT_TRUE(cancel_callback_received);
  ASSERT_EQ(2ul, cancel_response->goals_canceling.size());
  EXPECT_EQ(goal_handle0->get_goal_id(), cancel_response->goals_canceling[0].goal_id.uuid);
  EXPECT_EQ(goal_handle1->get_goal_id(), cancel_response->goals_canceling[1].goal_id.uuid);
  EXPECT_EQ(rclcpp_action::GoalStatus::STATUS_CANCELED, goal_handle0->get_status());
  EXPECT_EQ(rclcpp_action::GoalStatus::STATUS_CANCELED, goal_handle1->get_status());
}

TEST_F(TestClientAgainstServer, async_cancel_some_goals)
{
  auto action_client = rclcpp_action::create_client<ActionType>(client_node, action_name);
  ASSERT_TRUE(action_client->wait_for_action_server(WAIT_FOR_SERVER_TIMEOUT));

  ActionGoal goal;
  goal.order = 6;
  auto future_goal_handle0 = action_client->async_send_goal(goal);
  dual_spin_until_future_complete(future_goal_handle0);
  auto goal_handle0 = future_goal_handle0.get();

  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(clock.get_clock_handle(), RCL_S_TO_NS(2)));

  goal.order = 8;
  auto future_goal_handle1 = action_client->async_send_goal(goal);
  dual_spin_until_future_complete(future_goal_handle1);
  auto goal_handle1 = future_goal_handle1.get();

  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(clock.get_clock_handle(), RCL_S_TO_NS(3)));

  auto future_cancel_some =
    action_client->async_cancel_goals_before(goal_handle1->get_goal_stamp());
  dual_spin_until_future_complete(future_cancel_some);
  auto cancel_response = future_cancel_some.get();

  EXPECT_EQ(ActionCancelGoalResponse::ERROR_NONE, cancel_response->return_code);
  ASSERT_EQ(1ul, cancel_response->goals_canceling.size());
  EXPECT_EQ(goal_handle0->get_goal_id(), cancel_response->goals_canceling[0].goal_id.uuid);
  EXPECT_EQ(rclcpp_action::GoalStatus::STATUS_CANCELED, goal_handle0->get_status());
}

TEST_F(TestClientAgainstServer, async_cancel_some_goals_with_callback)
{
  auto action_client = rclcpp_action::create_client<ActionType>(client_node, action_name);
  ASSERT_TRUE(action_client->wait_for_action_server(WAIT_FOR_SERVER_TIMEOUT));

  ActionGoal goal;
  goal.order = 6;
  auto future_goal_handle0 = action_client->async_send_goal(goal);
  dual_spin_until_future_complete(future_goal_handle0);
  auto goal_handle0 = future_goal_handle0.get();

  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(clock.get_clock_handle(), RCL_S_TO_NS(2)));

  goal.order = 8;
  auto future_goal_handle1 = action_client->async_send_goal(goal);
  dual_spin_until_future_complete(future_goal_handle1);
  auto goal_handle1 = future_goal_handle1.get();

  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(clock.get_clock_handle(), RCL_S_TO_NS(3)));

  bool cancel_callback_received = false;
  auto future_cancel_some = action_client->async_cancel_goals_before(
    goal_handle1->get_goal_stamp(),
    [&cancel_callback_received, goal_handle0](ActionCancelGoalResponse::SharedPtr response)
    {
      if (
        response &&
        1ul == response->goals_canceling.size() &&
        goal_handle0->get_goal_id() == response->goals_canceling[0].goal_id.uuid)
      {
        cancel_callback_received = true;
      }
    });
  dual_spin_until_future_complete(future_cancel_some);
  auto cancel_response = future_cancel_some.get();

  EXPECT_EQ(ActionCancelGoalResponse::ERROR_NONE, cancel_response->return_code);
  EXPECT_TRUE(cancel_callback_received);
  ASSERT_EQ(1ul, cancel_response->goals_canceling.size());
  EXPECT_EQ(goal_handle0->get_goal_id(), cancel_response->goals_canceling[0].goal_id.uuid);
  EXPECT_EQ(rclcpp_action::GoalStatus::STATUS_CANCELED, goal_handle0->get_status());
}

TEST_F(TestClientAgainstServer, send_rcl_errors)
{
  auto action_client = rclcpp_action::create_client<ActionType>(client_node, action_name);
  ASSERT_TRUE(action_client->wait_for_action_server(WAIT_FOR_SERVER_TIMEOUT));

  auto send_goal_ops = rclcpp_action::Client<ActionType>::SendGoalOptions();
  send_goal_ops.result_callback =
    [](const typename ActionGoalHandle::WrappedResult &) {};
  send_goal_ops.feedback_callback =
    [](typename ActionGoalHandle::SharedPtr,
      const std::shared_ptr<const ActionFeedback>) {};

  {
    ActionGoal goal;
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp_action", rcl_action_send_goal_request, RCL_RET_ERROR);
    EXPECT_THROW(
      action_client->async_send_goal(goal, send_goal_ops),
      rclcpp::exceptions::RCLError);
  }
  // TODO(anyone): Review this test
  // {
  //   ActionGoal goal;
  //   auto mock = mocking_utils::patch_and_return(
  //     "lib:rclcpp_action", rcl_action_send_result_request, RCL_RET_ERROR);
  //   auto future_goal_handle = action_client->async_send_goal(goal, send_goal_ops);
  //   dual_spin_until_future_complete(future_goal_handle);
  //   auto goal_handle = future_goal_handle.get();
  //   EXPECT_EQ(rclcpp_action::GoalStatus::STATUS_UNKNOWN, goal_handle->get_status());
  // }
  {
    ActionGoal goal;
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp_action", rcl_action_send_cancel_request, RCL_RET_ERROR);
    auto future_goal_handle = action_client->async_send_goal(goal, send_goal_ops);
    dual_spin_until_future_complete(future_goal_handle);
    auto goal_handle = future_goal_handle.get();
    EXPECT_THROW(
      action_client->async_cancel_goals_before(goal_handle->get_goal_stamp()),
      rclcpp::exceptions::RCLError);
  }
}

TEST_F(TestClientAgainstServer, execute_rcl_errors)
{
  auto action_client = rclcpp_action::create_client<ActionType>(client_node, action_name);
  ASSERT_TRUE(action_client->wait_for_action_server(WAIT_FOR_SERVER_TIMEOUT));

  auto send_goal_ops = rclcpp_action::Client<ActionType>::SendGoalOptions();
  send_goal_ops.result_callback =
    [](const typename ActionGoalHandle::WrappedResult &) {};
  send_goal_ops.feedback_callback =
    [](typename ActionGoalHandle::SharedPtr,
      const std::shared_ptr<const ActionFeedback>) {};

  {
    ActionGoal goal;
    goal.order = 5;
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp_action", rcl_action_take_feedback, RCL_RET_ERROR);

    auto future_goal_handle = action_client->async_send_goal(goal, send_goal_ops);
    dual_spin_until_future_complete(future_goal_handle);
    auto goal_handle = future_goal_handle.get();
    auto future_result = action_client->async_get_result(goal_handle);
    EXPECT_THROW(
      dual_spin_until_future_complete(future_result),
      rclcpp::exceptions::RCLError);
  }
  {
    ActionGoal goal;
    goal.order = 5;
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp_action", rcl_action_take_goal_response, RCL_RET_ERROR);

    auto future_goal_handle = action_client->async_send_goal(goal, send_goal_ops);
    EXPECT_THROW(
      dual_spin_until_future_complete(future_goal_handle),
      rclcpp::exceptions::RCLError);
  }
  {
    ActionGoal goal;
    goal.order = 5;
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp_action", rcl_action_take_result_response, RCL_RET_ERROR);

    auto future_goal_handle = action_client->async_send_goal(goal, send_goal_ops);
    dual_spin_until_future_complete(future_goal_handle);
    auto goal_handle = future_goal_handle.get();
    auto future_result = action_client->async_get_result(goal_handle);
    EXPECT_THROW(
      dual_spin_until_future_complete(future_result),
      rclcpp::exceptions::RCLError);
  }
  {
    ActionGoal goal;
    goal.order = 5;
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp_action", rcl_action_take_cancel_response, RCL_RET_ERROR);

    auto future_goal_handle = action_client->async_send_goal(goal, send_goal_ops);
    dual_spin_until_future_complete(future_goal_handle);
    auto goal_handle = future_goal_handle.get();
    auto future_cancel_some =
      action_client->async_cancel_goals_before(goal_handle->get_goal_stamp());
    EXPECT_THROW(
      dual_spin_until_future_complete(future_cancel_some),
      rclcpp::exceptions::RCLError);
  }
}
