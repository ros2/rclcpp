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

#include <rclcpp/clock.hpp>
#include <rclcpp/exceptions.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/time.hpp>

#include <test_msgs/action/fibonacci.hpp>

#include <map>
#include <memory>
#include <string>

#include "rclcpp_action/exceptions.hpp"
#include "rclcpp_action/create_client.hpp"
#include "rclcpp_action/client.hpp"
#include "rclcpp_action/types.hpp"

class TestActionClient : public ::testing::Test
{
protected:
  using ActionType = test_msgs::action::Fibonacci;
  using ActionGoal = ActionType::Goal;
  using ActionGoalHandle = rclcpp_action::ClientGoalHandle<ActionType>;
  using ActionGoalRequestService = ActionType::GoalRequestService;
  using ActionGoalRequest = ActionGoalRequestService::Request;
  using ActionGoalResponse = ActionGoalRequestService::Response;
  using ActionGoalResultService = ActionType::GoalRequestService;
  using ActionGoalResultRequest = ActionGoalResultService::Request;
  using ActionGoalResultResponse = ActionGoalResultService::Response;
  using ActionCancelGoalService = ActionType::CancelGoalService;
  using ActionCancelGoalRequest = ActionType::CancelGoalService::Request;
  using ActionCancelGoalResponse = ActionType::CancelGoalService::Response;
  using ActionStatusMessage = ActionType::GoalStatusMessage;
  using ActionFeedbackMessage = ActionType::Feedback;
  using ActionFeedback = ActionType::Feedback;

  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>(node_name, namespace_name);
    rcl_allocator_t allocator = rcl_get_default_allocator();

    char * goal_service_name = nullptr;
    rcl_ret_t ret = rcl_action_get_goal_service_name(
      action_name, allocator, &goal_service_name);
    ASSERT_EQ(RCL_RET_OK, ret);
    goal_service = node->create_service<ActionGoalRequestService>(
      goal_service_name,
      [this] (
        const ActionGoalRequest::SharedPtr request,
        const ActionGoalResponse::SharedPtr response)
      {
        response->stamp = clock.now();
        response->accepted = (request->goal.order >= 0);
        if (response->accepted) {
          goals[request->uuid] = {request, response};
        }
      });
    ASSERT_FALSE(goal_service != nullptr);
    allocator.deallocate(goal_service_name, allocator.state);

    char * result_service_name = nullptr;
    ret = rcl_action_get_result_service_name(
      action_name, allocator, &result_service_name);
    ASSERT_EQ(RCL_RET_OK, ret);
    result_service = node->create_service<ActionGoalResultService>(
      result_service_name,
      [this](
        const ActionGoalResultRequest::SharedPtr request,
        const ActionGoalResultResponse::SharedPtr response)
      {
        if (goals.count(request->uuid) == 1) {
          auto goal_request = goals[request->uuid].first;
          auto goal_response = goals[request->uuid].second;
          ActionStatusMessage status_message;
          rclcpp_action::GoalStatus goal_status;
          goal_status.goal_info.uuid = goal_request->uuid;
          goal_status.goal_info.stamp = goal_response->stamp;
          goal_status.status = rclcpp_action::GoalStatus::STATUS_EXECUTING;
          status_message.status_list.push_back(goal_status);
          status_publisher->publish(status_message):
          ActionFeedbackMessage feedback_message;
          feedback_message->uuid = goal_request->uuid;
          response->sequence.push_back(0);
          feedback_message->sequence = response->sequence;
          feedback_publisher->publish(feedback_message);
          if (request->order > 0)
          {
            response->sequence.push_back(1);
            feedback_message->sequence = response->sequence;
            feedback_publisher->publish(feedback_message);
            for (int i = 1; i < goal_request->order; ++i)
            {
              response->sequence.push_back(response->sequence[i] + response->sequence[i - 1]);
              feedback_message->sequence = response->sequence;
              feedback_publisher->publish(feedback_message);
            }
          }
          response->status = rclcpp_action::GoalStatus::STATUS_SUCCEEDED;
          status_message.status_list.back().status = rclcpp_action::GoalStatus::STATUS_SUCCEEDED;
          status_publisher->publish(status_message):
          goals.erase(request->uuid);
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
    cancel_service = node->create_service<ActionCancelService>(
        cancel_service_name,
        [this] (
          const ActionCancelRequest::SharedPtr request,
          const ActionCancelResponse::SharedPtr response) mutable
        {
          ActionStatusMessage status_message;
          bool cancel_all = (
            request->goal_info.stamp == zero_stamp &&
            request->goal_info.uuid == zero_uuid);
          auto it = goals.begin();
          while (it != goals.end()) {
            auto goal_request = it->second.first;
            auto goal_response = it->second.second;
            bool cancel_this = (
              request->goal_info.uuid == goal_request->uuid ||
              request->goal_info.stamp > goal_response->stamp);
            if (cancel_all || cancel_this)
            {
              rclcpp_action::GoalStatus goal_status;
              goal_status.goal_info.uuid = goal_request->uuid;
              goal_status.goal_info.stamp = goal_response->stamp;
              goal_status.status = rclcpp_action::GoalStatus::STATUS_CANCELED;
              status_message.status_list.push_back(goal_status);
              response->goals_canceling.push_back(goal_status.goal_info);
              it = goals.erase(it);
            }
          }
          status_publisher->publish(status_message);
        });
    ASSERT_TRUE(cancel_service != nullptr);
    allocator.deallocate(cancel_service_name, allocator.state);

    char * feedback_topic_name = nullptr;
    ret = rcl_action_get_feedback_topic_name(
      action_name, allocator, &feedback_topic_name);
    ASSERT_EQ(RCL_RET_OK, ret);
    feedback_publisher = node->create_publisher<ActionFeedbackMessage>(feedback_topic_name);
    ASSERT_TRUE(feedback_publisher != nullptr);
    allocator.deallocate(feedback_topic_name, allocator.state);

    char * status_topic_name = nullptr;
    ret = rcl_action_get_status_topic_name(
      action_name, allocator, &status_topic_name);
    ASSERT_EQ(RCL_RET_OK, ret);
    status_publisher = node->create_publisher<ActionStatusMessage>(status_topic_name);
    ASSERT_TRUE(status_publisher != nullptr);
    allocator.deallocate(status_topic_name, allocator.state);

    ASSERT_EQ(RCL_RET_OK, rcl_enable_ros_time_override(clock.get_clock_handle()));
    ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(clock.get_clock_handle(), RCL_S_TO_NS(1)));
  }

  void Teardown()
  {
    status_publisher.reset();
    feedback_publisher.reset();
    cancel_service.reset();
    result_service.reset();
    goal_servicer.reset();
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::Clock clock{RCL_ROS_TIME};
  const char * const node_name{"fibonacci_action_test_node"};
  const char * const namespace_name{"/rclcpp_action/test/client"};
  const char * const action_name{"fibonacci_test"};
  typename rclcpp::Service<ActionGoalRequestService>::SharedPtr goal_service;
  typename rclcpp::Service<ActionGoalResultService>::SharedPtr result_service;
  typename rclcpp::Service<ActionCancelGoalService>::SharedPtr cancel_service;
  typename rclcpp::Publisher<ActionFeedbackMessage>::SharedPtr feedback_publisher;
  typename rclcpp::Publisher<ActionStatusMessage>::SharedPtr status_publisher;
  std::map<rclcpp_action::GoalID, typename ActionGoalRequest::SharedPtr> goals;
};

TEST_F(TestClient, construction_and_destruction)
{
  ASSERT_NO_THROW(rclcpp_action::create_client<ActionType>(node, action_name).reset());
}

TEST_F(TestClient, async_send_goal_but_ignore_feedback_and_result)
{
  auto action_client = rclcpp_action::create_client<ActionType>(node, action_name);

  ActionGoal bad_goal;
  bad_goal.order = -5;
  auto future_goal_handle = action_client->async_send_goal(bad_goal, nullptr, true);
  ASSERT_EQ(
    rclcpp::executor::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, future));
  EXPECT_THROW(future.get(), rclcpp_action::exceptions::RejectedGoalError);

  ActionGoal good_goal;
  good_goal.order = 5;
  future_goal_handle = action_client->async_send_goal(good_goal, nullptr, true);
  ASSERT_EQ(
    rclcpp::executor::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, future));
  auto goal_handle = future.get();
  EXPECT_EQ(GoalStatus::ACCEPTED, goal_handle->get_status());
  EXPECT_FALSE(goal_handle->is_feedback_aware());
  EXPECT_FALSE(goal_handle->is_result_aware());
  EXPECT_THROW(goal_handle->async_result(), rclcpp_action::exceptions::UnawareGoalHandleError);
}

TEST_F(TestClient, async_send_goal_and_ignore_feedback_but_wait_for_result)
{
  auto action_client = rclcpp_action::create_client<ActionType>(node, action_name);

  ActionGoal goal;
  goal.order = 5;
  auto future_goal_handle = action_client->async_send_goal(goal);
  ASSERT_EQ(
    rclcpp::executor::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, future_goal_handle));
  auto goal_handle = future_goal_handle.get();
  EXPECT_EQ(GoalStatus::ACCEPTED, goal_handle->get_status());
  EXPECT_FALSE(goal_handle->is_feedback_aware());
  EXPECT_TRUE(goal_handle->is_result_aware());

  auto future_result = goal_handle->async_result();
  ASSERT_EQ(
    rclcpp::executor::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, future_result));
  auto result = future_result.get();
  ASSERT_EQ(6, result->response.sequence.size());
  EXPECT_EQ(0, result->response.sequence[0]);
  EXPECT_EQ(1, result->response.sequence[1]);
  EXPECT_EQ(5, result->response.sequence[5]);
}

TEST_F(TestClient, async_send_goal_with_feedback_and_result)
{
  auto action_client = rclcpp_action::create_client<ActionType>(node, action_name);

  ActionGoal goal;
  goal.order = 4;
  int feedback_count = 0;
  auto future_goal_handle = action_client->async_send_goal(
    goal,
    [&feedback_count] (
      typename ActionGoalHandle::SharedPtr goal_handle,
      const ActionFeedback & feedback) mutable
    {
      feedback_count++;
    });

  ASSERT_EQ(
    rclcpp::executor::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, future_goal_handle));
  auto goal_handle = future_goal_handle.get();
  EXPECT_EQ(GoalStatus::ACCEPTED, goal_handle->get_status());
  EXPECT_TRUE(goal_handle->is_feedback_aware());
  EXPECT_TRUE(goal_handle->is_result_aware());

  auto future_result = goal_handle->async_result();
  ASSERT_EQ(
    rclcpp::executor::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, future_result));
  auto result = future_result.get();
  ASSERT_EQ(5, result->response.sequence.size());
  EXPECT_EQ(3, result->response.sequence.back());
  EXPECT_EQ(5, feedback_count);
}

TEST_F(TestClient, async_cancel_one_goal)
{
  auto action_client = rclcpp_action::create_client<ActionType>(node, action_name);

  ActionGoal goal;
  goal.order = 5;
  auto future_goal_handle = action_client->async_send_goal(goal, nullptr, true);
  ASSERT_EQ(
    rclcpp::executor::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, future_goal_handle));
  auto goal_handle = future_goal_handle.get();
  EXPECT_EQ(rclcpp_action::GoalStatus::ACCEPTED, goal_handle->get_status());

  auto future_cancel = action_client->async_cancel_goal(goal_handle);
  ASSERT_EQ(
    rclcpp::executor::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, future_cancel_all));
  auto cancel_response = future_cancel.get();
  ASSERT_EQ(1, cancel_response->goals_canceling.size());
  const rclcpp_action::GoalInfo & canceled_goal_info = cancel_response->goals_canceling[0];
  EXPECT_EQ(canceled_goal_info.uuid, goal_handle->get_goal_id());
  EXPECT_EQ(rclcpp_action::GoalStatus::STATUS_CANCELED, goal_handle->get_status());
}

TEST_F(TestClient, async_cancel_all_goals)
{
  auto action_client = rclcpp_action::create_client<ActionType>(node, action_name);

  ActionGoal goal;
  goal.order = 6;
  auto future_goal_handle0 = action_client->async_send_goal(goal, nullptr, true);
  ASSERT_EQ(
    rclcpp::executor::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, future_goal_handle0));
  auto goal_handle0 = future_goal_handle0.get();

  goal.order = 8;
  auto future_goal_handle1 = action_client->async_send_goal(goal, nullptr, true);
  ASSERT_EQ(
    rclcpp::executor::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, future_goal_handle1));
  auto goal_handle1 = future_goal_handle1.get();

  auto future_cancel_all = action_client->async_cancel_all_goals();
  ASSERT_EQ(
    rclcpp::executor::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, future_cancel_all));
  auto cancel_response = future_cancel_all.get();

  ASSERT_EQ(2, cancel_response->goals_canceling.size());
  EXPECT_EQ(goal_handle0->get_goal_id(), cancel_response->goals_canceling[0].uuid);
  EXPECT_EQ(goal_handle1->get_goal_id(), cancel_response->goals_canceling[1].uuid);
}

TEST_F(TestClient, async_cancel_some_goals)
{
  auto action_client = rclcpp_action::create_client<ActionType>(node, action_name);

  ActionGoal goal;
  goal.order = 6;
  auto future_goal_handle0 = action_client->async_send_goal(goal, nullptr, true);
  ASSERT_EQ(
    rclcpp::executor::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, future_goal_handle0));
  auto goal_handle0 = future_goal_handle0.get();

  // Move time forward
  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(clock.get_clock_handle(), RCL_S_TO_NS(2)));

  goal.order = 8;
  auto future_goal_handle1 = action_client->async_send_goal(goal, nullptr, true);
  ASSERT_EQ(
    rclcpp::executor::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, future_goal_handle1));
  auto goal_handle1 = future_goal_handle1.get();

  auto future_cancel_some = action_client->async_cancel_goals_before(goal_handle1->get_goal_stamp());
  ASSERT_EQ(
    rclcpp::executor::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, future_cancel_some));
  auto cancel_response = future_cancel_some.get();

  ASSERT_EQ(1, cancel_response->goals_canceling.size());
  EXPECT_EQ(goal_handle0->get_goal_id(), cancel_response->goals_canceling[0].uuid);
}
