// Copyright 2024 Open Source Robotics Foundation, Inc.
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
// limitations under the License.#include <gtest/gtest.h>

#include <gtest/gtest.h>
#include "test_actions.hpp"

struct actions_test_data_t
{
    bool use_events_executor;
    bool use_server_ipc;
    bool use_client_ipc;
};

class ActionsTest
: public testing::Test, public testing::WithParamInterface<actions_test_data_t>
{
public:
    void SetUp() override
    {
        test_info = std::make_shared<TestInfo>();
        rclcpp::init(0, nullptr);
        auto p = GetParam();
        std::cout << "Test permutation: "
                  << (p.use_events_executor ? "{ EventsExecutor, " : "{ SingleThreadedExecutor, ")
                  << (p.use_server_ipc ? "IPC Server, " : "Non-IPC Server, ")
                  << (p.use_client_ipc ? "IPC Client }" : "Non-IPC Client }") << std::endl;

        executor = test_info->create_executor(p.use_events_executor);
        executor_thread = std::thread([&]() {
          executor->spin();
        });
        client_node = test_info->create_node("client_node", p.use_client_ipc);
        server_node = test_info->create_node("server_node", p.use_server_ipc);
        action_client = test_info->create_action_client(client_node);
        action_server = test_info->create_action_server(server_node);
        send_goal_options = test_info->create_goal_options();
        goal_msg = Fibonacci::Goal();
    }

    void TearDown() override
    {
        test_info.reset();
        executor->cancel();
        if (executor_thread.joinable()) {
            executor_thread.join();
        }
        rclcpp::shutdown();
    }

    rclcpp::Executor::UniquePtr executor;
    std::thread executor_thread;
    rclcpp::Node::SharedPtr client_node;
    rclcpp::Node::SharedPtr server_node;
    rclcpp_action::Client<Fibonacci>::SharedPtr action_client;
    rclcpp_action::Server<Fibonacci>::SharedPtr action_server;
    rclcpp_action::Client<Fibonacci>::SendGoalOptions send_goal_options;
    Fibonacci::Goal goal_msg;
    std::shared_ptr<TestInfo> test_info;
};

INSTANTIATE_TEST_SUITE_P(
    ActionsTest,
    ActionsTest,
    testing::Values(
    /*  <UseEventsExecutor> <ServerIsIntraProcess> <ClientIsIntraProcess>  */
        actions_test_data_t{ false, false, false },
        actions_test_data_t{ false, false, true  },
        actions_test_data_t{ false, true,  false },
        actions_test_data_t{ false, true,  true  },
        actions_test_data_t{ true,  false, false },
        actions_test_data_t{ true,  false, true  },
        actions_test_data_t{ true,  true,  false },
        actions_test_data_t{ true,  true,  true  }
    ));

TEST_P(ActionsTest, SucceedGoal)
{
    executor->add_node(server_node);
    executor->add_node(client_node);

    bool server_available = action_client->wait_for_action_server(std::chrono::seconds(1));
    ASSERT_TRUE(server_available);

    auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);
    auto accepted_response_wait = goal_handle_future.wait_for(std::chrono::seconds(5));
    ASSERT_TRUE(accepted_response_wait == std::future_status::ready) << "Goal was rejected by server";

    auto goal_handle = goal_handle_future.get();
    ASSERT_TRUE(goal_handle != nullptr) << "Invalid goal";

    auto result_future = action_client->async_get_result(goal_handle);

    test_info->succeed_goal();

    auto result_wait = result_future.wait_for(std::chrono::seconds(5));
    ASSERT_TRUE(result_wait == std::future_status::ready) << "Goal not completed on time";

    auto wrapped_result = result_future.get();
    EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_TRUE(test_info->result_is_correct(
        wrapped_result.result->sequence, rclcpp_action::ResultCode::SUCCEEDED));
}

TEST_P(ActionsTest, CancelGoal)
{
    executor->add_node(server_node);
    executor->add_node(client_node);
    send_goal_options.result_callback = nullptr;

    bool server_available = action_client->wait_for_action_server(std::chrono::seconds(1));
    ASSERT_TRUE(server_available);

    auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);
    auto accepted_response_wait = goal_handle_future.wait_for(std::chrono::seconds(5));
    ASSERT_TRUE(accepted_response_wait == std::future_status::ready) << "Goal was rejected by server";
    auto goal_handle = goal_handle_future.get();
    ASSERT_TRUE(goal_handle != nullptr) << "Invalid goal";

    auto cancel_result_future = action_client->async_cancel_goal(goal_handle);
    auto cancel_response_wait = cancel_result_future.wait_for(std::chrono::seconds(5));
    ASSERT_TRUE(cancel_response_wait == std::future_status::ready) << "Cancel response not on time";
    auto cancel_result = cancel_result_future.get();
    ASSERT_TRUE(cancel_result != nullptr) << "Invalid cancel result";
    EXPECT_NE(cancel_result->return_code, action_msgs::srv::CancelGoal::Response::ERROR_REJECTED);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    auto result_future = action_client->async_get_result(goal_handle);
    auto result_response_wait = result_future.wait_for(std::chrono::seconds(5));
    ASSERT_TRUE(result_response_wait == std::future_status::ready) << "Cancel result response not on time";
    auto wrapped_result = result_future.get();
    EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::CANCELED);
    EXPECT_TRUE(test_info->result_is_correct(
        wrapped_result.result->sequence, rclcpp_action::ResultCode::CANCELED));
}

TEST_P(ActionsTest, AbortGoal)
{
    executor->add_node(server_node);
    executor->add_node(client_node);

    bool server_available = action_client->wait_for_action_server(std::chrono::seconds(1));
    ASSERT_TRUE(server_available);

    auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);
    auto accepted_response_wait = goal_handle_future.wait_for(std::chrono::seconds(5));
    ASSERT_TRUE(accepted_response_wait == std::future_status::ready) << "Goal was rejected by server";
    auto goal_handle = goal_handle_future.get();
    ASSERT_TRUE(goal_handle != nullptr) << "Invalid goal";
    auto result_future = action_client->async_get_result(goal_handle);

    test_info->abort_goal();

    auto result_wait = result_future.wait_for(std::chrono::seconds(5));
    ASSERT_TRUE(result_wait == std::future_status::ready) << "Abort response not arrived";
    auto wrapped_result = result_future.get();
    EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::ABORTED);
    EXPECT_TRUE(test_info->result_is_correct(
        wrapped_result.result->sequence, rclcpp_action::ResultCode::ABORTED));
}

TEST_P(ActionsTest, TestReject)
{
    executor->add_node(server_node);
    executor->add_node(client_node);

    bool server_available = action_client->wait_for_action_server(std::chrono::seconds(1));
    ASSERT_TRUE(server_available);

    goal_msg.order = 21; // Goals over 20 rejected

    auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);
    auto result_wait = goal_handle_future.wait_for(std::chrono::seconds(5));
    ASSERT_TRUE(result_wait == std::future_status::ready) << "Response not arrived";
    auto goal_handle = goal_handle_future.get();

    ASSERT_TRUE(goal_handle == nullptr);
}

TEST_P(ActionsTest, TestOnReadyCallback)
{
    bool server_available = action_client->wait_for_action_server(std::chrono::seconds(1));
    ASSERT_TRUE(server_available);

    auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);

    // Add node: set "on_ready" callback and process the "unread" event
    executor->add_node(server_node);

    // Give time to server to be executed and generate event into client
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // Add node: set "on_ready" callback and process the "unread" event
    executor->add_node(client_node);
    auto result_wait = goal_handle_future.wait_for(std::chrono::seconds(5));
    ASSERT_TRUE(result_wait == std::future_status::ready) << "Response not arrived";

    auto goal_handle = goal_handle_future.get();
    ASSERT_TRUE(goal_handle != nullptr) << "Invalid goal";

    auto result_future = action_client->async_get_result(goal_handle);
    test_info->succeed_goal();

    auto succeed_wait = result_future.wait_for(std::chrono::seconds(5));
    ASSERT_TRUE(succeed_wait == std::future_status::ready) << "Response not arrived";

    auto wrapped_result = result_future.get();
    EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_TRUE(test_info->result_is_correct(
        wrapped_result.result->sequence, rclcpp_action::ResultCode::SUCCEEDED));
}

TEST_P(ActionsTest, InstantSuccess)
{
    executor->add_node(server_node);
    executor->add_node(client_node);

    // Unset result callback, we want to test having the result before
    // having a callback set
    send_goal_options.result_callback = nullptr;

    bool server_available = action_client->wait_for_action_server(std::chrono::seconds(1));
    ASSERT_TRUE(server_available);

    auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);
    auto result_wait = goal_handle_future.wait_for(std::chrono::seconds(5));
    ASSERT_TRUE(result_wait == std::future_status::ready) << "Response not arrived";

    auto goal_handle = goal_handle_future.get();
    ASSERT_TRUE(goal_handle != nullptr) << "Invalid goal";

    test_info->succeed_goal();

    auto result_future = action_client->async_get_result(goal_handle);
    auto succeed_wait = result_future.wait_for(std::chrono::seconds(5));
    ASSERT_TRUE(succeed_wait == std::future_status::ready) << "Response not arrived";

    auto wrapped_result = result_future.get();
    EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_TRUE(test_info->result_is_correct(
        wrapped_result.result->sequence, rclcpp_action::ResultCode::SUCCEEDED));
}

// See https://github.com/ros2/rclcpp/issues/2451#issuecomment-1999749919
TEST_P(ActionsTest, FeedbackRace)
{
    executor->add_node(server_node);

    auto test_params = GetParam();
    auto client_executor = test_info->create_executor(test_params.use_events_executor);
    client_executor->add_node(client_node);

    bool server_available = action_client->wait_for_action_server(std::chrono::seconds(1));
    ASSERT_TRUE(server_available);

    auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);

    rclcpp::Rate spin_rate(double(test_info->server_rate_hz) * 0.4);  // A bit slower than the server's feedback rate

    for (size_t i = 0; i < 10 && !test_info->result_callback_called(); i++) {
      spin_rate.sleep();
      client_executor->spin_some();
      if (i == 5) {
        test_info->succeed_goal();
      }
    }

    EXPECT_TRUE(test_info->result_callback_called());
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
