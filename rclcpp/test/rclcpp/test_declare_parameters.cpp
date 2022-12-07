// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rcpputils/filesystem_helper.hpp"
#include "rcpputils/scope_exit.hpp"

#include "rmw/validate_namespace.h"

#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/msg/empty.hpp"
#include "test_msgs/srv/empty.hpp"

#include "../mocking_utils/patch.hpp"

using namespace std::chrono_literals;

class TestNode : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp()
  {
    server_node = rclcpp::Node::make_shared("test_declare_parameters_server");
    client_node = rclcpp::Node::make_shared("test_declare_parameters_client");
  }

  void TearDown()
  {
    server_node.reset();
    client_node.reset();
  }

  rclcpp::Node::SharedPtr server_node;
  rclcpp::Node::SharedPtr client_node;
};

TEST_F(TestNode, DeclareParameters)
{
  auto param_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(server_node);
  rclcpp::executors::SingleThreadedExecutor exec;
  int result = 0;

  auto cb = [this, &exec, &result](const rcl_interfaces::msg::ParameterEvent & event) {
      if (event.node == "/test_declare_parameters_client") {
        auto params = rclcpp::ParameterEventHandler::get_parameters_from_event(event);
        if (params.size() == 3) {
          for (auto & p : params) {
            if (p.get_name() == "namespace1.parameter_a") {
              result++;
            }
            if (p.get_name() == "namespace1.parameter_b") {
              result++;
            }
            if (p.get_name() == "namespace1.parameter_c") {
              result++;
            }
          }
          exec.cancel();
        }
      }
    };
  auto handle = param_subscriber->add_parameter_event_callback(cb);
  exec.add_node(server_node->get_node_base_interface());
  std::thread thr([&exec] {exec.spin();});
  std::this_thread::sleep_for(1s);
  auto values = client_node->declare_parameters(
    "namespace1", {
    {"parameter_a", 42},
    {"parameter_b", "test_string"},
    {"parameter_c", true},
  });
  thr.join();
  EXPECT_EQ(result, 3);
}

TEST_F(TestNode, DeclareParametersTemplate)
{
  auto param_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(server_node);
  rclcpp::executors::SingleThreadedExecutor exec;
  int result = 0;

  auto cb = [this, &exec, &result](const rcl_interfaces::msg::ParameterEvent & event) {
      if (event.node == "/test_declare_parameters_client") {
        auto params = rclcpp::ParameterEventHandler::get_parameters_from_event(event);
        if (params.size() == 3) {
          for (auto & p : params) {
            if (p.get_name() == "namespace1.parameter_a") {
              result++;
            }
            if (p.get_name() == "namespace1.parameter_b") {
              result++;
            }
            if (p.get_name() == "namespace1.parameter_c") {
              result++;
            }
          }
          exec.cancel();
        }
      }
    };
  auto handle = param_subscriber->add_parameter_event_callback(cb);
  exec.add_node(server_node->get_node_base_interface());
  std::thread thr([&exec] {exec.spin();});
  std::this_thread::sleep_for(1s);
  auto values = client_node->declare_parameters<int64_t>(
    "namespace1", {
    {"parameter_a", 42},
    {"parameter_b", 256},
    {"parameter_c", 128},
  });
  thr.join();
  EXPECT_EQ(result, 3);
}

TEST_F(TestNode, DeclareParametersException)
{
  EXPECT_THROW(
  {
    client_node->declare_parameters(
      "namespace1", {
      {"parameter_a", 42},
      {"parameter_b", "test_string"},
      {"", true},
    });
  }, rclcpp::exceptions::InvalidParametersException);
  EXPECT_FALSE(client_node->has_parameter("namespace1.parameter_a"));
  EXPECT_FALSE(client_node->has_parameter("namespace1.parameter_b"));
}

TEST_F(TestNode, DeclareParametersTemplateException)
{
  EXPECT_THROW(
  {
    auto values = client_node->declare_parameters<int64_t>(
      "namespace1", {
      {"parameter_a", 42},
      {"parameter_b", 256},
      {"", 100},
    });
  }, rclcpp::exceptions::InvalidParametersException);
  EXPECT_FALSE(client_node->has_parameter("namespace1.parameter_a"));
  EXPECT_FALSE(client_node->has_parameter("namespace1.parameter_b"));
}
