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

#include <gtest/gtest.h>

#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/graph_listener.hpp"
#include "rclcpp/rclcpp.hpp"

#include "../mocking_utils/patch.hpp"
#include "../utils/rclcpp_gtest_macros.hpp"

namespace
{

constexpr char node_name[] = "node";
constexpr char node_namespace[] = "ns";
constexpr char shutdown_error_str[] = "GraphListener already shutdown";

}  // namespace

class TestGraphListener : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>(node_name, node_namespace);

    using rclcpp::contexts::get_global_default_context;
    graph_listener_ =
      std::make_shared<rclcpp::graph_listener::GraphListener>(get_global_default_context());
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<rclcpp::Node> node() {return node_;}
  std::shared_ptr<rclcpp::graph_listener::GraphListener> graph_listener() {return graph_listener_;}

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::graph_listener::GraphListener> graph_listener_;
};

/* Run base class init/shutdown */
TEST_F(TestGraphListener, construction_and_destruction) {
  void();
}

// Required for mocking_utils below
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcl_guard_condition_options_t, ==)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcl_guard_condition_options_t, !=)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcl_guard_condition_options_t, <)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcl_guard_condition_options_t, >)

/* Error creating a new graph listener */
TEST_F(TestGraphListener, error_construct_graph_listener) {
  using rclcpp::contexts::get_global_default_context;
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_guard_condition_init, RCL_RET_ERROR);

  using rclcpp::contexts::get_global_default_context;
  RCLCPP_EXPECT_THROW_EQ({
  auto graph_listener_error =
    std::make_shared<rclcpp::graph_listener::GraphListener>(get_global_default_context());
    }, std::runtime_error("failed to create interrupt guard condition: error not set"));
}

/* Errors that occur when initializing the graph_listener */
TEST_F(TestGraphListener, error_start_graph_listener) {
  {
    // Invalid incomplete type mock, skip
    /*
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_wait_set_init, RCL_RET_ERROR);
    RCLCPP_EXPECT_THROW_EQ({
      graph_listener()->start_if_not_started();
    }, std::runtime_error("failed to initialize wait set: error not set"));
    */
  }
  {
    auto context_to_destroy = std::make_shared<rclcpp::contexts::DefaultContext>();
    context_to_destroy->init(0, nullptr);
    auto graph_listener_error =
      std::make_shared<rclcpp::graph_listener::GraphListener>(context_to_destroy);
    context_to_destroy.reset();
    RCLCPP_EXPECT_THROW_EQ({
      graph_listener_error->start_if_not_started();
    }, std::runtime_error("parent context was destroyed"));
  }
  {
    EXPECT_NO_THROW(graph_listener()->shutdown());
    RCLCPP_EXPECT_THROW_EQ({
      graph_listener()->start_if_not_started();
      }, std::runtime_error(shutdown_error_str));
  }
}
