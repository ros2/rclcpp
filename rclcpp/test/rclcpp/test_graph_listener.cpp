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
#include <memory>

#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/graph_listener.hpp"
#include "rclcpp/node_interfaces/node_graph.hpp"
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

    node_graph_ = node_->get_node_graph_interface();
    ASSERT_NE(nullptr, node_graph_);

    graph_listener_ =
      std::make_shared<rclcpp::graph_listener::GraphListener>(
      rclcpp::contexts::get_global_default_context());
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<rclcpp::Node> node() {return node_;}
  rclcpp::node_interfaces::NodeGraphInterface * node_graph() {return node_graph_.get();}
  std::shared_ptr<rclcpp::graph_listener::GraphListener> graph_listener() {return graph_listener_;}

private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_;
  std::shared_ptr<rclcpp::graph_listener::GraphListener> graph_listener_;
};

/* Run base class init/shutdown */
TEST_F(TestGraphListener, construction_and_destruction) {
  EXPECT_FALSE(graph_listener()->has_node(node_graph()));
  EXPECT_FALSE(graph_listener()->is_shutdown());
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

  RCLCPP_EXPECT_THROW_EQ(
  {
    auto graph_listener_error =
    std::make_shared<rclcpp::graph_listener::GraphListener>(get_global_default_context());
    graph_listener_error.reset();
  }, std::runtime_error("failed to create guard condition: error not set"));
}

// Required for mocking_utils below
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcutils_allocator_t, ==)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcutils_allocator_t, !=)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcutils_allocator_t, <)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcutils_allocator_t, >)

/* Errors that occur when initializing the graph_listener */
TEST_F(TestGraphListener, error_start_graph_listener) {
  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_wait_set_init, RCL_RET_ERROR);
    RCLCPP_EXPECT_THROW_EQ(
      graph_listener()->start_if_not_started(),
      std::runtime_error("failed to initialize wait set: error not set"));
  }
  {
    EXPECT_NO_THROW(graph_listener()->shutdown());
    RCLCPP_EXPECT_THROW_EQ(
      graph_listener()->start_if_not_started(),
      std::runtime_error(shutdown_error_str));
  }
}

class TestGraphListenerProtectedMethods : public rclcpp::graph_listener::GraphListener
{
public:
  explicit TestGraphListenerProtectedMethods(std::shared_ptr<rclcpp::Context> parent_context)
  : rclcpp::graph_listener::GraphListener{parent_context}
  {}

  void run_protected()
  {
    this->run();
  }

  void mock_init_wait_set()
  {
    this->init_wait_set();
  }

  void mock_cleanup_wait_set()
  {
    this->cleanup_wait_set();
  }
};

/* Errors running graph protected methods */
TEST_F(TestGraphListener, error_run_graph_listener_destroy_context) {
  auto context_to_destroy = std::make_shared<rclcpp::contexts::DefaultContext>();
  context_to_destroy->init(0, nullptr);
  auto graph_listener_error =
    std::make_shared<TestGraphListenerProtectedMethods>(context_to_destroy);
  context_to_destroy.reset();
  EXPECT_THROW(
    graph_listener_error->run_protected(),
    rclcpp::exceptions::RCLError);
}

TEST_F(TestGraphListener, error_run_graph_listener_mock_wait_set_clear) {
  auto global_context = rclcpp::contexts::get_global_default_context();
  auto graph_listener_test =
    std::make_shared<TestGraphListenerProtectedMethods>(global_context);
  graph_listener_test->mock_init_wait_set();
  auto mock_wait_set_clear = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_wait_set_clear, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    graph_listener_test->run_protected(),
    std::runtime_error("failed to clear wait set: error not set"));
}

TEST_F(TestGraphListener, error_run_graph_listener_mock_wait_set_add_guard_condition) {
  auto global_context = rclcpp::contexts::get_global_default_context();
  auto graph_listener_test =
    std::make_shared<TestGraphListenerProtectedMethods>(global_context);
  graph_listener_test->mock_init_wait_set();
  auto mock_wait_set_clear = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_wait_set_add_guard_condition, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    graph_listener_test->run_protected(),
    std::runtime_error("failed to add guard condition to wait set: error not set"));
}

TEST_F(TestGraphListener, error_run_graph_listener_mock_wait_error) {
  auto global_context = rclcpp::contexts::get_global_default_context();
  auto graph_listener_test =
    std::make_shared<TestGraphListenerProtectedMethods>(global_context);
  graph_listener_test->mock_init_wait_set();
  auto mock_wait_set_clear = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_wait, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    graph_listener_test->run_protected(),
    std::runtime_error("failed to wait on wait set: error not set"));
}

TEST_F(TestGraphListener, error_run_graph_listener_mock_wait_timeout) {
  auto global_context = rclcpp::contexts::get_global_default_context();
  auto graph_listener_test =
    std::make_shared<TestGraphListenerProtectedMethods>(global_context);
  graph_listener_test->mock_init_wait_set();
  auto mock_wait_set_clear = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_wait, RCL_RET_TIMEOUT);
  RCLCPP_EXPECT_THROW_EQ(
    graph_listener_test->run_protected(),
    std::runtime_error("rcl_wait unexpectedly timed out"));
}

/* Add/Remove node usage */
TEST_F(TestGraphListener, test_graph_listener_add_remove_node) {
  EXPECT_FALSE(graph_listener()->has_node(node_graph()));

  graph_listener()->add_node(node_graph());
  EXPECT_TRUE(graph_listener()->has_node(node_graph()));

  graph_listener()->remove_node(node_graph());
  EXPECT_FALSE(graph_listener()->has_node(node_graph()));
}

/* Add/Remove node error usage */
TEST_F(TestGraphListener, test_errors_graph_listener_add_remove_node) {
  // nullptrs tests
  EXPECT_FALSE(graph_listener()->has_node(nullptr));

  RCLCPP_EXPECT_THROW_EQ(
    graph_listener()->add_node(nullptr),
    std::invalid_argument("node is nullptr"));

  RCLCPP_EXPECT_THROW_EQ(
    graph_listener()->remove_node(nullptr),
    std::invalid_argument("node is nullptr"));

  // Already added
  graph_listener()->add_node(node_graph());
  EXPECT_TRUE(graph_listener()->has_node(node_graph()));
  RCLCPP_EXPECT_THROW_EQ(
    graph_listener()->add_node(node_graph()),
    std::runtime_error("node already added"));

  // Remove node not found
  graph_listener()->remove_node(node_graph());
  EXPECT_FALSE(graph_listener()->has_node(node_graph()));
  RCLCPP_EXPECT_THROW_EQ(
    graph_listener()->remove_node(node_graph()),
    std::runtime_error("node not found"));

  // Add and remove after shutdown
  EXPECT_NO_THROW(graph_listener()->shutdown());
  RCLCPP_EXPECT_THROW_EQ(
    graph_listener()->add_node(node_graph()),
    std::runtime_error(shutdown_error_str));
  // Remove works the same
  RCLCPP_EXPECT_THROW_EQ(
    graph_listener()->remove_node(node_graph()),
    std::runtime_error("node not found"));
}

/* Shutdown errors */
TEST_F(TestGraphListener, test_graph_listener_shutdown_wait_fini_error_nothrow) {
  auto global_context = rclcpp::contexts::get_global_default_context();
  auto graph_listener_test =
    std::make_shared<TestGraphListenerProtectedMethods>(global_context);

  graph_listener_test->start_if_not_started();

  {
    auto mock_wait_set_fini = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_wait_set_fini, RCL_RET_ERROR);
    // Exception is logged when using nothrow_t
    EXPECT_NO_THROW(graph_listener_test->shutdown(std::nothrow_t()));
  }

  graph_listener_test->mock_cleanup_wait_set();
}

TEST_F(TestGraphListener, test_graph_listener_shutdown_wait_fini_error_throw) {
  auto global_context = rclcpp::contexts::get_global_default_context();
  auto graph_listener_test =
    std::make_shared<TestGraphListenerProtectedMethods>(global_context);

  graph_listener()->start_if_not_started();

  {
    auto mock_wait_set_fini = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_wait_set_fini, RCL_RET_ERROR);

    RCLCPP_EXPECT_THROW_EQ(
      graph_listener()->shutdown(),
      std::runtime_error("failed to finalize wait set: error not set"));
  }

  graph_listener_test->mock_cleanup_wait_set();
}

TEST_F(TestGraphListener, test_graph_listener_shutdown_guard_fini_error_throw) {
  auto global_context = rclcpp::contexts::get_global_default_context();
  auto graph_listener_test =
    std::make_shared<TestGraphListenerProtectedMethods>(global_context);

  graph_listener_test->start_if_not_started();

  auto mock_wait_set_fini = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_guard_condition_fini, RCL_RET_ERROR);

  EXPECT_NO_THROW(graph_listener_test->shutdown());

  graph_listener_test->mock_cleanup_wait_set();
}
