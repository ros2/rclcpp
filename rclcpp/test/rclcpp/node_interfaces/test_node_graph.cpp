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

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rcl/graph.h"
#include "rcl/node_options.h"
#include "rcl/remap.h"
#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_base.hpp"
#include "rclcpp/node_interfaces/node_graph.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/strdup.h"
#include "test_msgs/msg/empty.hpp"
#include "test_msgs/srv/empty.hpp"

#include "../../mocking_utils/patch.hpp"
#include "../../utils/rclcpp_gtest_macros.hpp"

namespace
{

constexpr char node_name[] = "node";
constexpr char node_namespace[] = "ns";
constexpr char absolute_namespace[] = "/ns";

}  // namespace

class TestNodeGraph : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>(node_name, node_namespace);

    // This dynamic cast is not necessary for the unittests, but instead is used to ensure
    // the proper type is being tested and covered.
    node_graph_ =
      dynamic_cast<rclcpp::node_interfaces::NodeGraph *>(node_->get_node_graph_interface().get());
    ASSERT_NE(nullptr, node_graph_);
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<rclcpp::Node> node() {return node_;}

  const rclcpp::node_interfaces::NodeGraph * node_graph() const {return node_graph_;}

  size_t get_num_graph_things(std::function<size_t()> predicate)
  {
    constexpr std::chrono::milliseconds timeout(100);

    size_t tries = 0;
    size_t num_things = 0;
    while (tries++ < 5) {
      num_things = predicate();
      if (num_things >= 1) {
        break;
      }

      auto event = node()->get_graph_event();
      EXPECT_NO_THROW(node()->wait_for_graph_change(event, timeout));
    }

    return num_things;
  }

  size_t get_num_topics()
  {
    return get_num_graph_things(
      [this]() -> size_t {
        auto topic_names_and_types = node_graph()->get_topic_names_and_types();
        return topic_names_and_types.size();
      });
  }

  size_t get_num_services()
  {
    return get_num_graph_things(
      [this]() -> size_t {
        auto service_names_and_types = node_graph()->get_service_names_and_types();
        return service_names_and_types.size();
      });
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::node_interfaces::NodeGraph * node_graph_;
};

TEST_F(TestNodeGraph, construct_from_node)
{
  EXPECT_LT(0u, get_num_topics());

  EXPECT_LT(0u, get_num_services());

  auto names = node_graph()->get_node_names();
  EXPECT_EQ(1u, names.size());

  auto names_and_namespaces = node_graph()->get_node_names_and_namespaces();
  EXPECT_EQ(1u, names_and_namespaces.size());

  auto names_namespaces_and_enclaves =
    node_graph()->get_node_names_with_enclaves();
  EXPECT_EQ(1u, names_namespaces_and_enclaves.size());

  EXPECT_EQ(0u, node_graph()->count_publishers("not_a_topic"));
  EXPECT_EQ(0u, node_graph()->count_subscribers("not_a_topic"));
  EXPECT_NE(nullptr, node_graph()->get_graph_guard_condition());

  // get_graph_event is non-const
  EXPECT_NE(nullptr, node()->get_node_graph_interface()->get_graph_event());
  EXPECT_LE(1u, node_graph()->count_graph_users());
}

TEST_F(TestNodeGraph, get_topic_names_and_types)
{
  ASSERT_LT(0u, get_num_topics());
}

TEST_F(TestNodeGraph, get_topic_names_and_types_rcl_error)
{
  auto mock_get_topic_names = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_get_topic_names_and_types, RCL_RET_ERROR);
  auto mock_names_fini = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_names_and_types_fini, RCL_RET_ERROR);

  RCLCPP_EXPECT_THROW_EQ(
    node_graph()->get_topic_names_and_types(),
    std::runtime_error(
      "failed to get topic names and types: error not set, failed also to cleanup topic names and"
      " types, leaking memory: error not set"));
}

TEST_F(TestNodeGraph, get_topic_names_and_types_rcl_names_and_types_fini_error)
{
  auto mock_names_fini = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_names_and_types_fini, RCL_RET_ERROR);

  RCLCPP_EXPECT_THROW_EQ(
    node_graph()->get_topic_names_and_types(),
    std::runtime_error("could not destroy topic names and types: error not set"));
}

TEST_F(TestNodeGraph, get_service_names_and_types)
{
  ASSERT_LT(0u, get_num_services());
}

TEST_F(TestNodeGraph, get_service_names_and_types_rcl_error)
{
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_get_service_names_and_types, RCL_RET_ERROR);
  auto mock_names_fini = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_names_and_types_fini, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    node_graph()->get_service_names_and_types(),
    std::runtime_error(
      "failed to get service names and types: error not set, failed also to cleanup service names"
      " and types, leaking memory: error not set"));
}

TEST_F(TestNodeGraph, get_service_names_and_types_rcl_names_and_types_fini)
{
  auto mock_names_fini = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_names_and_types_fini, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    node_graph()->get_service_names_and_types(),
    std::runtime_error("could not destroy service names and types: error not set"));
}

TEST_F(TestNodeGraph, get_service_names_and_types_by_node)
{
  auto callback = [](
    const test_msgs::srv::Empty::Request::SharedPtr,
    test_msgs::srv::Empty::Response::SharedPtr) {};
  auto service =
    node()->create_service<test_msgs::srv::Empty>("node1_service", std::move(callback));

  const std::string node2_name = "node2";
  auto node2 = std::make_shared<rclcpp::Node>(node2_name, node_namespace);

  // rcl_get_service_names_and_types_by_node() expects the node to exist, otherwise it fails
  EXPECT_THROW(
    node_graph()->get_service_names_and_types_by_node("not_a_node", "not_absolute_namespace"),
    std::runtime_error);

  // Check that node1_service exists for node1 but not node2. This shouldn't exercise graph
  // discovery as node_graph belongs to node1 anyway. This is just to test the API itself.
  auto services_of_node1 =
    node_graph()->get_service_names_and_types_by_node(node_name, absolute_namespace);
  auto services_of_node2 =
    node_graph()->get_service_names_and_types_by_node(node2_name, absolute_namespace);

  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(3)) {
    services_of_node1 =
      node_graph()->get_service_names_and_types_by_node(node_name, absolute_namespace);
    services_of_node2 =
      node_graph()->get_service_names_and_types_by_node(node2_name, absolute_namespace);
    if (services_of_node1.find("/ns/node1_service") != services_of_node1.end()) {
      break;
    }
  }

  EXPECT_TRUE(services_of_node1.find("/ns/node1_service") != services_of_node1.end());
  EXPECT_FALSE(services_of_node2.find("/ns/node1_service") != services_of_node2.end());
}

TEST_F(TestNodeGraph, get_client_names_and_types_by_node)
{
  auto client = node()->create_client<test_msgs::srv::Empty>("node1_service");

  const std::string node2_name = "node2";
  auto node2 = std::make_shared<rclcpp::Node>(node2_name, node_namespace);

  // rcl_get_client_names_and_types_by_node() expects the node to exist, otherwise it fails
  EXPECT_THROW(
    node_graph()->get_client_names_and_types_by_node("not_a_node", "not_absolute_namespace"),
    rclcpp::exceptions::RCLError);

  // Check that node1_service exists for node1 but not node2. This shouldn't exercise graph
  // discovery as node_graph belongs to node1 anyway. This is just to test the API itself.
  auto services_of_node1 =
    node_graph()->get_client_names_and_types_by_node(node_name, absolute_namespace);
  auto services_of_node2 =
    node_graph()->get_client_names_and_types_by_node(node2_name, absolute_namespace);

  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(3)) {
    services_of_node1 =
      node_graph()->get_client_names_and_types_by_node(node_name, absolute_namespace);
    services_of_node2 =
      node_graph()->get_client_names_and_types_by_node(node2_name, absolute_namespace);
    if (services_of_node1.find("/ns/node1_service") != services_of_node1.end()) {
      break;
    }
  }

  EXPECT_TRUE(services_of_node1.find("/ns/node1_service") != services_of_node1.end());
  EXPECT_FALSE(services_of_node2.find("/ns/node1_service") != services_of_node2.end());
}

TEST_F(TestNodeGraph, get_service_names_and_types_by_node_rcl_errors)
{
  auto callback = [](
    const test_msgs::srv::Empty::Request::SharedPtr,
    test_msgs::srv::Empty::Response::SharedPtr) {};
  auto service =
    node()->create_service<test_msgs::srv::Empty>("node1_service", std::move(callback));

  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_get_service_names_and_types_by_node, RCL_RET_ERROR);
  auto mock_names_fini = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_names_and_types_fini, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    node_graph()->get_service_names_and_types_by_node(node_name, node_namespace),
    std::runtime_error(
      "failed to get service names and types by node: error not set, failed also to cleanup"
      " service names and types, leaking memory: error not set"));
}


TEST_F(TestNodeGraph, get_client_names_and_types_by_node_rcl_errors)
{
  auto client = node()->create_client<test_msgs::srv::Empty>("node1_service");

  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_get_client_names_and_types_by_node, RCL_RET_ERROR);
  auto mock_names_fini = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_names_and_types_fini, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    node_graph()->get_client_names_and_types_by_node(node_name, node_namespace),
    std::runtime_error(
      "failed to get service names and types by node: error not set"));
}

TEST_F(TestNodeGraph, get_service_names_and_types_by_node_names_and_types_fini_error)
{
  auto callback = [](
    const test_msgs::srv::Empty::Request::SharedPtr,
    test_msgs::srv::Empty::Response::SharedPtr) {};
  auto service =
    node()->create_service<test_msgs::srv::Empty>("node1_service", std::move(callback));
  auto mock_names_fini = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_names_and_types_fini, RCL_RET_ERROR);

  EXPECT_THROW(
    node_graph()->get_service_names_and_types_by_node(node_name, absolute_namespace),
    rclcpp::exceptions::RCLError);
}

TEST_F(TestNodeGraph, get_client_names_and_types_by_node_names_and_types_fini_error)
{
  auto client = node()->create_client<test_msgs::srv::Empty>("node1_service");
  auto mock_names_fini = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_names_and_types_fini, RCL_RET_ERROR);

  EXPECT_NO_THROW(
    node_graph()->get_client_names_and_types_by_node(node_name, absolute_namespace));
}

TEST_F(TestNodeGraph, get_publisher_names_and_types_by_node)
{
  const rclcpp::QoS publisher_qos(1);
  auto publisher = node()->create_publisher<test_msgs::msg::Empty>("node1_topic", publisher_qos);

  const std::string node2_name = "node2";
  auto node2 = std::make_shared<rclcpp::Node>(node2_name, node_namespace);

  // rcl_get_publisher_names_and_types_by_node() expects the node to exist, otherwise it fails
  EXPECT_THROW(
    node_graph()->get_publisher_names_and_types_by_node("not_a_node", "not_absolute_namespace"),
    rclcpp::exceptions::RCLError);

  // Check that node1_topic exists for node1 but not node2. This shouldn't exercise graph
  // discovery as node_graph belongs to node1 anyway. This is just to test the API itself.
  auto topics_of_node1 =
    node_graph()->get_publisher_names_and_types_by_node(node_name, absolute_namespace);
  auto topics_of_node2 =
    node_graph()->get_publisher_names_and_types_by_node(node2_name, absolute_namespace);

  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(3)) {
    topics_of_node1 =
      node_graph()->get_publisher_names_and_types_by_node(node_name, absolute_namespace);
    topics_of_node2 =
      node_graph()->get_publisher_names_and_types_by_node(node2_name, absolute_namespace);
    if (topics_of_node1.find("/ns/node1_topic") != topics_of_node1.end()) {
      break;
    }
  }

  EXPECT_TRUE(topics_of_node1.find("/ns/node1_topic") != topics_of_node1.end());
  EXPECT_FALSE(topics_of_node2.find("/ns/node1_topic") != topics_of_node2.end());
}

TEST_F(TestNodeGraph, get_subscriber_names_and_types_by_node)
{
  const rclcpp::QoS subscriber_qos(10);
  auto callback = [](test_msgs::msg::Empty::ConstSharedPtr) {};
  auto subscription =
    node()->create_subscription<test_msgs::msg::Empty>(
    "node1_topic", subscriber_qos, std::move(callback));

  const std::string node2_name = "node2";
  auto node2 = std::make_shared<rclcpp::Node>(node2_name, node_namespace);

  // rcl_get_subscriber_names_and_types_by_node() expects the node to exist, otherwise it fails
  EXPECT_THROW(
    node_graph()->get_subscriber_names_and_types_by_node("not_a_node", "not_absolute_namespace"),
    rclcpp::exceptions::RCLError);

  // Check that node1_topic exists for node1 but not node2. This shouldn't exercise graph
  // discovery as node_graph belongs to node1 anyway. This is just to test the API itself.
  auto topics_of_node1 =
    node_graph()->get_subscriber_names_and_types_by_node(node_name, absolute_namespace);
  auto topics_of_node2 =
    node_graph()->get_subscriber_names_and_types_by_node(node2_name, absolute_namespace);

  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(3)) {
    topics_of_node1 =
      node_graph()->get_subscriber_names_and_types_by_node(node_name, absolute_namespace);
    topics_of_node2 =
      node_graph()->get_subscriber_names_and_types_by_node(node2_name, absolute_namespace);
    if (topics_of_node1.find("/ns/node1_topic") != topics_of_node1.end()) {
      break;
    }
  }

  EXPECT_TRUE(topics_of_node1.find("/ns/node1_topic") != topics_of_node1.end());
  EXPECT_FALSE(topics_of_node2.find("/ns/node1_topic") != topics_of_node2.end());
}

TEST_F(TestNodeGraph, get_publisher_names_and_types_by_node_rcl_errors)
{
  const rclcpp::QoS publisher_qos(1);
  auto publisher = node()->create_publisher<test_msgs::msg::Empty>("topic", publisher_qos);

  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_get_publisher_names_and_types_by_node, RCL_RET_ERROR);
  auto mock_names_fini = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_names_and_types_fini, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    node_graph()->get_publisher_names_and_types_by_node(node_name, node_namespace),
    std::runtime_error(
      "failed to get topic names and types by node: error not set"));
}


TEST_F(TestNodeGraph, get_subscriber_names_and_types_by_node_rcl_errors)
{
  const rclcpp::QoS subscriber_qos(10);
  auto callback = [](test_msgs::msg::Empty::ConstSharedPtr) {};
  auto subscription =
    node()->create_subscription<test_msgs::msg::Empty>(
    "topic", subscriber_qos, std::move(callback));

  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_get_subscriber_names_and_types_by_node, RCL_RET_ERROR);
  auto mock_names_fini = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_names_and_types_fini, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    node_graph()->get_subscriber_names_and_types_by_node(node_name, node_namespace),
    std::runtime_error(
      "failed to get topic names and types by node: error not set"));
}

TEST_F(TestNodeGraph, get_publisher_names_and_types_by_node_names_and_types_fini_error)
{
  const rclcpp::QoS publisher_qos(1);
  auto publisher = node()->create_publisher<test_msgs::msg::Empty>("topic", publisher_qos);

  auto mock_names_fini = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_names_and_types_fini, RCL_RET_ERROR);

  EXPECT_NO_THROW(
    node_graph()->get_publisher_names_and_types_by_node(node_name, absolute_namespace));
}

TEST_F(TestNodeGraph, get_subscriber_names_and_types_by_node_names_and_types_fini_error)
{
  const rclcpp::QoS subscriber_qos(10);
  auto callback = [](test_msgs::msg::Empty::ConstSharedPtr) {};
  auto subscription =
    node()->create_subscription<test_msgs::msg::Empty>(
    "topic", subscriber_qos, std::move(callback));

  auto mock_names_fini = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_names_and_types_fini, RCL_RET_ERROR);

  EXPECT_NO_THROW(
    node_graph()->get_subscriber_names_and_types_by_node(node_name, absolute_namespace));
}

TEST_F(TestNodeGraph, get_node_names_and_namespaces)
{
  auto names_and_namespaces = node_graph()->get_node_names_and_namespaces();
  EXPECT_EQ(1u, names_and_namespaces.size());
}

TEST_F(TestNodeGraph, get_node_names_with_enclaves)
{
  auto names_namespaces_and_enclaves =
    node_graph()->get_node_names_with_enclaves();
  EXPECT_EQ(1u, names_namespaces_and_enclaves.size());
}

TEST_F(TestNodeGraph, get_node_names_and_namespaces_rcl_errors)
{
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_get_node_names, RCL_RET_ERROR);
  auto mock_names_fini = mocking_utils::patch_and_return(
    "lib:rclcpp", rcutils_string_array_fini, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    node_graph()->get_node_names_and_namespaces(),
    std::runtime_error(
      "failed to get node names: error not set, failed also to cleanup node names, leaking memory:"
      " error not set, failed also to cleanup node namespaces, leaking memory: error not set"));
}

TEST_F(TestNodeGraph, get_node_names_with_enclaves_rcl_errors)
{
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_get_node_names_with_enclaves, RCL_RET_ERROR);
  auto mock_names_fini = mocking_utils::patch_and_return(
    "lib:rclcpp", rcutils_string_array_fini, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    node_graph()->get_node_names_with_enclaves(),
    std::runtime_error(
      "failed to get node names with enclaves: error not set, failed also to cleanup node names, "
      "leaking memory: error not set, failed also to cleanup node namespaces, leaking memory: "
      "error not set, failed also to cleanup node enclaves, leaking memory: error not set"));
}

TEST_F(TestNodeGraph, get_node_names_and_namespaces_fini_errors)
{
  auto mock_names_fini = mocking_utils::patch_and_return(
    "lib:rclcpp", rcutils_string_array_fini, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    node_graph()->get_node_names_and_namespaces(),
    std::runtime_error("could not destroy node names, could not destroy node namespaces"));
}

TEST_F(TestNodeGraph, get_node_names_with_enclaves_fini_errors)
{
  auto mock_names_fini = mocking_utils::patch_and_return(
    "lib:rclcpp", rcutils_string_array_fini, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    node_graph()->get_node_names_with_enclaves(),
    std::runtime_error(
      "failed to finalize array, could not destroy node names, leaking memory: error not set"
      ", could not destroy node namespaces, leaking memory: error not set"
      ", could not destroy node enclaves, leaking memory: error not set"));
}

TEST_F(TestNodeGraph, count_publishers_rcl_error)
{
  auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_count_publishers, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    node_graph()->count_publishers("topic"),
    std::runtime_error("could not count publishers: error not set"));
}

TEST_F(TestNodeGraph, count_subscribers_rcl_error)
{
  auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_count_subscribers, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    node_graph()->count_subscribers("topic"),
    std::runtime_error("could not count subscribers: error not set"));
}

TEST_F(TestNodeGraph, notify_shutdown)
{
  EXPECT_NO_THROW(node()->get_node_graph_interface()->notify_shutdown());
}

TEST_F(TestNodeGraph, wait_for_graph_change)
{
  auto node_graph_interface = node()->get_node_graph_interface();
  EXPECT_NO_THROW(node_graph_interface->notify_graph_change());
  EXPECT_THROW(
    node_graph_interface->wait_for_graph_change(nullptr, std::chrono::milliseconds(1)),
    rclcpp::exceptions::InvalidEventError);

  auto event = std::make_shared<rclcpp::Event>();
  EXPECT_THROW(
    node_graph_interface->wait_for_graph_change(event, std::chrono::milliseconds(0)),
    rclcpp::exceptions::EventNotRegisteredError);
}

TEST_F(TestNodeGraph, notify_graph_change_rcl_error)
{
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_trigger_guard_condition, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    node()->get_node_graph_interface()->notify_graph_change(),
    std::runtime_error("failed to notify wait set on graph change: error not set"));
}

TEST_F(TestNodeGraph, get_info_by_topic)
{
  const rclcpp::QoS publisher_qos(1);
  auto publisher = node()->create_publisher<test_msgs::msg::Empty>("topic", publisher_qos);
  auto callback = [](test_msgs::msg::Empty::ConstSharedPtr) {};

  const rclcpp::QoS subscriber_qos(10);
  auto subscription =
    node()->create_subscription<test_msgs::msg::Empty>(
    "topic", subscriber_qos, std::move(callback));

  EXPECT_EQ(0u, node_graph()->get_publishers_info_by_topic("topic", true).size());

  std::vector<rclcpp::TopicEndpointInfo> publishers;
  size_t num_publishers = get_num_graph_things(
    [this, &publishers]() {
      publishers = node_graph()->get_publishers_info_by_topic("topic", false);
      return publishers.size();
    });
  ASSERT_EQ(1u, num_publishers);

  auto publisher_endpoint_info = publishers[0];
  const auto const_publisher_endpoint_info = publisher_endpoint_info;
  EXPECT_STREQ(node_name, publisher_endpoint_info.node_name().c_str());
  EXPECT_STREQ(node_name, const_publisher_endpoint_info.node_name().c_str());
  EXPECT_STREQ(absolute_namespace, publisher_endpoint_info.node_namespace().c_str());
  EXPECT_STREQ(absolute_namespace, const_publisher_endpoint_info.node_namespace().c_str());
  EXPECT_STREQ("test_msgs/msg/Empty", publisher_endpoint_info.topic_type().c_str());
  EXPECT_STREQ("test_msgs/msg/Empty", const_publisher_endpoint_info.topic_type().c_str());
  EXPECT_EQ(rclcpp::EndpointType::Publisher, publisher_endpoint_info.endpoint_type());
  EXPECT_EQ(rclcpp::EndpointType::Publisher, const_publisher_endpoint_info.endpoint_type());

  rclcpp::QoS actual_qos = publisher_endpoint_info.qos_profile();
  EXPECT_EQ(actual_qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);

  rclcpp::QoS const_actual_qos = const_publisher_endpoint_info.qos_profile();
  EXPECT_EQ(const_actual_qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);

  auto endpoint_gid = publisher_endpoint_info.endpoint_gid();
  auto const_endpoint_gid = const_publisher_endpoint_info.endpoint_gid();
  bool endpoint_gid_is_all_zeros = true;
  for (size_t i = 0; i < RMW_GID_STORAGE_SIZE; ++i) {
    endpoint_gid_is_all_zeros &= (endpoint_gid[i] == 0);
    EXPECT_EQ(endpoint_gid[i], const_endpoint_gid[i]);
  }
  EXPECT_FALSE(endpoint_gid_is_all_zeros);
}

TEST_F(TestNodeGraph, get_info_by_topic_rcl_node_get_options_error)
{
  const rclcpp::QoS publisher_qos(1);
  auto publisher = node()->create_publisher<test_msgs::msg::Empty>("topic", publisher_qos);

  auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_node_get_options, nullptr);
  RCLCPP_EXPECT_THROW_EQ(
    node_graph()->get_publishers_info_by_topic("topic", false),
    std::runtime_error("Need valid node options in get_info_by_topic()"));
}

// Required for mocking_utils below
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcutils_allocator_t, ==)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcutils_allocator_t, !=)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcutils_allocator_t, <)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcutils_allocator_t, >)

TEST_F(TestNodeGraph, get_info_by_topic_rcl_remap_topic_name_error)
{
  const rclcpp::QoS publisher_qos(1);
  auto publisher = node()->create_publisher<test_msgs::msg::Empty>("topic", publisher_qos);

  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_remap_topic_name, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    node_graph()->get_publishers_info_by_topic("topic", false),
    std::runtime_error("Failed to remap topic name /ns/topic: error not set"));
}

TEST_F(TestNodeGraph, get_info_by_topic_rcl_remap_topic_name_nullptr)
{
  const rclcpp::QoS publisher_qos(1);
  auto publisher = node()->create_publisher<test_msgs::msg::Empty>("topic", publisher_qos);

  // Should be cleaned up by get_info_by_topic
  char * some_string = rcutils_strdup("", rcl_get_default_allocator());
  ASSERT_NE(nullptr, some_string);
  auto mock =
    mocking_utils::patch(
    "lib:rclcpp", rcl_remap_topic_name, [&some_string](
      const rcl_arguments_t *, const rcl_arguments_t *, const char *, const char *, const char *,
      rcl_allocator_t, char ** output_name)
    {
      *output_name = some_string;
      return RCL_RET_OK;
    });
  EXPECT_NO_THROW(node_graph()->get_publishers_info_by_topic("topic", false));
}

TEST_F(TestNodeGraph, get_info_by_topic_rcl_errors)
{
  const rclcpp::QoS publisher_qos(1);
  auto publisher = node()->create_publisher<test_msgs::msg::Empty>("topic", publisher_qos);

  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_get_publishers_info_by_topic, RCL_RET_ERROR);
  auto mock_info_array_fini = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_topic_endpoint_info_array_fini, RCL_RET_ERROR);
  EXPECT_THROW(
    node_graph()->get_publishers_info_by_topic("topic", false),
    rclcpp::exceptions::RCLError);
}

TEST_F(TestNodeGraph, get_info_by_topic_unsupported)
{
  const rclcpp::QoS publisher_qos(1);
  auto publisher = node()->create_publisher<test_msgs::msg::Empty>("topic", publisher_qos);

  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_get_publishers_info_by_topic, RCL_RET_UNSUPPORTED);
  EXPECT_THROW(
    node_graph()->get_publishers_info_by_topic("topic", false),
    rclcpp::exceptions::RCLError);
}

TEST_F(TestNodeGraph, get_info_by_topic_endpoint_info_array_fini_error)
{
  const rclcpp::QoS publisher_qos(1);
  auto publisher = node()->create_publisher<test_msgs::msg::Empty>("topic", publisher_qos);

  auto mock_info_array_fini = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_topic_endpoint_info_array_fini, RCL_RET_ERROR);
  EXPECT_THROW(
    node_graph()->get_publishers_info_by_topic("topic", false),
    rclcpp::exceptions::RCLError);
}
