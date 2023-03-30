// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/executor_entities_collector.hpp"

#include "../../utils/rclcpp_gtest_macros.hpp"

class TestExecutorEntitiesCollector : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }
};

TEST_F(TestExecutorEntitiesCollector, add_remove_node) {
  auto entities_collector = rclcpp::executors::ExecutorEntitiesCollector();

  auto node1 = std::make_shared<rclcpp::Node>("node1", "ns");
  auto node2 = std::make_shared<rclcpp::Node>("node2", "ns");

  // Add a node
  EXPECT_NO_THROW(entities_collector.add_node(node1->get_node_base_interface()));

  // Add the same node a second time
  RCLCPP_EXPECT_THROW_EQ(
    entities_collector.add_node(node1->get_node_base_interface()),
    std::runtime_error("Node '/ns/node1' has already been added to an executor."));

  // Remove a node before adding
  RCLCPP_EXPECT_THROW_EQ(
    entities_collector.remove_node(node2->get_node_base_interface()),
    std::runtime_error("Node needs to be associated with an executor."));

  // Simulate node being associated somewhere else
  auto & has_executor = node2->get_node_base_interface()->get_associated_with_executor_atomic();
  has_executor.store(true);

  // Add an already-associated node
  RCLCPP_EXPECT_THROW_EQ(
    entities_collector.remove_node(node2->get_node_base_interface()),
    std::runtime_error("Node needs to be associated with this executor."));

  has_executor.store(false);

  // Add the now-disassociated node
  EXPECT_NO_THROW(entities_collector.add_node(node2->get_node_base_interface()));

  // Remove an existing node
  EXPECT_NO_THROW(entities_collector.remove_node(node1->get_node_base_interface()));
  // Remove the same node a second time
  RCLCPP_EXPECT_THROW_EQ(
    entities_collector.remove_node(node1->get_node_base_interface()),
    std::runtime_error("Node needs to be associated with an executor."));

  // Remove an existing node
  EXPECT_NO_THROW(entities_collector.remove_node(node2->get_node_base_interface()));
}

TEST_F(TestExecutorEntitiesCollector, add_callback_group) {
  auto entities_collector = rclcpp::executors::ExecutorEntitiesCollector();

  auto node = std::make_shared<rclcpp::Node>("node1", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  entities_collector.add_callback_group(cb_group);
  ASSERT_EQ(entities_collector.get_all_callback_groups().size(), 1u);
  ASSERT_EQ(entities_collector.get_manually_added_callback_groups().size(), 1u);
  ASSERT_EQ(entities_collector.get_automatically_added_callback_groups().size(), 0u);
}

TEST_F(TestExecutorEntitiesCollector, add_node_default_callback_group) {
  auto entities_collector = rclcpp::executors::ExecutorEntitiesCollector();

  auto node = std::make_shared<rclcpp::Node>("node1", "ns");
  entities_collector.add_node(node->get_node_base_interface());

  ASSERT_EQ(entities_collector.get_all_callback_groups().size(), 1u);
  ASSERT_EQ(entities_collector.get_manually_added_callback_groups().size(), 0u);
  ASSERT_EQ(entities_collector.get_automatically_added_callback_groups().size(), 1u);
}

TEST_F(TestExecutorEntitiesCollector, add_callback_group_after_add_node) {
  auto entities_collector = rclcpp::executors::ExecutorEntitiesCollector();

  auto node = std::make_shared<rclcpp::Node>("node1", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  entities_collector.add_node(node->get_node_base_interface());
  RCLCPP_EXPECT_THROW_EQ(
    entities_collector.add_callback_group(cb_group),
    std::runtime_error("Callback group has already been added to an executor."));
}

TEST_F(TestExecutorEntitiesCollector, add_callback_group_twice) {
  auto entities_collector = rclcpp::executors::ExecutorEntitiesCollector();

  auto node = std::make_shared<rclcpp::Node>("node1", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  entities_collector.add_callback_group(cb_group);
  ASSERT_EQ(entities_collector.get_all_callback_groups().size(), 1u);
  ASSERT_EQ(entities_collector.get_manually_added_callback_groups().size(), 1u);
  ASSERT_EQ(entities_collector.get_automatically_added_callback_groups().size(), 0u);

  cb_group->get_associated_with_executor_atomic().exchange(false);
  RCLCPP_EXPECT_THROW_EQ(
    entities_collector.add_callback_group(cb_group),
    std::runtime_error("Callback group has already been added to this executor."));
}

TEST_F(TestExecutorEntitiesCollector, remove_callback_group_after_node) {
  auto entities_collector = rclcpp::executors::ExecutorEntitiesCollector();

  auto node = std::make_shared<rclcpp::Node>("node1", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  entities_collector.add_callback_group(cb_group);
  ASSERT_EQ(entities_collector.get_all_callback_groups().size(), 1u);
  ASSERT_EQ(entities_collector.get_manually_added_callback_groups().size(), 1u);
  ASSERT_EQ(entities_collector.get_automatically_added_callback_groups().size(), 0u);

  node.reset();

  ASSERT_FALSE(cb_group->has_valid_node());

  RCLCPP_EXPECT_THROW_EQ(
    entities_collector.remove_callback_group(cb_group),
    std::runtime_error("Node must not be deleted before its callback group(s)."));
}

TEST_F(TestExecutorEntitiesCollector, remove_callback_group_twice) {
  auto entities_collector = rclcpp::executors::ExecutorEntitiesCollector();

  auto node = std::make_shared<rclcpp::Node>("node1", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  entities_collector.add_callback_group(cb_group);
  ASSERT_EQ(entities_collector.get_all_callback_groups().size(), 1u);
  ASSERT_EQ(entities_collector.get_manually_added_callback_groups().size(), 1u);
  ASSERT_EQ(entities_collector.get_automatically_added_callback_groups().size(), 0u);

  entities_collector.remove_callback_group(cb_group);

  RCLCPP_EXPECT_THROW_EQ(
    entities_collector.remove_callback_group(cb_group),
    std::runtime_error("Callback group needs to be associated with executor."));
}

TEST_F(TestExecutorEntitiesCollector, remove_node_opposite_order) {
  auto entities_collector = rclcpp::executors::ExecutorEntitiesCollector();

  auto node1 = std::make_shared<rclcpp::Node>("node1", "ns");
  EXPECT_NO_THROW(entities_collector.add_node(node1->get_node_base_interface()));

  auto node2 = std::make_shared<rclcpp::Node>("node2", "ns");
  EXPECT_NO_THROW(entities_collector.add_node(node2->get_node_base_interface()));

  EXPECT_NO_THROW(entities_collector.remove_node(node2->get_node_base_interface()));
}
