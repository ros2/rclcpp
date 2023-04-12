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

#include "rclcpp/executors/executor_notify_waitable.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/executor_entities_collector.hpp"

#include "../../utils/rclcpp_gtest_macros.hpp"

class TestExecutorEntitiesCollector : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);

    notify_waitable = std::make_shared<rclcpp::executors::ExecutorNotifyWaitable>();
    entities_collector = std::make_shared<rclcpp::executors::ExecutorEntitiesCollector>(
      notify_waitable);
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::executors::ExecutorNotifyWaitable> notify_waitable;
  std::shared_ptr<rclcpp::executors::ExecutorEntitiesCollector> entities_collector;
};

TEST_F(TestExecutorEntitiesCollector, add_remove_node) {
  auto node1 = std::make_shared<rclcpp::Node>("node1", "ns");

  // Add a node
  EXPECT_NO_THROW(entities_collector->add_node(node1->get_node_base_interface()));
  EXPECT_NO_THROW(entities_collector->update_collections());

  // Remove a node
  EXPECT_NO_THROW(entities_collector->remove_node(node1->get_node_base_interface()));
  EXPECT_NO_THROW(entities_collector->update_collections());
}

TEST_F(TestExecutorEntitiesCollector, add_node_twice) {
  auto node1 = std::make_shared<rclcpp::Node>("node1", "ns");

  EXPECT_NO_THROW(entities_collector->add_node(node1->get_node_base_interface()));

  RCLCPP_EXPECT_THROW_EQ(
    entities_collector->add_node(node1->get_node_base_interface()),
    std::runtime_error("Node '/ns/node1' has already been added to an executor."));

  EXPECT_NO_THROW(entities_collector->update_collections());
}

TEST_F(TestExecutorEntitiesCollector, add_associated_node) {
  auto node1 = std::make_shared<rclcpp::Node>("node1", "ns");

  // Simulate node being associated somewhere else
  auto & has_executor = node1->get_node_base_interface()->get_associated_with_executor_atomic();
  has_executor.store(true);

  // Add an already-associated node
  RCLCPP_EXPECT_THROW_EQ(
    entities_collector->remove_node(node1->get_node_base_interface()),
    std::runtime_error("Node '/ns/node1' needs to be associated with this executor."));

  has_executor.store(false);
}

TEST_F(TestExecutorEntitiesCollector, remove_unassociated_node) {
  auto node1 = std::make_shared<rclcpp::Node>("node1", "ns");

  // Add an already-associated node
  RCLCPP_EXPECT_THROW_EQ(
    entities_collector->remove_node(node1->get_node_base_interface()),
    std::runtime_error("Node '/ns/node1' needs to be associated with an executor."));

  // Simulate node being associated somewhere else
  auto & has_executor = node1->get_node_base_interface()->get_associated_with_executor_atomic();
  has_executor.store(true);

  // Add an already-associated node
  RCLCPP_EXPECT_THROW_EQ(
    entities_collector->remove_node(node1->get_node_base_interface()),
    std::runtime_error("Node '/ns/node1' needs to be associated with this executor."));
}

TEST_F(TestExecutorEntitiesCollector, add_remove_node_before_update) {
  auto notify_waitable = std::make_shared<rclcpp::executors::ExecutorNotifyWaitable>();
  auto entities_collector = rclcpp::executors::ExecutorEntitiesCollector(notify_waitable);

  auto node1 = std::make_shared<rclcpp::Node>("node1", "ns");
  auto node2 = std::make_shared<rclcpp::Node>("node2", "ns");

  // Add and remove nodes without running updatenode
  EXPECT_NO_THROW(entities_collector.add_node(node1->get_node_base_interface()));
  EXPECT_NO_THROW(entities_collector.add_node(node2->get_node_base_interface()));
  EXPECT_NO_THROW(entities_collector.remove_node(node1->get_node_base_interface()));
  EXPECT_NO_THROW(entities_collector.remove_node(node2->get_node_base_interface()));
  EXPECT_NO_THROW(entities_collector.update_collections());
}

TEST_F(TestExecutorEntitiesCollector, add_callback_group) {
  auto notify_waitable = std::make_shared<rclcpp::executors::ExecutorNotifyWaitable>();
  auto entities_collector = rclcpp::executors::ExecutorEntitiesCollector(notify_waitable);

  auto node = std::make_shared<rclcpp::Node>("node1", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // Add a callback group and update
  entities_collector.add_callback_group(cb_group);

  ASSERT_EQ(entities_collector.get_all_callback_groups().size(), 0u);
  ASSERT_EQ(entities_collector.get_manually_added_callback_groups().size(), 0u);
  ASSERT_EQ(entities_collector.get_automatically_added_callback_groups().size(), 0u);

  entities_collector.update_collections();

  ASSERT_EQ(entities_collector.get_all_callback_groups().size(), 1u);
  ASSERT_EQ(entities_collector.get_manually_added_callback_groups().size(), 1u);
  ASSERT_EQ(entities_collector.get_automatically_added_callback_groups().size(), 0u);

  // Remove callback group and update
  entities_collector.remove_callback_group(cb_group);

  ASSERT_EQ(entities_collector.get_all_callback_groups().size(), 1u);
  ASSERT_EQ(entities_collector.get_manually_added_callback_groups().size(), 1u);
  ASSERT_EQ(entities_collector.get_automatically_added_callback_groups().size(), 0u);

  entities_collector.update_collections();

  ASSERT_EQ(entities_collector.get_all_callback_groups().size(), 0u);
  ASSERT_EQ(entities_collector.get_manually_added_callback_groups().size(), 0u);
  ASSERT_EQ(entities_collector.get_automatically_added_callback_groups().size(), 0u);
}

TEST_F(TestExecutorEntitiesCollector, add_node_default_callback_group) {
  auto notify_waitable = std::make_shared<rclcpp::executors::ExecutorNotifyWaitable>();
  auto entities_collector = rclcpp::executors::ExecutorEntitiesCollector(notify_waitable);

  auto node = std::make_shared<rclcpp::Node>("node1", "ns");
  entities_collector.add_node(node->get_node_base_interface());

  ASSERT_EQ(entities_collector.get_all_callback_groups().size(), 0u);
  ASSERT_EQ(entities_collector.get_manually_added_callback_groups().size(), 0u);
  ASSERT_EQ(entities_collector.get_automatically_added_callback_groups().size(), 0u);

  entities_collector.update_collections();

  ASSERT_EQ(entities_collector.get_all_callback_groups().size(), 1u);
  ASSERT_EQ(entities_collector.get_manually_added_callback_groups().size(), 0u);
  ASSERT_EQ(entities_collector.get_automatically_added_callback_groups().size(), 1u);
}

TEST_F(TestExecutorEntitiesCollector, add_callback_group_after_add_node) {
  auto notify_waitable = std::make_shared<rclcpp::executors::ExecutorNotifyWaitable>();
  auto entities_collector = rclcpp::executors::ExecutorEntitiesCollector(notify_waitable);

  auto node = std::make_shared<rclcpp::Node>("node1", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  entities_collector.add_node(node->get_node_base_interface());

  ASSERT_EQ(entities_collector.get_all_callback_groups().size(), 0u);
  ASSERT_EQ(entities_collector.get_manually_added_callback_groups().size(), 0u);
  ASSERT_EQ(entities_collector.get_automatically_added_callback_groups().size(), 0u);

  entities_collector.update_collections();

  ASSERT_EQ(entities_collector.get_all_callback_groups().size(), 2u);
  ASSERT_EQ(entities_collector.get_manually_added_callback_groups().size(), 0u);
  ASSERT_EQ(entities_collector.get_automatically_added_callback_groups().size(), 2u);

  RCLCPP_EXPECT_THROW_EQ(
    entities_collector.add_callback_group(cb_group),
    std::runtime_error("Callback group has already been added to an executor."));
}

TEST_F(TestExecutorEntitiesCollector, add_callback_group_twice) {
  auto notify_waitable = std::make_shared<rclcpp::executors::ExecutorNotifyWaitable>();
  auto entities_collector = rclcpp::executors::ExecutorEntitiesCollector(notify_waitable);

  auto node = std::make_shared<rclcpp::Node>("node1", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  entities_collector.add_callback_group(cb_group);

  ASSERT_EQ(entities_collector.get_all_callback_groups().size(), 0u);
  ASSERT_EQ(entities_collector.get_manually_added_callback_groups().size(), 0u);
  ASSERT_EQ(entities_collector.get_automatically_added_callback_groups().size(), 0u);

  entities_collector.update_collections();

  ASSERT_EQ(entities_collector.get_all_callback_groups().size(), 1u);
  ASSERT_EQ(entities_collector.get_manually_added_callback_groups().size(), 1u);
  ASSERT_EQ(entities_collector.get_automatically_added_callback_groups().size(), 0u);

  cb_group->get_associated_with_executor_atomic().exchange(false);
  RCLCPP_EXPECT_THROW_EQ(
    entities_collector.add_callback_group(cb_group),
    std::runtime_error("Callback group has already been added to this executor."));
}

TEST_F(TestExecutorEntitiesCollector, remove_callback_group_after_node) {
  auto notify_waitable = std::make_shared<rclcpp::executors::ExecutorNotifyWaitable>();
  auto entities_collector = rclcpp::executors::ExecutorEntitiesCollector(notify_waitable);

  auto node = std::make_shared<rclcpp::Node>("node1", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  entities_collector.add_callback_group(cb_group);

  ASSERT_EQ(entities_collector.get_all_callback_groups().size(), 0u);
  ASSERT_EQ(entities_collector.get_manually_added_callback_groups().size(), 0u);
  ASSERT_EQ(entities_collector.get_automatically_added_callback_groups().size(), 0u);

  entities_collector.update_collections();

  ASSERT_EQ(entities_collector.get_all_callback_groups().size(), 1u);
  ASSERT_EQ(entities_collector.get_manually_added_callback_groups().size(), 1u);
  ASSERT_EQ(entities_collector.get_automatically_added_callback_groups().size(), 0u);

  node.reset();

  /**
   * TODO(mjcarroll): Assert this when we are enforcing that nodes must be destroyed
   * after their created callback groups.
  RCLCPP_EXPECT_THROW_EQ(
    entities_collector.remove_callback_group(cb_group),
    std::runtime_error("Node must not be deleted before its callback group(s)."));
  */
  EXPECT_NO_THROW(entities_collector.update_collections());
}

TEST_F(TestExecutorEntitiesCollector, remove_callback_group_after_node2) {
  auto notify_waitable = std::make_shared<rclcpp::executors::ExecutorNotifyWaitable>();
  auto entities_collector = rclcpp::executors::ExecutorEntitiesCollector(notify_waitable);

  auto node = std::make_shared<rclcpp::Node>("node1", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  entities_collector.add_callback_group(cb_group);

  ASSERT_EQ(entities_collector.get_all_callback_groups().size(), 0u);
  ASSERT_EQ(entities_collector.get_manually_added_callback_groups().size(), 0u);
  ASSERT_EQ(entities_collector.get_automatically_added_callback_groups().size(), 0u);

  entities_collector.update_collections();

  ASSERT_EQ(entities_collector.get_all_callback_groups().size(), 1u);
  ASSERT_EQ(entities_collector.get_manually_added_callback_groups().size(), 1u);
  ASSERT_EQ(entities_collector.get_automatically_added_callback_groups().size(), 0u);

  EXPECT_NO_THROW(entities_collector.remove_callback_group(cb_group));

  node.reset();

  /**
   * TODO(mjcarroll): Assert this when we are enforcing that nodes must be destroyed
   * after their created callback groups.
  RCLCPP_EXPECT_THROW_EQ(
    entities_collector.remove_callback_group(cb_group),
    std::runtime_error("Node must not be deleted before its callback group(s)."));
  */
  EXPECT_NO_THROW(entities_collector.update_collections());
}

TEST_F(TestExecutorEntitiesCollector, remove_callback_group_twice) {
  auto notify_waitable = std::make_shared<rclcpp::executors::ExecutorNotifyWaitable>();
  auto entities_collector = rclcpp::executors::ExecutorEntitiesCollector(notify_waitable);

  auto node = std::make_shared<rclcpp::Node>("node1", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  entities_collector.add_callback_group(cb_group);
  entities_collector.update_collections();

  ASSERT_EQ(entities_collector.get_all_callback_groups().size(), 1u);
  ASSERT_EQ(entities_collector.get_manually_added_callback_groups().size(), 1u);
  ASSERT_EQ(entities_collector.get_automatically_added_callback_groups().size(), 0u);

  entities_collector.remove_callback_group(cb_group);
  entities_collector.update_collections();

  RCLCPP_EXPECT_THROW_EQ(
    entities_collector.remove_callback_group(cb_group),
    std::runtime_error("Callback group needs to be associated with an executor."));
}

TEST_F(TestExecutorEntitiesCollector, remove_node_opposite_order) {
  auto notify_waitable = std::make_shared<rclcpp::executors::ExecutorNotifyWaitable>();
  auto entities_collector = rclcpp::executors::ExecutorEntitiesCollector(notify_waitable);

  auto node1 = std::make_shared<rclcpp::Node>("node1", "ns");
  EXPECT_NO_THROW(entities_collector.add_node(node1->get_node_base_interface()));

  auto node2 = std::make_shared<rclcpp::Node>("node2", "ns");
  EXPECT_NO_THROW(entities_collector.add_node(node2->get_node_base_interface()));

  EXPECT_NO_THROW(entities_collector.remove_node(node2->get_node_base_interface()));
}
