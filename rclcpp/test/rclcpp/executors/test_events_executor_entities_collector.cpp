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

#include "rclcpp/rclcpp.hpp"

#include "../../mocking_utils/patch.hpp"

class TestEventsExecutorEntitiesCollector : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
    dummy_executor_ = std::make_shared<rclcpp::executors::EventsExecutor>();
    entities_collector_ =
      std::make_shared<rclcpp::executors::EventsExecutorEntitiesCollector>(dummy_executor_.get());
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }

  rclcpp::executors::EventsExecutorEntitiesCollector::SharedPtr entities_collector_;

private:
  rclcpp::executors::EventsExecutor::SharedPtr dummy_executor_;
};

TEST_F(TestEventsExecutorEntitiesCollector, bad_init)
{
  EXPECT_THROW(
    std::make_shared<rclcpp::executors::EventsExecutorEntitiesCollector>(nullptr),
    std::runtime_error);
}

TEST_F(TestEventsExecutorEntitiesCollector, add_remove_node)
{
  auto node1 = std::make_shared<rclcpp::Node>("node1", "ns");
  EXPECT_NO_THROW(entities_collector_->add_node(node1->get_node_base_interface()));

  // Check adding second time
  EXPECT_THROW(
    entities_collector_->add_node(node1->get_node_base_interface()),
    std::runtime_error);

  auto node2 = std::make_shared<rclcpp::Node>("node2", "ns");
  EXPECT_THROW(
    entities_collector_->remove_node(node2->get_node_base_interface()),
    std::runtime_error);
  EXPECT_NO_THROW(entities_collector_->add_node(node2->get_node_base_interface()));

  EXPECT_NO_THROW(entities_collector_->remove_node(node1->get_node_base_interface()));
  EXPECT_THROW(
    entities_collector_->remove_node(node1->get_node_base_interface()),
    std::runtime_error);
  EXPECT_NO_THROW(entities_collector_->remove_node(node2->get_node_base_interface()));

  auto node3 = std::make_shared<rclcpp::Node>("node3", "ns");
  node3->get_node_base_interface()->get_associated_with_executor_atomic().exchange(true);
  EXPECT_THROW(
    entities_collector_->remove_node(node3->get_node_base_interface()),
    std::runtime_error);
}

TEST_F(TestEventsExecutorEntitiesCollector, add_callback_group)
{
  auto node = std::make_shared<rclcpp::Node>("node1", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  entities_collector_->add_callback_group(cb_group, node->get_node_base_interface());
  ASSERT_EQ(entities_collector_->get_all_callback_groups().size(), 1u);
}

TEST_F(TestEventsExecutorEntitiesCollector, add_callback_group_after_add_node)
{
  auto node = std::make_shared<rclcpp::Node>("node1", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  entities_collector_->add_node(node->get_node_base_interface());
  EXPECT_THROW(
    entities_collector_->add_callback_group(cb_group, node->get_node_base_interface()),
    std::runtime_error);
}

TEST_F(TestEventsExecutorEntitiesCollector, add_callback_group_twice)
{
  auto node = std::make_shared<rclcpp::Node>("node1", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  entities_collector_->add_callback_group(cb_group, node->get_node_base_interface());
  ASSERT_EQ(entities_collector_->get_all_callback_groups().size(), 1u);
  cb_group->get_associated_with_executor_atomic().exchange(false);
  EXPECT_THROW(
    entities_collector_->add_callback_group(cb_group, node->get_node_base_interface()),
    std::runtime_error);
}

TEST_F(TestEventsExecutorEntitiesCollector, remove_callback_group_after_node)
{
  auto node = std::make_shared<rclcpp::Node>("node1", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  entities_collector_->add_callback_group(cb_group, node->get_node_base_interface());
  ASSERT_EQ(entities_collector_->get_all_callback_groups().size(), 1u);

  node.reset();

  EXPECT_THROW(
    entities_collector_->remove_callback_group(cb_group),
    std::runtime_error);
}

TEST_F(TestEventsExecutorEntitiesCollector, remove_callback_group_twice)
{
  auto node = std::make_shared<rclcpp::Node>("node1", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  entities_collector_->add_callback_group(cb_group, node->get_node_base_interface());
  ASSERT_EQ(entities_collector_->get_all_callback_groups().size(), 1u);

  entities_collector_->remove_callback_group(cb_group);

  EXPECT_THROW(
    entities_collector_->remove_callback_group(cb_group),
    std::runtime_error);
}

TEST_F(TestEventsExecutorEntitiesCollector, remove_node_opposite_order)
{
  auto node1 = std::make_shared<rclcpp::Node>("node1", "ns");
  EXPECT_NO_THROW(entities_collector_->add_node(node1->get_node_base_interface()));

  auto node2 = std::make_shared<rclcpp::Node>("node2", "ns");
  EXPECT_NO_THROW(entities_collector_->add_node(node2->get_node_base_interface()));

  EXPECT_NO_THROW(entities_collector_->remove_node(node2->get_node_base_interface()));
}

TEST_F(TestEventsExecutorEntitiesCollector, test_fancy_name)
{
  auto node1 = std::make_shared<rclcpp::Node>("node1", "ns");
  auto node2 = std::make_shared<rclcpp::Node>("node2", "ns");
  entities_collector_->add_node(node1->get_node_base_interface());

  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_guard_condition_set_events_executor_callback, RCL_RET_ERROR);

    EXPECT_THROW(
      entities_collector_->add_node(node2->get_node_base_interface()),
      std::runtime_error);

    EXPECT_THROW(
      entities_collector_->remove_node(node1->get_node_base_interface()),
      std::runtime_error);
  }
}
