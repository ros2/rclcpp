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

#include <chrono>
#include <memory>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"

#include "rcpputils/scope_exit.hpp"

#include "../../utils/rclcpp_gtest_macros.hpp"

namespace
{

class DummyEntitiesCollector
  : public rclcpp::executors::detail::EntitiesCollector
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(DummyEntitiesCollector)

  DummyEntitiesCollector() = default;

  void
  add_to_wait_set(rcl_wait_set_t * wait_set) override
  {
    (void)wait_set;
  }

  bool
  is_ready(rcl_wait_set_t * wait_set) override
  {
    (void)wait_set;
    return false;
  }

  std::shared_ptr<void>
  take_data() override
  {
    return nullptr;
  }

  void
  execute(std::shared_ptr<void> & data) override
  {
    (void)data;
  }

protected:
  void
  callback_group_added_impl(
    rclcpp::CallbackGroup::SharedPtr group) override
  {
    (void)group;
  }

  void
  node_added_impl(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node) override
  {
    (void)node;
  }

  void
  callback_group_removed_impl(
    rclcpp::CallbackGroup::SharedPtr group) override
  {
    (void)group;
  }

  void
  node_removed_impl(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node) override
  {
    (void)node;
  }
};

}  // namespace

class TestEntitiesCollector : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
    entities_collector_ =
      std::make_shared<DummyEntitiesCollector>();
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }

  DummyEntitiesCollector::SharedPtr entities_collector_;
};

TEST_F(TestEntitiesCollector, add_remove_node) {
  auto node1 = std::make_shared<rclcpp::Node>("node1", "ns");
  EXPECT_NO_THROW(entities_collector_->add_node(node1->get_node_base_interface()));

  // Check adding second time
  RCLCPP_EXPECT_THROW_EQ(
    entities_collector_->add_node(node1->get_node_base_interface()),
    std::runtime_error("Node has already been added to an executor."));

  auto node2 = std::make_shared<rclcpp::Node>("node2", "ns");
  EXPECT_FALSE(entities_collector_->remove_node(node2->get_node_base_interface()));
  EXPECT_NO_THROW(entities_collector_->add_node(node2->get_node_base_interface()));

  EXPECT_TRUE(entities_collector_->remove_node(node1->get_node_base_interface()));
  EXPECT_FALSE(entities_collector_->remove_node(node1->get_node_base_interface()));
  EXPECT_TRUE(entities_collector_->remove_node(node2->get_node_base_interface()));

  auto node3 = std::make_shared<rclcpp::Node>("node3", "ns");
  node3->get_node_base_interface()->get_associated_with_executor_atomic().exchange(true);
  EXPECT_FALSE(entities_collector_->remove_node(node3->get_node_base_interface()));
}

TEST_F(TestEntitiesCollector, add_callback_group) {
  auto node = std::make_shared<rclcpp::Node>("node1", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  entities_collector_->add_callback_group(cb_group, node->get_node_base_interface());
  ASSERT_EQ(entities_collector_->get_all_callback_groups().size(), 1u);
}

TEST_F(TestEntitiesCollector, add_callback_group_after_add_node) {
  auto node = std::make_shared<rclcpp::Node>("node1", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  entities_collector_->add_node(node->get_node_base_interface());
  RCLCPP_EXPECT_THROW_EQ(
    entities_collector_->add_callback_group(cb_group, node->get_node_base_interface()),
    std::runtime_error("Callback group has already been added to an executor."));
}

TEST_F(TestEntitiesCollector, add_callback_group_twice) {
  auto node = std::make_shared<rclcpp::Node>("node1", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  entities_collector_->add_callback_group(cb_group, node->get_node_base_interface());
  ASSERT_EQ(entities_collector_->get_all_callback_groups().size(), 1u);
  cb_group->get_associated_with_executor_atomic().exchange(false);
  RCLCPP_EXPECT_THROW_EQ(
    entities_collector_->add_callback_group(cb_group, node->get_node_base_interface()),
    std::runtime_error("Callback group was already added to executor."));
}

TEST_F(TestEntitiesCollector, remove_callback_group_after_node) {
  auto node = std::make_shared<rclcpp::Node>("node1", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  entities_collector_->add_callback_group(cb_group, node->get_node_base_interface());
  ASSERT_EQ(entities_collector_->get_all_callback_groups().size(), 1u);

  node.reset();

  RCLCPP_EXPECT_THROW_EQ(
    entities_collector_->remove_callback_group(cb_group),
    std::runtime_error("Node must not be deleted before its callback group(s)."));
}

TEST_F(TestEntitiesCollector, remove_callback_group_twice) {
  auto node = std::make_shared<rclcpp::Node>("node1", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  entities_collector_->add_callback_group(cb_group, node->get_node_base_interface());
  ASSERT_EQ(entities_collector_->get_all_callback_groups().size(), 1u);

  entities_collector_->remove_callback_group(cb_group);

  RCLCPP_EXPECT_THROW_EQ(
    entities_collector_->remove_callback_group(cb_group),
    std::runtime_error("Callback group needs to be associated with executor."));
}

TEST_F(TestEntitiesCollector, remove_node_opposite_order) {
  auto node1 = std::make_shared<rclcpp::Node>("node1", "ns");
  EXPECT_NO_THROW(entities_collector_->add_node(node1->get_node_base_interface()));

  auto node2 = std::make_shared<rclcpp::Node>("node2", "ns");
  EXPECT_NO_THROW(entities_collector_->add_node(node2->get_node_base_interface()));

  EXPECT_TRUE(entities_collector_->remove_node(node2->get_node_base_interface()));
}
