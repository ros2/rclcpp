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
#include <string>

#include "rcl/node_options.h"
#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_waitables.hpp"
#include "rclcpp/rclcpp.hpp"

#include "../../mocking_utils/patch.hpp"
#include "../../utils/rclcpp_gtest_macros.hpp"

class TestWaitable : public rclcpp::Waitable
{
public:
  void add_to_wait_set(rcl_wait_set_t *) override {}
  bool is_ready(rcl_wait_set_t *) override {return false;}

  std::shared_ptr<void>
  take_data() override
  {
    return nullptr;
  }

  void execute(std::shared_ptr<void> & data) override
  {
    (void) data;
  }
};

class TestNodeWaitables : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("node", "ns");

    node_waitables =
      dynamic_cast<rclcpp::node_interfaces::NodeWaitables *>(
      node->get_node_waitables_interface().get());
    ASSERT_NE(nullptr, node_waitables);
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::node_interfaces::NodeWaitables * node_waitables;
};

TEST_F(TestNodeWaitables, add_remove_waitable)
{
  std::shared_ptr<rclcpp::Node> node2 = std::make_shared<rclcpp::Node>("node2", "ns");

  auto callback_group1 = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto callback_group2 = node2->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto waitable = std::make_shared<TestWaitable>();
  EXPECT_NO_THROW(
    node_waitables->add_waitable(waitable, callback_group1));
  RCLCPP_EXPECT_THROW_EQ(
    node_waitables->add_waitable(waitable, callback_group2),
    std::runtime_error("Cannot create waitable, group not in node."));
  EXPECT_NO_THROW(node_waitables->remove_waitable(waitable, callback_group1));
  EXPECT_NO_THROW(node_waitables->remove_waitable(waitable, callback_group2));

  auto waitable2 = std::make_shared<TestWaitable>();
  EXPECT_NO_THROW(node_waitables->add_waitable(waitable2, nullptr));
  EXPECT_NO_THROW(node_waitables->remove_waitable(waitable2, nullptr));
}

TEST_F(TestNodeWaitables, add_waitable_rcl_trigger_guard_condition_error)
{
  auto callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto waitable = std::make_shared<TestWaitable>();

  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_trigger_guard_condition, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    node_waitables->add_waitable(waitable, callback_group),
    std::runtime_error("failed to notify wait set on waitable creation: error not set"));
}
