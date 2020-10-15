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

#include "test_msgs/msg/empty.hpp"
#include "test_msgs/srv/empty.hpp"

namespace
{

struct NumberOfEntities
{
  size_t subscriptions = 0;
  size_t timers = 0;
  size_t services = 0;
  size_t clients = 0;
  size_t waitables = 0;
};

std::unique_ptr<NumberOfEntities> get_number_of_default_entities(rclcpp::Node::SharedPtr node)
{
  auto number_of_entities = std::make_unique<NumberOfEntities>();
  for (auto & weak_group : node->get_callback_groups()) {
    auto group = weak_group.lock();
    EXPECT_NE(nullptr, group);
    if (!group || !group->can_be_taken_from().load()) {
      return nullptr;
    }
    group->find_subscription_ptrs_if(
      [&number_of_entities](rclcpp::SubscriptionBase::SharedPtr &)
      {
        number_of_entities->subscriptions++; return false;
      });
    group->find_timer_ptrs_if(
      [&number_of_entities](rclcpp::TimerBase::SharedPtr &)
      {
        number_of_entities->timers++; return false;
      });
    group->find_service_ptrs_if(
      [&number_of_entities](rclcpp::ServiceBase::SharedPtr &)
      {
        number_of_entities->services++; return false;
      });
    group->find_client_ptrs_if(
      [&number_of_entities](rclcpp::ClientBase::SharedPtr &)
      {
        number_of_entities->clients++; return false;
      });
    group->find_waitable_ptrs_if(
      [&number_of_entities](rclcpp::Waitable::SharedPtr &)
      {
        number_of_entities->waitables++; return false;
      });
  }

  return number_of_entities;
}

}  // namespace

class TestStaticExecutorEntitiesCollector : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
    entities_collector_ =
      std::make_shared<rclcpp::executors::StaticExecutorEntitiesCollector>();
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }

  rclcpp::executors::StaticExecutorEntitiesCollector::SharedPtr entities_collector_;
};

TEST_F(TestStaticExecutorEntitiesCollector, construct_destruct) {
  EXPECT_EQ(0u, entities_collector_->get_number_of_subscriptions());
  EXPECT_EQ(0u, entities_collector_->get_number_of_timers());
  EXPECT_EQ(0u, entities_collector_->get_number_of_services());
  EXPECT_EQ(0u, entities_collector_->get_number_of_clients());
  EXPECT_EQ(0u, entities_collector_->get_number_of_waitables());
}

TEST_F(TestStaticExecutorEntitiesCollector, add_remove_node) {
  auto node1 = std::make_shared<rclcpp::Node>("node1", "ns");
  EXPECT_NO_THROW(entities_collector_->add_node(node1->get_node_base_interface()));

  // Check adding second time
  EXPECT_THROW(entities_collector_->add_node(node1->get_node_base_interface()), std::runtime_error);

  auto node2 = std::make_shared<rclcpp::Node>("node2", "ns");
  EXPECT_FALSE(entities_collector_->remove_node(node2->get_node_base_interface()));
  EXPECT_NO_THROW(entities_collector_->add_node(node2->get_node_base_interface()));

  EXPECT_TRUE(entities_collector_->remove_node(node1->get_node_base_interface()));
  EXPECT_FALSE(entities_collector_->remove_node(node1->get_node_base_interface()));
  EXPECT_TRUE(entities_collector_->remove_node(node2->get_node_base_interface()));
}

TEST_F(TestStaticExecutorEntitiesCollector, init_bad_arguments) {
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  entities_collector_->add_node(node->get_node_base_interface());

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  auto shared_context = node->get_node_base_interface()->get_context();
  rcl_context_t * context = shared_context->get_rcl_context().get();
  EXPECT_EQ(
    RCL_RET_OK,
    rcl_wait_set_init(&wait_set, 100, 100, 100, 100, 100, 100, context, allocator));
  RCLCPP_SCOPE_EXIT({EXPECT_EQ(RCL_RET_OK, rcl_wait_set_fini(&wait_set));});

  rclcpp::GuardCondition guard_condition(shared_context);
  rcl_guard_condition_t rcl_guard_condition = guard_condition.get_rcl_guard_condition();

  // Check memory strategy is nullptr
  rclcpp::memory_strategy::MemoryStrategy::SharedPtr memory_strategy = nullptr;
  EXPECT_THROW(
    entities_collector_->init(&wait_set, memory_strategy, &rcl_guard_condition),
    std::runtime_error);
}

TEST_F(TestStaticExecutorEntitiesCollector, add_remove_basic_node) {
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  const auto expected_number_of_entities = get_number_of_default_entities(node);
  EXPECT_NE(nullptr, expected_number_of_entities);
  entities_collector_->add_node(node->get_node_base_interface());

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  auto shared_context = node->get_node_base_interface()->get_context();
  rcl_context_t * context = shared_context->get_rcl_context().get();
  EXPECT_EQ(
    RCL_RET_OK,
    rcl_wait_set_init(&wait_set, 100, 100, 100, 100, 100, 100, context, allocator));
  RCLCPP_SCOPE_EXIT({EXPECT_EQ(RCL_RET_OK, rcl_wait_set_fini(&wait_set));});

  auto memory_strategy = rclcpp::memory_strategies::create_default_strategy();
  rclcpp::GuardCondition guard_condition(shared_context);
  rcl_guard_condition_t rcl_guard_condition = guard_condition.get_rcl_guard_condition();

  entities_collector_->init(&wait_set, memory_strategy, &rcl_guard_condition);
  RCLCPP_SCOPE_EXIT(entities_collector_->fini());
  EXPECT_EQ(
    expected_number_of_entities->subscriptions,
    entities_collector_->get_number_of_subscriptions());
  EXPECT_EQ(expected_number_of_entities->timers, entities_collector_->get_number_of_timers());
  EXPECT_EQ(expected_number_of_entities->services, entities_collector_->get_number_of_services());
  EXPECT_EQ(expected_number_of_entities->clients, entities_collector_->get_number_of_clients());
  // One extra for the executor
  EXPECT_EQ(
    1u + expected_number_of_entities->waitables,
    entities_collector_->get_number_of_waitables());

  EXPECT_TRUE(entities_collector_->remove_node(node->get_node_base_interface()));
  entities_collector_->init(&wait_set, memory_strategy, &rcl_guard_condition);
  EXPECT_EQ(0u, entities_collector_->get_number_of_subscriptions());
  EXPECT_EQ(0u, entities_collector_->get_number_of_timers());
  EXPECT_EQ(0u, entities_collector_->get_number_of_services());
  EXPECT_EQ(0u, entities_collector_->get_number_of_clients());

  // Still one for the executor
  EXPECT_EQ(1u, entities_collector_->get_number_of_waitables());
}

TEST_F(TestStaticExecutorEntitiesCollector, add_remove_node_out_of_scope) {
  rclcpp::Context::SharedPtr shared_context = nullptr;
  {
    auto node1 = std::make_shared<rclcpp::Node>("node1", "ns");
    auto node2 = std::make_shared<rclcpp::Node>("node2", "ns");
    auto node3 = std::make_shared<rclcpp::Node>("node3", "ns");
    entities_collector_->add_node(node1->get_node_base_interface());
    entities_collector_->add_node(node2->get_node_base_interface());
    entities_collector_->add_node(node3->get_node_base_interface());
    shared_context = node1->get_node_base_interface()->get_context();
  }
  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_context_t * context = shared_context->get_rcl_context().get();
  EXPECT_EQ(
    RCL_RET_OK,
    rcl_wait_set_init(&wait_set, 100, 100, 100, 100, 100, 100, context, allocator));
  RCLCPP_SCOPE_EXIT({EXPECT_EQ(RCL_RET_OK, rcl_wait_set_fini(&wait_set));});

  auto memory_strategy = rclcpp::memory_strategies::create_default_strategy();
  rclcpp::GuardCondition guard_condition(shared_context);
  rcl_guard_condition_t rcl_guard_condition = guard_condition.get_rcl_guard_condition();

  // Expect weak_node pointers to be cleaned up and used
  entities_collector_->init(&wait_set, memory_strategy, &rcl_guard_condition);
  RCLCPP_SCOPE_EXIT(entities_collector_->fini());
  EXPECT_EQ(0u, entities_collector_->get_number_of_subscriptions());
  EXPECT_EQ(0u, entities_collector_->get_number_of_timers());
  EXPECT_EQ(0u, entities_collector_->get_number_of_services());
  EXPECT_EQ(0u, entities_collector_->get_number_of_clients());

  // Still one for the executor
  EXPECT_EQ(1u, entities_collector_->get_number_of_waitables());
}

class TestWaitable : public rclcpp::Waitable
{
public:
  bool add_to_wait_set(rcl_wait_set_t *) override {return true;}

  bool is_ready(rcl_wait_set_t *) override {return true;}

  void execute() override {}
};

TEST_F(TestStaticExecutorEntitiesCollector, add_remove_node_with_entities) {
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  auto expected_number_of_entities = get_number_of_default_entities(node);
  EXPECT_NE(nullptr, expected_number_of_entities);

  // Create 1 of each entity type
  auto subscription =
    node->create_subscription<test_msgs::msg::Empty>(
    "topic", rclcpp::QoS(10), [](test_msgs::msg::Empty::SharedPtr) {});
  auto timer =
    node->create_wall_timer(std::chrono::seconds(60), []() {});
  auto service =
    node->create_service<test_msgs::srv::Empty>(
    "service",
    [](
      const test_msgs::srv::Empty::Request::SharedPtr,
      test_msgs::srv::Empty::Response::SharedPtr) {});
  auto client = node->create_client<test_msgs::srv::Empty>("service");
  auto waitable = std::make_shared<TestWaitable>();

  // Adding a subscription with rmw_connext_cpp adds another waitable, so we need to get the
  // current number of waitables just before adding the new waitable.
  expected_number_of_entities->waitables = get_number_of_default_entities(node)->waitables;
  node->get_node_waitables_interface()->add_waitable(waitable, nullptr);

  entities_collector_->add_node(node->get_node_base_interface());

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  auto shared_context = node->get_node_base_interface()->get_context();
  rcl_context_t * context = shared_context->get_rcl_context().get();
  EXPECT_EQ(
    RCL_RET_OK,
    rcl_wait_set_init(&wait_set, 100, 100, 100, 100, 100, 100, context, allocator));
  RCLCPP_SCOPE_EXIT({EXPECT_EQ(RCL_RET_OK, rcl_wait_set_fini(&wait_set));});

  auto memory_strategy = rclcpp::memory_strategies::create_default_strategy();

  rclcpp::GuardCondition guard_condition(shared_context);
  rcl_guard_condition_t rcl_guard_condition = guard_condition.get_rcl_guard_condition();

  entities_collector_->init(&wait_set, memory_strategy, &rcl_guard_condition);
  RCLCPP_SCOPE_EXIT(entities_collector_->fini());

  EXPECT_EQ(
    1u + expected_number_of_entities->subscriptions,
    entities_collector_->get_number_of_subscriptions());
  EXPECT_EQ(1u + expected_number_of_entities->timers, entities_collector_->get_number_of_timers());
  EXPECT_EQ(
    1u + expected_number_of_entities->services,
    entities_collector_->get_number_of_services());
  EXPECT_EQ(
    1u + expected_number_of_entities->clients,
    entities_collector_->get_number_of_clients());

  // One extra for the executor
  EXPECT_EQ(
    2u + expected_number_of_entities->waitables,
    entities_collector_->get_number_of_waitables());

  entities_collector_->remove_node(node->get_node_base_interface());
  entities_collector_->init(&wait_set, memory_strategy, &rcl_guard_condition);
  EXPECT_EQ(0u, entities_collector_->get_number_of_subscriptions());
  EXPECT_EQ(0u, entities_collector_->get_number_of_timers());
  EXPECT_EQ(0u, entities_collector_->get_number_of_services());
  EXPECT_EQ(0u, entities_collector_->get_number_of_clients());
  // Still one for the executor
  EXPECT_EQ(1u, entities_collector_->get_number_of_waitables());
}
