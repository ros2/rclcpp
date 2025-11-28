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

#include "test_msgs/msg/empty.hpp"
#include "test_msgs/srv/empty.hpp"

#include "../../mocking_utils/patch.hpp"
#include "../../utils/rclcpp_gtest_macros.hpp"

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
  node->for_each_callback_group(
    [&number_of_entities](rclcpp::CallbackGroup::SharedPtr group)
    {
      if (!group->can_be_taken_from().load()) {
        return;
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
    });

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
  RCPPUTILS_SCOPE_EXIT({EXPECT_EQ(RCL_RET_OK, rcl_wait_set_fini(&wait_set));});

  rclcpp::GuardCondition guard_condition(shared_context);

  // Check memory strategy is nullptr
  rclcpp::memory_strategy::MemoryStrategy::SharedPtr memory_strategy = nullptr;
  EXPECT_THROW(
    entities_collector_->init(&wait_set, memory_strategy),
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
  RCPPUTILS_SCOPE_EXIT({EXPECT_EQ(RCL_RET_OK, rcl_wait_set_fini(&wait_set));});

  auto memory_strategy = rclcpp::memory_strategies::create_default_strategy();
  rclcpp::GuardCondition guard_condition(shared_context);

  entities_collector_->init(&wait_set, memory_strategy);
  RCPPUTILS_SCOPE_EXIT(entities_collector_->fini());
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
  entities_collector_->init(&wait_set, memory_strategy);
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
  RCPPUTILS_SCOPE_EXIT({EXPECT_EQ(RCL_RET_OK, rcl_wait_set_fini(&wait_set));});

  auto memory_strategy = rclcpp::memory_strategies::create_default_strategy();
  rclcpp::GuardCondition guard_condition(shared_context);

  // Expect weak_node pointers to be cleaned up and used
  entities_collector_->init(&wait_set, memory_strategy);
  RCPPUTILS_SCOPE_EXIT(entities_collector_->fini());
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
  void add_to_wait_set(rcl_wait_set_t &) override {}

  bool is_ready(const rcl_wait_set_t &) override {return true;}

  std::shared_ptr<void>
  take_data() override
  {
    return nullptr;
  }
  void
  execute(const std::shared_ptr<void> &) override {}
};

TEST_F(TestStaticExecutorEntitiesCollector, add_remove_node_with_entities) {
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  auto expected_number_of_entities = get_number_of_default_entities(node);
  EXPECT_NE(nullptr, expected_number_of_entities);

  // Create 1 of each entity type
  auto subscription =
    node->create_subscription<test_msgs::msg::Empty>(
    "topic", rclcpp::QoS(10), [](test_msgs::msg::Empty::ConstSharedPtr) {});
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

  // Adding a subscription could add another waitable, so we need to get the
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
  RCPPUTILS_SCOPE_EXIT({EXPECT_EQ(RCL_RET_OK, rcl_wait_set_fini(&wait_set));});

  auto memory_strategy = rclcpp::memory_strategies::create_default_strategy();

  rclcpp::GuardCondition guard_condition(shared_context);

  entities_collector_->init(&wait_set, memory_strategy);
  RCPPUTILS_SCOPE_EXIT(entities_collector_->fini());

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
  entities_collector_->init(&wait_set, memory_strategy);
  EXPECT_EQ(0u, entities_collector_->get_number_of_subscriptions());
  EXPECT_EQ(0u, entities_collector_->get_number_of_timers());
  EXPECT_EQ(0u, entities_collector_->get_number_of_services());
  EXPECT_EQ(0u, entities_collector_->get_number_of_clients());
  // Still one for the executor
  EXPECT_EQ(1u, entities_collector_->get_number_of_waitables());
}

TEST_F(TestStaticExecutorEntitiesCollector, add_callback_group) {
  auto node = std::make_shared<rclcpp::Node>("node1", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  entities_collector_->add_callback_group(cb_group, node->get_node_base_interface());
  ASSERT_EQ(entities_collector_->get_all_callback_groups().size(), 1u);
}

TEST_F(TestStaticExecutorEntitiesCollector, add_callback_group_after_add_node) {
  auto node = std::make_shared<rclcpp::Node>("node1", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  entities_collector_->add_node(node->get_node_base_interface());
  RCLCPP_EXPECT_THROW_EQ(
    entities_collector_->add_callback_group(cb_group, node->get_node_base_interface()),
    std::runtime_error("Callback group has already been added to an executor."));
}

TEST_F(TestStaticExecutorEntitiesCollector, add_callback_group_twice) {
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

TEST_F(TestStaticExecutorEntitiesCollector, prepare_wait_set_rcl_wait_set_clear_error) {
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  entities_collector_->add_node(node->get_node_base_interface());

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  auto shared_context = node->get_node_base_interface()->get_context();
  rcl_context_t * context = shared_context->get_rcl_context().get();
  EXPECT_EQ(
    RCL_RET_OK,
    rcl_wait_set_init(&wait_set, 100, 100, 100, 100, 100, 100, context, allocator));
  RCPPUTILS_SCOPE_EXIT({EXPECT_EQ(RCL_RET_OK, rcl_wait_set_fini(&wait_set));});

  auto memory_strategy = rclcpp::memory_strategies::create_default_strategy();
  rclcpp::GuardCondition guard_condition(shared_context);

  entities_collector_->init(&wait_set, memory_strategy);
  RCPPUTILS_SCOPE_EXIT(entities_collector_->fini());

  {
    auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_wait_set_clear, RCL_RET_ERROR);
    std::shared_ptr<void> data = entities_collector_->take_data();
    RCLCPP_EXPECT_THROW_EQ(
      entities_collector_->execute(data),
      std::runtime_error("Couldn't clear wait set"));
  }

  EXPECT_TRUE(entities_collector_->remove_node(node->get_node_base_interface()));
}

TEST_F(TestStaticExecutorEntitiesCollector, prepare_wait_set_rcl_wait_set_resize_error) {
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  entities_collector_->add_node(node->get_node_base_interface());

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  auto shared_context = node->get_node_base_interface()->get_context();
  rcl_context_t * context = shared_context->get_rcl_context().get();
  EXPECT_EQ(
    RCL_RET_OK,
    rcl_wait_set_init(&wait_set, 100, 100, 100, 100, 100, 100, context, allocator));
  RCPPUTILS_SCOPE_EXIT({EXPECT_EQ(RCL_RET_OK, rcl_wait_set_fini(&wait_set));});

  auto memory_strategy = rclcpp::memory_strategies::create_default_strategy();
  rclcpp::GuardCondition guard_condition(shared_context);

  entities_collector_->init(&wait_set, memory_strategy);
  RCPPUTILS_SCOPE_EXIT(entities_collector_->fini());

  {
    auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_wait_set_resize, RCL_RET_ERROR);
    std::shared_ptr<void> data = entities_collector_->take_data();
    RCLCPP_EXPECT_THROW_EQ(
      entities_collector_->execute(data),
      std::runtime_error("Couldn't resize the wait set: error not set"));
  }

  EXPECT_TRUE(entities_collector_->remove_node(node->get_node_base_interface()));
}

TEST_F(TestStaticExecutorEntitiesCollector, refresh_wait_set_not_initialized) {
  RCLCPP_EXPECT_THROW_EQ(
    entities_collector_->refresh_wait_set(std::chrono::nanoseconds(1000)),
    std::runtime_error("Couldn't clear wait set"));
  rcl_reset_error();
}

TEST_F(TestStaticExecutorEntitiesCollector, refresh_wait_set_rcl_wait_failed) {
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  entities_collector_->add_node(node->get_node_base_interface());

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  auto shared_context = node->get_node_base_interface()->get_context();
  rcl_context_t * context = shared_context->get_rcl_context().get();
  EXPECT_EQ(
    RCL_RET_OK,
    rcl_wait_set_init(&wait_set, 100, 100, 100, 100, 100, 100, context, allocator));
  RCPPUTILS_SCOPE_EXIT({EXPECT_EQ(RCL_RET_OK, rcl_wait_set_fini(&wait_set));});

  auto memory_strategy = rclcpp::memory_strategies::create_default_strategy();
  rclcpp::GuardCondition guard_condition(shared_context);

  entities_collector_->init(&wait_set, memory_strategy);
  RCPPUTILS_SCOPE_EXIT(entities_collector_->fini());

  {
    auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_wait, RCL_RET_ERROR);
    RCLCPP_EXPECT_THROW_EQ(
      entities_collector_->refresh_wait_set(std::chrono::nanoseconds(1000)),
      std::runtime_error("rcl_wait() failed: error not set"));
  }

  EXPECT_TRUE(entities_collector_->remove_node(node->get_node_base_interface()));
}

TEST_F(TestStaticExecutorEntitiesCollector, refresh_wait_set_add_handles_to_wait_set_failed) {
  auto node = std::make_shared<rclcpp::Node>("node", "ns");

  // Create 1 of each entity type
  auto subscription =
    node->create_subscription<test_msgs::msg::Empty>(
    "topic", rclcpp::QoS(10), [](test_msgs::msg::Empty::ConstSharedPtr) {});
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

  node->get_node_waitables_interface()->add_waitable(waitable, nullptr);

  entities_collector_->add_node(node->get_node_base_interface());

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  auto shared_context = node->get_node_base_interface()->get_context();
  rcl_context_t * context = shared_context->get_rcl_context().get();
  EXPECT_EQ(
    RCL_RET_OK,
    rcl_wait_set_init(&wait_set, 100, 100, 100, 100, 100, 100, context, allocator));
  RCPPUTILS_SCOPE_EXIT({EXPECT_EQ(RCL_RET_OK, rcl_wait_set_fini(&wait_set));});

  auto memory_strategy = rclcpp::memory_strategies::create_default_strategy();

  rclcpp::GuardCondition guard_condition(shared_context);

  entities_collector_->init(&wait_set, memory_strategy);
  RCPPUTILS_SCOPE_EXIT(entities_collector_->fini());

  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_wait_set_add_subscription,
      RCL_RET_ERROR);
    RCLCPP_EXPECT_THROW_EQ(
      entities_collector_->refresh_wait_set(std::chrono::nanoseconds(1000)),
      std::runtime_error("Couldn't fill wait set"));
  }

  entities_collector_->remove_node(node->get_node_base_interface());
}

TEST_F(TestStaticExecutorEntitiesCollector, add_to_wait_set_nullptr) {
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  entities_collector_->add_node(node->get_node_base_interface());

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  auto shared_context = node->get_node_base_interface()->get_context();
  rcl_context_t * context = shared_context->get_rcl_context().get();
  EXPECT_EQ(
    RCL_RET_OK,
    rcl_wait_set_init(&wait_set, 100, 100, 100, 100, 100, 100, context, allocator));
  RCPPUTILS_SCOPE_EXIT({EXPECT_EQ(RCL_RET_OK, rcl_wait_set_fini(&wait_set));});

  auto memory_strategy = rclcpp::memory_strategies::create_default_strategy();
  rclcpp::GuardCondition guard_condition(shared_context);

  entities_collector_->init(&wait_set, memory_strategy);
  RCPPUTILS_SCOPE_EXIT(entities_collector_->fini());

  EXPECT_TRUE(entities_collector_->remove_node(node->get_node_base_interface()));
}

TEST_F(TestStaticExecutorEntitiesCollector, fill_memory_strategy_invalid_group) {
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  entities_collector_->add_node(node->get_node_base_interface());

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  auto shared_context = node->get_node_base_interface()->get_context();
  rcl_context_t * context = shared_context->get_rcl_context().get();
  EXPECT_EQ(
    RCL_RET_OK,
    rcl_wait_set_init(&wait_set, 100, 100, 100, 100, 100, 100, context, allocator));
  RCPPUTILS_SCOPE_EXIT({EXPECT_EQ(RCL_RET_OK, rcl_wait_set_fini(&wait_set));});

  auto memory_strategy = rclcpp::memory_strategies::create_default_strategy();
  rclcpp::GuardCondition guard_condition(shared_context);

  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  entities_collector_->add_callback_group(cb_group, node->get_node_base_interface());
  ASSERT_EQ(entities_collector_->get_all_callback_groups().size(), 2u);

  cb_group.reset();

  entities_collector_->init(&wait_set, memory_strategy);
  RCPPUTILS_SCOPE_EXIT(entities_collector_->fini());
  ASSERT_EQ(entities_collector_->get_all_callback_groups().size(), 1u);

  EXPECT_TRUE(entities_collector_->remove_node(node->get_node_base_interface()));
}

TEST_F(TestStaticExecutorEntitiesCollector, remove_callback_group_after_node) {
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

TEST_F(TestStaticExecutorEntitiesCollector, remove_callback_group_twice) {
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

TEST_F(TestStaticExecutorEntitiesCollector, remove_node_opposite_order) {
  auto node1 = std::make_shared<rclcpp::Node>("node1", "ns");
  EXPECT_NO_THROW(entities_collector_->add_node(node1->get_node_base_interface()));

  auto node2 = std::make_shared<rclcpp::Node>("node2", "ns");
  EXPECT_NO_THROW(entities_collector_->add_node(node2->get_node_base_interface()));

  EXPECT_TRUE(entities_collector_->remove_node(node2->get_node_base_interface()));
}

TEST_F(
  TestStaticExecutorEntitiesCollector,
  add_callback_groups_from_nodes_associated_to_executor_add) {
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  entities_collector_->add_callback_group(cb_group, node->get_node_base_interface());
  entities_collector_->add_node(node->get_node_base_interface());

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  auto shared_context = node->get_node_base_interface()->get_context();
  rcl_context_t * context = shared_context->get_rcl_context().get();
  EXPECT_EQ(
    RCL_RET_OK,
    rcl_wait_set_init(&wait_set, 100, 100, 100, 100, 100, 100, context, allocator));
  RCPPUTILS_SCOPE_EXIT({EXPECT_EQ(RCL_RET_OK, rcl_wait_set_fini(&wait_set));});

  auto memory_strategy = rclcpp::memory_strategies::create_default_strategy();
  rclcpp::GuardCondition guard_condition(shared_context);

  entities_collector_->init(&wait_set, memory_strategy);
  RCPPUTILS_SCOPE_EXIT(entities_collector_->fini());

  cb_group->get_associated_with_executor_atomic().exchange(false);
  std::shared_ptr<void> data = entities_collector_->take_data();
  entities_collector_->execute(data);

  EXPECT_TRUE(entities_collector_->remove_node(node->get_node_base_interface()));
}
