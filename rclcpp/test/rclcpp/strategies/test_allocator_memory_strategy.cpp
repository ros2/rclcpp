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

#include <list>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "gtest/gtest.h"

#include "rclcpp/scope_exit.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"
#include "test_msgs/msg/empty.hpp"
#include "test_msgs/srv/empty.hpp"

using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;
using WeakNodeList = std::list<rclcpp::node_interfaces::NodeBaseInterface::WeakPtr>;

static bool test_waitable_result = false;

/**
 * Mock Waitable class, with a globally setable boolean result.
 */
class TestWaitable : public rclcpp::Waitable
{
public:
  bool add_to_wait_set(rcl_wait_set_t *) override
  {
    return test_waitable_result;
  }

  bool is_ready(rcl_wait_set_t *) override
  {
    return test_waitable_result;
  }

  void execute() override {}
};

struct RclWaitSetSizes
{
  size_t size_of_subscriptions = 0;
  size_t size_of_guard_conditions = 0;
  size_t size_of_timers = 0;
  size_t size_of_clients = 0;
  size_t size_of_services = 0;
  size_t size_of_events = 0;
  size_t size_of_waitables = 0;
};

// For a standard rclcpp node, this should be more than enough capacity for each type.
RclWaitSetSizes SufficientWaitSetCapacities()
{
  return {100, 100, 100, 100, 100, 100, 100};
}

class TestAllocatorMemoryStrategy : public ::testing::Test
{
public:
  TestAllocatorMemoryStrategy()
  : allocator_memory_strategy_(nullptr),
    num_ready_subscriptions_of_basic_node_(0),
    num_ready_services_of_basic_node_(0),
    num_ready_events_of_basic_node_(0),
    num_ready_clients_of_basic_node_(0),
    num_guard_conditions_of_basic_node_(0),
    num_ready_timers_of_basic_node_(0),
    num_waitables_of_basic_node_(0) {}

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    allocator_ = std::make_shared<std::allocator<void>>();
    // Even though this is just using a basic allocator, the custom allocator constructor was
    // not covered in general testing.
    allocator_memory_strategy_ = std::make_shared<AllocatorMemoryStrategy<>>(allocator_);

    auto basic_node = std::make_shared<rclcpp::Node>("basic_node", "ns");
    auto dummy_memory_strategy = std::make_shared<AllocatorMemoryStrategy<>>();
    WeakNodeList nodes;
    nodes.push_back(basic_node->get_node_base_interface());
    dummy_memory_strategy->collect_entities(nodes);

    num_ready_subscriptions_of_basic_node_ =
      dummy_memory_strategy->number_of_ready_subscriptions();
    num_ready_services_of_basic_node_ = dummy_memory_strategy->number_of_ready_services();
    num_ready_events_of_basic_node_ = dummy_memory_strategy->number_of_ready_events();
    num_ready_clients_of_basic_node_ = dummy_memory_strategy->number_of_ready_clients();
    num_guard_conditions_of_basic_node_ = dummy_memory_strategy->number_of_guard_conditions();
    num_ready_timers_of_basic_node_ = dummy_memory_strategy->number_of_ready_timers();
    num_waitables_of_basic_node_ = dummy_memory_strategy->number_of_waitables();
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<AllocatorMemoryStrategy<>> allocator_memory_strategy()
  {
    return allocator_memory_strategy_;
  }

  //
  // Convience methods, but it adds entities to vectors so the weak pointers kept by the node
  // interfaces remain alive and valid
  //

  std::shared_ptr<rclcpp::Node> create_node_with_subscription(
    const std::string & name, bool with_exclusive_callback_group)
  {
    auto subscription_callback = [](const test_msgs::msg::Empty::SharedPtr) {};
    const rclcpp::QoS qos(10);
    auto node_with_subscription = std::make_shared<rclcpp::Node>(name, "ns");

    rclcpp::SubscriptionOptions subscription_options;

    if (with_exclusive_callback_group) {
      auto callback_group =
        node_with_subscription->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

      subscription_options.callback_group = callback_group;
      callback_groups_.push_back(callback_group);
    }

    auto subscription = node_with_subscription->create_subscription<
      test_msgs::msg::Empty, decltype(subscription_callback)>(
      "topic", qos, std::move(subscription_callback), subscription_options);
    subscriptions_.push_back(subscription);

    return node_with_subscription;
  }

  std::shared_ptr<rclcpp::Node> create_node_with_service(
    const std::string & name, bool with_exclusive_callback_group = false)
  {
    auto service_callback =
      [](const test_msgs::srv::Empty::Request::SharedPtr,
        test_msgs::srv::Empty::Response::SharedPtr) {};
    auto node_with_service = std::make_shared<rclcpp::Node>(name, "ns");

    rclcpp::CallbackGroup::SharedPtr callback_group = nullptr;

    if (with_exclusive_callback_group) {
      callback_group =
        node_with_service->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

      callback_groups_.push_back(callback_group);
    }

    services_.push_back(
      node_with_service->create_service<test_msgs::srv::Empty>(
        "service", std::move(service_callback), rmw_qos_profile_services_default, callback_group));
    return node_with_service;
  }

  std::shared_ptr<rclcpp::Node> create_node_with_client(
    const std::string & name, bool with_exclusive_callback_group = false)
  {
    auto node_with_client = std::make_shared<rclcpp::Node>(name, "ns");
    rclcpp::CallbackGroup::SharedPtr callback_group = nullptr;
    if (with_exclusive_callback_group) {
      callback_group =
        node_with_client->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

      callback_groups_.push_back(callback_group);
    }

    clients_.push_back(
      node_with_client->create_client<test_msgs::srv::Empty>(
        "service", rmw_qos_profile_services_default, callback_group));
    return node_with_client;
  }

  std::shared_ptr<rclcpp::Node> create_node_with_timer(
    const std::string & name, bool with_exclusive_callback_group = false)
  {
    auto timer_callback = []() {};
    auto node_with_timer = std::make_shared<rclcpp::Node>(name, "ns");

    rclcpp::CallbackGroup::SharedPtr callback_group = nullptr;
    if (with_exclusive_callback_group) {
      callback_group =
        node_with_timer->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
      callback_groups_.push_back(callback_group);
    }

    timers_.push_back(
      node_with_timer->create_wall_timer(
        std::chrono::milliseconds(1), timer_callback, callback_group));
    return node_with_timer;
  }

  size_t number_of_ready_subscriptions_in_addition_to_basic_node() const
  {
    return
      allocator_memory_strategy_->number_of_ready_subscriptions() -
      num_ready_subscriptions_of_basic_node_;
  }

  size_t number_of_ready_services_in_addition_to_basic_node() const
  {
    return
      allocator_memory_strategy_->number_of_ready_services() -
      num_ready_services_of_basic_node_;
  }

  size_t number_of_ready_events_in_addition_to_basic_node() const
  {
    return
      allocator_memory_strategy_->number_of_ready_events() -
      num_ready_events_of_basic_node_;
  }

  size_t number_of_ready_clients_in_addition_to_basic_node() const
  {
    return
      allocator_memory_strategy_->number_of_ready_clients() -
      num_ready_clients_of_basic_node_;
  }

  size_t number_of_guard_conditions_in_addition_to_basic_node() const
  {
    return
      allocator_memory_strategy_->number_of_guard_conditions() -
      num_guard_conditions_of_basic_node_;
  }

  size_t number_of_ready_timers_in_addition_to_basic_node() const
  {
    return
      allocator_memory_strategy_->number_of_ready_timers() -
      num_ready_timers_of_basic_node_;
  }

  size_t number_of_waitables_in_addition_to_basic_node() const
  {
    return
      allocator_memory_strategy_->number_of_waitables() -
      num_waitables_of_basic_node_;
  }

  ::testing::AssertionResult TestNumberOfEntitiesAfterCollection(
    std::shared_ptr<rclcpp::Node> node,
    const RclWaitSetSizes & expected)
  {
    WeakNodeList nodes;
    nodes.push_back(node->get_node_base_interface());

    allocator_memory_strategy()->collect_entities(nodes);
    EXPECT_EQ(
      expected.size_of_subscriptions, number_of_ready_subscriptions_in_addition_to_basic_node());
    EXPECT_EQ(
      expected.size_of_guard_conditions, number_of_guard_conditions_in_addition_to_basic_node());
    EXPECT_EQ(expected.size_of_timers, number_of_ready_timers_in_addition_to_basic_node());
    EXPECT_EQ(expected.size_of_clients, number_of_ready_clients_in_addition_to_basic_node());
    EXPECT_EQ(expected.size_of_services, number_of_ready_services_in_addition_to_basic_node());
    EXPECT_EQ(expected.size_of_events, number_of_ready_events_in_addition_to_basic_node());
    EXPECT_EQ(expected.size_of_waitables, number_of_waitables_in_addition_to_basic_node());
    if (::testing::Test::HasFailure()) {
      return ::testing::AssertionFailure() <<
             "Expected number of entities did not match actual counts";
    }
    return ::testing::AssertionSuccess();
  }

  ::testing::AssertionResult TestAddHandlesToWaitSet(
    std::shared_ptr<rclcpp::Node> node,
    const RclWaitSetSizes & insufficient_capacities)
  {
    WeakNodeList nodes;
    nodes.push_back(node->get_node_base_interface());
    allocator_memory_strategy()->collect_entities(nodes);

    auto context = node->get_node_base_interface()->get_context();
    rcl_context_t * rcl_context = context->get_rcl_context().get();
    rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
    rcl_allocator_t allocator = rcl_get_default_allocator();
    RclWaitSetSizes sufficient_capacities = SufficientWaitSetCapacities();

    rcl_ret_t ret = rcl_wait_set_init(
      &wait_set,
      sufficient_capacities.size_of_subscriptions,
      sufficient_capacities.size_of_guard_conditions,
      sufficient_capacities.size_of_timers,
      sufficient_capacities.size_of_clients,
      sufficient_capacities.size_of_services,
      sufficient_capacities.size_of_events,
      rcl_context,
      allocator);
    if (ret != RCL_RET_OK) {
      return ::testing::AssertionFailure() <<
             "Calling rcl_wait_set_init() with expected sufficient capacities failed";
    }

    RCLCPP_SCOPE_EXIT(
    {
      EXPECT_EQ(RCL_RET_OK, rcl_wait_set_fini(&wait_set));
    });

    if (!allocator_memory_strategy()->add_handles_to_wait_set(&wait_set)) {
      return ::testing::AssertionFailure() <<
             "Calling add_handles_to_wait_set() with a wait_set with expected sufficient capacities"
             " failed";
    }

    rcl_wait_set_t wait_set_no_capacity = rcl_get_zero_initialized_wait_set();
    ret = rcl_wait_set_init(
      &wait_set_no_capacity,
      insufficient_capacities.size_of_subscriptions,
      insufficient_capacities.size_of_guard_conditions,
      insufficient_capacities.size_of_timers,
      insufficient_capacities.size_of_clients,
      insufficient_capacities.size_of_services,
      insufficient_capacities.size_of_events,
      rcl_context,
      allocator);

    if (ret != RCL_RET_OK) {
      return ::testing::AssertionFailure() <<
             "Calling rcl_wait_set_init() with expected insufficient capacities failed";
    }

    RCLCPP_SCOPE_EXIT(
    {
      EXPECT_EQ(RCL_RET_OK, rcl_wait_set_fini(&wait_set_no_capacity));
    });

    if (allocator_memory_strategy()->add_handles_to_wait_set(&wait_set_no_capacity)) {
      return ::testing::AssertionFailure() <<
             "Calling add_handles_to_wait_set() with a wait_set with insufficient capacities"
             " unexpectedly succeeded";
    }
    return ::testing::AssertionSuccess();
  }

  std::vector<std::shared_ptr<rclcpp::CallbackGroup>> callback_groups_;

private:
  std::shared_ptr<std::allocator<void>> allocator_;
  std::shared_ptr<AllocatorMemoryStrategy<>> allocator_memory_strategy_;

  size_t num_ready_subscriptions_of_basic_node_;
  size_t num_ready_services_of_basic_node_;
  size_t num_ready_events_of_basic_node_;
  size_t num_ready_clients_of_basic_node_;
  size_t num_guard_conditions_of_basic_node_;
  size_t num_ready_timers_of_basic_node_;
  size_t num_waitables_of_basic_node_;

  // These are generally kept as weak pointers in the rclcpp::Node interfaces, so they need to be
  // owned by this class.
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
  std::vector<rclcpp::ClientBase::SharedPtr> clients_;
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
  std::vector<rclcpp::Waitable::SharedPtr> waitables_;
};

TEST_F(TestAllocatorMemoryStrategy, construct_destruct) {
  WeakNodeList nodes;
  auto basic_node = std::make_shared<rclcpp::Node>("node", "ns");
  nodes.push_back(basic_node->get_node_base_interface());
  allocator_memory_strategy()->collect_entities(nodes);
  EXPECT_EQ(0u, number_of_ready_subscriptions_in_addition_to_basic_node());
  EXPECT_EQ(0u, number_of_ready_services_in_addition_to_basic_node());
  EXPECT_EQ(0u, number_of_ready_events_in_addition_to_basic_node());
  EXPECT_EQ(0u, number_of_ready_clients_in_addition_to_basic_node());
  EXPECT_EQ(0u, number_of_guard_conditions_in_addition_to_basic_node());
  EXPECT_EQ(0u, number_of_ready_timers_in_addition_to_basic_node());
  EXPECT_EQ(0u, number_of_waitables_in_addition_to_basic_node());
}

TEST_F(TestAllocatorMemoryStrategy, add_remove_guard_conditions) {
  rcl_guard_condition_t guard_condition1 = rcl_get_zero_initialized_guard_condition();
  rcl_guard_condition_t guard_condition2 = rcl_get_zero_initialized_guard_condition();
  rcl_guard_condition_t guard_condition3 = rcl_get_zero_initialized_guard_condition();

  EXPECT_NO_THROW(allocator_memory_strategy()->add_guard_condition(&guard_condition1));
  EXPECT_NO_THROW(allocator_memory_strategy()->add_guard_condition(&guard_condition2));
  EXPECT_NO_THROW(allocator_memory_strategy()->add_guard_condition(&guard_condition3));
  EXPECT_EQ(3u, allocator_memory_strategy()->number_of_guard_conditions());

  // Adding a second time should not add to vector
  EXPECT_NO_THROW(allocator_memory_strategy()->add_guard_condition(&guard_condition1));
  EXPECT_NO_THROW(allocator_memory_strategy()->add_guard_condition(&guard_condition2));
  EXPECT_NO_THROW(allocator_memory_strategy()->add_guard_condition(&guard_condition3));
  EXPECT_EQ(3u, allocator_memory_strategy()->number_of_guard_conditions());

  EXPECT_NO_THROW(allocator_memory_strategy()->remove_guard_condition(&guard_condition1));
  EXPECT_NO_THROW(allocator_memory_strategy()->remove_guard_condition(&guard_condition2));
  EXPECT_NO_THROW(allocator_memory_strategy()->remove_guard_condition(&guard_condition3));
  EXPECT_EQ(0u, allocator_memory_strategy()->number_of_guard_conditions());

  // Removing second time should have no effect
  EXPECT_NO_THROW(allocator_memory_strategy()->remove_guard_condition(&guard_condition1));
  EXPECT_NO_THROW(allocator_memory_strategy()->remove_guard_condition(&guard_condition2));
  EXPECT_NO_THROW(allocator_memory_strategy()->remove_guard_condition(&guard_condition3));
  EXPECT_EQ(0u, allocator_memory_strategy()->number_of_guard_conditions());
}

TEST_F(TestAllocatorMemoryStrategy, add_remove_waitables) {
  EXPECT_THROW(allocator_memory_strategy()->add_waitable_handle(nullptr), std::runtime_error);
  EXPECT_EQ(0u, allocator_memory_strategy()->number_of_waitables());

  rclcpp::Waitable::SharedPtr waitable = std::make_shared<TestWaitable>();
  EXPECT_NO_THROW(allocator_memory_strategy()->add_waitable_handle(waitable));
  EXPECT_EQ(1u, allocator_memory_strategy()->number_of_waitables());

  EXPECT_NO_THROW(allocator_memory_strategy()->clear_handles());
  EXPECT_EQ(0u, allocator_memory_strategy()->number_of_waitables());
}

TEST_F(TestAllocatorMemoryStrategy, number_of_entities_with_subscription) {
  RclWaitSetSizes expected_sizes = {};
  expected_sizes.size_of_subscriptions = 1;
  auto node_with_subscription = create_node_with_subscription("subscription_node", false);
  EXPECT_TRUE(TestNumberOfEntitiesAfterCollection(node_with_subscription, expected_sizes));
}

TEST_F(TestAllocatorMemoryStrategy, number_of_entities_with_service) {
  RclWaitSetSizes expected_sizes = {};
  expected_sizes.size_of_services = 1;
  auto node_with_service = create_node_with_service("service_node");
  EXPECT_TRUE(TestNumberOfEntitiesAfterCollection(node_with_service, expected_sizes));
}

TEST_F(TestAllocatorMemoryStrategy, number_of_entities_with_client) {
  RclWaitSetSizes expected_sizes = {};
  expected_sizes.size_of_clients = 1;
  auto node_with_client = create_node_with_client("client_node");
  EXPECT_TRUE(TestNumberOfEntitiesAfterCollection(node_with_client, expected_sizes));
}

TEST_F(TestAllocatorMemoryStrategy, number_of_entities_with_timer) {
  RclWaitSetSizes expected_sizes = {};
  expected_sizes.size_of_timers = 1;
  auto node_with_timer = create_node_with_timer("timer_node");
  EXPECT_TRUE(TestNumberOfEntitiesAfterCollection(node_with_timer, expected_sizes));
}

TEST_F(TestAllocatorMemoryStrategy, add_handles_to_wait_set_bad_arguments) {
  auto node = create_node_with_subscription("subscription_node", false);
  WeakNodeList nodes;
  nodes.push_back(node->get_node_base_interface());
  allocator_memory_strategy()->collect_entities(nodes);
  EXPECT_FALSE(allocator_memory_strategy()->add_handles_to_wait_set(nullptr));
  EXPECT_TRUE(rcl_error_is_set());
  rcl_reset_error();
}

TEST_F(TestAllocatorMemoryStrategy, add_handles_to_wait_set_subscription) {
  auto node_with_subscription = create_node_with_subscription("subscription_node", false);
  RclWaitSetSizes insufficient_capacities = SufficientWaitSetCapacities();
  insufficient_capacities.size_of_subscriptions = 0;
  EXPECT_TRUE(TestAddHandlesToWaitSet(node_with_subscription, insufficient_capacities));
}

TEST_F(TestAllocatorMemoryStrategy, add_handles_to_wait_set_service) {
  auto node_with_service = create_node_with_service("service_node");
  RclWaitSetSizes insufficient_capacities = SufficientWaitSetCapacities();
  insufficient_capacities.size_of_services = 0;
  EXPECT_TRUE(TestAddHandlesToWaitSet(node_with_service, insufficient_capacities));
}

TEST_F(TestAllocatorMemoryStrategy, add_handles_to_wait_set_client) {
  auto node_with_client = create_node_with_client("client_node");
  RclWaitSetSizes insufficient_capacities = SufficientWaitSetCapacities();
  insufficient_capacities.size_of_clients = 0;
  EXPECT_TRUE(TestAddHandlesToWaitSet(node_with_client, insufficient_capacities));
}

TEST_F(TestAllocatorMemoryStrategy, add_handles_to_wait_set_guard_condition) {
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  rcl_guard_condition_t guard_condition = rcl_get_zero_initialized_guard_condition();
  auto context = node->get_node_base_interface()->get_context();
  rcl_context_t * rcl_context = context->get_rcl_context().get();
  rcl_guard_condition_options_t guard_condition_options = {
    rcl_get_default_allocator()};

  EXPECT_EQ(
    RCL_RET_OK,
    rcl_guard_condition_init(&guard_condition, rcl_context, guard_condition_options));
  RCLCPP_SCOPE_EXIT(
  {
    EXPECT_EQ(RCL_RET_OK, rcl_guard_condition_fini(&guard_condition));
  });

  allocator_memory_strategy()->add_guard_condition(&guard_condition);
  RclWaitSetSizes insufficient_capacities = SufficientWaitSetCapacities();
  insufficient_capacities.size_of_guard_conditions = 0;
  EXPECT_TRUE(TestAddHandlesToWaitSet(node, insufficient_capacities));
}

TEST_F(TestAllocatorMemoryStrategy, add_handles_to_wait_set_timer) {
  auto node_with_timer = create_node_with_timer("timer_node");
  RclWaitSetSizes insufficient_capacities = SufficientWaitSetCapacities();
  insufficient_capacities.size_of_timers = 0;
  EXPECT_TRUE(TestAddHandlesToWaitSet(node_with_timer, insufficient_capacities));
}

TEST_F(TestAllocatorMemoryStrategy, add_handles_to_wait_set_waitable) {
  rclcpp::Waitable::SharedPtr waitable = std::make_shared<TestWaitable>();
  EXPECT_NO_THROW(allocator_memory_strategy()->add_waitable_handle(waitable));
  EXPECT_EQ(1u, allocator_memory_strategy()->number_of_waitables());

  test_waitable_result = true;
  EXPECT_TRUE(allocator_memory_strategy()->add_handles_to_wait_set(nullptr));

  test_waitable_result = false;
  EXPECT_FALSE(allocator_memory_strategy()->add_handles_to_wait_set(nullptr));

  // This calls TestWaitable's functions, so rcl errors are not set
  EXPECT_FALSE(rcl_error_is_set());
}
