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
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "gtest/gtest.h"

#include "rclcpp/strategies/allocator_memory_strategy.hpp"
#include "rcpputils/scope_exit.hpp"
#include "test_msgs/msg/empty.hpp"
#include "test_msgs/srv/empty.hpp"

using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;
typedef std::map<rclcpp::CallbackGroup::WeakPtr,
    rclcpp::node_interfaces::NodeBaseInterface::WeakPtr,
    std::owner_less<rclcpp::CallbackGroup::WeakPtr>> WeakCallbackGroupsToNodesMap;

static bool test_waitable_result = false;

/**
 * Mock Waitable class, with a globally setable boolean result.
 */
class TestWaitable : public rclcpp::Waitable
{
public:
  void add_to_wait_set(rcl_wait_set_t *) override
  {
    if (!test_waitable_result) {
      throw std::runtime_error("TestWaitable add_to_wait_set failed");
    }
  }

  bool is_ready(rcl_wait_set_t *) override
  {
    return test_waitable_result;
  }

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
  : allocator_memory_strategy_(nullptr) {}

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    allocator_ = std::make_shared<std::allocator<void>>();
    // Even though this is just using a basic allocator, the custom allocator constructor was
    // not covered in general testing.
    allocator_memory_strategy_ = std::make_shared<AllocatorMemoryStrategy<>>(allocator_);
  }

  void TearDown() override
  {
    allocator_memory_strategy_.reset();
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

  std::shared_ptr<rclcpp::Node> create_node_with_disabled_callback_groups(const std::string & name)
  {
    auto node = std::make_shared<rclcpp::Node>(name, "ns");

    node->for_each_callback_group(
      [](rclcpp::CallbackGroup::SharedPtr group)
      {
        group->can_be_taken_from() = false;
      });
    return node;
  }

  std::shared_ptr<rclcpp::Node> create_node_with_subscription(const std::string & name)
  {
    auto subscription_callback = [](test_msgs::msg::Empty::ConstSharedPtr) {};
    const rclcpp::QoS qos(10);
    auto node_with_subscription = create_node_with_disabled_callback_groups(name);

    rclcpp::SubscriptionOptions subscription_options;

    auto callback_group =
      node_with_subscription->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    subscription_options.callback_group = callback_group;
    callback_groups_.push_back(callback_group);

    auto subscription = node_with_subscription->create_subscription<
      test_msgs::msg::Empty, decltype(subscription_callback)>(
      "topic", qos, std::move(subscription_callback), subscription_options);
    subscriptions_.push_back(subscription);

    return node_with_subscription;
  }

  std::shared_ptr<rclcpp::Node> create_node_with_service(const std::string & name)
  {
    auto service_callback =
      [](const test_msgs::srv::Empty::Request::SharedPtr,
        test_msgs::srv::Empty::Response::SharedPtr) {};
    auto node_with_service = create_node_with_disabled_callback_groups(name);

    auto callback_group =
      node_with_service->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    callback_groups_.push_back(callback_group);

    services_.push_back(
      node_with_service->create_service<test_msgs::srv::Empty>(
        "service", std::move(service_callback), rmw_qos_profile_services_default, callback_group));
    return node_with_service;
  }

  std::shared_ptr<rclcpp::Node> create_node_with_client(const std::string & name)
  {
    auto node_with_client = create_node_with_disabled_callback_groups(name);
    auto callback_group =
      node_with_client->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    callback_groups_.push_back(callback_group);

    clients_.push_back(
      node_with_client->create_client<test_msgs::srv::Empty>(
        "service", rmw_qos_profile_services_default, callback_group));
    return node_with_client;
  }

  std::shared_ptr<rclcpp::Node> create_node_with_timer(const std::string & name)
  {
    auto timer_callback = []() {};
    auto node_with_timer = create_node_with_disabled_callback_groups(name);

    auto callback_group =
      node_with_timer->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_groups_.push_back(callback_group);

    timers_.push_back(
      node_with_timer->create_wall_timer(
        std::chrono::milliseconds(1), timer_callback, callback_group));
    return node_with_timer;
  }

  ::testing::AssertionResult TestNumberOfEntitiesAfterCollection(
    std::shared_ptr<rclcpp::Node> node,
    const RclWaitSetSizes & expected)
  {
    WeakCallbackGroupsToNodesMap weak_groups_to_nodes;
    node->for_each_callback_group(
      [node, &weak_groups_to_nodes](rclcpp::CallbackGroup::SharedPtr group_ptr)
      {
        weak_groups_to_nodes.insert(
          std::pair<rclcpp::CallbackGroup::WeakPtr,
          rclcpp::node_interfaces::NodeBaseInterface::WeakPtr>(
            group_ptr,
            node->get_node_base_interface()));
      });
    allocator_memory_strategy()->collect_entities(weak_groups_to_nodes);
    EXPECT_EQ(
      expected.size_of_subscriptions, allocator_memory_strategy()->number_of_ready_subscriptions());
    EXPECT_EQ(
      expected.size_of_guard_conditions, allocator_memory_strategy()->number_of_guard_conditions());
    EXPECT_EQ(expected.size_of_timers, allocator_memory_strategy()->number_of_ready_timers());
    EXPECT_EQ(expected.size_of_clients, allocator_memory_strategy()->number_of_ready_clients());
    EXPECT_EQ(expected.size_of_services, allocator_memory_strategy()->number_of_ready_services());
    EXPECT_EQ(expected.size_of_events, allocator_memory_strategy()->number_of_ready_events());
    EXPECT_EQ(expected.size_of_waitables, allocator_memory_strategy()->number_of_waitables());
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
    WeakCallbackGroupsToNodesMap weak_groups_to_nodes;
    node->for_each_callback_group(
      [node, &weak_groups_to_nodes](rclcpp::CallbackGroup::SharedPtr group_ptr)
      {
        weak_groups_to_nodes.insert(
          std::pair<rclcpp::CallbackGroup::WeakPtr,
          rclcpp::node_interfaces::NodeBaseInterface::WeakPtr>(
            group_ptr,
            node->get_node_base_interface()));
      });
    allocator_memory_strategy()->collect_entities(weak_groups_to_nodes);

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

    RCPPUTILS_SCOPE_EXIT(
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

    RCPPUTILS_SCOPE_EXIT(
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

  ::testing::AssertionResult TestGetNextEntity(
    std::shared_ptr<rclcpp::Node> node_with_entity1,
    std::shared_ptr<rclcpp::Node> node_with_entity2,
    std::function<rclcpp::AnyExecutable(WeakCallbackGroupsToNodesMap)> get_next_entity_func)
  {
    auto basic_node = create_node_with_disabled_callback_groups("basic_node");
    WeakCallbackGroupsToNodesMap weak_groups_to_nodes;
    basic_node->for_each_callback_group(
      [basic_node, &weak_groups_to_nodes](rclcpp::CallbackGroup::SharedPtr group_ptr)
      {
        weak_groups_to_nodes.insert(
          std::pair<rclcpp::CallbackGroup::WeakPtr,
          rclcpp::node_interfaces::NodeBaseInterface::WeakPtr>(
            group_ptr,
            basic_node->get_node_base_interface()));
      });
    node_with_entity1->for_each_callback_group(
      [node_with_entity1, &weak_groups_to_nodes](rclcpp::CallbackGroup::SharedPtr group_ptr)
      {
        weak_groups_to_nodes.insert(
          std::pair<rclcpp::CallbackGroup::WeakPtr,
          rclcpp::node_interfaces::NodeBaseInterface::WeakPtr>(
            group_ptr,
            node_with_entity1->get_node_base_interface()));
      });
    allocator_memory_strategy()->collect_entities(weak_groups_to_nodes);

    rclcpp::AnyExecutable result = get_next_entity_func(weak_groups_to_nodes);
    if (result.node_base != node_with_entity1->get_node_base_interface()) {
      return ::testing::AssertionFailure() <<
             "Failed to get expected entity with specified get_next_*() function";
    }

    auto basic_node2 = std::make_shared<rclcpp::Node>("basic_node2", "ns");
    WeakCallbackGroupsToNodesMap weak_groups_to_uncollected_nodes;
    basic_node2->for_each_callback_group(
      [basic_node2, &weak_groups_to_uncollected_nodes](rclcpp::CallbackGroup::SharedPtr group_ptr)
      {
        weak_groups_to_uncollected_nodes.insert(
          std::pair<rclcpp::CallbackGroup::WeakPtr,
          rclcpp::node_interfaces::NodeBaseInterface::WeakPtr>(
            group_ptr,
            basic_node2->get_node_base_interface()));
      });
    node_with_entity2->for_each_callback_group(
      [node_with_entity2,
      &weak_groups_to_uncollected_nodes](rclcpp::CallbackGroup::SharedPtr group_ptr)
      {
        weak_groups_to_uncollected_nodes.insert(
          std::pair<rclcpp::CallbackGroup::WeakPtr,
          rclcpp::node_interfaces::NodeBaseInterface::WeakPtr>(
            group_ptr,
            node_with_entity2->get_node_base_interface()));
      });
    rclcpp::AnyExecutable failed_result = get_next_entity_func(weak_groups_to_uncollected_nodes);
    if (nullptr != failed_result.node_base) {
      return ::testing::AssertionFailure() <<
             "A node was retrieved with the specified get_next_*() function despite"
             " none of the nodes that were passed to it were added to the"
             " allocator_memory_strategy. Retrieved node: " << failed_result.node_base->get_name();
    }
    return ::testing::AssertionSuccess();
  }

  ::testing::AssertionResult TestGetNextEntityMutuallyExclusive(
    std::shared_ptr<rclcpp::Node> node_with_entity,
    std::function<rclcpp::AnyExecutable(WeakCallbackGroupsToNodesMap)> get_next_entity_func)
  {
    auto basic_node = std::make_shared<rclcpp::Node>("basic_node", "ns");
    auto basic_node_base = basic_node->get_node_base_interface();
    auto node_base = node_with_entity->get_node_base_interface();
    WeakCallbackGroupsToNodesMap weak_groups_to_nodes;
    basic_node_base->for_each_callback_group(
      [basic_node_base, &weak_groups_to_nodes](rclcpp::CallbackGroup::SharedPtr group_ptr)
      {
        weak_groups_to_nodes.insert(
          std::pair<rclcpp::CallbackGroup::WeakPtr,
          rclcpp::node_interfaces::NodeBaseInterface::WeakPtr>(
            group_ptr,
            basic_node_base));
      });
    node_base->for_each_callback_group(
      [node_base, &weak_groups_to_nodes](rclcpp::CallbackGroup::SharedPtr group_ptr)
      {
        weak_groups_to_nodes.insert(
          std::pair<rclcpp::CallbackGroup::WeakPtr,
          rclcpp::node_interfaces::NodeBaseInterface::WeakPtr>(
            group_ptr,
            node_base));
      });
    allocator_memory_strategy()->collect_entities(weak_groups_to_nodes);

    // It's important that these be set after collect_entities() otherwise collect_entities() will
    // not do anything
    node_base->get_default_callback_group()->can_be_taken_from() = false;
    basic_node_base->get_default_callback_group()->can_be_taken_from() = false;
    for (auto & callback_group : callback_groups_) {
      callback_group->can_be_taken_from() = false;
    }

    rclcpp::AnyExecutable result = get_next_entity_func(weak_groups_to_nodes);

    if (nullptr != result.node_base) {
      return ::testing::AssertionFailure() <<
             "A node was retrieved with the specified get_next_*() function despite"
             " setting can_be_taken_from() to false for all nodes and callback_groups";
    }

    return ::testing::AssertionSuccess();
  }

  std::vector<std::shared_ptr<rclcpp::CallbackGroup>> callback_groups_;

private:
  std::shared_ptr<std::allocator<void>> allocator_;
  std::shared_ptr<AllocatorMemoryStrategy<>> allocator_memory_strategy_;

  // These are generally kept as weak pointers in the rclcpp::Node interfaces, so they need to be
  // owned by this class.
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
  std::vector<rclcpp::ClientBase::SharedPtr> clients_;
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
  std::vector<rclcpp::Waitable::SharedPtr> waitables_;
};

TEST_F(TestAllocatorMemoryStrategy, construct_destruct) {
  auto basic_node = create_node_with_disabled_callback_groups("basic_node");
  WeakCallbackGroupsToNodesMap weak_groups_to_nodes;
  basic_node->for_each_callback_group(
    [basic_node, &weak_groups_to_nodes](rclcpp::CallbackGroup::SharedPtr group_ptr)
    {
      weak_groups_to_nodes.insert(
        std::pair<rclcpp::CallbackGroup::WeakPtr,
        rclcpp::node_interfaces::NodeBaseInterface::WeakPtr>(
          group_ptr,
          basic_node->get_node_base_interface()));
    });
  allocator_memory_strategy()->collect_entities(weak_groups_to_nodes);
  EXPECT_EQ(0u, allocator_memory_strategy()->number_of_ready_subscriptions());
  EXPECT_EQ(0u, allocator_memory_strategy()->number_of_ready_services());
  EXPECT_EQ(0u, allocator_memory_strategy()->number_of_ready_events());
  EXPECT_EQ(0u, allocator_memory_strategy()->number_of_ready_clients());
  EXPECT_EQ(0u, allocator_memory_strategy()->number_of_guard_conditions());
  EXPECT_EQ(0u, allocator_memory_strategy()->number_of_ready_timers());
  EXPECT_EQ(0u, allocator_memory_strategy()->number_of_waitables());
}

TEST_F(TestAllocatorMemoryStrategy, add_remove_guard_conditions) {
  rclcpp::GuardCondition guard_condition1;
  rclcpp::GuardCondition guard_condition2;
  rclcpp::GuardCondition guard_condition3;

  EXPECT_NO_THROW(allocator_memory_strategy()->add_guard_condition(guard_condition1));
  EXPECT_NO_THROW(allocator_memory_strategy()->add_guard_condition(guard_condition2));
  EXPECT_NO_THROW(allocator_memory_strategy()->add_guard_condition(guard_condition3));
  EXPECT_EQ(3u, allocator_memory_strategy()->number_of_guard_conditions());

  // Adding a second time should not add to vector
  EXPECT_NO_THROW(allocator_memory_strategy()->add_guard_condition(guard_condition1));
  EXPECT_NO_THROW(allocator_memory_strategy()->add_guard_condition(guard_condition2));
  EXPECT_NO_THROW(allocator_memory_strategy()->add_guard_condition(guard_condition3));
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
  expected_sizes.size_of_events = 1;
  expected_sizes.size_of_waitables = 1;
  auto node_with_subscription = create_node_with_subscription("subscription_node");
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
  auto node = create_node_with_subscription("subscription_node");
  WeakCallbackGroupsToNodesMap weak_groups_to_nodes;
  node->for_each_callback_group(
    [node, &weak_groups_to_nodes](rclcpp::CallbackGroup::SharedPtr group_ptr)
    {
      weak_groups_to_nodes.insert(
        std::pair<rclcpp::CallbackGroup::WeakPtr,
        rclcpp::node_interfaces::NodeBaseInterface::WeakPtr>(
          group_ptr,
          node->get_node_base_interface()));
    });
  allocator_memory_strategy()->collect_entities(weak_groups_to_nodes);
  EXPECT_FALSE(allocator_memory_strategy()->add_handles_to_wait_set(nullptr));
  EXPECT_TRUE(rcl_error_is_set());
  rcl_reset_error();
}

TEST_F(TestAllocatorMemoryStrategy, add_handles_to_wait_set_subscription) {
  auto node_with_subscription = create_node_with_subscription("subscription_node");
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
  auto node = create_node_with_disabled_callback_groups("node");
  auto context = node->get_node_base_interface()->get_context();

  rclcpp::GuardCondition guard_condition(context);

  EXPECT_NO_THROW(rclcpp::GuardCondition guard_condition(context););

  allocator_memory_strategy()->add_guard_condition(guard_condition);

  RclWaitSetSizes insufficient_capacities = SufficientWaitSetCapacities();
  insufficient_capacities.size_of_guard_conditions = 0;
  EXPECT_THROW(TestAddHandlesToWaitSet(node, insufficient_capacities), std::runtime_error);
}

TEST_F(TestAllocatorMemoryStrategy, add_handles_to_wait_set_timer) {
  auto node_with_timer = create_node_with_timer("timer_node");
  RclWaitSetSizes insufficient_capacities = SufficientWaitSetCapacities();
  insufficient_capacities.size_of_timers = 0;
  EXPECT_TRUE(TestAddHandlesToWaitSet(node_with_timer, insufficient_capacities));
}

TEST_F(TestAllocatorMemoryStrategy, add_handles_to_wait_set_waitable) {
  rcl_reset_error();

  rclcpp::Waitable::SharedPtr waitable = std::make_shared<TestWaitable>();
  EXPECT_NO_THROW(allocator_memory_strategy()->add_waitable_handle(waitable));
  EXPECT_EQ(1u, allocator_memory_strategy()->number_of_waitables());

  test_waitable_result = true;
  EXPECT_TRUE(allocator_memory_strategy()->add_handles_to_wait_set(nullptr));

  test_waitable_result = false;
  EXPECT_THROW(
    allocator_memory_strategy()->add_handles_to_wait_set(nullptr),
    std::runtime_error);

  // This calls TestWaitable's functions, so rcl errors are not set
  EXPECT_FALSE(rcl_error_is_set());
}

TEST_F(TestAllocatorMemoryStrategy, get_next_subscription) {
  auto node1 = create_node_with_subscription("node1");
  auto node2 = create_node_with_subscription("node2");

  auto get_next_entity = [this](const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes) {
      rclcpp::AnyExecutable result;
      allocator_memory_strategy()->get_next_subscription(result, weak_groups_to_nodes);
      return result;
    };

  EXPECT_TRUE(TestGetNextEntity(node1, node2, get_next_entity));
}

TEST_F(TestAllocatorMemoryStrategy, get_next_service) {
  auto node1 = create_node_with_service("node1");
  auto node2 = create_node_with_service("node2");

  auto get_next_entity = [this](const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes) {
      rclcpp::AnyExecutable result;
      allocator_memory_strategy()->get_next_service(result, weak_groups_to_nodes);
      return result;
    };

  EXPECT_TRUE(TestGetNextEntity(node1, node2, get_next_entity));
}

TEST_F(TestAllocatorMemoryStrategy, get_next_client) {
  auto node1 = create_node_with_client("node1");
  auto node2 = create_node_with_client("node2");

  auto get_next_entity = [this](const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes) {
      rclcpp::AnyExecutable result;
      allocator_memory_strategy()->get_next_client(result, weak_groups_to_nodes);
      return result;
    };

  EXPECT_TRUE(TestGetNextEntity(node1, node2, get_next_entity));
}

TEST_F(TestAllocatorMemoryStrategy, get_next_timer) {
  auto node1 = create_node_with_timer("node1");
  auto node2 = create_node_with_timer("node2");

  auto get_next_entity = [this](const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes) {
      rclcpp::AnyExecutable result;
      allocator_memory_strategy()->get_next_timer(result, weak_groups_to_nodes);
      return result;
    };

  EXPECT_TRUE(TestGetNextEntity(node1, node2, get_next_entity));
}

TEST_F(TestAllocatorMemoryStrategy, get_next_waitable) {
  auto node1 = std::make_shared<rclcpp::Node>("waitable_node", "ns");
  auto node2 = std::make_shared<rclcpp::Node>("waitable_node2", "ns");
  rclcpp::Waitable::SharedPtr waitable1 = std::make_shared<TestWaitable>();
  rclcpp::Waitable::SharedPtr waitable2 = std::make_shared<TestWaitable>();
  node1->get_node_waitables_interface()->add_waitable(waitable1, nullptr);
  node2->get_node_waitables_interface()->add_waitable(waitable2, nullptr);

  auto get_next_entity = [this](const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes) {
      rclcpp::AnyExecutable result;
      allocator_memory_strategy()->get_next_waitable(result, weak_groups_to_nodes);
      return result;
    };

  EXPECT_TRUE(TestGetNextEntity(node1, node2, get_next_entity));
}

TEST_F(TestAllocatorMemoryStrategy, get_next_subscription_mutually_exclusive) {
  auto node = create_node_with_subscription("node");

  auto get_next_entity = [this](const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes) {
      rclcpp::AnyExecutable result;
      allocator_memory_strategy()->get_next_subscription(result, weak_groups_to_nodes);
      return result;
    };

  EXPECT_TRUE(TestGetNextEntityMutuallyExclusive(node, get_next_entity));
}

TEST_F(TestAllocatorMemoryStrategy, get_next_service_mutually_exclusive) {
  auto node = create_node_with_service("node");

  auto get_next_entity = [this](const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes) {
      rclcpp::AnyExecutable result;
      allocator_memory_strategy()->get_next_service(result, weak_groups_to_nodes);
      return result;
    };

  EXPECT_TRUE(TestGetNextEntityMutuallyExclusive(node, get_next_entity));
}

TEST_F(TestAllocatorMemoryStrategy, get_next_client_mutually_exclusive) {
  auto node = create_node_with_client("node");

  auto get_next_entity = [this](const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes) {
      rclcpp::AnyExecutable result;
      allocator_memory_strategy()->get_next_client(result, weak_groups_to_nodes);
      return result;
    };

  EXPECT_TRUE(TestGetNextEntityMutuallyExclusive(node, get_next_entity));
}

TEST_F(TestAllocatorMemoryStrategy, get_next_timer_mutually_exclusive) {
  auto node = create_node_with_timer("node");

  auto get_next_entity = [this](const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes) {
      rclcpp::AnyExecutable result;
      allocator_memory_strategy()->get_next_timer(result, weak_groups_to_nodes);
      return result;
    };

  EXPECT_TRUE(TestGetNextEntityMutuallyExclusive(node, get_next_entity));
}

TEST_F(TestAllocatorMemoryStrategy, get_next_waitable_mutually_exclusive) {
  auto node = std::make_shared<rclcpp::Node>("waitable_node", "ns");
  rclcpp::Waitable::SharedPtr waitable = std::make_shared<TestWaitable>();
  auto callback_group =
    node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  node->get_node_waitables_interface()->add_waitable(waitable, callback_group);

  auto get_next_entity =
    [this, callback_group](const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes) {
      // This callback group isn't in the base class' callback_group list, so this needs to be done
      // before get_next_waitable() is called.
      callback_group->can_be_taken_from() = false;

      rclcpp::AnyExecutable result;
      allocator_memory_strategy()->get_next_waitable(result, weak_groups_to_nodes);
      return result;
    };

  EXPECT_TRUE(TestGetNextEntityMutuallyExclusive(node, get_next_entity));
}

TEST_F(TestAllocatorMemoryStrategy, get_next_subscription_out_of_scope) {
  WeakCallbackGroupsToNodesMap weak_groups_to_nodes;
  auto node = create_node_with_disabled_callback_groups("node");
  // Force subscription to go out of scope and cleanup after collecting entities.
  {
    rclcpp::SubscriptionOptions subscription_options;

    auto callback_group =
      node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    subscription_options.callback_group = callback_group;

    auto subscription_callback = [](test_msgs::msg::Empty::ConstSharedPtr) {};
    const rclcpp::QoS qos(10);

    auto subscription = node->create_subscription<
      test_msgs::msg::Empty, decltype(subscription_callback)>(
      "topic", qos, std::move(subscription_callback), subscription_options);

    node->for_each_callback_group(
      [node, &weak_groups_to_nodes](rclcpp::CallbackGroup::SharedPtr group_ptr)
      {
        weak_groups_to_nodes.insert(
          std::pair<rclcpp::CallbackGroup::WeakPtr,
          rclcpp::node_interfaces::NodeBaseInterface::WeakPtr>(
            group_ptr,
            node->get_node_base_interface()));
      });
    allocator_memory_strategy()->collect_entities(weak_groups_to_nodes);
  }
  EXPECT_EQ(1u, allocator_memory_strategy()->number_of_ready_subscriptions());

  rclcpp::AnyExecutable result;
  allocator_memory_strategy()->get_next_subscription(result, weak_groups_to_nodes);
  EXPECT_EQ(nullptr, result.node_base);
}

TEST_F(TestAllocatorMemoryStrategy, get_next_service_out_of_scope) {
  WeakCallbackGroupsToNodesMap weak_groups_to_nodes;
  auto node = create_node_with_disabled_callback_groups("node");
  // Force service to go out of scope and cleanup after collecting entities.
  {
    auto callback_group =
      node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    auto service_callback =
      [](const test_msgs::srv::Empty::Request::SharedPtr,
        test_msgs::srv::Empty::Response::SharedPtr) {};
    auto service = node->create_service<test_msgs::srv::Empty>(
      "service", std::move(service_callback), rmw_qos_profile_services_default, callback_group);

    node->for_each_callback_group(
      [node, &weak_groups_to_nodes](rclcpp::CallbackGroup::SharedPtr group_ptr)
      {
        weak_groups_to_nodes.insert(
          std::pair<rclcpp::CallbackGroup::WeakPtr,
          rclcpp::node_interfaces::NodeBaseInterface::WeakPtr>(
            group_ptr,
            node->get_node_base_interface()));
      });
    allocator_memory_strategy()->collect_entities(weak_groups_to_nodes);
  }
  EXPECT_EQ(1u, allocator_memory_strategy()->number_of_ready_services());

  rclcpp::AnyExecutable result;
  allocator_memory_strategy()->get_next_service(result, weak_groups_to_nodes);
  EXPECT_EQ(nullptr, result.node_base);
}

TEST_F(TestAllocatorMemoryStrategy, get_next_client_out_of_scope) {
  auto node = create_node_with_disabled_callback_groups("node");
  WeakCallbackGroupsToNodesMap weak_groups_to_nodes;
  node->for_each_callback_group(
    [node, &weak_groups_to_nodes](rclcpp::CallbackGroup::SharedPtr group_ptr)
    {
      weak_groups_to_nodes.insert(
        std::pair<rclcpp::CallbackGroup::WeakPtr,
        rclcpp::node_interfaces::NodeBaseInterface::WeakPtr>(
          group_ptr,
          node->get_node_base_interface()));
    });
  // Force client to go out of scope and cleanup after collecting entities.
  {
    auto callback_group =
      node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    auto client = node->create_client<test_msgs::srv::Empty>(
      "service", rmw_qos_profile_services_default, callback_group);

    weak_groups_to_nodes.insert(
      std::pair<rclcpp::CallbackGroup::WeakPtr,
      rclcpp::node_interfaces::NodeBaseInterface::WeakPtr>(
        rclcpp::CallbackGroup::WeakPtr(callback_group),
        node->get_node_base_interface()));

    allocator_memory_strategy()->collect_entities(weak_groups_to_nodes);
  }
  EXPECT_EQ(1u, allocator_memory_strategy()->number_of_ready_clients());

  rclcpp::AnyExecutable result;
  allocator_memory_strategy()->get_next_client(result, weak_groups_to_nodes);
  EXPECT_EQ(nullptr, result.node_base);
}

TEST_F(TestAllocatorMemoryStrategy, get_next_timer_out_of_scope) {
  auto node = create_node_with_disabled_callback_groups("node");
  WeakCallbackGroupsToNodesMap weak_groups_to_nodes;
  // Force timer to go out of scope and cleanup after collecting entities.
  {
    auto callback_group =
      node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    auto timer = node->create_wall_timer(
      std::chrono::seconds(10), []() {}, callback_group);
    node->for_each_callback_group(
      [node, &weak_groups_to_nodes](rclcpp::CallbackGroup::SharedPtr group_ptr)
      {
        weak_groups_to_nodes.insert(
          std::pair<rclcpp::CallbackGroup::WeakPtr,
          rclcpp::node_interfaces::NodeBaseInterface::WeakPtr>(
            group_ptr,
            node->get_node_base_interface()));
      });
    allocator_memory_strategy()->collect_entities(weak_groups_to_nodes);
  }
  EXPECT_EQ(1u, allocator_memory_strategy()->number_of_ready_timers());

  rclcpp::AnyExecutable result;
  allocator_memory_strategy()->get_next_timer(result, weak_groups_to_nodes);
  EXPECT_EQ(nullptr, result.node_base);
}

TEST_F(TestAllocatorMemoryStrategy, get_next_waitable_out_of_scope) {
  auto node = create_node_with_disabled_callback_groups("node");
  WeakCallbackGroupsToNodesMap weak_groups_to_nodes;
  // Force waitable to go out of scope and cleanup after collecting entities.
  {
    auto callback_group =
      node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    node->for_each_callback_group(
      [node, &weak_groups_to_nodes](rclcpp::CallbackGroup::SharedPtr group_ptr)
      {
        weak_groups_to_nodes.insert(
          std::pair<rclcpp::CallbackGroup::WeakPtr,
          rclcpp::node_interfaces::NodeBaseInterface::WeakPtr>(
            group_ptr,
            node->get_node_base_interface()));
      });
    allocator_memory_strategy()->collect_entities(weak_groups_to_nodes);
    auto waitable = std::make_shared<TestWaitable>();
    node->get_node_waitables_interface()->add_waitable(waitable, callback_group);
    allocator_memory_strategy()->add_waitable_handle(waitable);
  }
  // Since all callback groups have been locked, except the one we added, this should only be 1
  EXPECT_EQ(1u, allocator_memory_strategy()->number_of_waitables());

  rclcpp::AnyExecutable result;
  allocator_memory_strategy()->get_next_waitable(result, weak_groups_to_nodes);
  EXPECT_EQ(nullptr, result.node_base);
}
