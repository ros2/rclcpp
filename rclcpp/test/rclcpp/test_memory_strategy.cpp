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

#include <list>
#include <memory>
#include <utility>

#include "rclcpp/strategies/allocator_memory_strategy.hpp"
#include "rclcpp/memory_strategy.hpp"
#include "test_msgs/msg/empty.hpp"
#include "test_msgs/srv/empty.hpp"

using rclcpp::memory_strategy::MemoryStrategy;
using WeakNodeList = std::list<rclcpp::node_interfaces::NodeBaseInterface::WeakPtr>;

/**
 * Mock Waitable class
 */
class TestWaitable : public rclcpp::Waitable
{
public:
  bool add_to_wait_set(rcl_wait_set_t *) override {return true;}
  bool is_ready(rcl_wait_set_t *) override {return true;}
  void execute() override {}
};

class TestMemoryStrategy : public ::testing::Test
{
public:
  TestMemoryStrategy()
  : memory_strategy_(nullptr) {}

  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    // This doesn't test AllocatorMemoryStrategy directly, so we cast to the base class.
    // AllocatorMemoryStrategy is more commonly used than MessagePoolMemoryStrategy
    // so we use this derived class for these tests.
    memory_strategy_ =
      std::make_shared<
      rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy<>>();
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<MemoryStrategy> memory_strategy()
  {
    return memory_strategy_;
  }

private:
  std::shared_ptr<MemoryStrategy> memory_strategy_;
};

TEST_F(TestMemoryStrategy, construct_destruct) {
  EXPECT_NE(nullptr, memory_strategy());
}

TEST_F(TestMemoryStrategy, get_subscription_by_handle) {
  WeakNodeList nodes;
  std::shared_ptr<const rcl_subscription_t> subscription_handle;
  rclcpp::SubscriptionBase::SharedPtr found_subscription = nullptr;
  {
    auto node = std::make_shared<rclcpp::Node>("node", "ns");
    nodes.push_back(node->get_node_base_interface());
    memory_strategy()->collect_entities(nodes);
    {
      auto callback_group =
        node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      auto subscription_callback = [](const test_msgs::msg::Empty::SharedPtr) {};
      const rclcpp::QoS qos(10);

      {
        auto subscription = node->create_subscription<
          test_msgs::msg::Empty, decltype(subscription_callback)>(
          "topic", qos, std::move(subscription_callback));

        subscription_handle = subscription->get_subscription_handle();

        EXPECT_EQ(
          subscription,
          memory_strategy()->get_subscription_by_handle(subscription_handle, nodes));
      }  // subscription goes out of scope
      EXPECT_EQ(
        nullptr,
        memory_strategy()->get_subscription_by_handle(subscription_handle, nodes));
    }  // callback_group goes out of scope
    EXPECT_EQ(
      nullptr,
      memory_strategy()->get_subscription_by_handle(subscription_handle, nodes));
  }  // Node goes out of scope
  EXPECT_EQ(
    nullptr,
    memory_strategy()->get_subscription_by_handle(subscription_handle, nodes));
}

TEST_F(TestMemoryStrategy, get_service_by_handle) {
  WeakNodeList nodes;
  std::shared_ptr<const rcl_service_t> service_handle;
  rclcpp::ServiceBase::SharedPtr found_service = nullptr;
  {
    auto node = std::make_shared<rclcpp::Node>("node", "ns");
    nodes.push_back(node->get_node_base_interface());
    memory_strategy()->collect_entities(nodes);
    {
      auto callback_group =
        node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      auto service_callback =
        [](const test_msgs::srv::Empty::Request::SharedPtr,
          test_msgs::srv::Empty::Response::SharedPtr) {};
      const rclcpp::QoS qos(10);

      {
        auto service = node->create_service<test_msgs::srv::Empty>(
          "service", std::move(service_callback),
          rmw_qos_profile_services_default, callback_group);

        service_handle = service->get_service_handle();

        EXPECT_EQ(
          service,
          memory_strategy()->get_service_by_handle(service_handle, nodes));
      }  // service goes out of scope
      EXPECT_EQ(
        nullptr,
        memory_strategy()->get_service_by_handle(service_handle, nodes));
    }  // callback_group goes out of scope
    EXPECT_EQ(
      nullptr,
      memory_strategy()->get_service_by_handle(service_handle, nodes));
  }  // Node goes out of scope
  EXPECT_EQ(
    nullptr,
    memory_strategy()->get_service_by_handle(service_handle, nodes));
}

TEST_F(TestMemoryStrategy, get_client_by_handle) {
  WeakNodeList nodes;
  std::shared_ptr<const rcl_client_t> client_handle;
  rclcpp::ClientBase::SharedPtr found_client = nullptr;
  {
    auto node = std::make_shared<rclcpp::Node>("node", "ns");
    nodes.push_back(node->get_node_base_interface());
    memory_strategy()->collect_entities(nodes);
    {
      auto callback_group =
        node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      {
        auto client = node->create_client<test_msgs::srv::Empty>(
          "service", rmw_qos_profile_services_default, callback_group);

        client_handle = client->get_client_handle();

        EXPECT_EQ(
          client,
          memory_strategy()->get_client_by_handle(client_handle, nodes));
      }  // client goes out of scope
      EXPECT_EQ(
        nullptr,
        memory_strategy()->get_client_by_handle(client_handle, nodes));
    }  // callback_group goes out of scope
    EXPECT_EQ(
      nullptr,
      memory_strategy()->get_client_by_handle(client_handle, nodes));
  }  // Node goes out of scope
  EXPECT_EQ(
    nullptr,
    memory_strategy()->get_client_by_handle(client_handle, nodes));
}

TEST_F(TestMemoryStrategy, get_timer_by_handle) {
  WeakNodeList nodes;
  std::shared_ptr<const rcl_timer_t> timer_handle;
  rclcpp::TimerBase::SharedPtr found_timer = nullptr;
  {
    auto node = std::make_shared<rclcpp::Node>("node", "ns");
    nodes.push_back(node->get_node_base_interface());
    memory_strategy()->collect_entities(nodes);
    {
      auto callback_group =
        node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      {
        auto timer_callback = []() {};
        auto timer = node->create_wall_timer(
          std::chrono::milliseconds(1), timer_callback, callback_group);

        timer_handle = timer->get_timer_handle();

        EXPECT_EQ(
          timer,
          memory_strategy()->get_timer_by_handle(timer_handle, nodes));
      }  // timer goes out of scope
      EXPECT_EQ(
        nullptr,
        memory_strategy()->get_timer_by_handle(timer_handle, nodes));
    }  // callback_group goes out of scope
    EXPECT_EQ(
      nullptr,
      memory_strategy()->get_timer_by_handle(timer_handle, nodes));
  }  // Node goes out of scope
  EXPECT_EQ(
    nullptr,
    memory_strategy()->get_timer_by_handle(timer_handle, nodes));
}

TEST_F(TestMemoryStrategy, get_node_by_group) {
  WeakNodeList nodes;
  rclcpp::CallbackGroup::SharedPtr callback_group = nullptr;
  {
    auto node = std::make_shared<rclcpp::Node>("node", "ns");
    auto node_handle = node->get_node_base_interface();
    nodes.push_back(node_handle);
    memory_strategy()->collect_entities(nodes);
    EXPECT_EQ(
      nullptr,
      memory_strategy()->get_node_by_group(nullptr, nodes));

    callback_group =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    EXPECT_EQ(
      node_handle,
      memory_strategy()->get_node_by_group(callback_group, nodes));
  }  // Node goes out of scope
  EXPECT_EQ(
    nullptr,
    memory_strategy()->get_node_by_group(callback_group, nodes));
}

TEST_F(TestMemoryStrategy, get_group_by_subscription) {
  WeakNodeList nodes;
  rclcpp::SubscriptionBase::SharedPtr subscription = nullptr;
  rclcpp::CallbackGroup::SharedPtr callback_group = nullptr;
  {
    auto node = std::make_shared<rclcpp::Node>("node", "ns");
    nodes.push_back(node->get_node_base_interface());
    memory_strategy()->collect_entities(nodes);
    {
      // This group is just used to test that a callback group that is held as a weak pointer
      // by node, doesn't confuse get_group_by_subscription() when it goes out of scope
      auto non_persistant_group =
        node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

      callback_group =
        node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      auto subscription_callback = [](const test_msgs::msg::Empty::SharedPtr) {};
      const rclcpp::QoS qos(10);

      rclcpp::SubscriptionOptions subscription_options;

      // This callback group is held as a shared_ptr in subscription_options, which means it
      // stays alive as long as subscription does.
      subscription_options.callback_group = callback_group;

      subscription = node->create_subscription<
        test_msgs::msg::Empty, decltype(subscription_callback)>(
        "topic", qos, std::move(subscription_callback), subscription_options);

      EXPECT_EQ(
        callback_group,
        memory_strategy()->get_group_by_subscription(subscription, nodes));
    }  // callback_group goes out of scope
    EXPECT_EQ(
      callback_group,
      memory_strategy()->get_group_by_subscription(subscription, nodes));
  }  // Node goes out of scope
  EXPECT_EQ(
    nullptr,
    memory_strategy()->get_group_by_subscription(subscription, nodes));
}

TEST_F(TestMemoryStrategy, get_group_by_service) {
  WeakNodeList nodes;
  rclcpp::ServiceBase::SharedPtr service = nullptr;
  {
    auto node = std::make_shared<rclcpp::Node>("node", "ns");
    nodes.push_back(node->get_node_base_interface());
    memory_strategy()->collect_entities(nodes);
    {
      auto callback_group =
        node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      auto service_callback =
        [](const test_msgs::srv::Empty::Request::SharedPtr,
          test_msgs::srv::Empty::Response::SharedPtr) {};
      const rclcpp::QoS qos(10);

      service = node->create_service<test_msgs::srv::Empty>(
        "service", std::move(service_callback),
        rmw_qos_profile_services_default, callback_group);

      EXPECT_EQ(
        callback_group,
        memory_strategy()->get_group_by_service(service, nodes));
    }  // callback_group goes out of scope
    EXPECT_EQ(
      nullptr,
      memory_strategy()->get_group_by_service(service, nodes));
  }  // Node goes out of scope
  EXPECT_EQ(
    nullptr,
    memory_strategy()->get_group_by_service(service, nodes));
}

TEST_F(TestMemoryStrategy, get_group_by_client) {
  WeakNodeList nodes;
  rclcpp::ClientBase::SharedPtr client = nullptr;
  {
    auto node = std::make_shared<rclcpp::Node>("node", "ns");
    nodes.push_back(node->get_node_base_interface());
    memory_strategy()->collect_entities(nodes);
    {
      auto callback_group =
        node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

      client = node->create_client<test_msgs::srv::Empty>(
        "service", rmw_qos_profile_services_default, callback_group);

      EXPECT_EQ(
        callback_group,
        memory_strategy()->get_group_by_client(client, nodes));
    }  // callback_group goes out of scope
    EXPECT_EQ(
      nullptr,
      memory_strategy()->get_group_by_client(client, nodes));
  }  // Node goes out of scope
  EXPECT_EQ(
    nullptr,
    memory_strategy()->get_group_by_client(client, nodes));
}

TEST_F(TestMemoryStrategy, get_group_by_timer) {
  WeakNodeList nodes;
  rclcpp::TimerBase::SharedPtr timer = nullptr;
  {
    auto node = std::make_shared<rclcpp::Node>("node", "ns");
    nodes.push_back(node->get_node_base_interface());
    memory_strategy()->collect_entities(nodes);
    {
      auto callback_group =
        node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      auto timer_callback = []() {};
      timer = node->create_wall_timer(
        std::chrono::milliseconds(1), timer_callback, callback_group);

      EXPECT_EQ(
        callback_group,
        memory_strategy()->get_group_by_timer(timer, nodes));
    }  // callback_group goes out of scope
    EXPECT_EQ(
      nullptr,
      memory_strategy()->get_group_by_timer(timer, nodes));
  }  // Node goes out of scope
  EXPECT_EQ(
    nullptr,
    memory_strategy()->get_group_by_timer(timer, nodes));
}

TEST_F(TestMemoryStrategy, get_group_by_waitable) {
  WeakNodeList nodes;
  rclcpp::Waitable::SharedPtr waitable = nullptr;
  {
    auto node = std::make_shared<rclcpp::Node>("node", "ns");
    nodes.push_back(node->get_node_base_interface());
    memory_strategy()->collect_entities(nodes);
    {
      waitable = std::make_shared<TestWaitable>();
      auto callback_group =
        node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      node->get_node_waitables_interface()->add_waitable(waitable, callback_group);

      EXPECT_EQ(
        callback_group,
        memory_strategy()->get_group_by_waitable(waitable, nodes));
    }  // callback_group goes out of scope
    EXPECT_EQ(
      nullptr,
      memory_strategy()->get_group_by_waitable(waitable, nodes));
  }  // Node goes out of scope
  EXPECT_EQ(
    nullptr,
    memory_strategy()->get_group_by_waitable(waitable, nodes));
}
