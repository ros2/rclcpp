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
#include <vector>

#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "test_msgs/msg/basic_types.hpp"

class TestWaitSet : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }
};

/*
 * Testing normal construction and destruction.
 */
TEST_F(TestWaitSet, construction_and_destruction) {
  {
    rclcpp::WaitSet wait_set;
    (void)wait_set;
  }

  {
    rclcpp::WaitSet wait_set(
      std::vector<rclcpp::WaitSet::SubscriptionEntry>{},
      std::vector<rclcpp::GuardCondition::SharedPtr>{},
      std::vector<rclcpp::TimerBase::SharedPtr>{},
      std::vector<rclcpp::ClientBase::SharedPtr>{},
      std::vector<rclcpp::ServiceBase::SharedPtr>{},
      std::vector<rclcpp::WaitSet::WaitableEntry>{});
    (void)wait_set;
  }

  {
    auto gc = std::make_shared<rclcpp::GuardCondition>();
    rclcpp::WaitSet wait_set({}, {gc});
    (void)wait_set;
  }

  {
    auto context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);
    auto gc = std::make_shared<rclcpp::GuardCondition>(context);
    rclcpp::WaitSet wait_set({}, {gc}, {}, {}, {}, {}, context);
    (void)wait_set;
  }

  {
    // invalid context (nullptr)
    ASSERT_THROW(
    {
      rclcpp::WaitSet wait_set(
        std::vector<rclcpp::WaitSet::SubscriptionEntry>{},
        std::vector<rclcpp::GuardCondition::SharedPtr>{},
        std::vector<rclcpp::TimerBase::SharedPtr>{},
        std::vector<rclcpp::ClientBase::SharedPtr>{},
        std::vector<rclcpp::ServiceBase::SharedPtr>{},
        std::vector<rclcpp::WaitSet::WaitableEntry>{},
        nullptr);
      (void)wait_set;
    }, std::invalid_argument);
  }

  {
    // invalid context (uninitialized)
    auto context = std::make_shared<rclcpp::Context>();
    ASSERT_THROW(
    {
      rclcpp::WaitSet wait_set(
        std::vector<rclcpp::WaitSet::SubscriptionEntry>{},
        std::vector<rclcpp::GuardCondition::SharedPtr>{},
        std::vector<rclcpp::TimerBase::SharedPtr>{},
        std::vector<rclcpp::ClientBase::SharedPtr>{},
        std::vector<rclcpp::ServiceBase::SharedPtr>{},
        std::vector<rclcpp::WaitSet::WaitableEntry>{},
        context);
      (void)wait_set;
    }, rclcpp::exceptions::RCLInvalidArgument);
  }
}

/*
 * Testing rcl wait set accessor.
 */
TEST_F(TestWaitSet, get_rcl_wait_set) {
  {
    rclcpp::WaitSet wait_set;
    wait_set.get_rcl_wait_set();
  }
}

/*
 * Testing add/remove for guard condition methods.
 */
TEST_F(TestWaitSet, add_remove_guard_condition) {
  // normal, mixed initialization
  {
    auto gc = std::make_shared<rclcpp::GuardCondition>();
    auto gc2 = std::make_shared<rclcpp::GuardCondition>();
    rclcpp::WaitSet wait_set({}, {gc});
    wait_set.add_guard_condition(gc2);
    wait_set.remove_guard_condition(gc2);
    wait_set.remove_guard_condition(gc);
  }

  // out of order removal
  {
    auto gc = std::make_shared<rclcpp::GuardCondition>();
    auto gc2 = std::make_shared<rclcpp::GuardCondition>();
    rclcpp::WaitSet wait_set({}, {gc});
    wait_set.add_guard_condition(gc2);
    wait_set.remove_guard_condition(gc);
    wait_set.remove_guard_condition(gc2);
  }

  // start empty, normal
  {
    auto gc = std::make_shared<rclcpp::GuardCondition>();
    rclcpp::WaitSet wait_set;
    wait_set.add_guard_condition(gc);
    wait_set.remove_guard_condition(gc);
  }

  // add invalid (nullptr)
  {
    rclcpp::WaitSet wait_set;
    ASSERT_THROW(
    {
      wait_set.add_guard_condition(nullptr);
    }, std::invalid_argument);
  }

  // double add
  {
    auto gc = std::make_shared<rclcpp::GuardCondition>();
    rclcpp::WaitSet wait_set;
    wait_set.add_guard_condition(gc);
    ASSERT_THROW(
    {
      wait_set.add_guard_condition(gc);
    }, std::runtime_error);
  }

  // remove invalid (nullptr)
  {
    rclcpp::WaitSet wait_set;
    ASSERT_THROW(
    {
      wait_set.remove_guard_condition(nullptr);
    }, std::invalid_argument);
  }

  // remove unrelated
  {
    auto gc = std::make_shared<rclcpp::GuardCondition>();
    auto gc2 = std::make_shared<rclcpp::GuardCondition>();
    rclcpp::WaitSet wait_set({}, {gc});
    ASSERT_THROW(
    {
      wait_set.remove_guard_condition(gc2);
    }, std::runtime_error);
  }

  // double remove
  {
    auto gc = std::make_shared<rclcpp::GuardCondition>();
    rclcpp::WaitSet wait_set({}, {gc});
    wait_set.remove_guard_condition(gc);
    ASSERT_THROW(
    {
      wait_set.remove_guard_condition(gc);
    }, std::runtime_error);
  }

  // remove from empty
  {
    auto gc = std::make_shared<rclcpp::GuardCondition>();
    rclcpp::WaitSet wait_set;
    ASSERT_THROW(
    {
      wait_set.remove_guard_condition(gc);
    }, std::runtime_error);
  }

  // remove after delete, checking weak ownership behavior
  // {
  //   auto gc = std::make_shared<rclcpp::GuardCondition>();
  //   rclcpp::WaitSet wait_set;
  //   wait_set.add_guard_condition(gc);
  //   gc.reset();
  //   ASSERT_THROW(
  //   {
  //     // gc should be missing at this point
  //     wait_set.remove_guard_condition(gc);
  //   }, std::runtime_error);
  // }
  // Note this case does not fail because you cannot pass a "reset" shared pointer to gc
  // and expect it to try and find the original pointer.
  // Instead it throws due to gc being nullptr.
}

/*
 * Testing adding each entity to two separate wait sets.
 */
TEST_F(TestWaitSet, add_guard_condition_to_two_different_wait_set) {
  {
    rclcpp::WaitSet wait_set1;
    rclcpp::WaitSet wait_set2;
    auto node = std::make_shared<rclcpp::Node>("add_guard_condition_to_two_different_wait_set");

    auto guard_condition = std::make_shared<rclcpp::GuardCondition>();
    wait_set1.add_guard_condition(guard_condition);
    ASSERT_THROW(
    {
      wait_set2.add_guard_condition(guard_condition);
    }, std::runtime_error);

    auto do_nothing = [](const std::shared_ptr<test_msgs::msg::BasicTypes>) {};
    auto sub = node->create_subscription<test_msgs::msg::BasicTypes>("~/test", 1, do_nothing);
    wait_set1.add_subscription(sub);
    ASSERT_THROW(
    {
      wait_set2.add_subscription(sub);
    }, std::runtime_error);

    auto timer = node->create_wall_timer(std::chrono::seconds(1), []() {});
    wait_set1.add_timer(timer);
    ASSERT_THROW(
    {
      wait_set2.add_timer(timer);
    }, std::runtime_error);

    auto client = node->create_client<rcl_interfaces::srv::ListParameters>("~/test");
    wait_set1.add_client(client);
    ASSERT_THROW(
    {
      wait_set2.add_client(client);
    }, std::runtime_error);

    auto srv_do_nothing = [](
      const std::shared_ptr<rcl_interfaces::srv::ListParameters::Request>,
      std::shared_ptr<rcl_interfaces::srv::ListParameters::Response>) {};
    auto service =
      node->create_service<rcl_interfaces::srv::ListParameters>("~/test", srv_do_nothing);
    wait_set1.add_service(service);
    ASSERT_THROW(
    {
      wait_set2.add_service(service);
    }, std::runtime_error);

    rclcpp::PublisherOptions po;
    po.event_callbacks.deadline_callback = [](rclcpp::QOSDeadlineOfferedInfo &) {};
    auto pub = node->create_publisher<test_msgs::msg::BasicTypes>("~/test", 1, po);
    auto qos_event = pub->get_event_handlers()[0];
    wait_set1.add_waitable(qos_event, pub);
    ASSERT_THROW(
    {
      wait_set2.add_waitable(qos_event, pub);
    }, std::runtime_error);
  }
}
