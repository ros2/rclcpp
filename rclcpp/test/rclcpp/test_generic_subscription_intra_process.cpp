// Copyright 2026 Open Source Robotics Foundation, Inc.
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
#include <string>
#include <utility>
#include <vector>

#include "test_msgs/msg/strings.hpp"

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

using namespace std::chrono_literals;

template<>
struct rclcpp::TypeAdapter<std::string, test_msgs::msg::Strings>
{
  using is_specialized = std::true_type;
  using custom_type = std::string;
  using ros_message_type = test_msgs::msg::Strings;

  static void
  convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    destination.string_value = source;
  }

  static void
  convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    destination = source.string_value;
  }
};

namespace
{

constexpr char kType[] = "test_msgs/msg/Strings";

std::string
deserialize_string(const rclcpp::SerializedMessage & serialized_message)
{
  test_msgs::msg::Strings msg;
  rclcpp::Serialization<test_msgs::msg::Strings> serializer;
  serializer.deserialize_message(&serialized_message, &msg);
  return msg.string_value;
}

/// What a generic subscription received.
struct Received
{
  std::vector<std::string> data;
  std::vector<bool> from_intra_process;
  std::vector<rcutils_time_point_value_t> received_timestamps;
  std::vector<rcutils_time_point_value_t> source_timestamps;

  void add(const rclcpp::SerializedMessage & msg, const rclcpp::MessageInfo & info)
  {
    data.push_back(deserialize_string(msg));
    from_intra_process.push_back(info.get_rmw_message_info().from_intra_process);
    received_timestamps.push_back(info.get_rmw_message_info().received_timestamp);
    source_timestamps.push_back(info.get_rmw_message_info().source_timestamp);
  }
};

}  // namespace

class TestGenericSubscriptionIntraProcess : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

protected:
  void SetUp() override
  {
    const auto * test_info = ::testing::UnitTest::GetInstance()->current_test_info();
    topic_ = std::string("/generic_ipc_") + test_info->name();
    node_ = std::make_shared<rclcpp::Node>(
      "generic_ipc_node",
      rclcpp::NodeOptions().start_parameter_event_publisher(false).use_intra_process_comms(true));
    executor_.add_node(node_);
  }

  void TearDown() override
  {
    executor_.remove_node(node_);
    node_.reset();
  }

  rclcpp::GenericSubscription::SharedPtr
  create_shared_generic_sub(Received & received, const rclcpp::QoS & qos = rclcpp::QoS(10))
  {
    return node_->create_generic_subscription(
      topic_, kType, qos,
      [&received](
        std::shared_ptr<const rclcpp::SerializedMessage> msg, const rclcpp::MessageInfo & info) {
        received.add(*msg, info);
      });
  }

  rclcpp::GenericSubscription::SharedPtr
  create_unique_generic_sub(Received & received, const rclcpp::QoS & qos = rclcpp::QoS(10))
  {
    return node_->create_generic_subscription(
      topic_, kType, qos,
      [&received](
        std::unique_ptr<rclcpp::SerializedMessage> msg, const rclcpp::MessageInfo & info) {
        received.add(*msg, info);
      });
  }

  /// Spin until condition holds (or timeout), then spin a little more to catch duplicates.
  template<typename Condition>
  bool spin_until(Condition condition, std::chrono::milliseconds timeout = 5s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!condition()) {
      if (std::chrono::steady_clock::now() > deadline) {
        return false;
      }
      executor_.spin_some(10ms);
    }
    const auto settle = std::chrono::steady_clock::now() + 200ms;
    while (std::chrono::steady_clock::now() < settle) {
      executor_.spin_some(10ms);
    }
    return true;
  }

  static std::unique_ptr<test_msgs::msg::Strings> make_msg(const std::string & data)
  {
    auto msg = std::make_unique<test_msgs::msg::Strings>();
    msg->string_value = data;
    return msg;
  }

  std::string topic_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(TestGenericSubscriptionIntraProcess, unique_publish_delivered_once_intra_process)
{
  Received received;
  auto sub = create_shared_generic_sub(received);
  auto pub = node_->create_publisher<test_msgs::msg::Strings>(topic_, 10);
  EXPECT_EQ(1u, pub->get_intra_process_subscription_count());

  const auto before = std::chrono::system_clock::now().time_since_epoch().count();
  pub->publish(make_msg("a"));
  pub->publish(make_msg("b"));
  ASSERT_TRUE(spin_until([&]() {return received.data.size() >= 2;}));

  ASSERT_EQ((std::vector<std::string>{"a", "b"}), received.data);
  for (size_t i = 0; i < received.data.size(); ++i) {
    EXPECT_TRUE(received.from_intra_process[i]);
    EXPECT_GE(received.received_timestamps[i], before);
    EXPECT_GE(received.source_timestamps[i], before);
  }
}

TEST_F(TestGenericSubscriptionIntraProcess, const_ref_publish_subscription_after_publisher)
{
  auto pub = node_->create_publisher<test_msgs::msg::Strings>(topic_, 10);
  Received received;
  auto sub = create_shared_generic_sub(received);
  EXPECT_EQ(1u, pub->get_intra_process_subscription_count());

  test_msgs::msg::Strings msg;
  msg.string_value = "hello";
  pub->publish(msg);
  ASSERT_TRUE(spin_until([&]() {return !received.data.empty();}));
  ASSERT_EQ((std::vector<std::string>{"hello"}), received.data);
  EXPECT_TRUE(received.from_intra_process[0]);
}

TEST_F(TestGenericSubscriptionIntraProcess, shared_and_owning_generic_subscriptions)
{
  auto pub = node_->create_publisher<test_msgs::msg::Strings>(topic_, 10);
  Received shared1, shared2, owned1, owned2;
  auto s1 = create_shared_generic_sub(shared1);
  auto s2 = create_shared_generic_sub(shared2);
  auto o1 = create_unique_generic_sub(owned1);
  auto o2 = create_unique_generic_sub(owned2);
  EXPECT_EQ(4u, pub->get_intra_process_subscription_count());

  pub->publish(make_msg("x"));
  ASSERT_TRUE(
    spin_until(
      [&]() {
        return shared1.data.size() && shared2.data.size() && owned1.data.size() &&
               owned2.data.size();
      }));
  for (auto * r : {&shared1, &shared2, &owned1, &owned2}) {
    ASSERT_EQ((std::vector<std::string>{"x"}), r->data);
    EXPECT_TRUE(r->from_intra_process[0]);
  }
}

TEST_F(TestGenericSubscriptionIntraProcess, typed_and_generic_subscriptions)
{
  auto pub = node_->create_publisher<test_msgs::msg::Strings>(topic_, 10);
  Received generic;
  auto gsub = create_shared_generic_sub(generic);
  std::vector<std::string> typed;
  std::vector<bool> typed_intra;
  auto tsub = node_->create_subscription<test_msgs::msg::Strings>(
    topic_, 10,
    [&](std::unique_ptr<test_msgs::msg::Strings> msg, const rclcpp::MessageInfo & info) {
      typed.push_back(msg->string_value);
      typed_intra.push_back(info.get_rmw_message_info().from_intra_process);
    });
  EXPECT_EQ(2u, pub->get_intra_process_subscription_count());

  pub->publish(make_msg("m1"));
  test_msgs::msg::Strings m2;
  m2.string_value = "m2";
  pub->publish(m2);
  ASSERT_TRUE(spin_until([&]() {return generic.data.size() >= 2 && typed.size() >= 2;}));
  EXPECT_EQ((std::vector<std::string>{"m1", "m2"}), generic.data);
  EXPECT_EQ((std::vector<std::string>{"m1", "m2"}), typed);
  EXPECT_EQ((std::vector<bool>{true, true}), generic.from_intra_process);
  EXPECT_EQ((std::vector<bool>{true, true}), typed_intra);
}

TEST_F(TestGenericSubscriptionIntraProcess, type_adapted_publisher)
{
  using Adapter = rclcpp::TypeAdapter<std::string, test_msgs::msg::Strings>;
  auto pub = node_->create_publisher<Adapter>(topic_, 10);
  Received received;
  auto sub = create_shared_generic_sub(received);

  pub->publish(std::make_unique<std::string>("adapted unique"));
  pub->publish(std::string("adapted const ref"));
  test_msgs::msg::Strings ros_msg;
  ros_msg.string_value = "ros msg";
  pub->publish(ros_msg);
  ASSERT_TRUE(spin_until([&]() {return received.data.size() >= 3;}));
  EXPECT_EQ(
    (std::vector<std::string>{"adapted unique", "adapted const ref", "ros msg"}), received.data);
  EXPECT_EQ((std::vector<bool>{true, true, true}), received.from_intra_process);
}

TEST_F(TestGenericSubscriptionIntraProcess, transient_local_late_joiner)
{
  const auto qos = rclcpp::QoS(5).reliable().transient_local();
  auto pub = node_->create_publisher<test_msgs::msg::Strings>(topic_, qos);
  pub->publish(make_msg("old1"));
  pub->publish(make_msg("old2"));

  Received shared_late, owned_late;
  auto s = create_shared_generic_sub(shared_late, qos);
  auto o = create_unique_generic_sub(owned_late, qos);
  ASSERT_TRUE(spin_until([&]() {
      return shared_late.data.size() >= 2 && owned_late.data.size() >= 2;
    }));
  for (auto * r : {&shared_late, &owned_late}) {
    EXPECT_EQ((std::vector<std::string>{"old1", "old2"}), r->data);
    EXPECT_EQ((std::vector<bool>{true, true}), r->from_intra_process);
  }

  pub->publish(make_msg("new"));
  ASSERT_TRUE(spin_until([&]() {
      return shared_late.data.size() >= 3 && owned_late.data.size() >= 3;
    }));
  EXPECT_EQ("new", shared_late.data.back());
  EXPECT_EQ(3u, shared_late.data.size());
  EXPECT_EQ(3u, owned_late.data.size());
}

TEST_F(TestGenericSubscriptionIntraProcess, lowest_available_capacity_counts_generic)
{
  auto pub = node_->create_publisher<test_msgs::msg::Strings>(topic_, 10);
  EXPECT_EQ(0u, pub->lowest_available_ipm_capacity());
  Received received;
  auto sub = create_shared_generic_sub(received, rclcpp::QoS(3));
  EXPECT_EQ(3u, pub->lowest_available_ipm_capacity());
  pub->publish(make_msg("1"));
  EXPECT_EQ(2u, pub->lowest_available_ipm_capacity());
}

TEST_F(TestGenericSubscriptionIntraProcess, keep_all_falls_back_to_inter_process)
{
  auto pub = node_->create_publisher<test_msgs::msg::Strings>(topic_, 10);
  Received received;
  rclcpp::GenericSubscription::SharedPtr sub;
  ASSERT_NO_THROW(sub = create_shared_generic_sub(received, rclcpp::QoS(10).keep_all()));
  EXPECT_EQ(0u, pub->get_intra_process_subscription_count());

  // Delivered through the middleware: wait for discovery first.
  ASSERT_TRUE(
    spin_until([&]() {return pub->get_subscription_count() >= 1;}));
  pub->publish(make_msg("via rmw"));
  ASSERT_TRUE(spin_until([&]() {return !received.data.empty();}));
  ASSERT_EQ((std::vector<std::string>{"via rmw"}), received.data);
  EXPECT_FALSE(received.from_intra_process[0]);
}

TEST_F(TestGenericSubscriptionIntraProcess, disabled_callbacks)
{
  auto pub = node_->create_publisher<test_msgs::msg::Strings>(topic_, 10);
  Received received;
  auto sub = create_shared_generic_sub(received);
  sub->disable_callbacks();
  pub->publish(make_msg("ignored"));
  EXPECT_FALSE(spin_until([&]() {return !received.data.empty();}, 300ms));

  sub->enable_callbacks();
  pub->publish(make_msg("delivered"));
  ASSERT_TRUE(spin_until([&]() {return !received.data.empty();}));
  EXPECT_EQ("delivered", received.data.back());
}

TEST_F(TestGenericSubscriptionIntraProcess, subscription_and_publisher_lifetime)
{
  auto pub = node_->create_publisher<test_msgs::msg::Strings>(topic_, 10);
  {
    Received received;
    auto sub = create_shared_generic_sub(received);
    EXPECT_EQ(1u, pub->get_intra_process_subscription_count());
  }
  EXPECT_EQ(0u, pub->get_intra_process_subscription_count());
  // No generic subscription anymore: publishing must not throw.
  EXPECT_NO_THROW(pub->publish(make_msg("nobody")));

  Received received;
  auto sub = create_shared_generic_sub(received);
  pub.reset();
  auto pub2 = node_->create_publisher<test_msgs::msg::Strings>(topic_, 10);
  pub2->publish(make_msg("second publisher"));
  ASSERT_TRUE(spin_until([&]() {return !received.data.empty();}));
  EXPECT_EQ((std::vector<std::string>{"second publisher"}), received.data);
}
