// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include "rcl/publisher.h"

#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"

#include "../mocking_utils/patch.hpp"
#include "../utils/rclcpp_gtest_macros.hpp"

#include "test_msgs/msg/empty.hpp"
#include "test_msgs/msg/strings.hpp"

class TestPublisher : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

protected:
  void initialize(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  {
    node = std::make_shared<rclcpp::Node>("my_node", "/ns", node_options);
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
};

struct TestParameters
{
  TestParameters(rclcpp::QoS qos, const std::string & description)
  : qos(qos), description(description) {}
  rclcpp::QoS qos;
  std::string description;
};

std::ostream & operator<<(std::ostream & out, const TestParameters & params)
{
  out << params.description;
  return out;
}

class TestPublisherInvalidIntraprocessQos
  : public TestPublisher,
  public ::testing::WithParamInterface<TestParameters>
{};

class TestPublisherSub : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>("my_node", "/ns");
    subnode = node->create_sub_node("sub_ns");
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr subnode;
};

/*
   Testing publisher construction and destruction.
 */
TEST_F(TestPublisher, construction_and_destruction) {
  initialize();
  using test_msgs::msg::Empty;
  {
    auto publisher = node->create_publisher<Empty>("topic", 42);
    (void)publisher;
  }

  {
    ASSERT_THROW(
    {
      auto publisher = node->create_publisher<Empty>("invalid_topic?", 42);
    }, rclcpp::exceptions::InvalidTopicNameError);
  }
}

/*
   Testing publisher creation signatures.
 */
TEST_F(TestPublisher, various_creation_signatures) {
  initialize();
  using test_msgs::msg::Empty;
  {
    auto publisher = node->create_publisher<Empty>("topic", 42);
    (void)publisher;
  }
  {
    auto publisher = node->create_publisher<Empty>("topic", rclcpp::QoS(42));
    (void)publisher;
  }
  {
    auto publisher =
      node->create_publisher<Empty>("topic", rclcpp::QoS(rclcpp::KeepLast(42)));
    (void)publisher;
  }
  {
    auto publisher =
      node->create_publisher<Empty>("topic", rclcpp::QoS(rclcpp::KeepAll()));
    (void)publisher;
  }
  {
    auto publisher =
      node->create_publisher<Empty>("topic", 42, rclcpp::PublisherOptions());
    (void)publisher;
  }
  {
    auto publisher =
      rclcpp::create_publisher<Empty>(node, "topic", 42, rclcpp::PublisherOptions());
    (void)publisher;
  }
  {
    rclcpp::PublisherOptions options;
    options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    auto publisher =
      rclcpp::create_publisher<Empty>(node, "topic", 42, options);
    (void)publisher;
  }
  {
    auto publisher = rclcpp::create_publisher<Empty>(
      node->get_node_topics_interface(), "topic", 42, rclcpp::PublisherOptions());
    (void)publisher;
  }
  {
    auto node_topics_interface = node->get_node_topics_interface();
    auto publisher = rclcpp::create_publisher<Empty>(
      node_topics_interface, "topic", 42, rclcpp::PublisherOptions());
    (void)publisher;
  }
  {
    auto node_parameters = node->get_node_parameters_interface();
    auto node_topics = node->get_node_topics_interface();
    auto publisher = rclcpp::create_publisher<Empty>(
      node_parameters, node_topics, "topic", 42, rclcpp::PublisherOptions());
    (void)publisher;
  }
}

/*
   Testing publisher with intraprocess enabled and invalid QoS
 */
TEST_P(TestPublisherInvalidIntraprocessQos, test_publisher_throws) {
  initialize(rclcpp::NodeOptions().use_intra_process_comms(true));
  rclcpp::QoS qos = GetParam().qos;
  using test_msgs::msg::Empty;
  {
    ASSERT_THROW(
      {auto publisher = node->create_publisher<Empty>("topic", qos);},
      std::invalid_argument);
  }
}

static std::vector<TestParameters> invalid_qos_profiles()
{
  std::vector<TestParameters> parameters;

  parameters.reserve(2);
  parameters.push_back(
    TestParameters(
      rclcpp::QoS(rclcpp::KeepLast(10)).transient_local(),
      "transient_local_qos"));
  parameters.push_back(
    TestParameters(
      rclcpp::QoS(rclcpp::KeepAll()),
      "keep_all_qos"));

  return parameters;
}

INSTANTIATE_TEST_SUITE_P(
  TestPublisherThrows, TestPublisherInvalidIntraprocessQos,
  ::testing::ValuesIn(invalid_qos_profiles()),
  ::testing::PrintToStringParamName());

/*
   Testing publisher construction and destruction for subnodes.
 */
TEST_F(TestPublisherSub, construction_and_destruction) {
  using test_msgs::msg::Empty;
  {
    auto publisher = subnode->create_publisher<Empty>("topic", 42);

    EXPECT_STREQ(publisher->get_topic_name(), "/ns/sub_ns/topic");
  }

  {
    auto publisher = subnode->create_publisher<Empty>("/topic", 42);

    EXPECT_STREQ(publisher->get_topic_name(), "/topic");
  }

  {
    ASSERT_THROW(
    {
      auto publisher = subnode->create_publisher<Empty>("invalid_topic?", 42);
    }, rclcpp::exceptions::InvalidTopicNameError);
  }
}

// Auxiliary class used to test getter for const PublisherBase
const rosidl_message_type_support_t EmptyTypeSupport()
{
  return *rosidl_typesupport_cpp::get_message_type_support_handle<test_msgs::msg::Empty>();
}

const rcl_publisher_options_t PublisherOptions()
{
  return rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>().template
         to_rcl_publisher_options<test_msgs::msg::Empty>(rclcpp::QoS(10));
}

class TestPublisherBase : public rclcpp::PublisherBase
{
public:
  explicit TestPublisherBase(rclcpp::Node * node)
  : rclcpp::PublisherBase(
      node->get_node_base_interface().get(), "topic", EmptyTypeSupport(), PublisherOptions()) {}
};

/*
   Testing some publisher getters
 */
TEST_F(TestPublisher, basic_getters) {
  initialize();
  using test_msgs::msg::Empty;
  {
    using rclcpp::QoS;
    using rclcpp::KeepLast;
    const size_t qos_depth_size = 10u;
    auto publisher = node->create_publisher<Empty>("topic", QoS(KeepLast(qos_depth_size)));

    size_t publisher_queue_size = publisher->get_queue_size();
    EXPECT_EQ(qos_depth_size, publisher_queue_size);

    const rmw_gid_t & publisher_rmw_gid = publisher->get_gid();
    EXPECT_NE(nullptr, publisher_rmw_gid.implementation_identifier);

    std::shared_ptr<rcl_publisher_t> publisher_handle = publisher->get_publisher_handle();
    EXPECT_NE(nullptr, publisher_handle);

    EXPECT_TRUE(publisher->assert_liveliness());
  }
  {
    const TestPublisherBase publisher = TestPublisherBase(node.get());
    std::shared_ptr<const rcl_publisher_t> publisher_handle = publisher.get_publisher_handle();
    EXPECT_NE(nullptr, publisher_handle);

    const rmw_gid_t & publisher_rmw_gid = publisher.get_gid();
    EXPECT_NE(nullptr, publisher_rmw_gid.implementation_identifier);

    // Test == operator of publisher with rmw_gid_t
    EXPECT_EQ(publisher, publisher_rmw_gid);
  }
}

TEST_F(TestPublisher, serialized_message_publish) {
  initialize();
  rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options;
  // This is the default, but it's also important for this test to succeed.
  options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
  auto publisher = node->create_publisher<test_msgs::msg::Empty>("topic", 10, options);

  rclcpp::SerializedMessage serialized_msg;
  // Mock successful rcl publish because the serialized_msg above is poorly formed
  auto mock = mocking_utils::patch_and_return(
    "self", rcl_publish_serialized_message, RCL_RET_OK);
  EXPECT_NO_THROW(publisher->publish(serialized_msg));

  EXPECT_NO_THROW(publisher->publish(serialized_msg.get_rcl_serialized_message()));
}

TEST_F(TestPublisher, rcl_publisher_init_error) {
  initialize();
  auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_publisher_init, RCL_RET_ERROR);
  EXPECT_THROW(
    node->create_publisher<test_msgs::msg::Empty>("topic", 10).reset(),
    rclcpp::exceptions::RCLError);
}

TEST_F(TestPublisher, rcl_publisher_get_rmw_handle_error) {
  initialize();
  auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_publisher_get_rmw_handle, nullptr);
  RCLCPP_EXPECT_THROW_EQ(
    node->create_publisher<test_msgs::msg::Empty>("topic", 10),
    std::runtime_error("failed to get rmw handle: error not set"));
}

TEST_F(TestPublisher, rcl_publisher_get_gid_for_publisher_error) {
  initialize();
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rmw_get_gid_for_publisher, RMW_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    node->create_publisher<test_msgs::msg::Empty>("topic", 10),
    std::runtime_error("failed to get publisher gid: error not set"));
}

TEST_F(TestPublisher, rcl_publisher_fini_error) {
  initialize();
  auto mock = mocking_utils::inject_on_return("lib:rclcpp", rcl_publisher_fini, RCL_RET_ERROR);
  auto publisher = node->create_publisher<test_msgs::msg::Empty>("topic", 10);
  ASSERT_EQ(1, publisher.use_count());
  // Failure in rcl_publisher_fini should just log error
  EXPECT_NO_THROW(publisher.reset());
}

TEST_F(TestPublisher, rcl_publisher_get_options_error) {
  initialize();
  auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_publisher_get_options, nullptr);
  auto publisher = node->create_publisher<test_msgs::msg::Empty>("topic", 10);
  RCLCPP_EXPECT_THROW_EQ(
    publisher->get_queue_size(),
    std::runtime_error("failed to get publisher options: error not set"));
}

TEST_F(TestPublisher, rcl_publisher_get_subscription_count_publisher_invalid) {
  initialize();
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_publisher_get_subscription_count, RCL_RET_PUBLISHER_INVALID);
  auto publisher = node->create_publisher<test_msgs::msg::Empty>("topic", 10);
  EXPECT_THROW(
    publisher->get_subscription_count(),
    rclcpp::exceptions::RCLError);
}

TEST_F(TestPublisher, rcl_publisher_get_actual_qos_error) {
  initialize();
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_publisher_get_actual_qos, nullptr);
  auto publisher = node->create_publisher<test_msgs::msg::Empty>("topic", 10);
  RCLCPP_EXPECT_THROW_EQ(
    publisher->get_actual_qos(),
    std::runtime_error("failed to get qos settings: error not set"));
}

TEST_F(TestPublisher, publishers_equal_rmw_compare_gids_error) {
  initialize();
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rmw_compare_gids_equal, RMW_RET_ERROR);
  const auto publisher = node->create_publisher<test_msgs::msg::Empty>("topic", 10);
  const rmw_gid_t * gid = nullptr;
  auto throwing_fn = [publisher, gid]()
    {
      // The == operator is expected to throw here, but this lambda avoids unused result warning
      return (*publisher.get() == gid) ? true : false;
    };

  RCLCPP_EXPECT_THROW_EQ(
    throwing_fn(),
    std::runtime_error("failed to compare gids: error not set"));
}

TEST_F(TestPublisher, intra_process_publish_failures) {
  initialize();
  rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options;
  options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  auto publisher = node->create_publisher<test_msgs::msg::Empty>("topic", 10, options);

  auto msg_unique = std::make_unique<test_msgs::msg::Empty>();
  EXPECT_NO_THROW(publisher->publish(std::move(msg_unique)));

  rclcpp::SerializedMessage serialized_msg;
  RCLCPP_EXPECT_THROW_EQ(
    publisher->publish(serialized_msg),
    std::runtime_error("storing serialized messages in intra process is not supported yet"));

  std::allocator<void> allocator;
  {
    rclcpp::LoanedMessage<test_msgs::msg::Empty> loaned_msg(*publisher, allocator);
    RCLCPP_EXPECT_THROW_EQ(
      publisher->publish(std::move(loaned_msg)),
      std::runtime_error("storing loaned messages in intra process is not supported yet"));
  }

  {
    rclcpp::LoanedMessage<test_msgs::msg::Empty> loaned_msg(*publisher, allocator);
    loaned_msg.release();
    RCLCPP_EXPECT_THROW_EQ(
      publisher->publish(std::move(loaned_msg)),
      std::runtime_error("loaned message is not valid"));
  }
  RCLCPP_EXPECT_THROW_EQ(
    node->create_publisher<test_msgs::msg::Empty>(
      "topic", rclcpp::QoS(0), options),
    std::invalid_argument(
      "intraprocess communication is not allowed with a zero qos history depth value"));
}

TEST_F(TestPublisher, inter_process_publish_failures) {
  initialize();
  rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options;
  options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
  auto publisher = node->create_publisher<test_msgs::msg::Empty>("topic", 10, options);

  auto msg_unique = std::make_unique<test_msgs::msg::Empty>();
  EXPECT_NO_THROW(publisher->publish(std::move(msg_unique)));

  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_publish, RCL_RET_PUBLISHER_INVALID);
    test_msgs::msg::Empty msg;
    EXPECT_THROW(publisher->publish(msg), rclcpp::exceptions::RCLError);
  }

  {
    // Using 'self' instead of 'lib:rclcpp' because `rcl_publish_serialized_message` is entirely
    // defined in a header. Also, this one requires mocking because the serialized_msg is poorly
    // formed and this just tests rclcpp functionality.
    auto mock = mocking_utils::patch_and_return(
      "self", rcl_publish_serialized_message, RCL_RET_OK);
    rclcpp::SerializedMessage serialized_msg;
    EXPECT_NO_THROW(publisher->publish(serialized_msg));
  }

  {
    // Using 'self' instead of 'lib:rclcpp' because `rcl_publish_serialized_message` is entirely
    // defined in a header
    auto mock = mocking_utils::patch_and_return(
      "self", rcl_publish_serialized_message, RCL_RET_ERROR);
    rclcpp::SerializedMessage serialized_msg;
    EXPECT_THROW(publisher->publish(serialized_msg), rclcpp::exceptions::RCLError);
  }

  std::allocator<void> allocator;
  rclcpp::LoanedMessage<test_msgs::msg::Empty> loaned_msg(*publisher, allocator);
  EXPECT_NO_THROW(publisher->publish(std::move(loaned_msg)));
}

template<typename MessageT, typename AllocatorT = std::allocator<void>>
class TestPublisherProtectedMethods : public rclcpp::Publisher<MessageT, AllocatorT>
{
public:
  using rclcpp::Publisher<MessageT, AllocatorT>::Publisher;

  void publish_loaned_message(rclcpp::LoanedMessage<MessageT, AllocatorT> && loaned_msg)
  {
    this->do_loaned_message_publish(std::move(loaned_msg.release()));
  }

  void call_default_incompatible_qos_callback(rclcpp::QOSOfferedIncompatibleQoSInfo & event) const
  {
    this->default_incompatible_qos_callback(event);
  }
};

TEST_F(TestPublisher, do_loaned_message_publish_error) {
  initialize();
  using PublisherT = TestPublisherProtectedMethods<test_msgs::msg::Empty, std::allocator<void>>;
  auto publisher =
    node->create_publisher<test_msgs::msg::Empty, std::allocator<void>, PublisherT>("topic", 10);

  auto msg = publisher->borrow_loaned_message();

  {
    // Using 'self' instead of 'lib:rclcpp' because `rcl_publish_loaned_message` is entirely
    // defined in a header
    auto mock = mocking_utils::patch_and_return(
      "self", rcl_publish_loaned_message, RCL_RET_PUBLISHER_INVALID);
    EXPECT_THROW(publisher->publish_loaned_message(std::move(msg)), rclcpp::exceptions::RCLError);
  }
}

TEST_F(TestPublisher, default_incompatible_qos_callback) {
  initialize();
  using PublisherT = TestPublisherProtectedMethods<test_msgs::msg::Empty, std::allocator<void>>;
  auto publisher =
    node->create_publisher<test_msgs::msg::Empty, std::allocator<void>, PublisherT>("topic", 10);
  rclcpp::QOSOfferedIncompatibleQoSInfo event;
  event.last_policy_kind = RMW_QOS_POLICY_INVALID;
  // This message just logs an error message
  EXPECT_NO_THROW(publisher->call_default_incompatible_qos_callback(event));
}

TEST_F(TestPublisher, run_event_handlers) {
  initialize();
  auto publisher = node->create_publisher<test_msgs::msg::Empty>("topic", 10);

  for (const auto & key_event_pair : publisher->get_event_handlers()) {
    auto handler = key_event_pair.second;
    std::shared_ptr<void> data = handler->take_data();
    handler->execute(data);
  }
}

TEST_F(TestPublisher, get_network_flow_endpoints_errors) {
  initialize();
  const rclcpp::QoS publisher_qos(1);
  auto publisher = node->create_publisher<test_msgs::msg::Empty>("topic", publisher_qos);

  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_publisher_get_network_flow_endpoints, RCL_RET_ERROR);
    auto mock_network_flow_endpoint_array_fini = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_network_flow_endpoint_array_fini, RCL_RET_ERROR);
    EXPECT_THROW(
      publisher->get_network_flow_endpoints(),
      rclcpp::exceptions::RCLError);
  }
  {
    auto mock_network_flow_endpoint_array_fini = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_network_flow_endpoint_array_fini, RCL_RET_ERROR);
    EXPECT_THROW(
      publisher->get_network_flow_endpoints(),
      rclcpp::exceptions::RCLError);
  }
  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_publisher_get_network_flow_endpoints, RCL_RET_OK);
    auto mock_network_flow_endpoint_array_fini = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_network_flow_endpoint_array_fini, RCL_RET_OK);
    EXPECT_NO_THROW(publisher->get_network_flow_endpoints());
  }
}

TEST_F(TestPublisher, check_wait_for_all_acked_return) {
  initialize();
  const rclcpp::QoS publisher_qos(1);
  auto publisher = node->create_publisher<test_msgs::msg::Empty>("topic", publisher_qos);

  {
    // Using 'self' instead of 'lib:rclcpp' because `rcl_publisher_wait_for_all_acked` is entirely
    // defined in a header
    auto mock = mocking_utils::patch_and_return(
      "self", rcl_publisher_wait_for_all_acked, RCL_RET_OK);
    EXPECT_TRUE(publisher->wait_for_all_acked(std::chrono::milliseconds(-1)));
  }

  {
    // Using 'self' instead of 'lib:rclcpp' because `rcl_publisher_wait_for_all_acked` is entirely
    // defined in a header
    auto mock = mocking_utils::patch_and_return(
      "self", rcl_publisher_wait_for_all_acked, RCL_RET_TIMEOUT);
    EXPECT_FALSE(publisher->wait_for_all_acked(std::chrono::milliseconds(-1)));
  }

  {
    // Using 'self' instead of 'lib:rclcpp' because `rcl_publisher_wait_for_all_acked` is entirely
    // defined in a header
    auto mock = mocking_utils::patch_and_return(
      "self", rcl_publisher_wait_for_all_acked, RCL_RET_UNSUPPORTED);
    EXPECT_THROW(
      publisher->wait_for_all_acked(std::chrono::milliseconds(-1)),
      rclcpp::exceptions::RCLError);
  }

  {
    // Using 'self' instead of 'lib:rclcpp' because `rcl_publisher_wait_for_all_acked` is entirely
    // defined in a header
    auto mock = mocking_utils::patch_and_return(
      "self", rcl_publisher_wait_for_all_acked, RCL_RET_ERROR);
    EXPECT_THROW(
      publisher->wait_for_all_acked(std::chrono::milliseconds(-1)),
      rclcpp::exceptions::RCLError);
  }
}

class TestPublisherWaitForAllAcked
  : public TestPublisher, public ::testing::WithParamInterface<std::pair<rclcpp::QoS, rclcpp::QoS>>
{
};

TEST_P(TestPublisherWaitForAllAcked, check_wait_for_all_acked_with_QosPolicy) {
  initialize();

  auto do_nothing = [](std::shared_ptr<const test_msgs::msg::Strings>) {};
  auto pub = node->create_publisher<test_msgs::msg::Strings>("topic", std::get<0>(GetParam()));
  auto sub = node->create_subscription<test_msgs::msg::Strings>(
    "topic",
    std::get<1>(GetParam()),
    do_nothing);

  auto msg = std::make_shared<test_msgs::msg::Strings>();
  for (int i = 0; i < 20; i++) {
    ASSERT_NO_THROW(pub->publish(*msg));
  }
  EXPECT_TRUE(pub->wait_for_all_acked(std::chrono::milliseconds(6000)));
}

INSTANTIATE_TEST_SUITE_P(
  TestWaitForAllAckedWithParm,
  TestPublisherWaitForAllAcked,
  ::testing::Values(
    std::pair<rclcpp::QoS, rclcpp::QoS>(
      rclcpp::QoS(1).reliable(), rclcpp::QoS(1).reliable()),
    std::pair<rclcpp::QoS, rclcpp::QoS>(
      rclcpp::QoS(1).best_effort(), rclcpp::QoS(1).best_effort()),
    std::pair<rclcpp::QoS, rclcpp::QoS>(
      rclcpp::QoS(1).reliable(), rclcpp::QoS(1).best_effort())));
