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
#include "rclcpp/type_support_decl.hpp"
#include "test_msgs/msg/empty.hpp"

class TestTypeSupport : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  ::testing::AssertionResult test_type_support_init_fini(const rosidl_service_type_support_t * ts)
  {
    rcl_service_t service_handle = rcl_get_zero_initialized_service();
    rcl_service_options_t service_options = rcl_service_get_default_options();
    auto node_handle = node->get_node_base_interface()->get_rcl_node_handle();
    rcl_ret_t ret = rcl_service_init(
      &service_handle, node_handle, ts, "base_node_service", &service_options);
    if (ret != RCL_RET_OK) {
      return ::testing::AssertionFailure() <<
             "Failed rcl_service_init with error string: " << rcl_get_error_string().str;
    }
    ret = rcl_service_fini(&service_handle, node_handle);
    if (ret != RCL_RET_OK) {
      return ::testing::AssertionFailure() <<
             "Failed rcl_service_fini with error string: " << rcl_get_error_string().str;
    }
    return ::testing::AssertionSuccess();
  }

protected:
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("my_node", "/ns");
};

const rcl_publisher_options_t PublisherOptions()
{
  return rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>().template
         to_rcl_publisher_options<test_msgs::msg::Empty>(rclcpp::QoS(10));
}

// Auxiliary classes used to test rosidl_message_type_support_t getters
// defined in type_support.hpp
const rosidl_message_type_support_t * ts_parameter_event =
  rclcpp::type_support::get_parameter_event_msg_type_support();

class TestTSParameterEvent : public rclcpp::PublisherBase
{
public:
  explicit TestTSParameterEvent(rclcpp::Node * node)
  : rclcpp::PublisherBase(
      node->get_node_base_interface().get(),
      "topicTSParameterEvent",
      *ts_parameter_event,
      PublisherOptions()) {}
};

const rosidl_message_type_support_t * ts_set_parameter_result =
  rclcpp::type_support::get_set_parameters_result_msg_type_support();

class TestTSSetParameterResult : public rclcpp::PublisherBase
{
public:
  explicit TestTSSetParameterResult(rclcpp::Node * node)
  : rclcpp::PublisherBase(
      node->get_node_base_interface().get(),
      "topicTSSetParameterResult",
      *ts_set_parameter_result,
      PublisherOptions()) {}
};

const rosidl_message_type_support_t * ts_parameter_descriptor =
  rclcpp::type_support::get_parameter_descriptor_msg_type_support();

class TestTSParameterDescriptor : public rclcpp::PublisherBase
{
public:
  explicit TestTSParameterDescriptor(rclcpp::Node * node)
  : rclcpp::PublisherBase(
      node->get_node_base_interface().get(),
      "topicTSParameterDescriptor",
      *ts_parameter_descriptor,
      PublisherOptions()) {}
};

const rosidl_message_type_support_t * ts_list_parameter_result =
  rclcpp::type_support::get_list_parameters_result_msg_type_support();

class TestTSListParametersResult : public rclcpp::PublisherBase
{
public:
  explicit TestTSListParametersResult(rclcpp::Node * node)
  : rclcpp::PublisherBase(
      node->get_node_base_interface().get(),
      "topicTSListParametersResult",
      *ts_list_parameter_result,
      PublisherOptions()) {}
};

/*
   Test that the publisher is created properly for different msg typesupport
 */
TEST_F(TestTypeSupport, basic_getters) {
  {
    auto publisher = TestTSParameterEvent(node.get());
    std::shared_ptr<const rcl_publisher_t> publisher_handle = publisher.get_publisher_handle();
    EXPECT_NE(nullptr, publisher_handle);
  }
  {
    auto publisher = TestTSSetParameterResult(node.get());
    std::shared_ptr<const rcl_publisher_t> publisher_handle = publisher.get_publisher_handle();
    EXPECT_NE(nullptr, publisher_handle);
  }
  {
    auto publisher = TestTSParameterDescriptor(node.get());
    std::shared_ptr<const rcl_publisher_t> publisher_handle = publisher.get_publisher_handle();
    EXPECT_NE(nullptr, publisher_handle);
  }
  {
    auto publisher = TestTSListParametersResult(node.get());
    std::shared_ptr<const rcl_publisher_t> publisher_handle = publisher.get_publisher_handle();
    EXPECT_NE(nullptr, publisher_handle);
  }
}

/* Testing type support getters */
TEST_F(TestTypeSupport, test_service_ts_get_params_srv) {
  const rosidl_service_type_support_t * ts =
    rclcpp::type_support::get_get_parameters_srv_type_support();
  ASSERT_NE(nullptr, ts);
  EXPECT_TRUE(test_type_support_init_fini(ts));
}

TEST_F(TestTypeSupport, test_service_ts_get_params_srv_type) {
  const rosidl_service_type_support_t * ts =
    rclcpp::type_support::get_get_parameters_srv_type_support();
  ASSERT_NE(nullptr, ts);
  EXPECT_TRUE(test_type_support_init_fini(ts));
}

TEST_F(TestTypeSupport, test_service_ts_get_parameters_types_srv) {
  const rosidl_service_type_support_t * ts =
    rclcpp::type_support::get_get_parameter_types_srv_type_support();
  ASSERT_NE(nullptr, ts);
  EXPECT_TRUE(test_type_support_init_fini(ts));
}

TEST_F(TestTypeSupport, test_service_ts_set_params_srv) {
  const rosidl_service_type_support_t * ts =
    rclcpp::type_support::get_set_parameters_srv_type_support();
  ASSERT_NE(nullptr, ts);
  EXPECT_TRUE(test_type_support_init_fini(ts));
}

TEST_F(TestTypeSupport, test_service_ts_list_params_srv) {
  const rosidl_service_type_support_t * ts =
    rclcpp::type_support::get_list_parameters_srv_type_support();
  ASSERT_NE(nullptr, ts);
  EXPECT_TRUE(test_type_support_init_fini(ts));
}

TEST_F(TestTypeSupport, test_service_ts_describe_params_srv) {
  const rosidl_service_type_support_t * ts =
    rclcpp::type_support::get_describe_parameters_srv_type_support();
  ASSERT_NE(nullptr, ts);
  EXPECT_TRUE(test_type_support_init_fini(ts));
}

TEST_F(TestTypeSupport, test_service_ts_set_params_atomically_srv) {
  const rosidl_service_type_support_t * ts =
    rclcpp::type_support::get_set_parameters_atomically_srv_type_support();
  ASSERT_NE(nullptr, ts);
  EXPECT_TRUE(test_type_support_init_fini(ts));
}
