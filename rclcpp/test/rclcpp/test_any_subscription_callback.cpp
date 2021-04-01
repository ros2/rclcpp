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

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/any_subscription_callback.hpp"
#include "test_msgs/msg/empty.hpp"
#include "test_msgs/msg/empty.h"

class TestAnySubscriptionCallback : public ::testing::Test
{
public:
  TestAnySubscriptionCallback() {}

  static
  std::unique_ptr<test_msgs::msg::Empty>
  get_unique_ptr_msg()
  {
    return std::make_unique<test_msgs::msg::Empty>();
  }

protected:
  rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty> any_subscription_callback_;
  std::shared_ptr<test_msgs::msg::Empty> msg_shared_ptr_{std::make_shared<test_msgs::msg::Empty>()};
  rclcpp::MessageInfo message_info_;
};

void construct_with_null_allocator()
{
// suppress deprecated function warning
#if !defined(_WIN32)
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#else  // !defined(_WIN32)
# pragma warning(push)
# pragma warning(disable: 4996)
#endif

  // We need to wrap this in a function because `EXPECT_THROW` is a macro, and thinks
  // that the comma in here splits macro arguments, not the template arguments.
  rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty> any_subscription_callback(nullptr);

// remove warning suppression
#if !defined(_WIN32)
# pragma GCC diagnostic pop
#else  // !defined(_WIN32)
# pragma warning(pop)
#endif
}

TEST(AnySubscriptionCallback, null_allocator) {
  EXPECT_THROW(
    construct_with_null_allocator(),
    std::runtime_error);
}

TEST_F(TestAnySubscriptionCallback, construct_destruct) {
  // Default constructor.
  rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty> asc1;

  // Constructor with allocator.
  std::allocator<void> allocator;
  rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty> asc2(allocator);
}

TEST_F(TestAnySubscriptionCallback, unset_dispatch_throw) {
  EXPECT_THROW(
    any_subscription_callback_.dispatch(msg_shared_ptr_, message_info_),
    std::runtime_error);
  EXPECT_THROW(
    any_subscription_callback_.dispatch_intra_process(msg_shared_ptr_, message_info_),
    std::runtime_error);
  EXPECT_THROW(
    any_subscription_callback_.dispatch_intra_process(get_unique_ptr_msg(), message_info_),
    std::runtime_error);
}

//
// Parameterized test to test across all callback types and dispatch types.
//

class InstanceContextImpl
{
public:
  InstanceContextImpl() = default;
  virtual ~InstanceContextImpl() = default;

  explicit InstanceContextImpl(rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty> asc)
  : any_subscription_callback_(asc)
  {}

  virtual
  rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty>
  get_any_subscription_callback_to_test() const
  {
    return any_subscription_callback_;
  }

protected:
  rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty> any_subscription_callback_;
};

class InstanceContext
{
public:
  InstanceContext(const std::string & name, std::shared_ptr<InstanceContextImpl> impl)
  : name(name), impl_(impl)
  {}

  InstanceContext(
    const std::string & name,
    rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty> asc)
  : name(name), impl_(std::make_shared<InstanceContextImpl>(asc))
  {}

  InstanceContext(const InstanceContext & other)
  : InstanceContext(other.name, other.impl_) {}

  rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty>
  get_any_subscription_callback_to_test() const
  {
    return impl_->get_any_subscription_callback_to_test();
  }

  std::string name;

protected:
  std::shared_ptr<InstanceContextImpl> impl_;
};

class DispatchTests
  : public TestAnySubscriptionCallback,
  public ::testing::WithParamInterface<InstanceContext>
{};

auto
format_parameter(const ::testing::TestParamInfo<DispatchTests::ParamType> & info)
{
  return info.param.name;
}

// Testing dispatch with shared_ptr<MessageT> as input
TEST_P(DispatchTests, test_inter_shared_dispatch) {
  auto any_subscription_callback_to_test = GetParam().get_any_subscription_callback_to_test();
  any_subscription_callback_to_test.dispatch(msg_shared_ptr_, message_info_);
}

// Testing dispatch with shared_ptr<const MessageT> as input
TEST_P(DispatchTests, test_intra_shared_dispatch) {
  auto any_subscription_callback_to_test = GetParam().get_any_subscription_callback_to_test();
  any_subscription_callback_to_test.dispatch_intra_process(msg_shared_ptr_, message_info_);
}

// Testing dispatch with unique_ptr<MessageT> as input
TEST_P(DispatchTests, test_intra_unique_dispatch) {
  auto any_subscription_callback_to_test = GetParam().get_any_subscription_callback_to_test();
  any_subscription_callback_to_test.dispatch_intra_process(get_unique_ptr_msg(), message_info_);
}

// Generic classes for testing callbacks using std::bind to class methods.
template<typename ... CallbackArgs>
class BindContextImpl : public InstanceContextImpl
{
  static constexpr size_t number_of_callback_args{sizeof...(CallbackArgs)};

public:
  using InstanceContextImpl::InstanceContextImpl;
  virtual ~BindContextImpl() = default;

  void on_message(CallbackArgs ...) const {}

  rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty>
  get_any_subscription_callback_to_test() const override
  {
    if constexpr (number_of_callback_args == 1) {
      return rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty>().set(
        std::bind(&BindContextImpl::on_message, this, std::placeholders::_1)
      );
    } else {
      return rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty>().set(
        std::bind(&BindContextImpl::on_message, this, std::placeholders::_1, std::placeholders::_2)
      );
    }
  }
};

template<typename ... CallbackArgs>
class BindContext : public InstanceContext
{
public:
  explicit BindContext(const std::string & name)
  : InstanceContext(name, std::make_shared<BindContextImpl<CallbackArgs ...>>())
  {}
};

//
// Versions of `const MessageT &`
//
void const_ref_free_func(const test_msgs::msg::Empty &) {}
void const_ref_w_info_free_func(const test_msgs::msg::Empty &, const rclcpp::MessageInfo &) {}

INSTANTIATE_TEST_SUITE_P(
  ConstRefCallbackTests,
  DispatchTests,
  ::testing::Values(
    // lambda
    InstanceContext{"lambda", rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty>().set(
        [](const test_msgs::msg::Empty &) {})},
    InstanceContext{"lambda_with_info",
      rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty>().set(
        [](const test_msgs::msg::Empty &, const rclcpp::MessageInfo &) {})},
    // free function
    InstanceContext{"free_function", rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty>().set(
        const_ref_free_func)},
    InstanceContext{"free_function_with_info",
      rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty>().set(
        const_ref_w_info_free_func)},
    // bind function
    BindContext<const test_msgs::msg::Empty &>("bind_method"),
    BindContext<const test_msgs::msg::Empty &, const rclcpp::MessageInfo &>(
      "bind_method_with_info")
  ),
  format_parameter
);

//
// Versions of `std::unique_ptr<MessageT, MessageDeleter>`
//
void unique_ptr_free_func(std::unique_ptr<test_msgs::msg::Empty>) {}
void unique_ptr_w_info_free_func(
  std::unique_ptr<test_msgs::msg::Empty>, const rclcpp::MessageInfo &)
{}

INSTANTIATE_TEST_SUITE_P(
  UniquePtrCallbackTests,
  DispatchTests,
  ::testing::Values(
    // lambda
    InstanceContext{"lambda", rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty>().set(
        [](std::unique_ptr<test_msgs::msg::Empty>) {})},
    InstanceContext{"lambda_with_info",
      rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty>().set(
        [](std::unique_ptr<test_msgs::msg::Empty>, const rclcpp::MessageInfo &) {})},
    // free function
    InstanceContext{"free_function", rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty>().set(
        unique_ptr_free_func)},
    InstanceContext{"free_function_with_info",
      rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty>().set(
        unique_ptr_w_info_free_func)},
    // bind function
    BindContext<std::unique_ptr<test_msgs::msg::Empty>>("bind_method"),
    BindContext<std::unique_ptr<test_msgs::msg::Empty>, const rclcpp::MessageInfo &>(
      "bind_method_with_info")
  ),
  format_parameter
);

//
// Versions of `std::shared_ptr<const MessageT>`
//
void shared_const_ptr_free_func(std::shared_ptr<const test_msgs::msg::Empty>) {}
void shared_const_ptr_w_info_free_func(
  std::shared_ptr<const test_msgs::msg::Empty>, const rclcpp::MessageInfo &)
{}

INSTANTIATE_TEST_SUITE_P(
  SharedConstPtrCallbackTests,
  DispatchTests,
  ::testing::Values(
    // lambda
    InstanceContext{"lambda", rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty>().set(
        [](std::shared_ptr<const test_msgs::msg::Empty>) {})},
    InstanceContext{"lambda_with_info",
      rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty>().set(
        [](std::shared_ptr<const test_msgs::msg::Empty>, const rclcpp::MessageInfo &) {})},
    // free function
    InstanceContext{"free_function", rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty>().set(
        shared_const_ptr_free_func)},
    InstanceContext{"free_function_with_info",
      rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty>().set(
        shared_const_ptr_w_info_free_func)},
    // bind function
    BindContext<std::shared_ptr<const test_msgs::msg::Empty>>("bind_method"),
    BindContext<std::shared_ptr<const test_msgs::msg::Empty>, const rclcpp::MessageInfo &>(
      "bind_method_with_info")
  ),
  format_parameter
);

//
// Versions of `const std::shared_ptr<const MessageT> &`
//
void const_ref_shared_const_ptr_free_func(const std::shared_ptr<const test_msgs::msg::Empty> &) {}
void const_ref_shared_const_ptr_w_info_free_func(
  const std::shared_ptr<const test_msgs::msg::Empty> &, const rclcpp::MessageInfo &)
{}

INSTANTIATE_TEST_SUITE_P(
  ConstRefSharedConstPtrCallbackTests,
  DispatchTests,
  ::testing::Values(
    // lambda
    InstanceContext{"lambda", rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty>().set(
        [](const std::shared_ptr<const test_msgs::msg::Empty> &) {})},
    InstanceContext{"lambda_with_info",
      rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty>().set(
        [](const std::shared_ptr<const test_msgs::msg::Empty> &, const rclcpp::MessageInfo &) {})},
    // free function
    InstanceContext{"free_function", rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty>().set(
        const_ref_shared_const_ptr_free_func)},
    InstanceContext{"free_function_with_info",
      rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty>().set(
        const_ref_shared_const_ptr_w_info_free_func)},
    // bind function
    BindContext<const std::shared_ptr<const test_msgs::msg::Empty> &>("bind_method"),
    BindContext<const std::shared_ptr<const test_msgs::msg::Empty> &, const rclcpp::MessageInfo &>(
      "bind_method_with_info")
  ),
  format_parameter
);

//
// Versions of `std::shared_ptr<MessageT>`
//
void shared_ptr_free_func(std::shared_ptr<test_msgs::msg::Empty>) {}
void shared_ptr_w_info_free_func(
  std::shared_ptr<test_msgs::msg::Empty>, const rclcpp::MessageInfo &)
{}

INSTANTIATE_TEST_SUITE_P(
  SharedPtrCallbackTests,
  DispatchTests,
  ::testing::Values(
    // lambda
    InstanceContext{"lambda", rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty>().set(
        [](std::shared_ptr<test_msgs::msg::Empty>) {})},
    InstanceContext{"lambda_with_info",
      rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty>().set(
        [](std::shared_ptr<test_msgs::msg::Empty>, const rclcpp::MessageInfo &) {})},
    // free function
    InstanceContext{"free_function", rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty>().set(
        shared_ptr_free_func)},
    InstanceContext{"free_function_with_info",
      rclcpp::AnySubscriptionCallback<test_msgs::msg::Empty>().set(
        shared_ptr_w_info_free_func)},
    // bind function
    BindContext<std::shared_ptr<test_msgs::msg::Empty>>("bind_method"),
    BindContext<std::shared_ptr<test_msgs::msg::Empty>, const rclcpp::MessageInfo &>(
      "bind_method_with_info")
  ),
  format_parameter
);
