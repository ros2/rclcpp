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

// TODO(aprotyas): Figure out better way to suppress deprecation warnings.
#define RCLCPP_AVOID_DEPRECATIONS_FOR_UNIT_TESTS 1
#include "rclcpp/any_subscription_callback.hpp"
#include "test_msgs/msg/empty.hpp"
#include "test_msgs/msg/empty.h"

// Type adapter to be used in tests.
struct MyEmpty {};

template<>
struct rclcpp::TypeAdapter<MyEmpty, test_msgs::msg::Empty>
{
  using is_specialized = std::true_type;
  using custom_type = MyEmpty;
  using ros_message_type = test_msgs::msg::Empty;

  static
  void
  convert_to_ros_message(const custom_type &, ros_message_type &)
  {}

  static
  void
  convert_to_custom(const ros_message_type &, custom_type &)
  {}
};

using MyTA = rclcpp::TypeAdapter<MyEmpty, test_msgs::msg::Empty>;

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

class TestAnySubscriptionCallbackTA : public ::testing::Test
{
public:
  TestAnySubscriptionCallbackTA() {}

  static
  std::unique_ptr<MyEmpty>
  get_unique_ptr_msg()
  {
    return std::make_unique<MyEmpty>();
  }

protected:
  rclcpp::AnySubscriptionCallback<MyEmpty> any_subscription_callback_;
  std::shared_ptr<MyEmpty> msg_shared_ptr_{std::make_shared<MyEmpty>()};
  rclcpp::MessageInfo message_info_;
};

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

template<typename MessageT>
class InstanceContextImpl
{
public:
  InstanceContextImpl() = default;
  virtual ~InstanceContextImpl() = default;

  explicit InstanceContextImpl(rclcpp::AnySubscriptionCallback<MessageT> asc)
  : any_subscription_callback_(asc)
  {}

  virtual
  rclcpp::AnySubscriptionCallback<MessageT>
  get_any_subscription_callback_to_test() const
  {
    return any_subscription_callback_;
  }

protected:
  rclcpp::AnySubscriptionCallback<MessageT> any_subscription_callback_;
};

template<typename MessageT>
class InstanceContext
{
public:
  InstanceContext(const std::string & name, std::shared_ptr<InstanceContextImpl<MessageT>> impl)
  : name(name), impl_(impl)
  {}

  InstanceContext(
    const std::string & name,
    rclcpp::AnySubscriptionCallback<MessageT> asc)
  : name(name), impl_(std::make_shared<InstanceContextImpl<MessageT>>(asc))
  {}

  InstanceContext(const InstanceContext & other)
  : InstanceContext(other.name, other.impl_) {}

  rclcpp::AnySubscriptionCallback<MessageT>
  get_any_subscription_callback_to_test() const
  {
    return impl_->get_any_subscription_callback_to_test();
  }

  std::string name;

protected:
  std::shared_ptr<InstanceContextImpl<MessageT>> impl_;
};

class DispatchTests
  : public TestAnySubscriptionCallback,
  public ::testing::WithParamInterface<InstanceContext<test_msgs::msg::Empty>>
{};

class DispatchTestsWithTA
  : public TestAnySubscriptionCallbackTA,
  public ::testing::WithParamInterface<InstanceContext<MyTA>>
{};

auto
format_parameter(const ::testing::TestParamInfo<DispatchTests::ParamType> & info)
{
  return info.param.name;
}

auto
format_parameter_with_ta(const ::testing::TestParamInfo<DispatchTestsWithTA::ParamType> & info)
{
  return info.param.name;
}

/* Testing dispatch with shared_ptr<MessageT> as input */
TEST_P(DispatchTests, test_inter_shared_dispatch) {
  auto any_subscription_callback_to_test = GetParam().get_any_subscription_callback_to_test();
  any_subscription_callback_to_test.dispatch(msg_shared_ptr_, message_info_);
}

/* Testing dispatch with shared_ptr<const MessageT> as input */
TEST_P(DispatchTests, test_intra_shared_dispatch) {
  auto any_subscription_callback_to_test = GetParam().get_any_subscription_callback_to_test();
  any_subscription_callback_to_test.dispatch_intra_process(msg_shared_ptr_, message_info_);
}

/* Testing dispatch with unique_ptr<MessageT> as input */
TEST_P(DispatchTests, test_intra_unique_dispatch) {
  auto any_subscription_callback_to_test = GetParam().get_any_subscription_callback_to_test();
  any_subscription_callback_to_test.dispatch_intra_process(get_unique_ptr_msg(), message_info_);
}

/* Testing dispatch with shared_ptr<const MessageT> as input */
TEST_P(DispatchTestsWithTA, test_intra_shared_dispatch) {
  auto any_subscription_callback_to_test = GetParam().get_any_subscription_callback_to_test();
  any_subscription_callback_to_test.dispatch_intra_process(msg_shared_ptr_, message_info_);
}

/* Testing dispatch with unique_ptr<MessageT> as input */
TEST_P(DispatchTestsWithTA, test_intra_unique_dispatch) {
  auto any_subscription_callback_to_test = GetParam().get_any_subscription_callback_to_test();
  any_subscription_callback_to_test.dispatch_intra_process(get_unique_ptr_msg(), message_info_);
}

// Generic classes for testing callbacks using std::bind to class methods.
template<typename MessageT, typename ... CallbackArgs>
class BindContextImpl : public InstanceContextImpl<MessageT>
{
  static constexpr size_t number_of_callback_args{sizeof...(CallbackArgs)};

public:
  using InstanceContextImpl<MessageT>::InstanceContextImpl;
  virtual ~BindContextImpl() = default;

  void on_message(CallbackArgs ...) const {}

  rclcpp::AnySubscriptionCallback<MessageT>
  get_any_subscription_callback_to_test() const override
  {
    if constexpr (number_of_callback_args == 1) {
      return rclcpp::AnySubscriptionCallback<MessageT>().set(
        std::bind(&BindContextImpl::on_message, this, std::placeholders::_1)
      );
    } else {
      return rclcpp::AnySubscriptionCallback<MessageT>().set(
        std::bind(&BindContextImpl::on_message, this, std::placeholders::_1, std::placeholders::_2)
      );
    }
  }
};

template<typename MessageT, typename ... CallbackArgs>
class BindContext : public InstanceContext<MessageT>
{
public:
  explicit BindContext(const std::string & name)
  : InstanceContext<MessageT>(name, std::make_shared<BindContextImpl<MessageT, CallbackArgs ...>>())
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
    BindContext<test_msgs::msg::Empty, const test_msgs::msg::Empty &>("bind_method"),
    BindContext<test_msgs::msg::Empty, const test_msgs::msg::Empty &, const rclcpp::MessageInfo &>(
      "bind_method_with_info")
  ),
  format_parameter
);

void const_ref_ta_free_func(const MyEmpty &) {}
void const_ref_ta_w_info_free_func(const MyEmpty &, const rclcpp::MessageInfo &) {}

INSTANTIATE_TEST_SUITE_P(
  ConstRefTACallbackTests,
  DispatchTestsWithTA,
  ::testing::Values(
    // lambda
    InstanceContext<MyTA>{"lambda_ta", rclcpp::AnySubscriptionCallback<MyTA>().set(
        [](const MyEmpty &) {})},
    InstanceContext<MyTA>{"lambda_ta_with_info",
      rclcpp::AnySubscriptionCallback<MyTA>().set(
        [](const MyEmpty &, const rclcpp::MessageInfo &) {})},
    InstanceContext<MyTA>{"lambda", rclcpp::AnySubscriptionCallback<MyTA>().set(
        [](const test_msgs::msg::Empty &) {})},
    InstanceContext<MyTA>{"lambda_with_info",
      rclcpp::AnySubscriptionCallback<MyTA>().set(
        [](const test_msgs::msg::Empty &, const rclcpp::MessageInfo &) {})},
    // free function
    InstanceContext<MyTA>{"free_function_ta", rclcpp::AnySubscriptionCallback<MyTA>().set(
        const_ref_ta_free_func)},
    InstanceContext<MyTA>{"free_function_ta_with_info",
      rclcpp::AnySubscriptionCallback<MyTA>().set(
        const_ref_ta_w_info_free_func)},
    InstanceContext<MyTA>{"free_function", rclcpp::AnySubscriptionCallback<MyTA>().set(
        const_ref_free_func)},
    InstanceContext<MyTA>{"free_function_with_info",
      rclcpp::AnySubscriptionCallback<MyTA>().set(
        const_ref_w_info_free_func)},
    // bind function
    BindContext<MyTA, const MyEmpty &>("bind_method_ta"),
    BindContext<MyTA, const MyEmpty &, const rclcpp::MessageInfo &>(
      "bind_method_ta_with_info"),
    BindContext<MyTA, const test_msgs::msg::Empty &>("bind_method"),
    BindContext<MyTA, const test_msgs::msg::Empty &, const rclcpp::MessageInfo &>(
      "bind_method_with_info")
  ),
  format_parameter_with_ta
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
    BindContext<test_msgs::msg::Empty, std::unique_ptr<test_msgs::msg::Empty>>("bind_method"),
    BindContext<
      test_msgs::msg::Empty,
      std::unique_ptr<test_msgs::msg::Empty>,
      const rclcpp::MessageInfo &
    >("bind_method_with_info")
  ),
  format_parameter
);

void unique_ptr_ta_free_func(std::unique_ptr<MyEmpty>) {}
void unique_ptr_ta_w_info_free_func(std::unique_ptr<MyEmpty>, const rclcpp::MessageInfo &) {}

INSTANTIATE_TEST_SUITE_P(
  UniquePtrCallbackTests,
  DispatchTestsWithTA,
  ::testing::Values(
    // lambda
    InstanceContext<MyTA>{"lambda_ta", rclcpp::AnySubscriptionCallback<MyTA>().set(
        [](std::unique_ptr<MyEmpty>) {})},
    InstanceContext<MyTA>{"lambda_ta_with_info",
      rclcpp::AnySubscriptionCallback<MyTA>().set(
        [](std::unique_ptr<MyEmpty>, const rclcpp::MessageInfo &) {})},
    InstanceContext<MyTA>{"lambda", rclcpp::AnySubscriptionCallback<MyTA>().set(
        [](std::unique_ptr<test_msgs::msg::Empty>) {})},
    InstanceContext<MyTA>{"lambda_with_info",
      rclcpp::AnySubscriptionCallback<MyTA>().set(
        [](std::unique_ptr<test_msgs::msg::Empty>, const rclcpp::MessageInfo &) {})},
    // free function
    InstanceContext<MyTA>{"free_function_ta", rclcpp::AnySubscriptionCallback<MyTA>().set(
        unique_ptr_ta_free_func)},
    InstanceContext<MyTA>{"free_function_ta_with_info",
      rclcpp::AnySubscriptionCallback<MyTA>().set(
        unique_ptr_ta_w_info_free_func)},
    InstanceContext<MyTA>{"free_function", rclcpp::AnySubscriptionCallback<MyTA>().set(
        unique_ptr_free_func)},
    InstanceContext<MyTA>{"free_function_with_info",
      rclcpp::AnySubscriptionCallback<MyTA>().set(
        unique_ptr_w_info_free_func)},
    // bind function
    BindContext<MyTA, std::unique_ptr<test_msgs::msg::Empty>>("bind_method_ta"),
    BindContext<MyTA, std::unique_ptr<test_msgs::msg::Empty>, const rclcpp::MessageInfo &>(
      "bind_method_ta_with_info"),
    BindContext<MyTA, std::unique_ptr<test_msgs::msg::Empty>>("bind_method"),
    BindContext<MyTA, std::unique_ptr<test_msgs::msg::Empty>, const rclcpp::MessageInfo &>(
      "bind_method_with_info")
  ),
  format_parameter_with_ta
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
    BindContext<test_msgs::msg::Empty, std::shared_ptr<const test_msgs::msg::Empty>>("bind_method"),
    BindContext<test_msgs::msg::Empty, std::shared_ptr<const test_msgs::msg::Empty>,
    const rclcpp::MessageInfo &>(
      "bind_method_with_info")
  ),
  format_parameter
);

void shared_const_ptr_ta_free_func(std::shared_ptr<const MyEmpty>) {}
void shared_const_ptr_ta_w_info_free_func(
  std::shared_ptr<const MyEmpty>, const rclcpp::MessageInfo &)
{}

INSTANTIATE_TEST_SUITE_P(
  SharedConstPtrCallbackTests,
  DispatchTestsWithTA,
  ::testing::Values(
    // lambda
    InstanceContext<MyTA>{"lambda_ta", rclcpp::AnySubscriptionCallback<MyTA>().set(
        [](std::shared_ptr<const MyEmpty>) {})},
    InstanceContext<MyTA>{"lambda_ta_with_info",
      rclcpp::AnySubscriptionCallback<MyTA>().set(
        [](std::shared_ptr<const MyEmpty>, const rclcpp::MessageInfo &) {})},
    InstanceContext<MyTA>{"lambda", rclcpp::AnySubscriptionCallback<MyTA>().set(
        [](std::shared_ptr<const test_msgs::msg::Empty>) {})},
    InstanceContext<MyTA>{"lambda_with_info",
      rclcpp::AnySubscriptionCallback<MyTA>().set(
        [](std::shared_ptr<const test_msgs::msg::Empty>, const rclcpp::MessageInfo &) {})},
    // free function
    InstanceContext<MyTA>{"free_function_ta", rclcpp::AnySubscriptionCallback<MyTA>().set(
        shared_const_ptr_ta_free_func)},
    InstanceContext<MyTA>{"free_function_ta_with_info",
      rclcpp::AnySubscriptionCallback<MyTA>().set(
        shared_const_ptr_ta_w_info_free_func)},
    InstanceContext<MyTA>{"free_function", rclcpp::AnySubscriptionCallback<MyTA>().set(
        shared_const_ptr_free_func)},
    InstanceContext<MyTA>{"free_function_with_info",
      rclcpp::AnySubscriptionCallback<MyTA>().set(
        shared_const_ptr_w_info_free_func)},
    // bind function
    BindContext<MyTA, std::shared_ptr<const MyEmpty>>("bind_method_ta"),
    BindContext<MyTA, std::shared_ptr<const MyEmpty>, const rclcpp::MessageInfo &>(
      "bind_method_ta_with_info"),
    BindContext<MyTA, std::shared_ptr<const test_msgs::msg::Empty>>("bind_method"),
    BindContext<MyTA, std::shared_ptr<const test_msgs::msg::Empty>, const rclcpp::MessageInfo &>(
      "bind_method_with_info")
  ),
  format_parameter_with_ta
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
    BindContext<test_msgs::msg::Empty,
    const std::shared_ptr<const test_msgs::msg::Empty> &>("bind_method"),
    BindContext<test_msgs::msg::Empty, const std::shared_ptr<const test_msgs::msg::Empty> &,
    const rclcpp::MessageInfo &>(
      "bind_method_with_info")
  ),
  format_parameter
);

void const_ref_shared_const_ptr_ta_free_func(const std::shared_ptr<const MyEmpty> &) {}
void const_ref_shared_const_ptr_ta_w_info_free_func(
  const std::shared_ptr<const MyEmpty> &, const rclcpp::MessageInfo &)
{}

INSTANTIATE_TEST_SUITE_P(
  ConstRefSharedConstPtrCallbackTests,
  DispatchTestsWithTA,
  ::testing::Values(
    // lambda
    InstanceContext<MyTA>{"lambda_ta", rclcpp::AnySubscriptionCallback<MyTA>().set(
        [](const std::shared_ptr<const MyEmpty> &) {})},
    InstanceContext<MyTA>{"lambda_ta_with_info",
      rclcpp::AnySubscriptionCallback<MyTA>().set(
        [](const std::shared_ptr<const MyEmpty> &, const rclcpp::MessageInfo &) {})},
    InstanceContext<MyTA>{"lambda", rclcpp::AnySubscriptionCallback<MyTA>().set(
        [](const std::shared_ptr<const test_msgs::msg::Empty> &) {})},
    InstanceContext<MyTA>{"lambda_with_info",
      rclcpp::AnySubscriptionCallback<MyTA>().set(
        [](const std::shared_ptr<const test_msgs::msg::Empty> &, const rclcpp::MessageInfo &) {})},
    // free function
    InstanceContext<MyTA>{"free_function_ta", rclcpp::AnySubscriptionCallback<MyTA>().set(
        const_ref_shared_const_ptr_ta_free_func)},
    InstanceContext<MyTA>{"free_function_ta_with_info",
      rclcpp::AnySubscriptionCallback<MyTA>().set(
        const_ref_shared_const_ptr_ta_w_info_free_func)},
    InstanceContext<MyTA>{"free_function", rclcpp::AnySubscriptionCallback<MyTA>().set(
        const_ref_shared_const_ptr_free_func)},
    InstanceContext<MyTA>{"free_function_with_info",
      rclcpp::AnySubscriptionCallback<MyTA>().set(
        const_ref_shared_const_ptr_w_info_free_func)},
    // bind function
    BindContext<MyTA, const std::shared_ptr<const MyEmpty> &>("bind_method_ta"),
    BindContext<MyTA, const std::shared_ptr<const MyEmpty> &, const rclcpp::MessageInfo &>(
      "bind_method_ta_with_info"),
    BindContext<MyTA, const std::shared_ptr<const test_msgs::msg::Empty> &>("bind_method"),
    BindContext<MyTA, const std::shared_ptr<const test_msgs::msg::Empty> &,
    const rclcpp::MessageInfo &>(
      "bind_method_with_info")
  ),
  format_parameter_with_ta
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
    BindContext<test_msgs::msg::Empty, std::shared_ptr<test_msgs::msg::Empty>>("bind_method"),
    BindContext<test_msgs::msg::Empty, std::shared_ptr<test_msgs::msg::Empty>,
    const rclcpp::MessageInfo &>(
      "bind_method_with_info")
  ),
  format_parameter
);

void shared_ptr_ta_free_func(std::shared_ptr<MyEmpty>) {}
void shared_ptr_ta_w_info_free_func(
  std::shared_ptr<MyEmpty>, const rclcpp::MessageInfo &)
{}

INSTANTIATE_TEST_SUITE_P(
  SharedPtrCallbackTests,
  DispatchTestsWithTA,
  ::testing::Values(
    // lambda
    InstanceContext<MyTA>{"lambda_ta", rclcpp::AnySubscriptionCallback<MyTA>().set(
        [](std::shared_ptr<MyEmpty>) {})},
    InstanceContext<MyTA>{"lambda_ta_with_info",
      rclcpp::AnySubscriptionCallback<MyTA>().set(
        [](std::shared_ptr<MyEmpty>, const rclcpp::MessageInfo &) {})},
    InstanceContext<MyTA>{"lambda", rclcpp::AnySubscriptionCallback<MyTA>().set(
        [](std::shared_ptr<test_msgs::msg::Empty>) {})},
    InstanceContext<MyTA>{"lambda_with_info",
      rclcpp::AnySubscriptionCallback<MyTA>().set(
        [](std::shared_ptr<test_msgs::msg::Empty>, const rclcpp::MessageInfo &) {})},
    // free function
    InstanceContext<MyTA>{"free_function_ta", rclcpp::AnySubscriptionCallback<MyTA>().set(
        shared_ptr_ta_free_func)},
    InstanceContext<MyTA>{"free_function_ta_with_info",
      rclcpp::AnySubscriptionCallback<MyTA>().set(
        shared_ptr_ta_w_info_free_func)},
    InstanceContext<MyTA>{"free_function", rclcpp::AnySubscriptionCallback<MyTA>().set(
        shared_ptr_free_func)},
    InstanceContext<MyTA>{"free_function_with_info",
      rclcpp::AnySubscriptionCallback<MyTA>().set(
        shared_ptr_w_info_free_func)},
    // bind function
    BindContext<MyTA, std::shared_ptr<MyEmpty>>("bind_method_ta"),
    BindContext<MyTA, std::shared_ptr<MyEmpty>, const rclcpp::MessageInfo &>(
      "bind_method_ta_with_info"),
    BindContext<MyTA, std::shared_ptr<test_msgs::msg::Empty>>("bind_method"),
    BindContext<MyTA, std::shared_ptr<test_msgs::msg::Empty>, const rclcpp::MessageInfo &>(
      "bind_method_with_info")
  ),
  format_parameter_with_ta
);
