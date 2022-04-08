// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__ANY_SUBSCRIPTION_CALLBACK_HPP_
#define RCLCPP__ANY_SUBSCRIPTION_CALLBACK_HPP_

#include <functional>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <variant>  // NOLINT[build/include_order]

#include "rosidl_runtime_cpp/traits.hpp"
#include "tracetools/tracetools.h"
#include "tracetools/utils.hpp"

#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/detail/subscription_callback_type_helper.hpp"
#include "rclcpp/function_traits.hpp"
#include "rclcpp/message_info.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/type_adapter.hpp"


template<class>
inline constexpr bool always_false_v = false;

namespace rclcpp
{

namespace detail
{

template<typename MessageT, typename AllocatorT>
struct MessageDeleterHelper
{
  using AllocTraits = allocator::AllocRebind<MessageT, AllocatorT>;
  using Alloc = typename AllocTraits::allocator_type;
  using Deleter = allocator::Deleter<Alloc, MessageT>;
};

/// Struct which contains all possible callback signatures, with or without a TypeAdapter.s
template<typename MessageT, typename AllocatorT>
struct AnySubscriptionCallbackPossibleTypes
{
  /// MessageT::custom_type if MessageT is a TypeAdapter, otherwise just MessageT.
  using SubscribedType = typename rclcpp::TypeAdapter<MessageT>::custom_type;
  /// MessageT::ros_message_type if MessageT is a TypeAdapter, otherwise just MessageT.
  using ROSMessageType = typename rclcpp::TypeAdapter<MessageT>::ros_message_type;

  using SubscribedMessageDeleter =
    typename MessageDeleterHelper<SubscribedType, AllocatorT>::Deleter;
  using ROSMessageDeleter =
    typename MessageDeleterHelper<ROSMessageType, AllocatorT>::Deleter;
  using SerializedMessageDeleter =
    typename MessageDeleterHelper<rclcpp::SerializedMessage, AllocatorT>::Deleter;

  using ConstRefCallback =
    std::function<void (const SubscribedType &)>;
  using ConstRefROSMessageCallback =
    std::function<void (const ROSMessageType &)>;
  using ConstRefWithInfoCallback =
    std::function<void (const SubscribedType &, const rclcpp::MessageInfo &)>;
  using ConstRefWithInfoROSMessageCallback =
    std::function<void (const ROSMessageType &, const rclcpp::MessageInfo &)>;
  using ConstRefSerializedMessageCallback =
    std::function<void (const rclcpp::SerializedMessage &)>;
  using ConstRefSerializedMessageWithInfoCallback =
    std::function<void (const rclcpp::SerializedMessage &, const rclcpp::MessageInfo &)>;

  using UniquePtrCallback =
    std::function<void (std::unique_ptr<SubscribedType, SubscribedMessageDeleter>)>;
  using UniquePtrROSMessageCallback =
    std::function<void (std::unique_ptr<ROSMessageType, ROSMessageDeleter>)>;
  using UniquePtrWithInfoCallback =
    std::function<void (
        std::unique_ptr<SubscribedType, SubscribedMessageDeleter>,
        const rclcpp::MessageInfo &)>;
  using UniquePtrWithInfoROSMessageCallback =
    std::function<void (
        std::unique_ptr<ROSMessageType, ROSMessageDeleter>,
        const rclcpp::MessageInfo &)>;
  using UniquePtrSerializedMessageCallback =
    std::function<void (std::unique_ptr<rclcpp::SerializedMessage, SerializedMessageDeleter>)>;
  using UniquePtrSerializedMessageWithInfoCallback =
    std::function<void (
        std::unique_ptr<rclcpp::SerializedMessage, SerializedMessageDeleter>,
        const rclcpp::MessageInfo &)>;

  using SharedConstPtrCallback =
    std::function<void (std::shared_ptr<const SubscribedType>)>;
  using SharedConstPtrROSMessageCallback =
    std::function<void (std::shared_ptr<const ROSMessageType>)>;
  using SharedConstPtrWithInfoCallback =
    std::function<void (
        std::shared_ptr<const SubscribedType>,
        const rclcpp::MessageInfo &)>;
  using SharedConstPtrWithInfoROSMessageCallback =
    std::function<void (
        std::shared_ptr<const ROSMessageType>,
        const rclcpp::MessageInfo &)>;
  using SharedConstPtrSerializedMessageCallback =
    std::function<void (std::shared_ptr<const rclcpp::SerializedMessage>)>;
  using SharedConstPtrSerializedMessageWithInfoCallback =
    std::function<void (
        std::shared_ptr<const rclcpp::SerializedMessage>,
        const rclcpp::MessageInfo &)>;

  using ConstRefSharedConstPtrCallback =
    std::function<void (const std::shared_ptr<const SubscribedType> &)>;
  using ConstRefSharedConstPtrROSMessageCallback =
    std::function<void (const std::shared_ptr<const ROSMessageType> &)>;
  using ConstRefSharedConstPtrWithInfoCallback =
    std::function<void (
        const std::shared_ptr<const SubscribedType> &,
        const rclcpp::MessageInfo &)>;
  using ConstRefSharedConstPtrWithInfoROSMessageCallback =
    std::function<void (
        const std::shared_ptr<const ROSMessageType> &,
        const rclcpp::MessageInfo &)>;
  using ConstRefSharedConstPtrSerializedMessageCallback =
    std::function<void (const std::shared_ptr<const rclcpp::SerializedMessage> &)>;
  using ConstRefSharedConstPtrSerializedMessageWithInfoCallback =
    std::function<void (
        const std::shared_ptr<const rclcpp::SerializedMessage> &,
        const rclcpp::MessageInfo &)>;

  // Deprecated signatures:
  using SharedPtrCallback =
    std::function<void (std::shared_ptr<SubscribedType>)>;
  using SharedPtrROSMessageCallback =
    std::function<void (std::shared_ptr<ROSMessageType>)>;
  using SharedPtrWithInfoCallback =
    std::function<void (std::shared_ptr<SubscribedType>, const rclcpp::MessageInfo &)>;
  using SharedPtrWithInfoROSMessageCallback =
    std::function<void (
        std::shared_ptr<ROSMessageType>,
        const rclcpp::MessageInfo &)>;
  using SharedPtrSerializedMessageCallback =
    std::function<void (std::shared_ptr<rclcpp::SerializedMessage>)>;
  using SharedPtrSerializedMessageWithInfoCallback =
    std::function<void (std::shared_ptr<rclcpp::SerializedMessage>, const rclcpp::MessageInfo &)>;
};

/// Template helper to select the variant type based on whether or not MessageT is a TypeAdapter.
template<
  typename MessageT,
  typename AllocatorT,
  bool is_adapted_type = rclcpp::TypeAdapter<MessageT>::is_specialized::value
>
struct AnySubscriptionCallbackHelper;

/// Specialization for when MessageT is not a TypeAdapter.
template<typename MessageT, typename AllocatorT>
struct AnySubscriptionCallbackHelper<MessageT, AllocatorT, false>
{
  using CallbackTypes = AnySubscriptionCallbackPossibleTypes<MessageT, AllocatorT>;

  using variant_type = std::variant<
    typename CallbackTypes::ConstRefCallback,
    typename CallbackTypes::ConstRefWithInfoCallback,
    typename CallbackTypes::ConstRefSerializedMessageCallback,
    typename CallbackTypes::ConstRefSerializedMessageWithInfoCallback,
    typename CallbackTypes::UniquePtrCallback,
    typename CallbackTypes::UniquePtrWithInfoCallback,
    typename CallbackTypes::UniquePtrSerializedMessageCallback,
    typename CallbackTypes::UniquePtrSerializedMessageWithInfoCallback,
    typename CallbackTypes::SharedConstPtrCallback,
    typename CallbackTypes::SharedConstPtrWithInfoCallback,
    typename CallbackTypes::SharedConstPtrSerializedMessageCallback,
    typename CallbackTypes::SharedConstPtrSerializedMessageWithInfoCallback,
    typename CallbackTypes::ConstRefSharedConstPtrCallback,
    typename CallbackTypes::ConstRefSharedConstPtrWithInfoCallback,
    typename CallbackTypes::ConstRefSharedConstPtrSerializedMessageCallback,
    typename CallbackTypes::ConstRefSharedConstPtrSerializedMessageWithInfoCallback,
    typename CallbackTypes::SharedPtrCallback,
    typename CallbackTypes::SharedPtrWithInfoCallback,
    typename CallbackTypes::SharedPtrSerializedMessageCallback,
    typename CallbackTypes::SharedPtrSerializedMessageWithInfoCallback
  >;
};

/// Specialization for when MessageT is a TypeAdapter.
template<typename MessageT, typename AllocatorT>
struct AnySubscriptionCallbackHelper<MessageT, AllocatorT, true>
{
  using CallbackTypes = AnySubscriptionCallbackPossibleTypes<MessageT, AllocatorT>;

  using variant_type = std::variant<
    typename CallbackTypes::ConstRefCallback,
    typename CallbackTypes::ConstRefROSMessageCallback,
    typename CallbackTypes::ConstRefWithInfoCallback,
    typename CallbackTypes::ConstRefWithInfoROSMessageCallback,
    typename CallbackTypes::ConstRefSerializedMessageCallback,
    typename CallbackTypes::ConstRefSerializedMessageWithInfoCallback,
    typename CallbackTypes::UniquePtrCallback,
    typename CallbackTypes::UniquePtrROSMessageCallback,
    typename CallbackTypes::UniquePtrWithInfoCallback,
    typename CallbackTypes::UniquePtrWithInfoROSMessageCallback,
    typename CallbackTypes::UniquePtrSerializedMessageCallback,
    typename CallbackTypes::UniquePtrSerializedMessageWithInfoCallback,
    typename CallbackTypes::SharedConstPtrCallback,
    typename CallbackTypes::SharedConstPtrROSMessageCallback,
    typename CallbackTypes::SharedConstPtrWithInfoCallback,
    typename CallbackTypes::SharedConstPtrWithInfoROSMessageCallback,
    typename CallbackTypes::SharedConstPtrSerializedMessageCallback,
    typename CallbackTypes::SharedConstPtrSerializedMessageWithInfoCallback,
    typename CallbackTypes::ConstRefSharedConstPtrCallback,
    typename CallbackTypes::ConstRefSharedConstPtrROSMessageCallback,
    typename CallbackTypes::ConstRefSharedConstPtrWithInfoCallback,
    typename CallbackTypes::ConstRefSharedConstPtrWithInfoROSMessageCallback,
    typename CallbackTypes::ConstRefSharedConstPtrSerializedMessageCallback,
    typename CallbackTypes::ConstRefSharedConstPtrSerializedMessageWithInfoCallback,
    typename CallbackTypes::SharedPtrCallback,
    typename CallbackTypes::SharedPtrROSMessageCallback,
    typename CallbackTypes::SharedPtrWithInfoCallback,
    typename CallbackTypes::SharedPtrWithInfoROSMessageCallback,
    typename CallbackTypes::SharedPtrSerializedMessageCallback,
    typename CallbackTypes::SharedPtrSerializedMessageWithInfoCallback
  >;
};

}  // namespace detail

template<
  typename MessageT,
  typename AllocatorT = std::allocator<void>
>
class AnySubscriptionCallback
{
private:
  /// MessageT::custom_type if MessageT is a TypeAdapter, otherwise just MessageT.
  using SubscribedType = typename rclcpp::TypeAdapter<MessageT>::custom_type;
  /// MessageT::ros_message_type if MessageT is a TypeAdapter, otherwise just MessageT.
  using ROSMessageType = typename rclcpp::TypeAdapter<MessageT>::ros_message_type;

  using HelperT = typename rclcpp::detail::AnySubscriptionCallbackHelper<MessageT, AllocatorT>;

  using SubscribedTypeDeleterHelper =
    rclcpp::detail::MessageDeleterHelper<SubscribedType, AllocatorT>;
  using SubscribedTypeAllocatorTraits = typename SubscribedTypeDeleterHelper::AllocTraits;
  using SubscribedTypeAllocator = typename SubscribedTypeDeleterHelper::Alloc;
  using SubscribedTypeDeleter = typename SubscribedTypeDeleterHelper::Deleter;

  using ROSMessageTypeDeleterHelper =
    rclcpp::detail::MessageDeleterHelper<ROSMessageType, AllocatorT>;
  using ROSMessageTypeAllocatorTraits = typename ROSMessageTypeDeleterHelper::AllocTraits;
  using ROSMessageTypeAllocator = typename ROSMessageTypeDeleterHelper::Alloc;
  using ROSMessageTypeDeleter = typename ROSMessageTypeDeleterHelper::Deleter;

  using SerializedMessageDeleterHelper =
    rclcpp::detail::MessageDeleterHelper<rclcpp::SerializedMessage, AllocatorT>;
  using SerializedMessageAllocatorTraits = typename SerializedMessageDeleterHelper::AllocTraits;
  using SerializedMessageAllocator = typename SerializedMessageDeleterHelper::Alloc;
  using SerializedMessageDeleter = typename SerializedMessageDeleterHelper::Deleter;

  // See AnySubscriptionCallbackPossibleTypes for the types of these.
  using CallbackTypes = detail::AnySubscriptionCallbackPossibleTypes<MessageT, AllocatorT>;

  using ConstRefCallback =
    typename CallbackTypes::ConstRefCallback;
  using ConstRefROSMessageCallback =
    typename CallbackTypes::ConstRefROSMessageCallback;
  using ConstRefWithInfoCallback =
    typename CallbackTypes::ConstRefWithInfoCallback;
  using ConstRefWithInfoROSMessageCallback =
    typename CallbackTypes::ConstRefWithInfoROSMessageCallback;
  using ConstRefSerializedMessageCallback =
    typename CallbackTypes::ConstRefSerializedMessageCallback;
  using ConstRefSerializedMessageWithInfoCallback =
    typename CallbackTypes::ConstRefSerializedMessageWithInfoCallback;
  using UniquePtrCallback =
    typename CallbackTypes::UniquePtrCallback;
  using UniquePtrROSMessageCallback =
    typename CallbackTypes::UniquePtrROSMessageCallback;
  using UniquePtrWithInfoCallback =
    typename CallbackTypes::UniquePtrWithInfoCallback;
  using UniquePtrWithInfoROSMessageCallback =
    typename CallbackTypes::UniquePtrWithInfoROSMessageCallback;
  using UniquePtrSerializedMessageCallback =
    typename CallbackTypes::UniquePtrSerializedMessageCallback;
  using UniquePtrSerializedMessageWithInfoCallback =
    typename CallbackTypes::UniquePtrSerializedMessageWithInfoCallback;
  using SharedConstPtrCallback =
    typename CallbackTypes::SharedConstPtrCallback;
  using SharedConstPtrROSMessageCallback =
    typename CallbackTypes::SharedConstPtrROSMessageCallback;
  using SharedConstPtrWithInfoCallback =
    typename CallbackTypes::SharedConstPtrWithInfoCallback;
  using SharedConstPtrWithInfoROSMessageCallback =
    typename CallbackTypes::SharedConstPtrWithInfoROSMessageCallback;
  using SharedConstPtrSerializedMessageCallback =
    typename CallbackTypes::SharedConstPtrSerializedMessageCallback;
  using SharedConstPtrSerializedMessageWithInfoCallback =
    typename CallbackTypes::SharedConstPtrSerializedMessageWithInfoCallback;
  using ConstRefSharedConstPtrCallback =
    typename CallbackTypes::ConstRefSharedConstPtrCallback;
  using ConstRefSharedConstPtrROSMessageCallback =
    typename CallbackTypes::ConstRefSharedConstPtrROSMessageCallback;
  using ConstRefSharedConstPtrWithInfoCallback =
    typename CallbackTypes::ConstRefSharedConstPtrWithInfoCallback;
  using ConstRefSharedConstPtrWithInfoROSMessageCallback =
    typename CallbackTypes::ConstRefSharedConstPtrWithInfoROSMessageCallback;
  using ConstRefSharedConstPtrSerializedMessageCallback =
    typename CallbackTypes::ConstRefSharedConstPtrSerializedMessageCallback;
  using ConstRefSharedConstPtrSerializedMessageWithInfoCallback =
    typename CallbackTypes::ConstRefSharedConstPtrSerializedMessageWithInfoCallback;
  using SharedPtrCallback =
    typename CallbackTypes::SharedPtrCallback;
  using SharedPtrROSMessageCallback =
    typename CallbackTypes::SharedPtrROSMessageCallback;
  using SharedPtrWithInfoCallback =
    typename CallbackTypes::SharedPtrWithInfoCallback;
  using SharedPtrWithInfoROSMessageCallback =
    typename CallbackTypes::SharedPtrWithInfoROSMessageCallback;
  using SharedPtrSerializedMessageCallback =
    typename CallbackTypes::SharedPtrSerializedMessageCallback;
  using SharedPtrSerializedMessageWithInfoCallback =
    typename CallbackTypes::SharedPtrSerializedMessageWithInfoCallback;

  template<typename T>
  struct NotNull
  {
    NotNull(const T * pointer_in, const char * msg)
    : pointer(pointer_in)
    {
      if (pointer == nullptr) {
        throw std::invalid_argument(msg);
      }
    }

    const T * pointer;
  };

public:
  explicit
  AnySubscriptionCallback(const AllocatorT & allocator = AllocatorT())  // NOLINT[runtime/explicit]
  : subscribed_type_allocator_(allocator),
    ros_message_type_allocator_(allocator)
  {
    allocator::set_allocator_for_deleter(&subscribed_type_deleter_, &subscribed_type_allocator_);
    allocator::set_allocator_for_deleter(&ros_message_type_deleter_, &ros_message_type_allocator_);
  }

  AnySubscriptionCallback(const AnySubscriptionCallback &) = default;

  /// Generic function for setting the callback.
  /**
   * There are specializations that overload this in order to deprecate some
   * callback signatures, and also to fix ambiguity between shared_ptr and
   * unique_ptr callback signatures when using them with lambda functions.
   */
  template<typename CallbackT>
  AnySubscriptionCallback<MessageT, AllocatorT>
  set(CallbackT callback)
  {
    // Use the SubscriptionCallbackTypeHelper to determine the actual type of
    // the CallbackT, in terms of std::function<...>, which does not happen
    // automatically with lambda functions in cases where the arguments can be
    // converted to one another, e.g. shared_ptr and unique_ptr.
    using scbth = detail::SubscriptionCallbackTypeHelper<MessageT, CallbackT>;

    // Determine if the given CallbackT is a deprecated signature or not.
    constexpr auto is_deprecated =
      rclcpp::function_traits::same_arguments<
      typename scbth::callback_type,
      std::function<void(std::shared_ptr<MessageT>)>
      >::value
      ||
      rclcpp::function_traits::same_arguments<
      typename scbth::callback_type,
      std::function<void(std::shared_ptr<MessageT>, const rclcpp::MessageInfo &)>
      >::value;

    // Use the discovered type to force the type of callback when assigning
    // into the variant.
    if constexpr (is_deprecated) {
      // If deprecated, call sub-routine that is deprecated.
      set_deprecated(static_cast<typename scbth::callback_type>(callback));
    } else {
      // Otherwise just assign it.
      callback_variant_ = static_cast<typename scbth::callback_type>(callback);
    }

    // Return copy of self for easier testing, normally will be compiled out.
    return *this;
  }

  /// Function for shared_ptr to non-const MessageT, which is deprecated.
  template<typename SetT>
#if !defined(RCLCPP_AVOID_DEPRECATIONS_FOR_UNIT_TESTS)
  // suppress deprecation warnings in `test_any_subscription_callback.cpp`
  [[deprecated("use 'void(std::shared_ptr<const MessageT>)' instead")]]
#endif
  void
  set_deprecated(std::function<void(std::shared_ptr<SetT>)> callback)
  {
    callback_variant_ = callback;
  }

  /// Function for shared_ptr to non-const MessageT with MessageInfo, which is deprecated.
  template<typename SetT>
#if !defined(RCLCPP_AVOID_DEPRECATIONS_FOR_UNIT_TESTS)
  // suppress deprecation warnings in `test_any_subscription_callback.cpp`
  [[deprecated(
          "use 'void(std::shared_ptr<const MessageT>, const rclcpp::MessageInfo &)' instead"
  )]]
#endif
  void
  set_deprecated(std::function<void(std::shared_ptr<SetT>, const rclcpp::MessageInfo &)> callback)
  {
    callback_variant_ = callback;
  }

  std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter>
  create_ros_unique_ptr_from_ros_shared_ptr_message(
    const std::shared_ptr<const ROSMessageType> & message)
  {
    auto ptr = ROSMessageTypeAllocatorTraits::allocate(ros_message_type_allocator_, 1);
    ROSMessageTypeAllocatorTraits::construct(ros_message_type_allocator_, ptr, *message);
    return std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter>(ptr, ros_message_type_deleter_);
  }

  std::unique_ptr<rclcpp::SerializedMessage, SerializedMessageDeleter>
  create_serialized_message_unique_ptr_from_shared_ptr(
    const std::shared_ptr<const rclcpp::SerializedMessage> & serialized_message)
  {
    auto ptr = SerializedMessageAllocatorTraits::allocate(serialized_message_allocator_, 1);
    SerializedMessageAllocatorTraits::construct(
      serialized_message_allocator_, ptr, *serialized_message);
    return std::unique_ptr<
      rclcpp::SerializedMessage,
      SerializedMessageDeleter
    >(ptr, serialized_message_deleter_);
  }

  std::unique_ptr<SubscribedType, SubscribedTypeDeleter>
  create_custom_unique_ptr_from_custom_shared_ptr_message(
    const std::shared_ptr<const SubscribedType> & message)
  {
    auto ptr = SubscribedTypeAllocatorTraits::allocate(subscribed_type_allocator_, 1);
    SubscribedTypeAllocatorTraits::construct(subscribed_type_allocator_, ptr, *message);
    return std::unique_ptr<SubscribedType, SubscribedTypeDeleter>(ptr, subscribed_type_deleter_);
  }

  std::unique_ptr<SubscribedType, SubscribedTypeDeleter>
  convert_ros_message_to_custom_type_unique_ptr(const ROSMessageType & msg)
  {
    if constexpr (rclcpp::TypeAdapter<MessageT>::is_specialized::value) {
      auto ptr = SubscribedTypeAllocatorTraits::allocate(subscribed_type_allocator_, 1);
      SubscribedTypeAllocatorTraits::construct(subscribed_type_allocator_, ptr);
      rclcpp::TypeAdapter<MessageT>::convert_to_custom(msg, *ptr);
      return std::unique_ptr<SubscribedType, SubscribedTypeDeleter>(ptr, subscribed_type_deleter_);
    } else {
      throw std::runtime_error(
              "convert_ros_message_to_custom_type_unique_ptr "
              "unexpectedly called without TypeAdapter");
    }
  }

  std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter>
  convert_custom_type_to_ros_message_unique_ptr(const SubscribedType & msg)
  {
    if constexpr (rclcpp::TypeAdapter<MessageT>::is_specialized::value) {
      auto ptr = ROSMessageTypeAllocatorTraits::allocate(ros_message_type_allocator_, 1);
      ROSMessageTypeAllocatorTraits::construct(ros_message_type_allocator_, ptr);
      rclcpp::TypeAdapter<MessageT>::convert_to_ros_message(msg, *ptr);
      return std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter>(ptr, ros_message_type_deleter_);
    } else {
      static_assert(
        !sizeof(MessageT *),
        "convert_custom_type_to_ros_message_unique_ptr() "
        "unexpectedly called without specialized TypeAdapter");
    }
  }

  // Dispatch when input is a ros message and the output could be anything.
  void
  dispatch(
    std::shared_ptr<ROSMessageType> message,
    const rclcpp::MessageInfo & message_info)
  {
    TRACEPOINT(callback_start, static_cast<const void *>(this), false);
    // Check if the variant is "unset", throw if it is.
    if (callback_variant_.index() == 0) {
      if (std::get<0>(callback_variant_) == nullptr) {
        // This can happen if it is default initialized, or if it is assigned nullptr.
        throw std::runtime_error("dispatch called on an unset AnySubscriptionCallback");
      }
    }
    // Dispatch.
    std::visit(
      [&message, &message_info, this](auto && callback) {
        using T = std::decay_t<decltype(callback)>;
        static constexpr bool is_ta = rclcpp::TypeAdapter<MessageT>::is_specialized::value;

        // conditions for output is custom message
        if constexpr (is_ta && std::is_same_v<T, ConstRefCallback>) {
          // TODO(wjwwood): consider avoiding heap allocation for small messages
          //   maybe something like:
          // if constexpr (rosidl_generator_traits::has_fixed_size<T> && sizeof(T) < N) {
          //   ... on stack
          // }
          auto local_message = convert_ros_message_to_custom_type_unique_ptr(*message);
          callback(*local_message);
        } else if constexpr (is_ta && std::is_same_v<T, ConstRefWithInfoCallback>) {  // NOLINT
          auto local_message = convert_ros_message_to_custom_type_unique_ptr(*message);
          callback(*local_message, message_info);
        } else if constexpr (is_ta && std::is_same_v<T, UniquePtrCallback>) {
          callback(convert_ros_message_to_custom_type_unique_ptr(*message));
        } else if constexpr (is_ta && std::is_same_v<T, UniquePtrWithInfoCallback>) {
          callback(convert_ros_message_to_custom_type_unique_ptr(*message), message_info);
        } else if constexpr (  // NOLINT[readability/braces]
          is_ta && (
            std::is_same_v<T, SharedConstPtrCallback>||
            std::is_same_v<T, ConstRefSharedConstPtrCallback>||
            std::is_same_v<T, SharedPtrCallback>
        ))
        {
          callback(convert_ros_message_to_custom_type_unique_ptr(*message));
        } else if constexpr (  // NOLINT[readability/braces]
          is_ta && (
            std::is_same_v<T, SharedConstPtrWithInfoCallback>||
            std::is_same_v<T, ConstRefSharedConstPtrWithInfoCallback>||
            std::is_same_v<T, SharedPtrWithInfoCallback>
        ))
        {
          callback(convert_ros_message_to_custom_type_unique_ptr(*message), message_info);
        }
        // conditions for output is ros message
        else if constexpr (std::is_same_v<T, ConstRefROSMessageCallback>) {  // NOLINT
          callback(*message);
        } else if constexpr (std::is_same_v<T, ConstRefWithInfoROSMessageCallback>) {
          callback(*message, message_info);
        } else if constexpr (std::is_same_v<T, UniquePtrROSMessageCallback>) {
          callback(create_ros_unique_ptr_from_ros_shared_ptr_message(message));
        } else if constexpr (std::is_same_v<T, UniquePtrWithInfoROSMessageCallback>) {
          callback(create_ros_unique_ptr_from_ros_shared_ptr_message(message), message_info);
        } else if constexpr (  // NOLINT[readability/braces]
          std::is_same_v<T, SharedConstPtrROSMessageCallback>||
          std::is_same_v<T, ConstRefSharedConstPtrROSMessageCallback>||
          std::is_same_v<T, SharedPtrROSMessageCallback>)
        {
          callback(message);
        } else if constexpr (  // NOLINT[readability/braces]
          std::is_same_v<T, SharedConstPtrWithInfoROSMessageCallback>||
          std::is_same_v<T, ConstRefSharedConstPtrWithInfoROSMessageCallback>||
          std::is_same_v<T, SharedPtrWithInfoROSMessageCallback>)
        {
          callback(message, message_info);
        }
        // condition to catch SerializedMessage types
        else if constexpr (  // NOLINT[readability/braces]
          std::is_same_v<T, ConstRefSerializedMessageCallback>||
          std::is_same_v<T, ConstRefSerializedMessageWithInfoCallback>||
          std::is_same_v<T, UniquePtrSerializedMessageCallback>||
          std::is_same_v<T, UniquePtrSerializedMessageWithInfoCallback>||
          std::is_same_v<T, SharedConstPtrSerializedMessageCallback>||
          std::is_same_v<T, SharedConstPtrSerializedMessageWithInfoCallback>||
          std::is_same_v<T, ConstRefSharedConstPtrSerializedMessageCallback>||
          std::is_same_v<T, ConstRefSharedConstPtrSerializedMessageWithInfoCallback>||
          std::is_same_v<T, SharedPtrSerializedMessageCallback>||
          std::is_same_v<T, SharedPtrSerializedMessageWithInfoCallback>)
        {
          throw std::runtime_error(
            "Cannot dispatch std::shared_ptr<ROSMessageType> message "
            "to rclcpp::SerializedMessage");
        }
        // condition to catch unhandled callback types
        else {  // NOLINT[readability/braces]
          static_assert(always_false_v<T>, "unhandled callback type");
        }
      }, callback_variant_);
    TRACEPOINT(callback_end, static_cast<const void *>(this));
  }

  // Dispatch when input is a serialized message and the output could be anything.
  void
  dispatch(
    std::shared_ptr<rclcpp::SerializedMessage> serialized_message,
    const rclcpp::MessageInfo & message_info)
  {
    TRACEPOINT(callback_start, static_cast<const void *>(this), false);
    // Check if the variant is "unset", throw if it is.
    if (callback_variant_.index() == 0) {
      if (std::get<0>(callback_variant_) == nullptr) {
        // This can happen if it is default initialized, or if it is assigned nullptr.
        throw std::runtime_error("dispatch called on an unset AnySubscriptionCallback");
      }
    }
    // Dispatch.
    std::visit(
      [&serialized_message, &message_info, this](auto && callback) {
        using T = std::decay_t<decltype(callback)>;

        // condition to catch SerializedMessage types
        if constexpr (std::is_same_v<T, ConstRefSerializedMessageCallback>) {
          callback(*serialized_message);
        } else if constexpr (std::is_same_v<T, ConstRefSerializedMessageWithInfoCallback>) {
          callback(*serialized_message, message_info);
        } else if constexpr (std::is_same_v<T, UniquePtrSerializedMessageCallback>) {
          callback(create_serialized_message_unique_ptr_from_shared_ptr(serialized_message));
        } else if constexpr (std::is_same_v<T, UniquePtrSerializedMessageWithInfoCallback>) {
          callback(
            create_serialized_message_unique_ptr_from_shared_ptr(serialized_message),
            message_info);
        } else if constexpr (  // NOLINT[readability/braces]
          std::is_same_v<T, SharedConstPtrSerializedMessageCallback>||
          std::is_same_v<T, ConstRefSharedConstPtrSerializedMessageCallback>||
          std::is_same_v<T, SharedPtrSerializedMessageCallback>)
        {
          callback(create_serialized_message_unique_ptr_from_shared_ptr(serialized_message));
        } else if constexpr (  // NOLINT[readability/braces]
          std::is_same_v<T, SharedConstPtrSerializedMessageWithInfoCallback>||
          std::is_same_v<T, ConstRefSharedConstPtrSerializedMessageWithInfoCallback>||
          std::is_same_v<T, SharedPtrSerializedMessageWithInfoCallback>)
        {
          callback(
            create_serialized_message_unique_ptr_from_shared_ptr(serialized_message),
            message_info);
        }
        // conditions for output anything else
        else if constexpr (  // NOLINT[whitespace/newline]
          std::is_same_v<T, ConstRefCallback>||
          std::is_same_v<T, ConstRefROSMessageCallback>||
          std::is_same_v<T, ConstRefWithInfoCallback>||
          std::is_same_v<T, ConstRefWithInfoROSMessageCallback>||
          std::is_same_v<T, UniquePtrCallback>||
          std::is_same_v<T, UniquePtrROSMessageCallback>||
          std::is_same_v<T, UniquePtrWithInfoCallback>||
          std::is_same_v<T, UniquePtrWithInfoROSMessageCallback>||
          std::is_same_v<T, SharedConstPtrCallback>||
          std::is_same_v<T, SharedConstPtrROSMessageCallback>||
          std::is_same_v<T, SharedConstPtrWithInfoCallback>||
          std::is_same_v<T, SharedConstPtrWithInfoROSMessageCallback>||
          std::is_same_v<T, ConstRefSharedConstPtrCallback>||
          std::is_same_v<T, ConstRefSharedConstPtrROSMessageCallback>||
          std::is_same_v<T, ConstRefSharedConstPtrWithInfoCallback>||
          std::is_same_v<T, ConstRefSharedConstPtrWithInfoROSMessageCallback>||
          std::is_same_v<T, SharedPtrCallback>||
          std::is_same_v<T, SharedPtrROSMessageCallback>||
          std::is_same_v<T, SharedPtrWithInfoCallback>||
          std::is_same_v<T, SharedPtrWithInfoROSMessageCallback>)
        {
          throw std::runtime_error(
            "cannot dispatch rclcpp::SerializedMessage to "
            "non-rclcpp::SerializedMessage callbacks");
        }
        // condition to catch unhandled callback types
        else {  // NOLINT[readability/braces]
          static_assert(always_false_v<T>, "unhandled callback type");
        }
      }, callback_variant_);
    TRACEPOINT(callback_end, static_cast<const void *>(this));
  }

  void
  dispatch_intra_process(
    std::shared_ptr<const SubscribedType> message,
    const rclcpp::MessageInfo & message_info)
  {
    TRACEPOINT(callback_start, static_cast<const void *>(this), true);
    // Check if the variant is "unset", throw if it is.
    if (callback_variant_.index() == 0) {
      if (std::get<0>(callback_variant_) == nullptr) {
        // This can happen if it is default initialized, or if it is assigned nullptr.
        throw std::runtime_error("dispatch called on an unset AnySubscriptionCallback");
      }
    }
    // Dispatch.
    std::visit(
      [&message, &message_info, this](auto && callback) {
        using T = std::decay_t<decltype(callback)>;
        static constexpr bool is_ta = rclcpp::TypeAdapter<MessageT>::is_specialized::value;

        // conditions for custom type
        if constexpr (is_ta && std::is_same_v<T, ConstRefCallback>) {
          callback(*message);
        } else if constexpr (is_ta && std::is_same_v<T, ConstRefWithInfoCallback>) {  // NOLINT
          callback(*message, message_info);
        } else if constexpr (  // NOLINT[readability/braces]
          is_ta && (
            std::is_same_v<T, UniquePtrCallback>||
            std::is_same_v<T, SharedPtrCallback>
        ))
        {
          callback(create_custom_unique_ptr_from_custom_shared_ptr_message(message));
        } else if constexpr (  // NOLINT[readability/braces]
          is_ta && (
            std::is_same_v<T, UniquePtrWithInfoCallback>||
            std::is_same_v<T, SharedPtrWithInfoCallback>
        ))
        {
          callback(create_custom_unique_ptr_from_custom_shared_ptr_message(message), message_info);
        } else if constexpr (  // NOLINT[readability/braces]
          is_ta && (
            std::is_same_v<T, SharedConstPtrCallback>||
            std::is_same_v<T, ConstRefSharedConstPtrCallback>
        ))
        {
          callback(message);
        } else if constexpr (  // NOLINT[readability/braces]
          is_ta && (
            std::is_same_v<T, SharedConstPtrWithInfoCallback>||
            std::is_same_v<T, ConstRefSharedConstPtrWithInfoCallback>
        ))
        {
          callback(message, message_info);
        }
        // conditions for ros message type
        else if constexpr (std::is_same_v<T, ConstRefROSMessageCallback>) {  // NOLINT[readability/braces]
          if constexpr (is_ta) {
            auto local = convert_custom_type_to_ros_message_unique_ptr(*message);
            callback(*local);
          } else {
            callback(*message);
          }
        } else if constexpr (std::is_same_v<T, ConstRefWithInfoROSMessageCallback>) {  // NOLINT[readability/braces]
          if constexpr (is_ta) {
            auto local = convert_custom_type_to_ros_message_unique_ptr(*message);
            callback(*local, message_info);
          } else {
            callback(*message, message_info);
          }
        } else if constexpr (  // NOLINT[readability/braces]
          std::is_same_v<T, UniquePtrROSMessageCallback>||
          std::is_same_v<T, SharedPtrROSMessageCallback>)
        {
          if constexpr (is_ta) {
            callback(convert_custom_type_to_ros_message_unique_ptr(*message));
          } else {
            callback(create_ros_unique_ptr_from_ros_shared_ptr_message(message));
          }
        } else if constexpr (  // NOLINT[readability/braces]
          std::is_same_v<T, UniquePtrWithInfoROSMessageCallback>||
          std::is_same_v<T, SharedPtrWithInfoROSMessageCallback>)
        {
          if constexpr (is_ta) {
            callback(convert_custom_type_to_ros_message_unique_ptr(*message), message_info);
          } else {
            callback(create_ros_unique_ptr_from_ros_shared_ptr_message(message), message_info);
          }
        } else if constexpr (  // NOLINT[readability/braces]
          std::is_same_v<T, SharedConstPtrROSMessageCallback>||
          std::is_same_v<T, ConstRefSharedConstPtrROSMessageCallback>)
        {
          if constexpr (is_ta) {
            callback(convert_custom_type_to_ros_message_unique_ptr(*message));
          } else {
            callback(message);
          }
        } else if constexpr (  // NOLINT[readability/braces]
          std::is_same_v<T, SharedConstPtrWithInfoROSMessageCallback>||
          std::is_same_v<T, ConstRefSharedConstPtrWithInfoROSMessageCallback>)
        {
          if constexpr (is_ta) {
            callback(convert_custom_type_to_ros_message_unique_ptr(*message), message_info);
          } else {
            callback(message, message_info);
          }
        }
        // condition to catch SerializedMessage types
        else if constexpr (  // NOLINT[readability/braces]
          std::is_same_v<T, ConstRefSerializedMessageCallback>||
          std::is_same_v<T, ConstRefSerializedMessageWithInfoCallback>||
          std::is_same_v<T, UniquePtrSerializedMessageCallback>||
          std::is_same_v<T, UniquePtrSerializedMessageWithInfoCallback>||
          std::is_same_v<T, SharedConstPtrSerializedMessageCallback>||
          std::is_same_v<T, SharedConstPtrSerializedMessageWithInfoCallback>||
          std::is_same_v<T, ConstRefSharedConstPtrSerializedMessageCallback>||
          std::is_same_v<T, ConstRefSharedConstPtrSerializedMessageWithInfoCallback>||
          std::is_same_v<T, SharedPtrSerializedMessageCallback>||
          std::is_same_v<T, SharedPtrSerializedMessageWithInfoCallback>)
        {
          throw std::runtime_error(
            "Cannot dispatch std::shared_ptr<const ROSMessageType> message "
            "to rclcpp::SerializedMessage");
        }
        // condition to catch unhandled callback types
        else {  // NOLINT[readability/braces]
          static_assert(always_false_v<T>, "unhandled callback type");
        }
      }, callback_variant_);
    TRACEPOINT(callback_end, static_cast<const void *>(this));
  }

  void
  dispatch_intra_process(
    std::unique_ptr<SubscribedType, SubscribedTypeDeleter> message,
    const rclcpp::MessageInfo & message_info)
  {
    TRACEPOINT(callback_start, static_cast<const void *>(this), true);
    // Check if the variant is "unset", throw if it is.
    if (callback_variant_.index() == 0) {
      if (std::get<0>(callback_variant_) == nullptr) {
        // This can happen if it is default initialized, or if it is assigned nullptr.
        throw std::runtime_error("dispatch called on an unset AnySubscriptionCallback");
      }
    }
    // Dispatch.
    std::visit(
      [&message, &message_info, this](auto && callback) {
        // clang complains that 'this' lambda capture is unused, which is true
        // in *some* specializations of this template, but not others.  Just
        // quiet it down.
        (void)this;

        using T = std::decay_t<decltype(callback)>;
        static constexpr bool is_ta = rclcpp::TypeAdapter<MessageT>::is_specialized::value;

        // conditions for custom type
        if constexpr (is_ta && std::is_same_v<T, ConstRefCallback>) {
          callback(*message);
        } else if constexpr (is_ta && std::is_same_v<T, ConstRefWithInfoCallback>) {  // NOLINT
          callback(*message, message_info);
        } else if constexpr (  // NOLINT[readability/braces]
          is_ta && (
            std::is_same_v<T, UniquePtrCallback>||
            std::is_same_v<T, SharedPtrCallback>))
        {
          callback(std::move(message));
        } else if constexpr (  // NOLINT[readability/braces]
          is_ta && (
            std::is_same_v<T, UniquePtrWithInfoCallback>||
            std::is_same_v<T, SharedPtrWithInfoCallback>
        ))
        {
          callback(std::move(message), message_info);
        } else if constexpr (  // NOLINT[readability/braces]
          is_ta && (
            std::is_same_v<T, SharedConstPtrCallback>||
            std::is_same_v<T, ConstRefSharedConstPtrCallback>
        ))
        {
          callback(std::move(message));
        } else if constexpr (  // NOLINT[readability/braces]
          is_ta && (
            std::is_same_v<T, SharedConstPtrWithInfoCallback>||
            std::is_same_v<T, ConstRefSharedConstPtrWithInfoCallback>
        ))
        {
          callback(std::move(message), message_info);
        }
        // conditions for ros message type
        else if constexpr (std::is_same_v<T, ConstRefROSMessageCallback>) {  // NOLINT[readability/braces]
          if constexpr (is_ta) {
            auto local = convert_custom_type_to_ros_message_unique_ptr(*message);
            callback(*local);
          } else {
            callback(*message);
          }
        } else if constexpr (std::is_same_v<T, ConstRefWithInfoROSMessageCallback>) {  // NOLINT[readability/braces]
          if constexpr (is_ta) {
            auto local = convert_custom_type_to_ros_message_unique_ptr(*message);
            callback(*local, message_info);
          } else {
            callback(*message, message_info);
          }
        } else if constexpr (  // NOLINT[readability/braces]
          std::is_same_v<T, UniquePtrROSMessageCallback>||
          std::is_same_v<T, SharedPtrROSMessageCallback>)
        {
          if constexpr (is_ta) {
            callback(convert_custom_type_to_ros_message_unique_ptr(*message));
          } else {
            callback(std::move(message));
          }
        } else if constexpr (  // NOLINT[readability/braces]
          std::is_same_v<T, UniquePtrWithInfoROSMessageCallback>||
          std::is_same_v<T, SharedPtrWithInfoROSMessageCallback>)
        {
          if constexpr (is_ta) {
            callback(convert_custom_type_to_ros_message_unique_ptr(*message), message_info);
          } else {
            callback(std::move(message), message_info);
          }
        } else if constexpr (  // NOLINT[readability/braces]
          std::is_same_v<T, SharedConstPtrROSMessageCallback>||
          std::is_same_v<T, ConstRefSharedConstPtrROSMessageCallback>)
        {
          if constexpr (is_ta) {
            callback(convert_custom_type_to_ros_message_unique_ptr(*message));
          } else {
            callback(std::move(message));
          }
        } else if constexpr (  // NOLINT[readability/braces]
          std::is_same_v<T, SharedConstPtrWithInfoROSMessageCallback>||
          std::is_same_v<T, ConstRefSharedConstPtrWithInfoROSMessageCallback>)
        {
          if constexpr (is_ta) {
            callback(convert_custom_type_to_ros_message_unique_ptr(*message), message_info);
          } else {
            callback(std::move(message), message_info);
          }
        }
        // condition to catch SerializedMessage types
        else if constexpr (  // NOLINT[readability/braces]
          std::is_same_v<T, ConstRefSerializedMessageCallback>||
          std::is_same_v<T, ConstRefSerializedMessageWithInfoCallback>||
          std::is_same_v<T, UniquePtrSerializedMessageCallback>||
          std::is_same_v<T, UniquePtrSerializedMessageWithInfoCallback>||
          std::is_same_v<T, SharedConstPtrSerializedMessageCallback>||
          std::is_same_v<T, SharedConstPtrSerializedMessageWithInfoCallback>||
          std::is_same_v<T, ConstRefSharedConstPtrSerializedMessageCallback>||
          std::is_same_v<T, ConstRefSharedConstPtrSerializedMessageWithInfoCallback>||
          std::is_same_v<T, SharedPtrSerializedMessageCallback>||
          std::is_same_v<T, SharedPtrSerializedMessageWithInfoCallback>)
        {
          throw std::runtime_error(
            "Cannot dispatch std::unique_ptr<ROSMessageType, ROSMessageTypeDeleter> message "
            "to rclcpp::SerializedMessage");
        }
        // condition to catch unhandled callback types
        else {  // NOLINT[readability/braces]
          static_assert(always_false_v<T>, "unhandled callback type");
        }
      }, callback_variant_);
    TRACEPOINT(callback_end, static_cast<const void *>(this));
  }

  constexpr
  bool
  use_take_shared_method() const
  {
    return
      std::holds_alternative<SharedConstPtrCallback>(callback_variant_) ||
      std::holds_alternative<SharedConstPtrWithInfoCallback>(callback_variant_) ||
      std::holds_alternative<ConstRefSharedConstPtrCallback>(callback_variant_) ||
      std::holds_alternative<ConstRefSharedConstPtrWithInfoCallback>(callback_variant_);
  }

  constexpr
  bool
  is_serialized_message_callback() const
  {
    return
      std::holds_alternative<ConstRefSerializedMessageCallback>(callback_variant_) ||
      std::holds_alternative<UniquePtrSerializedMessageCallback>(callback_variant_) ||
      std::holds_alternative<SharedConstPtrSerializedMessageCallback>(callback_variant_) ||
      std::holds_alternative<ConstRefSharedConstPtrSerializedMessageCallback>(callback_variant_) ||
      std::holds_alternative<SharedPtrSerializedMessageCallback>(callback_variant_);
  }

  void
  register_callback_for_tracing()
  {
#ifndef TRACETOOLS_DISABLED
    std::visit(
      [this](auto && callback) {
        TRACEPOINT(
          rclcpp_callback_register,
          static_cast<const void *>(this),
          tracetools::get_symbol(callback));
      }, callback_variant_);
#endif  // TRACETOOLS_DISABLED
  }

  typename HelperT::variant_type &
  get_variant()
  {
    return callback_variant_;
  }

  const typename HelperT::variant_type &
  get_variant() const
  {
    return callback_variant_;
  }

private:
  // TODO(wjwwood): switch to inheriting from std::variant (i.e. HelperT::variant_type) once
  // inheriting from std::variant is realistic (maybe C++23?), see:
  //   http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2020/p2162r0.html
  // For now, compose the variant into this class as a private attribute.
  typename HelperT::variant_type callback_variant_;

  SubscribedTypeAllocator subscribed_type_allocator_;
  SubscribedTypeDeleter subscribed_type_deleter_;
  ROSMessageTypeAllocator ros_message_type_allocator_;
  ROSMessageTypeDeleter ros_message_type_deleter_;
  SerializedMessageAllocator serialized_message_allocator_;
  SerializedMessageDeleter serialized_message_deleter_;
};

}  // namespace rclcpp

#endif  // RCLCPP__ANY_SUBSCRIPTION_CALLBACK_HPP_
