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

#include "tracetools/tracetools.h"
#include "tracetools/utils.hpp"

#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/detail/subscription_callback_type_helper.hpp"
#include "rclcpp/function_traits.hpp"
#include "rclcpp/message_info.hpp"

template<class>
inline constexpr bool always_false_v = false;

namespace rclcpp
{

namespace detail
{

template<typename MessageT, typename AllocatorT>
struct MessageDeleterHelper
{
  using MessageAllocTraits = allocator::AllocRebind<MessageT, AllocatorT>;
  using MessageAlloc = typename MessageAllocTraits::allocator_type;
  using MessageDeleter = allocator::Deleter<MessageAlloc, MessageT>;
};

template<typename MessageT, typename AllocatorT>
struct AnySubscriptionCallbackHelper
{
  using MessageDeleter = typename MessageDeleterHelper<MessageT, AllocatorT>::MessageDeleter;

  using ConstRefCallback =
    std::function<void (const MessageT &)>;
  using ConstRefWithInfoCallback =
    std::function<void (const MessageT &, const rclcpp::MessageInfo &)>;

  using UniquePtrCallback =
    std::function<void (std::unique_ptr<MessageT, MessageDeleter>)>;
  using UniquePtrWithInfoCallback =
    std::function<void (std::unique_ptr<MessageT, MessageDeleter>, const rclcpp::MessageInfo &)>;

  using SharedConstPtrCallback =
    std::function<void (std::shared_ptr<const MessageT>)>;
  using SharedConstPtrWithInfoCallback =
    std::function<void (std::shared_ptr<const MessageT>, const rclcpp::MessageInfo &)>;

  using ConstRefSharedConstPtrCallback =
    std::function<void (const std::shared_ptr<const MessageT> &)>;
  using ConstRefSharedConstPtrWithInfoCallback =
    std::function<void (const std::shared_ptr<const MessageT> &, const rclcpp::MessageInfo &)>;

  // Deprecated signatures:
  using SharedPtrCallback =
    std::function<void (std::shared_ptr<MessageT>)>;
  using SharedPtrWithInfoCallback =
    std::function<void (std::shared_ptr<MessageT>, const rclcpp::MessageInfo &)>;

  using variant_type = std::variant<
    ConstRefCallback,
    ConstRefWithInfoCallback,
    UniquePtrCallback,
    UniquePtrWithInfoCallback,
    SharedConstPtrCallback,
    SharedConstPtrWithInfoCallback,
    ConstRefSharedConstPtrCallback,
    ConstRefSharedConstPtrWithInfoCallback,
    SharedPtrCallback,
    SharedPtrWithInfoCallback
  >;
};

}  // namespace detail

template<
  typename MessageT,
  typename AllocatorT
>
class AnySubscriptionCallback
{
private:
  using HelperT = typename rclcpp::detail::AnySubscriptionCallbackHelper<MessageT, AllocatorT>;
  using MessageDeleterHelper = rclcpp::detail::MessageDeleterHelper<MessageT, AllocatorT>;

  // using variant_type = HelperT::variant_type;

  // TODO(wjwwood): switch to inheriting from std::variant (i.e. HelperT::variant_type) once
  // inheriting from std::variant is realistic (maybe C++23?), see:
  //   http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2020/p2162r0.html
  // For now, compose the variant into this class as a private attribute.
  typename HelperT::variant_type callback_variant_;

  using MessageAllocTraits = typename MessageDeleterHelper::MessageAllocTraits;
  using MessageAlloc = typename MessageDeleterHelper::MessageAlloc;
  using MessageDeleter = typename MessageDeleterHelper::MessageDeleter;

  // See AnySubscriptionCallbackHelper for the types of these.
  using ConstRefCallback = typename HelperT::ConstRefCallback;
  using ConstRefWithInfoCallback = typename HelperT::ConstRefWithInfoCallback;

  using UniquePtrCallback = typename HelperT::UniquePtrCallback;
  using UniquePtrWithInfoCallback = typename HelperT::UniquePtrWithInfoCallback;

  using SharedConstPtrCallback = typename HelperT::SharedConstPtrCallback;
  using SharedConstPtrWithInfoCallback = typename HelperT::SharedConstPtrWithInfoCallback;

  using ConstRefSharedConstPtrCallback =
    typename HelperT::ConstRefSharedConstPtrCallback;
  using ConstRefSharedConstPtrWithInfoCallback =
    typename HelperT::ConstRefSharedConstPtrWithInfoCallback;

  using SharedPtrCallback = typename HelperT::SharedPtrCallback;
  using SharedPtrWithInfoCallback = typename HelperT::SharedPtrWithInfoCallback;

public:
  explicit
  AnySubscriptionCallback(const AllocatorT & allocator = AllocatorT())  // NOLINT[runtime/explicit]
  {
    message_allocator_ = allocator;
    allocator::set_allocator_for_deleter(&message_deleter_, &message_allocator_);
  }

  [[deprecated("use AnySubscriptionCallback(const AllocatorT & allocator) instead")]]
  explicit
  AnySubscriptionCallback(std::shared_ptr<AllocatorT> allocator)  // NOLINT[runtime/explicit]
  {
    if (allocator == nullptr) {
      throw std::runtime_error("invalid allocator");
    }
    message_allocator_ = *allocator;
    allocator::set_allocator_for_deleter(&message_deleter_, &message_allocator_);
  }

  AnySubscriptionCallback(const AnySubscriptionCallback &) = default;

  /// Generic function for setting the callback.
  /**
   * There are specializations that overload this in order to deprecate some
   * callback signatures, and also to fix ambiguity between shared_ptr and
   * unique_ptr callback signatures when using them with lambda functions.
   */
  template<typename CallbackT>
  void
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
  }

  /// Function for shared_ptr to non-const MessageT, which is deprecated.
  [[deprecated(
    "use 'void (std::shared_ptr<const MessageT>)' instead"
  )]]
  void
  set_deprecated(std::function<void(std::shared_ptr<MessageT>)> callback)
  // set(CallbackT callback)
  {
    callback_variant_ = callback;
  }

  /// Function for shared_ptr to non-const MessageT with MessageInfo, which is deprecated.
  [[deprecated(
    "use 'void (std::shared_ptr<const MessageT>, const rclcpp::MessageInfo &)' instead"
  )]]
  void
  set_deprecated(
    std::function<void(std::shared_ptr<MessageT>, const rclcpp::MessageInfo &)> callback)
  {
    callback_variant_ = callback;
  }

  void
  dispatch(
    std::shared_ptr<MessageT> message,
    const rclcpp::MessageInfo & message_info)
  {
    TRACEPOINT(callback_start, static_cast<const void *>(this), false);
    std::visit(
      [&message, &message_info, this](auto && callback) {
        using T = std::decay_t<decltype(callback)>;

        auto create_unique_ptr_from_shared_ptr_message =
        [this](const std::shared_ptr<MessageT> & message) {
          auto ptr = MessageAllocTraits::allocate(message_allocator_, 1);
          MessageAllocTraits::construct(message_allocator_, ptr, *message);
          return std::unique_ptr<MessageT, MessageDeleter>(ptr, message_deleter_);
        };

        if constexpr (std::is_same_v<T, ConstRefCallback>) {
          callback(*message);
        } else if constexpr (std::is_same_v<T, ConstRefWithInfoCallback>) {
          callback(*message, message_info);
        } else if constexpr (std::is_same_v<T, UniquePtrCallback>) {
          callback(create_unique_ptr_from_shared_ptr_message(message));
        } else if constexpr (std::is_same_v<T, UniquePtrWithInfoCallback>) {
          callback(create_unique_ptr_from_shared_ptr_message(message), message_info);
        } else if constexpr (  // NOLINT[readability/braces]
          std::is_same_v<T, SharedConstPtrCallback>||
          std::is_same_v<T, ConstRefSharedConstPtrCallback>||
          std::is_same_v<T, SharedPtrCallback>)
        {
          callback(message);
        } else if constexpr (  // NOLINT[readability/braces]
          std::is_same_v<T, SharedConstPtrWithInfoCallback>||
          std::is_same_v<T, ConstRefSharedConstPtrWithInfoCallback>||
          std::is_same_v<T, SharedPtrWithInfoCallback>)
        {
          callback(message, message_info);
        }
      }, callback_variant_);
    TRACEPOINT(callback_end, static_cast<const void *>(this));
  }

  void
  dispatch_intra_process(
    std::shared_ptr<const MessageT> message,
    const rclcpp::MessageInfo & message_info)
  {
    TRACEPOINT(callback_start, static_cast<const void *>(this), true);
    std::visit(
      [&message, &message_info](auto && callback) {
        using T = std::decay_t<decltype(callback)>;

        if constexpr (
          std::is_same_v<T, SharedConstPtrCallback>||
          std::is_same_v<T, ConstRefSharedConstPtrCallback>) {
          callback(message);
        } else if constexpr (  // NOLINT[readability/braces]
          std::is_same_v<T, SharedConstPtrWithInfoCallback>||
          std::is_same_v<T, ConstRefSharedConstPtrWithInfoCallback>) {
          callback(message, message_info);
        } else {
          throw std::runtime_error(
            "unexpected dispatch_intra_process const shared "
            "message call with no const shared_ptr callback");
        }
      }, callback_variant_);
    TRACEPOINT(callback_end, static_cast<const void *>(this));
  }

  void
  dispatch_intra_process(
    std::unique_ptr<MessageT, MessageDeleter> message,
    const rclcpp::MessageInfo & message_info)
  {
    TRACEPOINT(callback_start, static_cast<const void *>(this), true);
    std::visit(
      [&message, &message_info](auto && callback) {
        using T = std::decay_t<decltype(callback)>;

        if constexpr (
          std::is_same_v<T, SharedConstPtrCallback>||
          std::is_same_v<T, ConstRefSharedConstPtrCallback>) {
          callback(std::move(message));
        } else if constexpr (  // NOLINT[readability/braces]
          std::is_same_v<T, SharedConstPtrWithInfoCallback>||
          std::is_same_v<T, ConstRefSharedConstPtrWithInfoCallback>) {
          callback(std::move(message), message_info);
        } else if constexpr (std::is_same_v<T, UniquePtrCallback>) {
          callback(std::move(message));
        } else if constexpr (std::is_same_v<T, UniquePtrWithInfoCallback>) {
          callback(std::move(message), message_info);
        } else {
          throw std::runtime_error(
            "unexpected dispatch_intra_process unique message call"
            " with const shared_ptr callback");
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

private:
  MessageAlloc message_allocator_;
  MessageDeleter message_deleter_;
};

}  // namespace rclcpp

#endif  // RCLCPP__ANY_SUBSCRIPTION_CALLBACK_HPP_
