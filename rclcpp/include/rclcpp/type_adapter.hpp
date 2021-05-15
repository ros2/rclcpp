// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__TYPE_ADAPTER_HPP_
#define RCLCPP__TYPE_ADAPTER_HPP_

#include <type_traits>

namespace rclcpp
{

/// Template structure used to adapt custom, user-defined types to ROS types.
/**
 * Adapting a custom, user-defined type to a ROS type allows that custom type
 * to be used when publishing and subscribing in ROS.
 *
 * In order to adapt a custom type to a ROS type, the user must create a
 * template specialization of this structure for the custom type.
 * In that specialization they must:
 *
 *   - change `is_specialized` to `std::true_type`,
 *   - specify the custom type with `using custom_type = ...`,
 *   - specify the ROS type with `using ros_message_type = ...`,
 *   - provide static convert functions with the signatures:
 *     - static void convert_to_ros(const custom_type &, ros_message_type &)
 *     - static void convert_to_custom(const ros_message_type &, custom_type &)
 *
 * The convert functions must convert from one type to the other.
 *
 * For example, here is a theoretical example for adapting `std::string` to the
 * `std_msgs::msg::String` ROS message type:
 *
 *     template<>
 *     struct rclcpp::TypeAdapter<std::string, std_msgs::msg::String>
 *     {
 *       using is_specialized = std::true_type;
 *       using custom_type = std::string;
 *       using ros_message_type = std_msgs::msg::String;
 *
 *       static
 *       void
 *       convert_to_ros_message(
 *         const custom_type & source,
 *         ros_message_type & destination)
 *       {
 *         destination.data = source;
 *       }
 *
 *       static
 *       void
 *       convert_to_custom(
 *         const ros_message_type & source,
 *         custom_type & destination)
 *       {
 *         destination = source.data;
 *       }
 *     };
 *
 * The adapter can then be used when creating a publisher or subscription,
 * e.g.:
 *
 *     using MyAdaptedType = TypeAdapter<std::string, std_msgs::msg::String>;
 *     auto pub = node->create_publisher<MyAdaptedType>("topic", 10);
 *     auto sub = node->create_subscription<MyAdaptedType>(
 *       "topic",
 *       10,
 *       [](const std::string & msg) {...});
 *
 * You can also be more declarative by using the adapt_type::as metafunctions,
 * which are a bit less ambiguous to read:
 *
 *     using AdaptedType = rclcpp::adapt_type<std::string>::as<std_msgs::msg::String>;
 *     auto pub = node->create_publisher<AdaptedType>(...);
 *
 * If you wish, you may associate a custom type with a single ROS message type,
 * allowing you to be a bit more brief when creating entities, e.g.:
 *
 *     // First you must declare the association, this is similar to how you
 *     // would avoid using the namespace in C++ by doing `using std::vector;`.
 *     RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(std::string, std_msgs::msg::String);
 *
 *     // Then you can create things with just the custom type, and the ROS
 *     // message type is implied based on the previous statement.
 *     auto pub = node->create_publisher<std::string>(...);
 */
template<typename CustomType, typename ROSMessageType = void, class Enable = void>
struct TypeAdapter
{
  using is_specialized = std::false_type;
  using custom_type = CustomType;
  // In this case, the CustomType is the only thing given, or there is no specialization.
  // Assign ros_message_type to CustomType for the former case.
  using ros_message_type = CustomType;
};

/// Helper template to determine if a type is a TypeAdapter, false specialization.
template<typename T>
struct is_type_adapter : std::false_type {};

/// Helper template to determine if a type is a TypeAdapter, true specialization.
template<typename ... Ts>
struct is_type_adapter<TypeAdapter<Ts...>>: std::true_type {};

/// Identity specialization for TypeAdapter.
template<typename T>
struct TypeAdapter<T, void, std::enable_if_t<is_type_adapter<T>::value>>: T {};

namespace detail
{

template<typename CustomType, typename ROSMessageType>
struct assert_type_pair_is_specialized_type_adapter
{
  using type_adapter = TypeAdapter<CustomType, ROSMessageType>;
  static_assert(
    type_adapter::is_specialized::value,
    "No type adapter for this custom type/ros message type pair");
};

}  // namespace detail

/// Template metafunction that can make the type being adapted explicit.
template<typename CustomType>
struct adapt_type
{
  template<typename ROSMessageType>
  using as = typename ::rclcpp::detail::assert_type_pair_is_specialized_type_adapter<
    CustomType,
    ROSMessageType
    >::type_adapter;
};

/// Implicit type adapter used as a short hand way to create something with just the custom type.
/**
 * This is used when creating a publisher or subscription using just the custom
 * type in conjunction with RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE().
 * For example:
 *
 *     #include "type_adapter_for_std_string_to_std_msgs_String.hpp"
 *
 *     RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(std::string, std_msgs::msg::String);
 *
 *     int main(...) {
 *       // ...
 *       auto pub = node->create_publisher<std::string>(...);
 *     }
 *
 * \sa TypeAdapter for more examples.
 */
template<typename CustomType>
struct ImplicitTypeAdapter
{
  using is_specialized = std::false_type;
};

/// Specialization of TypeAdapter for ImplicitTypeAdapter.
/**
 * This allows for things like this:
 *
 *    RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(std::string, std_msgs::msg::String);
 *    auto pub = node->create_publisher<std::string>("topic", 10);
 *
 */
template<typename T>
struct TypeAdapter<T, void, std::enable_if_t<ImplicitTypeAdapter<T>::is_specialized::value>>
  : ImplicitTypeAdapter<T>
{};

/// Assigns the custom type implicitly to the given custom type/ros message type pair.
/**
 * Note: this macro needs to be used in the root namespace.
 * We cannot use ::rclcpp to protect against this, due to how GCC interprets the
 * syntax, see: https://stackoverflow.com/a/2781537
 *
 * \sa TypeAdapter
 * \sa ImplicitTypeAdapter
 */
#define RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(CustomType, ROSMessageType) \
  template<> \
  struct rclcpp::ImplicitTypeAdapter<CustomType> \
    : public rclcpp::TypeAdapter<CustomType, ROSMessageType> \
  { \
    static_assert( \
      is_specialized::value, \
      "Cannot use custom type as ros type when there is no TypeAdapter for that pair"); \
  }

}  // namespace rclcpp

#endif  // RCLCPP__TYPE_ADAPTER_HPP_
