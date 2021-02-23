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

#ifndef RCLCPP__TYPE_ADAPTOR_HPP_
#define RCLCPP__TYPE_ADAPTOR_HPP_

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
 * The convert functiions must convert from one message to the other.
 *
 * For example, here is a theoretical example for adapting `std::string` to the
 * `std_msgs::msg::String` ROS message type:
 *
 *     template<>
 *     struct rclcpp::TypeAdaptor<std::string>
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
 * These adaptors are used automatically when creating a publisher or
 * subscription, e.g.:
 *
 *     auto pub = node->create_publisher<std::string>("topic", 10);
 *     auto sub = node->create_subscription<std::string>(
 *       "topic",
 *       10,
 *       [](const std::string & msg) {...});
 *
 * You can also be explicit that you're using an adapted type here, though which
 * ROS message it is being adapted to is still implicit:
 *
 *     auto pub = node->create_publisher<rclcpp::TypeAdaptor<std::string>>(...);
 *
 * Or you can be explicit about which adaptor should be used, either for
 * readability or for disambiguating which ROS type is being used:
 *
 *     using AdaptedType = rclcpp::adapt_type<std::string>::as<std_msgs::msg::String>;
 *     auto pub = node->create_publisher<AdaptedType>(...);
 *
 */
template<typename CustomType>
struct TypeAdaptor
{
  using is_specialized = std::false_type;
};

/// Template metafunction that can make the type being adapted explicit.
template<typename CustomType>
struct adapt_type
{
  template<typename ROSMessageType>
  struct as : public TypeAdaptor<CustomType>
  {
    static_assert(
      std::is_same<ROSMessageType, typename TypeAdaptor<CustomType>::ros_message_type>::value,
      "No type adaptor for this custom type/ros message type pair");
  };
};

}  // namespace rclcpp

#endif  // RCLCPP__TYPE_ADAPTOR_HPP_
