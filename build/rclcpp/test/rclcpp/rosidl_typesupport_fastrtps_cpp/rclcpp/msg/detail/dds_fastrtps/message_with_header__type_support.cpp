// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from rclcpp:msg/MessageWithHeader.idl
// generated code does not contain a copyright notice
#include "rclcpp/msg/detail/message_with_header__rosidl_typesupport_fastrtps_cpp.hpp"
#include "rclcpp/msg/detail/message_with_header__functions.h"
#include "rclcpp/msg/detail/message_with_header__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace rclcpp
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const rclcpp::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  rclcpp::msg::Header &);
size_t get_serialized_size(
  const rclcpp::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace rclcpp


namespace rclcpp
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rclcpp
cdr_serialize(
  const rclcpp::msg::MessageWithHeader & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  rclcpp::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rclcpp
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  rclcpp::msg::MessageWithHeader & ros_message)
{
  // Member: header
  rclcpp::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rclcpp
get_serialized_size(
  const rclcpp::msg::MessageWithHeader & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header

  current_alignment +=
    rclcpp::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rclcpp
max_serialized_size_MessageWithHeader(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: header
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      current_alignment +=
        rclcpp::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  return current_alignment - initial_alignment;
}

static bool _MessageWithHeader__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const rclcpp::msg::MessageWithHeader *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _MessageWithHeader__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<rclcpp::msg::MessageWithHeader *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _MessageWithHeader__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const rclcpp::msg::MessageWithHeader *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _MessageWithHeader__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_MessageWithHeader(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _MessageWithHeader__callbacks = {
  "rclcpp::msg",
  "MessageWithHeader",
  _MessageWithHeader__cdr_serialize,
  _MessageWithHeader__cdr_deserialize,
  _MessageWithHeader__get_serialized_size,
  _MessageWithHeader__max_serialized_size
};

static rosidl_message_type_support_t _MessageWithHeader__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_MessageWithHeader__callbacks,
  get_message_typesupport_handle_function,
  &rclcpp__msg__MessageWithHeader__get_type_hash,
  &rclcpp__msg__MessageWithHeader__get_type_description,
  &rclcpp__msg__MessageWithHeader__get_type_description_sources,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace rclcpp

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_rclcpp
const rosidl_message_type_support_t *
get_message_type_support_handle<rclcpp::msg::MessageWithHeader>()
{
  return &rclcpp::msg::typesupport_fastrtps_cpp::_MessageWithHeader__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rclcpp, msg, MessageWithHeader)() {
  return &rclcpp::msg::typesupport_fastrtps_cpp::_MessageWithHeader__handle;
}

#ifdef __cplusplus
}
#endif
