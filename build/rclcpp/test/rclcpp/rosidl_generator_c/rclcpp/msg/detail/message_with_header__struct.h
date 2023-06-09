// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rclcpp:msg/MessageWithHeader.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "rclcpp/msg/message_with_header.h"


#ifndef RCLCPP__MSG__DETAIL__MESSAGE_WITH_HEADER__STRUCT_H_
#define RCLCPP__MSG__DETAIL__MESSAGE_WITH_HEADER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "rclcpp/msg/detail/header__struct.h"

/// Struct defined in msg/MessageWithHeader in the package rclcpp.
/**
  * Message containing a simple Header field.
 */
typedef struct rclcpp__msg__MessageWithHeader
{
  rclcpp__msg__Header header;
} rclcpp__msg__MessageWithHeader;

// Struct for a sequence of rclcpp__msg__MessageWithHeader.
typedef struct rclcpp__msg__MessageWithHeader__Sequence
{
  rclcpp__msg__MessageWithHeader * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rclcpp__msg__MessageWithHeader__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RCLCPP__MSG__DETAIL__MESSAGE_WITH_HEADER__STRUCT_H_
