// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from rclcpp:msg/String.idl
// generated code does not contain a copyright notice

#include "rclcpp/msg/detail/string__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_rclcpp
const rosidl_type_hash_t *
rclcpp__msg__String__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x55, 0x68, 0x4f, 0x5f, 0x6f, 0x2f, 0xfc, 0x5b,
      0x17, 0xc3, 0x62, 0x41, 0x88, 0x9b, 0xd3, 0xa3,
      0xcc, 0x1e, 0x6f, 0x5a, 0xf9, 0xe3, 0x21, 0xbd,
      0x88, 0x20, 0xc7, 0x4b, 0x4a, 0x2e, 0x31, 0xf5,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char rclcpp__msg__String__TYPE_NAME[] = "rclcpp/msg/String";

// Define type names, field names, and default values
static char rclcpp__msg__String__FIELD_NAME__data[] = "data";

static rosidl_runtime_c__type_description__Field rclcpp__msg__String__FIELDS[] = {
  {
    {rclcpp__msg__String__FIELD_NAME__data, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
rclcpp__msg__String__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rclcpp__msg__String__TYPE_NAME, 17, 17},
      {rclcpp__msg__String__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string data";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
rclcpp__msg__String__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {rclcpp__msg__String__TYPE_NAME, 17, 17},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 11, 11},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rclcpp__msg__String__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *rclcpp__msg__String__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
