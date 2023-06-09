// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from rclcpp:msg/MessageWithHeader.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "rclcpp/msg/message_with_header.h"


#ifndef RCLCPP__MSG__DETAIL__MESSAGE_WITH_HEADER__FUNCTIONS_H_
#define RCLCPP__MSG__DETAIL__MESSAGE_WITH_HEADER__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "rclcpp/msg/rosidl_generator_c__visibility_control.h"

#include "rclcpp/msg/detail/message_with_header__struct.h"

/// Initialize msg/MessageWithHeader message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rclcpp__msg__MessageWithHeader
 * )) before or use
 * rclcpp__msg__MessageWithHeader__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rclcpp
bool
rclcpp__msg__MessageWithHeader__init(rclcpp__msg__MessageWithHeader * msg);

/// Finalize msg/MessageWithHeader message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rclcpp
void
rclcpp__msg__MessageWithHeader__fini(rclcpp__msg__MessageWithHeader * msg);

/// Create msg/MessageWithHeader message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rclcpp__msg__MessageWithHeader__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rclcpp
rclcpp__msg__MessageWithHeader *
rclcpp__msg__MessageWithHeader__create();

/// Destroy msg/MessageWithHeader message.
/**
 * It calls
 * rclcpp__msg__MessageWithHeader__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rclcpp
void
rclcpp__msg__MessageWithHeader__destroy(rclcpp__msg__MessageWithHeader * msg);

/// Check for msg/MessageWithHeader message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rclcpp
bool
rclcpp__msg__MessageWithHeader__are_equal(const rclcpp__msg__MessageWithHeader * lhs, const rclcpp__msg__MessageWithHeader * rhs);

/// Copy a msg/MessageWithHeader message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rclcpp
bool
rclcpp__msg__MessageWithHeader__copy(
  const rclcpp__msg__MessageWithHeader * input,
  rclcpp__msg__MessageWithHeader * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_rclcpp
const rosidl_type_hash_t *
rclcpp__msg__MessageWithHeader__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_rclcpp
const rosidl_runtime_c__type_description__TypeDescription *
rclcpp__msg__MessageWithHeader__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_rclcpp
const rosidl_runtime_c__type_description__TypeSource *
rclcpp__msg__MessageWithHeader__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_rclcpp
const rosidl_runtime_c__type_description__TypeSource__Sequence *
rclcpp__msg__MessageWithHeader__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/MessageWithHeader messages.
/**
 * It allocates the memory for the number of elements and calls
 * rclcpp__msg__MessageWithHeader__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rclcpp
bool
rclcpp__msg__MessageWithHeader__Sequence__init(rclcpp__msg__MessageWithHeader__Sequence * array, size_t size);

/// Finalize array of msg/MessageWithHeader messages.
/**
 * It calls
 * rclcpp__msg__MessageWithHeader__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rclcpp
void
rclcpp__msg__MessageWithHeader__Sequence__fini(rclcpp__msg__MessageWithHeader__Sequence * array);

/// Create array of msg/MessageWithHeader messages.
/**
 * It allocates the memory for the array and calls
 * rclcpp__msg__MessageWithHeader__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rclcpp
rclcpp__msg__MessageWithHeader__Sequence *
rclcpp__msg__MessageWithHeader__Sequence__create(size_t size);

/// Destroy array of msg/MessageWithHeader messages.
/**
 * It calls
 * rclcpp__msg__MessageWithHeader__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rclcpp
void
rclcpp__msg__MessageWithHeader__Sequence__destroy(rclcpp__msg__MessageWithHeader__Sequence * array);

/// Check for msg/MessageWithHeader message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rclcpp
bool
rclcpp__msg__MessageWithHeader__Sequence__are_equal(const rclcpp__msg__MessageWithHeader__Sequence * lhs, const rclcpp__msg__MessageWithHeader__Sequence * rhs);

/// Copy an array of msg/MessageWithHeader messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rclcpp
bool
rclcpp__msg__MessageWithHeader__Sequence__copy(
  const rclcpp__msg__MessageWithHeader__Sequence * input,
  rclcpp__msg__MessageWithHeader__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // RCLCPP__MSG__DETAIL__MESSAGE_WITH_HEADER__FUNCTIONS_H_
