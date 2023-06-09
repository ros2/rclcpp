// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rclcpp:msg/MessageWithHeader.idl
// generated code does not contain a copyright notice
#include "rclcpp/msg/detail/message_with_header__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "rclcpp/msg/detail/header__functions.h"

bool
rclcpp__msg__MessageWithHeader__init(rclcpp__msg__MessageWithHeader * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!rclcpp__msg__Header__init(&msg->header)) {
    rclcpp__msg__MessageWithHeader__fini(msg);
    return false;
  }
  return true;
}

void
rclcpp__msg__MessageWithHeader__fini(rclcpp__msg__MessageWithHeader * msg)
{
  if (!msg) {
    return;
  }
  // header
  rclcpp__msg__Header__fini(&msg->header);
}

bool
rclcpp__msg__MessageWithHeader__are_equal(const rclcpp__msg__MessageWithHeader * lhs, const rclcpp__msg__MessageWithHeader * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!rclcpp__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  return true;
}

bool
rclcpp__msg__MessageWithHeader__copy(
  const rclcpp__msg__MessageWithHeader * input,
  rclcpp__msg__MessageWithHeader * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!rclcpp__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  return true;
}

rclcpp__msg__MessageWithHeader *
rclcpp__msg__MessageWithHeader__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rclcpp__msg__MessageWithHeader * msg = (rclcpp__msg__MessageWithHeader *)allocator.allocate(sizeof(rclcpp__msg__MessageWithHeader), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rclcpp__msg__MessageWithHeader));
  bool success = rclcpp__msg__MessageWithHeader__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rclcpp__msg__MessageWithHeader__destroy(rclcpp__msg__MessageWithHeader * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rclcpp__msg__MessageWithHeader__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rclcpp__msg__MessageWithHeader__Sequence__init(rclcpp__msg__MessageWithHeader__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rclcpp__msg__MessageWithHeader * data = NULL;

  if (size) {
    data = (rclcpp__msg__MessageWithHeader *)allocator.zero_allocate(size, sizeof(rclcpp__msg__MessageWithHeader), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rclcpp__msg__MessageWithHeader__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rclcpp__msg__MessageWithHeader__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rclcpp__msg__MessageWithHeader__Sequence__fini(rclcpp__msg__MessageWithHeader__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rclcpp__msg__MessageWithHeader__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rclcpp__msg__MessageWithHeader__Sequence *
rclcpp__msg__MessageWithHeader__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rclcpp__msg__MessageWithHeader__Sequence * array = (rclcpp__msg__MessageWithHeader__Sequence *)allocator.allocate(sizeof(rclcpp__msg__MessageWithHeader__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rclcpp__msg__MessageWithHeader__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rclcpp__msg__MessageWithHeader__Sequence__destroy(rclcpp__msg__MessageWithHeader__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rclcpp__msg__MessageWithHeader__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rclcpp__msg__MessageWithHeader__Sequence__are_equal(const rclcpp__msg__MessageWithHeader__Sequence * lhs, const rclcpp__msg__MessageWithHeader__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rclcpp__msg__MessageWithHeader__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rclcpp__msg__MessageWithHeader__Sequence__copy(
  const rclcpp__msg__MessageWithHeader__Sequence * input,
  rclcpp__msg__MessageWithHeader__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rclcpp__msg__MessageWithHeader);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rclcpp__msg__MessageWithHeader * data =
      (rclcpp__msg__MessageWithHeader *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rclcpp__msg__MessageWithHeader__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rclcpp__msg__MessageWithHeader__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rclcpp__msg__MessageWithHeader__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
