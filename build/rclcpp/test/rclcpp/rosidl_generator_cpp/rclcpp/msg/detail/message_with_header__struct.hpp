// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rclcpp:msg/MessageWithHeader.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "rclcpp/msg/message_with_header.hpp"


#ifndef RCLCPP__MSG__DETAIL__MESSAGE_WITH_HEADER__STRUCT_HPP_
#define RCLCPP__MSG__DETAIL__MESSAGE_WITH_HEADER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "rclcpp/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rclcpp__msg__MessageWithHeader __attribute__((deprecated))
#else
# define DEPRECATED__rclcpp__msg__MessageWithHeader __declspec(deprecated)
#endif

namespace rclcpp
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MessageWithHeader_
{
  using Type = MessageWithHeader_<ContainerAllocator>;

  explicit MessageWithHeader_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit MessageWithHeader_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    rclcpp::msg::Header_<ContainerAllocator>;
  _header_type header;

  // setters for named parameter idiom
  Type & set__header(
    const rclcpp::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rclcpp::msg::MessageWithHeader_<ContainerAllocator> *;
  using ConstRawPtr =
    const rclcpp::msg::MessageWithHeader_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rclcpp::msg::MessageWithHeader_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rclcpp::msg::MessageWithHeader_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rclcpp::msg::MessageWithHeader_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rclcpp::msg::MessageWithHeader_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rclcpp::msg::MessageWithHeader_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rclcpp::msg::MessageWithHeader_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rclcpp::msg::MessageWithHeader_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rclcpp::msg::MessageWithHeader_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rclcpp__msg__MessageWithHeader
    std::shared_ptr<rclcpp::msg::MessageWithHeader_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rclcpp__msg__MessageWithHeader
    std::shared_ptr<rclcpp::msg::MessageWithHeader_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MessageWithHeader_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    return true;
  }
  bool operator!=(const MessageWithHeader_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MessageWithHeader_

// alias to use template instance with default allocator
using MessageWithHeader =
  rclcpp::msg::MessageWithHeader_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rclcpp

#endif  // RCLCPP__MSG__DETAIL__MESSAGE_WITH_HEADER__STRUCT_HPP_
