// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rclcpp:msg/Header.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "rclcpp/msg/header.hpp"


#ifndef RCLCPP__MSG__DETAIL__HEADER__STRUCT_HPP_
#define RCLCPP__MSG__DETAIL__HEADER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rclcpp__msg__Header __attribute__((deprecated))
#else
# define DEPRECATED__rclcpp__msg__Header __declspec(deprecated)
#endif

namespace rclcpp
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Header_
{
  using Type = Header_<ContainerAllocator>;

  explicit Header_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    (void)_init;
  }

  explicit Header_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rclcpp::msg::Header_<ContainerAllocator> *;
  using ConstRawPtr =
    const rclcpp::msg::Header_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rclcpp::msg::Header_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rclcpp::msg::Header_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rclcpp::msg::Header_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rclcpp::msg::Header_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rclcpp::msg::Header_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rclcpp::msg::Header_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rclcpp::msg::Header_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rclcpp::msg::Header_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rclcpp__msg__Header
    std::shared_ptr<rclcpp::msg::Header_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rclcpp__msg__Header
    std::shared_ptr<rclcpp::msg::Header_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Header_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const Header_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Header_

// alias to use template instance with default allocator
using Header =
  rclcpp::msg::Header_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rclcpp

#endif  // RCLCPP__MSG__DETAIL__HEADER__STRUCT_HPP_
