// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP_RCLCPP_ALLOCATOR_FACTORY_HPP_
#define RCLCPP_RCLCPP_ALLOCATOR_FACTORY_HPP_

#include <memory>
#include <iostream>
#include <rclcpp/allocator/allocator_deleter.hpp>

template<template<typename> class Alloc, typename T, typename D>
D initialize_deleter(Alloc<T> * alloc)
{
  (void) alloc;
  throw std::runtime_error("Reached unexpected template specialization");
}

template<typename T>
std::default_delete<T> initialize_deleter(std::allocator<T> * alloc)
{
  (void) alloc;
  return std::default_delete<T>();
}

template<template<typename> class Alloc, typename T>
AllocatorDeleter<T, Alloc<T>> initialize_deleter(Alloc<T> * alloc)
{
  if (!alloc) {
    throw std::invalid_argument("Allocator argument was NULL");
  }
  return AllocatorDeleter<T, Alloc<T>>(alloc);
}

template<typename T, template<typename> class Alloc>
using Deleter = typename std::conditional<
    std::is_same<Alloc<T>, std::allocator<T>>::value,
    std::default_delete<T>,
    AllocatorDeleter<T, Alloc<T>>
    >::type;

#endif
