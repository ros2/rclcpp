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

#ifndef RCLCPP__DETAIL__CPP_CALLBACK_TRAMPOLINE_HPP_
#define RCLCPP__DETAIL__CPP_CALLBACK_TRAMPOLINE_HPP_

#include <functional>

namespace rclcpp
{

namespace detail
{

/// Trampoline pattern for wrapping std::function into C-style callbacks.
/**
 * A common pattern in C is for a function to take a function pointer and a
 * void pointer for "user data" which is passed to the function pointer when it
 * is called from within C.
 *
 * It works by using the user data pointer to store a pointer to a
 * std::function instance.
 * So when called from C, this function will cast the user data to the right
 * std::function type and call it.
 *
 * This should allow you to use free functions, lambdas with and without
 * captures, and various kinds of std::bind instances.
 *
 * The interior of this function is likely to be executed within a C runtime,
 * so no exceptions should be thrown at this point, and doing so results in
 * undefined behavior.
 *
 * \tparam UserDataT Deduced type based on what is passed for user data,
 *   usually this type is either `void *` or `const void *`.
 * \tparam Args the arguments being passed to the callback
 * \tparam ReturnT the return type of this function and the callback, default void
 * \param user_data the function pointer, possibly type erased
 * \param args the arguments to be forwarded to the callback
 * \returns whatever the callback returns, if anything
 */
template<
  typename UserDataT,
  typename ... Args,
  typename ReturnT = void
>
ReturnT
cpp_callback_trampoline(UserDataT user_data, Args ... args) noexcept
{
  auto & actual_callback = *reinterpret_cast<const std::function<ReturnT(Args...)> *>(user_data);
  return actual_callback(args ...);
}

}  // namespace detail

}  // namespace rclcpp

#endif  // RCLCPP__DETAIL__CPP_CALLBACK_TRAMPOLINE_HPP_
