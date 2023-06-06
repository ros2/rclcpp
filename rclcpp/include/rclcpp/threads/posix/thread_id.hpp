// Copyright 2023 eSOL Co.,Ltd.
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

#ifndef RCLCPP__THREADS__POSIX__THREAD_ID_HPP_
#define RCLCPP__THREADS__POSIX__THREAD_ID_HPP_

#include <pthread.h>

#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmismatched-tags"
#endif

struct Thread;

namespace detail
{

namespace thread_id_ns
{

struct ThreadId;

inline ThreadId get_id() noexcept;
inline bool operator==(ThreadId id1, ThreadId id2);
inline bool operator!=(ThreadId id1, ThreadId id2);
inline bool operator<(ThreadId id1, ThreadId id2);
inline bool operator>(ThreadId id1, ThreadId id2);
inline bool operator<=(ThreadId id1, ThreadId id2);
inline bool operator>=(ThreadId id1, ThreadId id2);
template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits> & operator<<(
  std::basic_ostream<CharT, Traits> &,
  ThreadId);

struct ThreadId
{
  ThreadId() = default;
  ThreadId(ThreadId const &) = default;
  ThreadId(ThreadId &&) = default;
  ThreadId & operator=(ThreadId const &) = default;
  ThreadId & operator=(ThreadId &&) = default;

  friend bool operator==(ThreadId id1, ThreadId id2)
  {
    return pthread_equal(id1.h, id2.h);
  }
  friend bool operator<(ThreadId id1, ThreadId id2)
  {
    return id1.h < id2.h;
  }
  template<typename CharT, typename Traits>
  friend std::basic_ostream<CharT, Traits> & operator<<(
    std::basic_ostream<CharT, Traits> & ost,
    ThreadId id)
  {
    return ost << id.h;
  }

private:
  friend class rclcpp::Thread;
  friend ThreadId get_id() noexcept;
  friend struct std::hash<ThreadId>;
  explicit ThreadId(pthread_t h)
  : h(h) {}
  pthread_t h;
};

ThreadId get_id() noexcept
{
  return ThreadId{pthread_self()};
}

bool operator!=(ThreadId id1, ThreadId id2)
{
  return !(id1 == id2);
}

bool operator>(ThreadId id1, ThreadId id2)
{
  return id2 < id1;
}

bool operator<=(ThreadId id1, ThreadId id2)
{
  return !(id1 > id2);
}

bool operator>=(ThreadId id1, ThreadId id2)
{
  return !(id1 < id2);
}

}  // namespace thread_id_ns

using thread_id_ns::ThreadId;
using thread_id_ns::operator==;
using thread_id_ns::operator!=;
using thread_id_ns::operator<;  // NOLINT
using thread_id_ns::operator>;  // NOLINT
using thread_id_ns::operator<=;
using thread_id_ns::operator>=;
using thread_id_ns::operator<<;

}  // namespace detail

namespace this_thread
{

using detail::thread_id_ns::get_id;

}  // namespace this_thread

}  // namespace rclcpp

namespace std
{

template<>
struct hash<rclcpp::detail::thread_id_ns::ThreadId>
{
  std::size_t operator()(rclcpp::detail::thread_id_ns::ThreadId id)
  {
    return id.h;
  }
};

}  // namespace std

#endif  // RCLCPP__THREADS__POSIX__THREAD_ID_HPP_
