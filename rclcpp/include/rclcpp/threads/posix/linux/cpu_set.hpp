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

#ifndef RCLCPP__THREADS__POSIX__LINUX__CPU_SET_HPP_
#define RCLCPP__THREADS__POSIX__LINUX__CPU_SET_HPP_

#include <pthread.h>
#include <vector>
#include <utility>
#include <memory>

#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

namespace detail
{

struct CpuSet
{
  using NativeCpuSetType = cpu_set_t;
  CpuSet() = default;
  explicit CpuSet(std::size_t cpu)
  {
    init_cpu_set();
    CPU_ZERO_S(alloc_size(), cpu_set_.get());
    CPU_SET_S(cpu, alloc_size(), cpu_set_.get());
  }
  CpuSet(const CpuSet & other)
  {
    if (other.cpu_set_) {
      init_cpu_set();
      memcpy(cpu_set_.get(), other.cpu_set_.get(), alloc_size());
    }
  }
  CpuSet & operator=(const CpuSet & other)
  {
    if (other.cpu_set_) {
      init_cpu_set();
      memcpy(cpu_set_.get(), other.cpu_set_.get(), alloc_size());
    } else {
      clear();
    }
    return *this;
  }
  CpuSet(CpuSet && other)
  : CpuSet()
  {
    swap(other);
  }
  CpuSet & operator=(CpuSet && other)
  {
    CpuSet tmp;
    other.swap(tmp);
    tmp.swap(*this);
    return *this;
  }
  void swap(CpuSet & other)
  {
    using std::swap;
    swap(cpu_set_, other.cpu_set_);
    swap(num_proc_, other.num_proc_);
  }
  void set(std::size_t cpu)
  {
    init_cpu_set();
    valid_cpu(cpu);
    CPU_SET_S(cpu, alloc_size(), cpu_set_.get());
  }
  void unset(std::size_t cpu)
  {
    init_cpu_set();
    valid_cpu(cpu);
    CPU_CLR_S(cpu, alloc_size(), cpu_set_.get());
  }
  void clear()
  {
    if (cpu_set_) {
      CPU_ZERO_S(alloc_size(), cpu_set_.get());
    }
  }
  bool is_set(std::size_t cpu) const
  {
    if (cpu_set_) {
      valid_cpu(cpu);
      return CPU_ISSET_S(cpu, alloc_size(), cpu_set_.get());
    } else {
      return false;
    }
  }

  std::size_t max_processors() const
  {
    return num_proc_;
  }
  std::size_t alloc_size() const
  {
    return CPU_ALLOC_SIZE(num_proc_);
  }
  NativeCpuSetType * native_cpu_set() const
  {
    return cpu_set_.get();
  }

private:
  void init_cpu_set()
  {
    if (cpu_set_) {
      return;
    }
    auto num_proc = sysconf(_SC_NPROCESSORS_ONLN);
    if (num_proc == -1) {
      return;
    }
    auto p = CPU_ALLOC(CPU_ALLOC_SIZE(num_proc));
    cpu_set_ = std::unique_ptr<NativeCpuSetType, CpuSetDeleter>(p);
    num_proc_ = num_proc;
  }
  void valid_cpu(std::size_t cpu) const
  {
    if (num_proc_ <= cpu) {
      auto ec = std::make_error_code(std::errc::invalid_argument);
      throw std::system_error{ec, "cpu number is invaild"};
    }
  }
  struct CpuSetDeleter
  {
    void operator()(NativeCpuSetType * cpu_set) const
    {
      CPU_FREE(cpu_set);
    }
  };
  std::unique_ptr<NativeCpuSetType, CpuSetDeleter> cpu_set_;
  std::size_t num_proc_;
};

inline void swap(CpuSet & a, CpuSet & b)
{
  a.swap(b);
}

}  // namespace detail

}  // namespace rclcpp

#endif  // RCLCPP__THREADS__POSIX__LINUX__CPU_SET_HPP_
