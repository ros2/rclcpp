// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__DETAIL__RESOLVE_USE_INTRA_PROCESS_HPP_
#define RCLCPP__DETAIL__RESOLVE_USE_INTRA_PROCESS_HPP_

#include <stdexcept>

#include "rclcpp/intra_process_setting.hpp"

namespace rclcpp
{

namespace detail
{

/// Return whether or not intra process is enabled, resolving "NodeDefault" if needed.
template<typename OptionsT, typename NodeBaseT>
bool
resolve_use_intra_process(const OptionsT & options, const NodeBaseT & node_base)
{
  bool use_intra_process;
  switch (options.use_intra_process_comm) {
    case IntraProcessSetting::Enable:
      use_intra_process = true;
      break;
    case IntraProcessSetting::Disable:
      use_intra_process = false;
      break;
    case IntraProcessSetting::NodeDefault:
      use_intra_process = node_base.get_use_intra_process_default();
      break;
    default:
      throw std::runtime_error("Unrecognized IntraProcessSetting value");
      break;
  }

  return use_intra_process;
}

}  // namespace detail

}  // namespace rclcpp

#endif  // RCLCPP__DETAIL__RESOLVE_USE_INTRA_PROCESS_HPP_
