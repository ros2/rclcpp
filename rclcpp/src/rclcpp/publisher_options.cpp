#include <rclcpp/publisher_options.hpp>

namespace rclcpp
{

bool
PublisherOptionsBase::resolve_use_intra_process_comm(bool default_from_node_topics) const
{
  bool use_intra_process;
  switch (this->use_intra_process_comm) {
    case IntraProcessSetting::Enable:
      use_intra_process = true;
      break;
    case IntraProcessSetting::Disable:
      use_intra_process = false;
      break;
    case IntraProcessSetting::NodeDefault:
      use_intra_process = default_from_node_topics;
      break;
    default:
      throw std::runtime_error("Unrecognized IntraProcessSetting value");
      break;
  }

  return use_intra_process;
}

}  // namespace rclcpp
