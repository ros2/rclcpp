#include "rclcpp/node_interfaces/node_services.hpp"

using namespace rclcpp::node_interfaces;

NodeServices::NodeServices(rclcpp::node_interfaces::NodeBaseInterface * node_base)
: node_base_(node_base)
{}

NodeServices::~NodeServices()
{}

void
NodeServices::add_service(
  rclcpp::service::ServiceBase::SharedPtr service_base_ptr,
  rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  if (group) {
    if (!node_base_->callback_group_in_node(group)) {
      // TODO(jacquelinekay): use custom exception
      throw std::runtime_error("Cannot create service, group not in node.");
    }
    group->add_service(service_base_ptr);
  } else {
    node_base_->get_default_callback_group()->add_service(service_base_ptr);
  }

  // Notify the executor that a new service was created using the parent Node.
  {
    auto notify_guard_condition_lock = node_base_->acquire_notify_guard_condition_lock();
    if (rcl_trigger_guard_condition(node_base_->get_notify_guard_condition()) != RCL_RET_OK) {
      throw std::runtime_error(
        std::string("Failed to notify waitset on service creation: ") + rmw_get_error_string()
      );
    }
  }
}

void
NodeServices::add_client(
  rclcpp::client::ClientBase::SharedPtr client_base_ptr,
  rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  if (group) {
    if (!node_base_->callback_group_in_node(group)) {
      // TODO(jacquelinekay): use custom exception
      throw std::runtime_error("Cannot create client, group not in node.");
    }
    group->add_client(client_base_ptr);
  } else {
    node_base_->get_default_callback_group()->add_client(client_base_ptr);
  }

  // Notify the executor that a new client was created using the parent Node.
  {
    auto notify_guard_condition_lock = node_base_->acquire_notify_guard_condition_lock();
    if (rcl_trigger_guard_condition(node_base_->get_notify_guard_condition()) != RCL_RET_OK) {
      throw std::runtime_error(
        std::string("Failed to notify waitset on client creation: ") + rmw_get_error_string()
      );
    }
  }
}
