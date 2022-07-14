#include "rclcpp/introspection.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"


using rclcpp::IntrospectionUtils;

IntrospectionUtils::IntrospectionUtils(
        rcl_node_t * rcl_node_ptr,
        const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_parameters)
: rcl_node_ptr_(rcl_node_ptr),
  node_parameters_(node_parameters)
{

  // declare service introspection parameters
  if (!node_parameters_->has_parameter(publish_service_events_param_name_)) {
    node_parameters_->declare_parameter(publish_service_events_param_name_,
        rclcpp::ParameterValue(true));
  }
  if (!node_parameters_->has_parameter(publish_client_events_param_name_)) {
    node_parameters_->declare_parameter(publish_client_events_param_name_,
        rclcpp::ParameterValue(true));
  }
  if (!node_parameters_->has_parameter(enable_service_event_content_param_name_)) {
    node_parameters_->declare_parameter(enable_service_event_content_param_name_,
        rclcpp::ParameterValue(true));
  }
  if (!node_parameters_->has_parameter(enable_client_event_content_param_name_)) {
    node_parameters_->declare_parameter(enable_client_event_content_param_name_,
        rclcpp::ParameterValue(true));
  }

  std::function<void(const std::vector<rclcpp::Parameter> &)>
    configure_service_introspection_callback = 
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_ret_t ret;
      for (const auto & param: parameters) {
        if (param.get_name() == publish_service_events_param_name_) {
          for (rcl_service_t * srv: services) {
            ret = rcl_service_introspection_configure_service_events(
                srv, this->rcl_node_ptr_, param.get_value<bool>());
            if (RCL_RET_OK != ret) {
              throw std::runtime_error("Could not configure service introspection events");
            }
          }
        } else if (param.get_name() == publish_client_events_param_name_) {
          for (rcl_client_t * clt: clients) {
            ret = rcl_service_introspection_configure_client_events(
                clt, this->rcl_node_ptr_, param.get_value<bool>());
            if (RCL_RET_OK != ret) {
              throw std::runtime_error("Could not configure client introspection events");
            }
          }
        } else if (param.get_name() == enable_service_event_content_param_name_) {
          for (rcl_service_t * srv: services) {
            rcl_service_introspection_configure_service_content(srv, param.get_value<bool>());
          }
        } else if (param.get_name() == enable_client_event_content_param_name_) {
          for (rcl_client_t * clt: clients) {
            rcl_service_introspection_configure_client_content(clt, param.get_value<bool>());
          }
        }
      }
    };

  // register callbacks
  node_parameters_->add_post_set_parameters_callback(configure_service_introspection_callback);
}

// IntrospectionUtils::~IntrospectionUtils();


// Alternatively this wrapper can be made to wrap a create_client call?

void IntrospectionUtils::register_service(
    const rclcpp::ServiceBase::SharedPtr& service){
  this->services.push_back(service->get_service_handle().get());
}

void IntrospectionUtils::register_client(
    const rclcpp::ClientBase::SharedPtr& client){
  this->clients.push_back(client->get_client_handle().get());
}


    

IntrospectionUtils::~IntrospectionUtils() = default;



