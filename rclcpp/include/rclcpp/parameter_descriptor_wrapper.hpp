#ifndef RCLCPP__PARAMETER_DESCRIPTOR_WRAPPER_HPP_
#define RCLCPP__PARAMETER_DESCRIPTOR_WRAPPER_HPP_

// Standard library includes
#include <functional>
#include <utility>
#include <memory>

// Additional ROS libraries needed
#include "node.hpp"
#include "rcl_interfaces/msg/describe_parameters.hpp"
#include "rclcpp/node.hpp"

namespace rclcpp
{

// Implments ParameterDesription class with builder design pattern
class ParameterDescription
{
public:
    // List of classes the builder manages
    class FloatingPointDescription;

    // Two constructors in case certain methods don't need to declare_parameters with a Type T in the form of a template
    ParameterDescription(std::string name, std::uint8_t type, std::string description, std::string additional_constraints, bool read_only. bool dynamic_typing) : m_name{name}, parameter_descriptor.type{type}, m_description{description}, m_additional_constraints{additional_constraints}, read_only{read_only}, dyanmic_typing{m_dyanmic_typing}{};

private:
    // The main descriptor object we're meant to adjust
    rcl_interfaces::msg::ParameterDescriptor parameter_descriptor = {};

    // Copies all the information in ParameterDescriptor.msg - https://github.com/ros2/rcl_interfaces/blob/rolling/rcl_interfaces/msg/ParameterDescriptor.msg
    std::string m_name;
    std::string m_description;

    // Parameter Constraints
    std::string m_additional_constraints;
    bool m_read_only;
    bool m_dynamic_typing;

};

class ParameterDescription::FloatingPointDescription
{
public:
    // Our Main build methods which will construct the base class
    ParameterDescription build() const;

    //Builder Method - Mains
    FloatingPointDescription& SetName(std::string name);
    FloatingPointDescription& SetType(std::uint8_t type);
    FloatingPointDescription& SetDescription(std::string description);
    FloatingPointDescription& SetAdditionalConstraints(std::string constraints);
    FloatingPointDescription& SetReadOnly(bool read_only);
    FloatingPointDescription& SetDynamicTyping(bool dynamic_typing);

    // Need the current node in order to begin the configuraiton state forit via the declare_parameter function which setups up the Node
    template<typename ParameterType>
    FloatingPointDescription& DeclareParameter<ParameterType>(ParameterType default_value, std::shared_ptr<rclcpp::Node> required_node)
{
    required_node->declare_parameter<ParameterType>(m_name, default_value, parameter_descriptor);
    return *this;
}
    // Extended build methods specific to this class
    FloatingPointDescription& SetMin(float min);
    FloatingPointDescription& SetMax(float max);
    FloatingPointDescription& SetStep(float step);

    // Simplification Method - Outside of the base clss that the user should be able to set up easily, they should then be able to call a function which setups up the specifics for this class (The floating point parameter description)
    // Here we will have a difference between generic types and templated types
    FloatingPointDescription& SetFloatingPointDescription(float min, float max, float step);
    // We will again need access to the current development node to declare its parameters
    template<typename ParameterType>
    FloatingPointDescription& SetFloatingPointDescription(std::string name, ParameterType default_value, float min, float max, float step)
    {
        parameter_descriptor.floating_point_range.resize(1);
        parameter_descriptor.floating_point_range.at(0).from_value = min;
        parameter_descriptor.floating_point_range.at(0).to_value = max;
        parameter_descriptor.floating_point_range.at(0).step = step;

        return *this;
    }


private:
    rcl_interfaces::msg::ParameterDescriptor parameter_descriptor = {};
    std::string m_name{""};
    parameter_descriptor.type{rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET};
    std::string m_description{""};

    std::string m_additional_constraints{""};
    bool m_read_only{false};
    bool m_dynamic_typing{false};

    // This is a floating point so we'll use floating points in the range
    float m_min{0.0f};
    float m_max{1.0f};
    float m_step{0.0f};
};

} // namespace rclcpp

#endif // headerguards for parameter_descriptor
