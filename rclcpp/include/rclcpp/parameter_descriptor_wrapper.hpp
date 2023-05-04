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

// Implements ParameterDesription class with builder design pattern
class ParameterDescription
{
public:
    ParameterDescription(const std::string& name, std::uint8_t type, std::string description, std::string additional_constraints, bool read_only. bool dynamic_typing) : m_name{name}, parameter_descriptor.type{type}, m_description{description}, m_additional_constraints{additional_constraints}, read_only{read_only}, dyanmic_typing{m_dyanmic_typing}{};
    // List of classes the builder manages
    ParameterDescription();

    // Our Main build methods which will construct the base class
    ParameterDescription build() const;

    //Builder Methods - Describes the instances in a parameter_descriptionobject
    ParameterDescription& SetName(const std::string& name);
    ParameterDescription& SetType(std::uint8_t type);
    ParameterDescription& SetDescriptionText(const std::string& description);
    ParameterDescription& SetAdditionalConstraints(const std::string& constraints);
    ParameterDescription& SetReadOnly(bool read_only);
    ParameterDescription& SetDynamicTyping(bool dynamic_typing);

    // Need the current node in order to begin the configuraiton state forit via the declare_parameter function which setups up the Node
    template<typename ParameterType>
    ParameterDescription& DeclareParameter<ParameterType>(ParameterType default_value, std::unique_ptr<rclcpp::Node> required_node)
{
    required_node->declare_parameter<ParameterType>(m_name, default_value, parameter_descriptor);
    return *this;
}

    // Simplification Methods - The user should be able to set up ranges easily, they should then be able to call a function which sets up the specifics for this class (The floating point range for parameter description)
    // Here we will have a difference between generic types and templated types
    // Also we can use default values for min, max, and float. The other option is to utilize SetMin methods and similar and check if we have a range, if not, then we should resize one
    ParameterDescription& SetFloatingPointDescriptionRange(float min = 0.0f, float max = 1.0f, float step = 0.0f);
    // We will again need access to the current development node to declare its parameters
    template<typename ParameterType>
    ParameterDescription& SetFloatingPointDescriptionRange(std::unique_ptr<rclcpp::Node> currentNode, const std::string& name, ParameterType default_value, float min = 0.0f, float max = 1.0f, float step = 0.0f)
    {
        parameter_descriptor.floating_point_range.resize(1);
        parameter_descriptor.floating_point_range.at(0).from_value = min;
        parameter_descriptor.floating_point_range.at(0).to_value = max;
        parameter_descriptor.floating_point_range.at(0).step = step;

        // For this version of the function we can outright declare the parameters using the specified type
        currentNode->declare_parameter<ParameterType>(name, default_value, parameter_descriptor);

        return *this;
    }

    ParameterDescription& SetIntegerDescriptionRange(int min = 0, int max = 1, int step = 0);
    // We will again need access to the current development node to declare its parameters
    template<typename ParameterType>
    ParameterDescription& SetIntegerDescriptionRange(std::unique_ptr<rclcpp::Node> currentNode, const std::string& name, ParameterType default_value, int min = 0, int max = 1, int step = 0)
    {
        parameter_descriptor.integer_range.resize(1);
        parameter_descriptor.integer_range.at(0).from_value = min;
        parameter_descriptor.integer_range.at(0).to_value = max;
        parameter_descriptor.integer_range.at(0).step = step;

        // For this version of the function we can outright declare the parameters using the specified type
        currentNode->declare_parameter<ParameterType>(name, default_value, parameter_descriptor);

        return *this;
    }

    // Extended build methods specific to the ranges, but with overloaded versions
    ParameterDescription& SetMin(float min);
    ParameterDescription& SetMax(float max);
    ParameterDescription& SetStep(float step);

    ParameterDescription& SetMin(int min);
    ParameterDescription& SetMax(int max);
    ParameterDescription& SetStep(int step);

private:
    // The main descriptor object we're meant to initialize and adjust
    rcl_interfaces::msg::ParameterDescriptor parameter_descriptor = {};

    // Copies all the information in ParameterDescriptor.msg - https://github.com/ros2/rcl_interfaces/blob/rolling/rcl_interfaces/msg/ParameterDescriptor.msg
    std::string m_name;
    std::string m_description;
    parameter_descriptor.type{rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET};

    // Parameter Constraints
    std::string m_additional_constraints;
    bool m_read_only;
    bool m_dynamic_typing;

    // This is a floating point so we'll use floating points in the range
    float m_min_float  {0.0f};
    float m_max_float  {1.0f};
    float m_step_float {0.0f};

    // Now here is the integer version so we'll use ints in the range
    int  m_min_int  {0};
    int  m_max_int  {1};
    int  m_step_int {0};
};

} // namespace rclcpp

#endif // headerguards for parameter_descriptor
