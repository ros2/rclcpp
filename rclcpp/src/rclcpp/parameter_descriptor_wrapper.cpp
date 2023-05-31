

#include "../../include/rclcpp/parameter_descriptor_wrapper.hpp"

namespace rclcpp {

ParameterDescription::ParameterDescription()
{
    // Copies all the information in ParameterDescriptor.msg - https://github.com/ros2/rcl_interfaces/blob/rolling/rcl_interfaces/msg/ParameterDescriptor.msg
    // Need to set this in the constructor, but it doesn't necessarily need to be used
    parameter_descriptor.type{rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET};
}

// ParameterDescription - ParameterDescription
// First the build methods to connect to the base class in the builder
rcl_interfaces::msg::ParameterDescriptor ParameterDescription::build() const
{
    // Return some some sort message
    return parameter_descriptor;
}

// Builder methods which set up the original class
// They all follow the same format of initing the value given within the base class, then returning this current class
ParameterDescription& ParameterDescription::SetName(const std::string& name)
{
    parameter_descriptor.name = name;
    return *this;
}

ParameterDescription& ParameterDescription::SetType(std::uint8_t type)
{
    parameter_descriptor.type = type;
    return *this;
}

ParameterDescription& ParameterDescription::SetDescriptionText(const std::string& description)
{
    parameter_descriptor.description = description;
    return *this;
}

ParameterDescription& ParameterDescription::SetAdditionalConstraints(const std::string& constraints)
{
    parameter_descriptor.constraints = constraints;
    return *this;
}

ParameterDescription& ParameterDescription::SetReadOnly(bool read_only)
{
    parameter_descriptor.read_only = read_only;
    return *this;
}

ParameterDescription& ParameterDescription::SetDynamicTyping(bool dynamic_typing)
{
    parameter_descriptor.dynamic_typing = dynamic_typing;
    return *this;
}

// Here is the Specific range function for this parameter description
ParameterDescription& ParameterDescription::SetFloatingPointDescriptionRange(float min, float max, float step)
{
        parameter_descriptor.floating_point_range.resize(1);
        parameter_descriptor.floating_point_range.at(0).from_value = min;
        parameter_descriptor.floating_point_range.at(0).to_value = max;
        parameter_descriptor.floating_point_range.at(0).step = step;
        return *this;
}

ParameterDescription& ParameterDescription::SetIntegerDescriptionRange(int min, int max, int step)
{
        parameter_descriptor.integer_range.resize(1);
        parameter_descriptor.integer_range.at(0).from_value = min;
        parameter_descriptor.integer_range.at(0).to_value = max;
        parameter_descriptor.integer_range.at(0).step = step;
        return *this;
}

} // rclcpp namespace
