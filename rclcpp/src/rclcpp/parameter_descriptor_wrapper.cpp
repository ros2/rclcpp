

#include "../../include/rclcpp/parameter_descriptor_wrapper.hpp"

namespace rclcpp {
// We use initializer lists in order to promote safety in uninitialzied state
ParameterDescription(std::string name, std::uint8 type, std::string description, std::string additional_constraints, bool read_only. bool dynamic_typing) : m_name{name}, parameter_descriptor.type{type}, m_description{description}, m_additional_constraints{additional_constraints}, read_only{read_only}, dyanmic_typing{m_dyanmic_typing}
{}


// ParameterDescription - FloatingPointDescription
// First the build methods to connect to the base class in the builder
ParameterDescription FloatingPointDescription::build() const
{
    return ParameterDescription(m_name, parameter_descriptor.type, m_description, m_additional_constraints, m_read_only, m_dynamic_typing)
}

// Builder methods which set up the original class
// They all follow the same format of initing the value given within the base class, then returning this current class
FloatingPointDescription& FloatingPointDescription::SetName(std::string name)
{
    m_name = name;
    return *this;
}

FloatingPointDescription& FloatingPointDescription::SetType(std::uint8_t type)
{
    parameter_descriptor.type = type;
    return *this;
}

FloatingPointDescription& FloatingPointDescription::SetDescription(std::string description)
{
    m_description = description;
    return *this;
}

FloatingPointDescription& FloatingPointDescription::SetAdditionalConstraints(std::string constraints)
{
    m_additional_constraints = constraints;
    return *this;
}

FloatingPointDescription& FloatingPointDescription::SetReadOnly(bool read_only)
{
    m_read_only = read_only;
    return *this;
}

FloatingPointDescription& FloatingPointDescription::SetDynamicTyping(bool dynamic_typing)
{
    m_dynamic_typing = dynamic_typing;
    return *this;
}

// These are the extension for this class that don't have access in the main class so we'll initialize here
FloatingPointDescription& FloatingPointDescription::SetMin(float min)
{
    m_min = min;
    return *this;
}

FloatingPointDescription& FloatingPointDescription::SetMax(float max)
{
    m_max = max;
    return *this;
}

FloatingPointDescription& FloatingPointDescription::SetStep(float step)
{
    m_step = step;
    return *this;
}

// Here is the Specific range function for this parameter description
FloatingPointDescription& FloatingPointDescription::SetFloatingPointDescripion(float min, float max, float step)
{
        parameter_descriptor.floating_point_range.resize(1);
        parameter_descriptor.floating_point_range.at(0).from_value = min;
        parameter_descriptor.floating_point_range.at(0).to_value = max;
        parameter_descriptor.floating_point_range.at(0).step = step;
        return *this;
}

} // rclcpp namespace
