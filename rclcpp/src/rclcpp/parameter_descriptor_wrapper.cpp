

#include "../../include/rclcpp/parameter_descriptor_wrapper.hpp"

namespace rclcpp {
// We use initializer lists in order to promote safety in uninitialzied state
    ParameterDescription::ParameterDescription(std::string name, std::uint8 type, std::string description, std::string additional_constraints, bool read_only. bool dynamic_typing) : m_name{name}, parameter_descriptor.type{type}, m_description{description}, m_additional_constraints{additional_constraints}, read_only{read_only}, dyanmic_typing{m_dyanmic_typing}
{}

ParameterDescription::ParameterDescription(){}

// ParameterDescription - ParameterDescription
// First the build methods to connect to the base class in the builder
ParameterDescription ParameterDescription::build() const
{
    // Return some some sort message
    return ParameterDescription(m_name, parameter_descriptor.type, m_description, m_additional_constraints, m_read_only, m_dynamic_typing)
}

// Builder methods which set up the original class
// They all follow the same format of initing the value given within the base class, then returning this current class
ParameterDescription& ParameterDescription::SetName(const std::string& name)
{
    m_name = name;
    return *this;
}

ParameterDescription& ParameterDescription::SetType(std::uint8_t type)
{
    parameter_descriptor.type = type;
    return *this;
}

ParameterDescription& ParameterDescription::SetDescriptionText(const std::string& description)
{
    m_description = description;
    return *this;
}

ParameterDescription& ParameterDescription::SetAdditionalConstraints(const std::string& constraints)
{
    m_additional_constraints = constraints;
    return *this;
}

ParameterDescription& ParameterDescription::SetReadOnly(bool read_only)
{
    m_read_only = read_only;
    return *this;
}

ParameterDescription& ParameterDescription::SetDynamicTyping(bool dynamic_typing)
{
    m_dynamic_typing = dynamic_typing;
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
