# `rclcpp_components`

Package containing tools for dynamically loadable components.

  - [ComponentManager](include/rclcpp_components/component_manager.hpp) Node to manage components. It has the services to load, unload and list current components.
  - [Node factory](include/rclcpp_components/node_factory.hpp) The NodeFactory interface is used by the class loader to instantiate components.
    - It allows for classes not derived from `rclcpp::Node` to be used as components.
    - It allows derived constructors to be called when components are loaded.

Some useful abstractions and utilities:
  - [RCLCPP_COMPONENTS_REGISTER_NODE](include/rclcpp_components/register_node_macro.hpp) Register a component that can be dynamically loaded at runtime.

Some useful internal abstractions and utilities:
  - Macros for controlling symbol visibility on the library
    - [rclcpp_components/visibility_control.h](include/rclcpp_components/visibility_control.hpp)

Package containing CMake tools for register components:
  - `rclcpp_components_register_node` Register an rclcpp component with the ament resource index and create an executable.
  - `rclcpp_components_register_nodes` Register an rclcpp component with the ament resource index. The passed library can contain multiple nodes each registered via macro.

## Quality Declaration

This package claims to be in the **Quality Level 4** category, see the [Quality Declaration](QUALITY_DECLARATION.md) for more details.
