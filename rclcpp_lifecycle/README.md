# `rclcpp_lifecycle`

Package containing a prototype for lifecycle implementation. Visit the [design document](https://design.ros2.org/articles/node_lifecycle.html) for more information about this package.

- [Lifecycle states](include/rclcpp_lifecycle/state.hpp) Define the State class
- [Lifecycle transitions](include/rclcpp_lifecycle/transition.hpp) Define the Transition class
- [Lifecycle publisher](include/rclcpp_lifecycle/publisher.hpp) A managed node can then deactivate or activate the publishing.
- [Lifecycle node](include/rclcpp_lifecycle/lifecycle_node.hpp) LifecycleNode for creating lifecycle components

Some useful internal abstractions and utilities:
- Macros for controlling symbol visibility on the library
 - [rclcpp_lifecycle/visibility_control.h](include/rclcpp_lifecycle/visibility_control.h)

## Quality Declaration

This package claims to be in the **Quality Level 4** category, see the [Quality Declaration](QUALITY_DECLARATION.md) for more details.
