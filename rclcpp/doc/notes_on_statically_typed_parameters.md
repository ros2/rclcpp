# Notes on statically typed parameters

## Introduction

Until ROS 2 Foxy, all parameters could change type anytime, except if the user installed a parameter callback to reject a change.
This could generate confusing errors, for example:

```
$ ros2 run demo_nodes_py listener &
$ ros2 param set /listener use_sim_time not_a_boolean
[ERROR] [1614712713.233733147] [listener]: use_sim_time parameter set to something besides a bool
Set parameter successful
$ ros2 param get /listener use_sim_time
String value is: not_a_boolean
```

For most use cases, having static parameter types is enough.
This article documents some of the decisions that were made when implementing static parameter types enforcement in:

* https://github.com/ros2/rclcpp/pull/1522
* https://github.com/ros2/rclpy/pull/683

## Allowing dynamically typed parameters

There might be some scenarios where dynamic typing is desired, so a `dynamic_typing` field was added to the [parameter descriptor](https://github.com/ros2/rcl_interfaces/blob/09b5ed93a733adb9deddc47f9a4a8c6e9f584667/rcl_interfaces/msg/ParameterDescriptor.msg#L25).
It defaults to `false`.

For example:

```cpp
rcl_interfaces::msg::ParameterDescriptor descriptor;
descriptor.dynamic_typing = true;

node->declare_parameter("dynamically_typed_parameter", rclcpp::ParameterValue{}, descriptor);
```

```py
rcl_interfaces.msg.ParameterDescriptor descriptor;
descriptor.dynamic_typing = True;

node.declare_parameter("dynamically_typed_parameter", None, descriptor);
```

## How is the parameter type specified?

The parameter type will be inferred from the default value provided when declaring it.

## Statically typed parameters when allowing undeclared parameters

When undeclared parameters are allowed and a parameter is set without a previous declaration, the parameter will be dynamically typed.
This is consistent with other allowed behaviors when undeclared parameters are allowed, e.g. trying to get an undeclared parameter returns "NOT_SET".
Parameter declarations will remain special and dynamic or static typing will be used based on the parameter descriptor (default to static).

## Declaring a parameter without a default value

There might be cases were a default value does not make sense and the user must always provide an override.
In those cases, the parameter type can be specified explicitly:

```cpp
// method signature
template<typename T>
Node::declare_parameter<T>(std::string name, rcl_interfaces::msg::ParameterDescriptor = rcl_interfaces::msg::ParameterDescriptor{});
// or alternatively
Node::declare_parameter(std::string name, rclcpp::ParameterType type, rcl_interfaces::msg::ParameterDescriptor = rcl_interfaces::msg::ParameterDescriptor{});

// examples
node->declare_parameter<int64_t>("my_integer_parameter");  // declare an integer parameter
node->declare_parameter("another_integer_parameter", rclcpp::ParameterType::PARAMETER_INTEGER);  // another way to do the same
```

```py
# method signature
Node.declare_parameter(name: str, param_type: rclpy.Parameter.Type, descriptor: rcl_interfaces.msg.ParameterDescriptor = rcl_interfaces.msg.ParameterDescriptor())

# example
node.declare_parameter('my_integer_parameter', rclpy.Parameter.Type.INTEGER);  # declare an integer parameter
```

If the parameter may be unused, but when used requires a parameter override, then you could conditionally declare it:

```cpp
auto mode = node->declare_parameter("mode", "modeA");  // "mode" parameter is an string
if (mode == "modeB") {
    node->declare_parameter<int64_t>("param_needed_when_mode_b");  // when "modeB", the user must provide "param_needed_when_mode_b"
}
```

## Other migration notes

Declaring a parameter with only a name is deprecated:

```cpp
node->declare_parameter("my_param");  // this generates a build warning
```

```py
node.declare_parameter("my_param");  # this generates a python user warning
```

Before, you could initialize a parameter with the "NOT SET" value (in cpp `rclcpp::ParameterValue{}`, in python `None`).
Now this will throw an exception in both cases:

```cpp
node->declare_parameter("my_param", rclcpp::ParameterValue{});  // not valid, will throw exception
```

```py
node.declare_parameter("my_param", None);  # not valid, will raise error
```

## Possible improvements

### Easier way to declare dynamically typed parameters

Declaring a dynamically typed parameter in `rclcpp` could be considered to be a bit verbose:

```cpp
rcl_interfaces::msg::ParameterDescriptor descriptor;
descriptor.dynamic_typing = true;

node->declare_parameter(name, rclcpp::ParameterValue{}, descriptor);
```

the following ways could be supported to make it simpler:

```cpp
node->declare_parameter(name, rclcpp::PARAMETER_DYNAMIC);
node->declare_parameter(name, default_value, rclcpp::PARAMETER_DYNAMIC);
```

or alternatively:

```cpp
node->declare_parameter(name, default_value, rclcpp::ParameterDescriptor{}.dynamic_typing());
```

For `rclpy`, there's already a short way to do it:

```py
node.declare_parameter(name, default_value, rclpy.ParameterDescriptor(dynamic_typing=true));
```
