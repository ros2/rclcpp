# `rclcpp_generic`

`rclcpp_generic` contains a publisher and subscription for serialized messages.

## How to use it
You specify a topic name, like "/my_node/velocity", and its type, like "geometry_msgs/Twist" or the newer variant "geometry_msgs/msg/Twist".

For a publisher, in a node:

```cpp
GenericPublisher::SharedPtr pub = rclcpp_generic::GenericPublisher::create(
  get_node_topics_interface(),
  "/my_node/velocity",
  "geometry_msgs/Twist",
  rclcpp::QoS{1});
```

And for a subscription, in a node:
```cpp
GenericSubscription::SharedPtr sub = rclcpp_generic::GenericSubscription::create(
  get_node_topics_interface(), "/my_node/velocity", "geometry_msgs/Twist", rclcpp::QoS{1},
  [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {RCLCPP_INFO(this->get_logger(), "Got message");});
```

It works for packages installed from the ROS repositories as well as locally built packages. Make sure the `AMENT_PREFIX_PATH` environment variable has been populated with its install location, usually by sourcing the appropriate install script.

## How it works
To handle messages, even in their serialized form, type support information needs to be provided. To obtain that from the message type, the corresponding package install location is looked up, and the package's dynamic library with the type support information is loaded.

For the above example, on Ubuntu 20.04 running ROS2 Foxy, that library is called `libgeometry_msgs__rosidl_typesupport_cpp.so` and is installed in `/opt/ros/foxy/lib` if you have installed the `ros-foxy-geometry-msgs` package.
