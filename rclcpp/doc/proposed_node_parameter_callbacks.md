# Proposed node parameters callback Design

## Introduction:

The original requirement came in **gazebo_ros_pkgs** for setting individual wheel slip parameters based on global wheel slip value [link to original issue](https://github.com/ros-simulation/gazebo_ros_pkgs/pull/1365). 

The main requirement is to set one or more parameters after another parameter is set successfully. 

Additionally, it would be nice if users could be notified locally (via a callback) when parameters have been set successfully (i.e. post validation).

Related discussion can be found in [#609](https://github.com/ros2/rclcpp/issues/609) [#1789](https://github.com/ros2/rclcpp/pull/1789)

With the current parameters API, the `add_on_set_parameters_callback` is intended for validation of parameter values before they are set, it should **not** cause any side-effects. 

There is also the `ParameterEventHandler` that publishes changes to node parameters on `/parameter_events` topic for external nodes to see. Though the node could subscribe to the `/parameter_events` topic to be notified of changes to its own parameters, it is less than ideal since there is a delay caused by waiting for an executor to process the callback.

We propose adding a `PostSetParametersCallbackHandle` for successful parameter set similar to `OnSetParametersCallbackHandle` for parameter validation. Also, we propose adding a `PreSetParametersCallbackHandle` useful for modifying list of parameters being set.

The validation callback is often abused to trigger side effects in the code, for instance updating class attributes even before a parameter has been set successfully. Instead of relying on the `/parameter_events` topic to be notified of parameter changes, users can register a callback with a new API, `add_post_set_parameters_callback`.

It is possible to use the proposed `add_post_set_parameters_callback` for setting additional parameters, but this might result in infinite recursion and does not allow those additional parameters to be set atomically with the original parameter(s) changed.
To workaround these issues, we propose adding a "pre set" callback type that can be registered with `add_pre_set_parameters_callback`, which will be triggered before the validation callbacks and can be used to modify the parameter list. 

![Desgin API](https://github.com/ros2/rclcpp/blob/deepanshu/local-param-changed-callback-support/rclcpp/doc/param_callback_design.png?raw=true)

## Alternatives

* Users could call `set_parameter` while processing a message from the `/parameter_events` topic, however, there is extra overhead in having to create subscription (as noted earlier).
* Users could call `set_parameter` inside the "on set" parameters callback, however it is not well-defined how side-effects should handle cases where parameter validation fails.