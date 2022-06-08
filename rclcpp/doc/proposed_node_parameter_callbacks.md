# Proposed node parameters callback Design

## Introduction:

 The original requirement came in **gazeb_ros_pkgs** for setting individual wheel slip params based on global wheel slip value [Link to original issue](https://github.com/ros-simulation/gazebo_ros_pkgs/pull/1365). The main idea was to set some parameters using ```set_parameter``` API once the a given param was set successfully. The requirement let to the discussion of supporting registered callbacks once the parameters are set and have been requested by users before. 

In the current Node API, the ```add_on_set_parameters_callback``` is used for doing any validation for parameters before setting the parameters successfully. Once the parameters are validated ```ParameterEventHandler``` object is used to publish any changes to node parameters on **/parameter_events** topic, which can be subscribed by any node to see the changed parameter. Note that the **/parameter_events** is a topic for all nodes on the network, and we have to rely on executors to process those.

 The validation callback is often abused to trigger side-effects in the code, for instance updating class attributes even before parameter has been set successfully. Instead of relying on **/parameter_events** it would be good to support an internal node interface API for registering callbacks for successful parameter set ```add_post_set_parameters_callback```.

 We can use the proposed ```add_post_set_parameters_callback``` for setting some param but this might result in unpredictable behaviour because of infinite recusion and due to the params being set attomically. In order to get around this we propose adding another registered callback ```add_pre_set_parameters_callback``` which will be triggered before the validation callbacks.

## Proposed Architecture

![Desgin API](https://github.com/ros2/rclcpp/blob/deepanshu/local-param-changed-callback-support/rclcpp/doc/param_callback_design.png?raw=true)

## Alternatives

* The **/parameter_events** topic can be used to monitor any changes to a node parameters and use ```set_parameter``` API if required to set any paramter further. However, it seems weird that we have to rely on network to notify nodes that their own parameter has been changed. Therefore, it makes sense to support a callback which gets triggered after the parameter is changed, so that it may trigger some more events
* The ```add_on_set_parameters_callback``` can be used to ```set_paramters```, however the ```ParameterMutationRecursionGuard``` will not allow this since the ```add_on_set_parameters_callback``` is supposed to be used for validation purposes only.
* The ```add_post_set_parameters_callback```  can be used to set parameters but this might result in different behaviours depending on whether the params were set atomically or not. Also, we might enter an infinite recursion depending on how params are being set. 