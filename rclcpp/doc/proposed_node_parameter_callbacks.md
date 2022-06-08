# Proposed node parameters callback Design

## Introduction:
In the current Node API, the ```add_on_set_parameters_callback```
is used for doing any validation for parameters before setting the parameters successfully.Once, the parameters are validated ```ParameterEventHandler``` object is used to publish any changes to node parameters on **/parameter_events** topic, which can be subscribed by any node to see the changed parameter. Note that the ```/parameter_events``` is a topic for all nodes on the network, and we have to rely on executors to process those.

 The validation callback is often abused to trigger side-effects in the code, for instance updating class attributes even before parameter has been set successfully. Instead of relying on          **/parameter_events** it would be good to support an internal node interface API ```add_post_set_parameters_callback``` for registering callbacks for successful parameter set. 

 Also it may be required to trigger setting of some other parameter once a given param is set. The original requirement came in **gazeb_ros_pkgs** for setting individual wheel slip params based on global wheel slip value. Currently, if we try to set param inside ```add_on_set_parameters_callback``` the **ParameterMutationRecursionGuard** will not allow it, which is good. We can potentially use the proposed ```add_post_set_parameters_callback``` for setting some param but this might result in unpredictable behaviour because of infinite recusion and due to the params being set attomically. In order to get around this we propose adding another registered callback ```add_pre_set_parameters_callback``` which will be triggered before the validation callbacks and can be used to modify the parameters at run time. 

## Proposed Architecture


## Alternatives