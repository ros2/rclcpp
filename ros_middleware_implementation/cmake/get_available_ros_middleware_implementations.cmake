#
# Get the package names of the available ROS middleware implemenations.
#
# :param var: the output variable name containing the package names
# :type var: list of strings
#
function(get_available_ros_middleware_implementations var)
  ament_index_get_resources(middleware_implementations "ros_middleware_implementation")
  set(${var} ${middleware_implementations} PARENT_SCOPE)
endfunction()
