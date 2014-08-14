#
# Get the package name of the default ROS middleware implemenation.
#
# Either selcting it using the variable ROS_MIDDLEWARE_IMPLEMENTATION or
# choosing a default from the available implementations.
#
# :param var: the output variable name containing the package name
# :type var: string
#
function(get_default_ros_middleware_implementation var)
  get_available_ros_middleware_implementations(middleware_implementations)

  if("${middleware_implementations}" STREQUAL "")
    message(FATAL_ERROR "Could not find any ROS middleware implementation.")
  endif()

  # option()
  if(NOT "${ROS_MIDDLEWARE_IMPLEMENTATION}" STREQUAL "")
    set(middleware_implementation "${ROS_MIDDLEWARE_IMPLEMENTATION}")
  elseif(NOT "$ENV{ROS_MIDDLEWARE_IMPLEMENTATION}" STREQUAL "")
    set(middleware_implementation "$ENV{ROS_MIDDLEWARE_IMPLEMENTATION}")
  else()
    # TODO detemine "default" implementation based on the available ones
    list(GET middleware_implementations 0 middleware_implementation)
  endif()

  # verify that the selection one is available
  list(FIND middleware_implementations "${middleware_implementation}" _index)
  if(_index EQUAL -1)
    string(REPLACE ";" ", " middleware_implementations_string "${middleware_implementations}")
    message(FATAL_ERROR "Could not find ROS middleware implementation '${middleware_implementation}'. Choose one of the following: ${middleware_implementations_string}")
  endif()
  find_package("${middleware_implementation}" REQUIRED)

  # persist implementation decision in cache
  set(
    ROS_MIDDLEWARE_IMPLEMENTATION "${middleware_implementation}"
    CACHE STRING "Select ROS middleware implementation to link against" FORCE
  )

  set(${var} ${middleware_implementation} PARENT_SCOPE)
endfunction()
