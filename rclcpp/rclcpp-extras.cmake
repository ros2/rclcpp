# copied from rclcpp/rclcpp-extras.cmake

# option()
set(_middleware_implementation "")
if(NOT "${ROS_MIDDLEWARE_IMPLEMENTATION}" STREQUAL "")
  set(_middleware_implementation "${ROS_MIDDLEWARE_IMPLEMENTATION}")
elseif(NOT "$ENV{ROS_MIDDLEWARE_IMPLEMENTATION}" STREQUAL "")
  set(_middleware_implementation "$ENV{ROS_MIDDLEWARE_IMPLEMENTATION}")
else()
  # detemine "default" implementation based on the available ones
  set(_middleware_implementation "ros_middleware_connext_cpp")
  if("${_middleware_implementation}" STREQUAL "")
    message(FATAL_ERROR "No middleware implementation available.")
  endif()
endif()

message("rclcpp_DEFINITIONS ${rclcpp_DEFINITIONS}")
message("rclcpp_INCLUDE_DIRS ${rclcpp_INCLUDE_DIRS}")
message("rclcpp_LIBRARIES ${rclcpp_LIBRARIES}")

message(STATUS "Using middleware implementation: ${_middleware_implementation}")
find_package("${_middleware_implementation}" REQUIRED)

# persist implementation decision in cache
set(
  ROS_MIDDLEWARE_IMPLEMENTATION "${_middleware_implementation}"
  CACHE STRING "Select ROS middleware implementation to link against" FORCE
)

list(APPEND rclcpp_DEFINITIONS ${${ROS_MIDDLEWARE_IMPLEMENTATION}_DEFINITIONS})
list(APPEND rclcpp_INCLUDE_DIRS ${${ROS_MIDDLEWARE_IMPLEMENTATION}_INCLUDE_DIRS})
list(APPEND rclcpp_LIBRARIES ${${ROS_MIDDLEWARE_IMPLEMENTATION}_LIBRARIES})

message("rclcpp_DEFINITIONS ${rclcpp_DEFINITIONS}")
message("rclcpp_INCLUDE_DIRS ${rclcpp_INCLUDE_DIRS}")
message("rclcpp_LIBRARIES ${rclcpp_LIBRARIES}")
