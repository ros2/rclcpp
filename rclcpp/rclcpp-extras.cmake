# copied from rclcpp/rclcpp-extras.cmake

ament_index_get_resources(_middleware_implementations "ros_middleware_implementation")
if("${_middleware_implementations}" STREQUAL "")
  message(FATAL_ERROR "Could not find any ROS middleware implementation.")
endif()

set(_middleware_implementation "")
# option()
if(NOT "${ROS_MIDDLEWARE_IMPLEMENTATION}" STREQUAL "")
  set(_middleware_implementation "${ROS_MIDDLEWARE_IMPLEMENTATION}")
elseif(NOT "$ENV{ROS_MIDDLEWARE_IMPLEMENTATION}" STREQUAL "")
  set(_middleware_implementation "$ENV{ROS_MIDDLEWARE_IMPLEMENTATION}")
else()
  # TODO detemine "default" implementation based on the available ones
  list(GET _middleware_implementations 1 _middleware_implementation)
endif()

list(FIND _middleware_implementations "${_middleware_implementation}" _index)
if(_index EQUAL -1)
  string(REPLACE ";" ", " _middleware_implementations_string "${_middleware_implementations}")
  message(FATAL_ERROR "Could not find ROS middleware implementation '${_middleware_implementation}'. Choose one of the following: ${_middleware_implementations_string}")
endif()

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
