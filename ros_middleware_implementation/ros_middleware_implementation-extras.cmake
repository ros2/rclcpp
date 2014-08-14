# copied from ros_middleware_implementation/ros_middleware_implementation-extras.cmake

include("${ros_middleware_implementation_DIR}/get_available_ros_middleware_implementations.cmake")
include("${ros_middleware_implementation_DIR}/get_default_ros_middleware_implementation.cmake")

get_default_ros_middleware_implementation(_middleware_implementation)
find_package("${_middleware_implementation}" REQUIRED)

# TODO should never need definitions and include dirs?
list(APPEND ros_middleware_implementation_DEFINITIONS ${${_middleware_implementation}_DEFINITIONS})
list(APPEND ros_middleware_implementation_INCLUDE_DIRS ${${_middleware_implementation}_INCLUDE_DIRS})
list(APPEND ros_middleware_implementation_LIBRARIES ${${_middleware_implementation}_LIBRARIES})
