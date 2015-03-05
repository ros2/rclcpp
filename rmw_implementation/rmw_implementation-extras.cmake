# copied from rmw_implementation/rmw_implementation-extras.cmake

include("${rmw_implementation_DIR}/get_available_rmw_implementations.cmake")
include("${rmw_implementation_DIR}/get_default_rmw_implementation.cmake")

get_default_rmw_implementation(_middleware_implementation)
find_package("${_middleware_implementation}" REQUIRED)

# TODO should never need definitions and include dirs?
list(APPEND rmw_implementation_DEFINITIONS ${${_middleware_implementation}_DEFINITIONS})
list(APPEND rmw_implementation_INCLUDE_DIRS ${${_middleware_implementation}_INCLUDE_DIRS})
list(APPEND rmw_implementation_LIBRARIES ${${_middleware_implementation}_LIBRARIES})
