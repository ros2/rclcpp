# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /opt/homebrew/Cellar/cmake/3.26.4/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/Cellar/cmake/3.26.4/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/rclcpp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp

# Include any dependencies generated for this target.
include test/benchmark/CMakeFiles/benchmark_init_shutdown.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include test/benchmark/CMakeFiles/benchmark_init_shutdown.dir/compiler_depend.make

# Include the progress variables for this target.
include test/benchmark/CMakeFiles/benchmark_init_shutdown.dir/progress.make

# Include the compile flags for this target's objects.
include test/benchmark/CMakeFiles/benchmark_init_shutdown.dir/flags.make

test/benchmark/CMakeFiles/benchmark_init_shutdown.dir/benchmark_init_shutdown.cpp.o: test/benchmark/CMakeFiles/benchmark_init_shutdown.dir/flags.make
test/benchmark/CMakeFiles/benchmark_init_shutdown.dir/benchmark_init_shutdown.cpp.o: /Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/rclcpp/test/benchmark/benchmark_init_shutdown.cpp
test/benchmark/CMakeFiles/benchmark_init_shutdown.dir/benchmark_init_shutdown.cpp.o: test/benchmark/CMakeFiles/benchmark_init_shutdown.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/benchmark/CMakeFiles/benchmark_init_shutdown.dir/benchmark_init_shutdown.cpp.o"
	cd /Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp/test/benchmark && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT test/benchmark/CMakeFiles/benchmark_init_shutdown.dir/benchmark_init_shutdown.cpp.o -MF CMakeFiles/benchmark_init_shutdown.dir/benchmark_init_shutdown.cpp.o.d -o CMakeFiles/benchmark_init_shutdown.dir/benchmark_init_shutdown.cpp.o -c /Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/rclcpp/test/benchmark/benchmark_init_shutdown.cpp

test/benchmark/CMakeFiles/benchmark_init_shutdown.dir/benchmark_init_shutdown.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/benchmark_init_shutdown.dir/benchmark_init_shutdown.cpp.i"
	cd /Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp/test/benchmark && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/rclcpp/test/benchmark/benchmark_init_shutdown.cpp > CMakeFiles/benchmark_init_shutdown.dir/benchmark_init_shutdown.cpp.i

test/benchmark/CMakeFiles/benchmark_init_shutdown.dir/benchmark_init_shutdown.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/benchmark_init_shutdown.dir/benchmark_init_shutdown.cpp.s"
	cd /Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp/test/benchmark && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/rclcpp/test/benchmark/benchmark_init_shutdown.cpp -o CMakeFiles/benchmark_init_shutdown.dir/benchmark_init_shutdown.cpp.s

# Object files for target benchmark_init_shutdown
benchmark_init_shutdown_OBJECTS = \
"CMakeFiles/benchmark_init_shutdown.dir/benchmark_init_shutdown.cpp.o"

# External object files for target benchmark_init_shutdown
benchmark_init_shutdown_EXTERNAL_OBJECTS =

test/benchmark/benchmark_init_shutdown: test/benchmark/CMakeFiles/benchmark_init_shutdown.dir/benchmark_init_shutdown.cpp.o
test/benchmark/benchmark_init_shutdown: test/benchmark/CMakeFiles/benchmark_init_shutdown.dir/build.make
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/google_benchmark_vendor/lib/libbenchmark_main.1.6.1.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/performance_test_fixture/lib/libperformance_test_fixture.dylib
test/benchmark/benchmark_init_shutdown: librclcpp.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/google_benchmark_vendor/lib/libbenchmark.1.6.1.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/osrf_testing_tools_cpp/lib/libmemory_tools.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/libstatistics_collector/lib/liblibstatistics_collector.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/rcl/lib/librcl.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/rcl_logging_interface/lib/librcl_logging_interface.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/rmw_implementation/lib/librmw_implementation.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/ament_index_cpp/lib/libament_index_cpp.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/type_description_interfaces/lib/libtype_description_interfaces__rosidl_typesupport_fastrtps_c.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/type_description_interfaces/lib/libtype_description_interfaces__rosidl_typesupport_introspection_c.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/type_description_interfaces/lib/libtype_description_interfaces__rosidl_typesupport_introspection_cpp.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/type_description_interfaces/lib/libtype_description_interfaces__rosidl_typesupport_fastrtps_cpp.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/type_description_interfaces/lib/libtype_description_interfaces__rosidl_typesupport_cpp.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/type_description_interfaces/lib/libtype_description_interfaces__rosidl_generator_py.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/type_description_interfaces/lib/libtype_description_interfaces__rosidl_typesupport_c.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/type_description_interfaces/lib/libtype_description_interfaces__rosidl_generator_c.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/rcl_interfaces/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/service_msgs/lib/libservice_msgs__rosidl_typesupport_fastrtps_c.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/rcl_interfaces/lib/librcl_interfaces__rosidl_typesupport_introspection_c.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/service_msgs/lib/libservice_msgs__rosidl_typesupport_introspection_c.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/rcl_interfaces/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/service_msgs/lib/libservice_msgs__rosidl_typesupport_introspection_cpp.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/rcl_interfaces/lib/librcl_interfaces__rosidl_typesupport_fastrtps_cpp.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/service_msgs/lib/libservice_msgs__rosidl_typesupport_fastrtps_cpp.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/rcl_interfaces/lib/librcl_interfaces__rosidl_typesupport_cpp.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/service_msgs/lib/libservice_msgs__rosidl_typesupport_cpp.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/rcl_interfaces/lib/librcl_interfaces__rosidl_generator_py.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/rcl_interfaces/lib/librcl_interfaces__rosidl_typesupport_c.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/rcl_interfaces/lib/librcl_interfaces__rosidl_generator_c.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/service_msgs/lib/libservice_msgs__rosidl_generator_py.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/service_msgs/lib/libservice_msgs__rosidl_typesupport_c.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/service_msgs/lib/libservice_msgs__rosidl_generator_c.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/rcl_yaml_param_parser/lib/librcl_yaml_param_parser.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/rosgraph_msgs/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_c.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/rosgraph_msgs/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_cpp.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/rosgraph_msgs/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/rosgraph_msgs/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/rosgraph_msgs/lib/librosgraph_msgs__rosidl_typesupport_cpp.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/rosgraph_msgs/lib/librosgraph_msgs__rosidl_generator_py.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/rosgraph_msgs/lib/librosgraph_msgs__rosidl_typesupport_c.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/rosgraph_msgs/lib/librosgraph_msgs__rosidl_generator_c.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/statistics_msgs/lib/libstatistics_msgs__rosidl_typesupport_fastrtps_c.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/builtin_interfaces/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_c.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/rosidl_typesupport_fastrtps_c/lib/librosidl_typesupport_fastrtps_c.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/statistics_msgs/lib/libstatistics_msgs__rosidl_typesupport_fastrtps_cpp.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/builtin_interfaces/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_cpp.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/rosidl_typesupport_fastrtps_cpp/lib/librosidl_typesupport_fastrtps_cpp.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/rmw/lib/librmw.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/rosidl_dynamic_typesupport/lib/librosidl_dynamic_typesupport.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/fastcdr/lib/libfastcdr.1.0.27.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/statistics_msgs/lib/libstatistics_msgs__rosidl_typesupport_introspection_c.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/builtin_interfaces/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/statistics_msgs/lib/libstatistics_msgs__rosidl_typesupport_introspection_cpp.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/builtin_interfaces/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/rosidl_typesupport_introspection_cpp/lib/librosidl_typesupport_introspection_cpp.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/rosidl_typesupport_introspection_c/lib/librosidl_typesupport_introspection_c.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/statistics_msgs/lib/libstatistics_msgs__rosidl_typesupport_cpp.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/builtin_interfaces/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/rosidl_typesupport_cpp/lib/librosidl_typesupport_cpp.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/statistics_msgs/lib/libstatistics_msgs__rosidl_generator_py.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/builtin_interfaces/lib/libbuiltin_interfaces__rosidl_generator_py.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/statistics_msgs/lib/libstatistics_msgs__rosidl_typesupport_c.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/builtin_interfaces/lib/libbuiltin_interfaces__rosidl_typesupport_c.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/statistics_msgs/lib/libstatistics_msgs__rosidl_generator_c.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/builtin_interfaces/lib/libbuiltin_interfaces__rosidl_generator_c.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/rosidl_typesupport_c/lib/librosidl_typesupport_c.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/rcpputils/lib/librcpputils.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/rosidl_runtime_c/lib/librosidl_runtime_c.dylib
test/benchmark/benchmark_init_shutdown: /Users/cursedrock17/ros2_rolling/install/rcutils/lib/librcutils.dylib
test/benchmark/benchmark_init_shutdown: /opt/homebrew/opt/python@3.11/Frameworks/Python.framework/Versions/3.11/lib/python3.11/config-3.11-darwin/libpython3.11.dylib
test/benchmark/benchmark_init_shutdown: test/benchmark/CMakeFiles/benchmark_init_shutdown.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable benchmark_init_shutdown"
	cd /Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp/test/benchmark && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/benchmark_init_shutdown.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/benchmark/CMakeFiles/benchmark_init_shutdown.dir/build: test/benchmark/benchmark_init_shutdown
.PHONY : test/benchmark/CMakeFiles/benchmark_init_shutdown.dir/build

test/benchmark/CMakeFiles/benchmark_init_shutdown.dir/clean:
	cd /Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp/test/benchmark && $(CMAKE_COMMAND) -P CMakeFiles/benchmark_init_shutdown.dir/cmake_clean.cmake
.PHONY : test/benchmark/CMakeFiles/benchmark_init_shutdown.dir/clean

test/benchmark/CMakeFiles/benchmark_init_shutdown.dir/depend:
	cd /Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/rclcpp /Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/rclcpp/test/benchmark /Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp /Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp/test/benchmark /Users/cursedrock17/Documents/Coding/CPP/ros2/rclcpp/build/rclcpp/test/benchmark/CMakeFiles/benchmark_init_shutdown.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/benchmark/CMakeFiles/benchmark_init_shutdown.dir/depend

