This document is a declaration of software quality for the `rclcpp` package, based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html).

# rclcpp Quality Declaration

The package `rclcpp` claims to be in the **Quality Level 1** category when it is used with a **Quality Level 1** middleware.

Below are the rationales, notes, and caveats for this claim, organized by each requirement listed in the [Package Quality Categories in REP-2004](https://www.ros.org/reps/rep-2004.html) of the ROS2 developer guide.

## Version Policy [1]

### Version Scheme [1.i]

`rclcpp` uses `semver` according to the recommendation for ROS Core packages in the [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#versioning).

### Version Stability [1.ii]

`rclcpp` is at a stable version, i.e. `>= 1.0.0`.
The current version can be found in its [package.xml](package.xml), and its change history can be found in its [CHANGELOG](CHANGELOG.rst).

### Public API Declaration [1.iii]

All symbols in the installed headers are considered part of the public API.

Except for the exclusions listed below, all installed headers are in the `include` directory of the package, headers in any other folders are not installed and considered private.
Headers under the folder `experimental` are not considered part of the public API as they have not yet been stabilized. These symbols are namespaced `rclcpp::experimental`.
Headers under the folder `detail` are not considered part of the public API and are subject to change without notice. These symbols are namespaced `rclcpp::detail`.

### API Stability Policy [1.iv]

`rclcpp` will not break public API within a released ROS distribution, i.e. no major releases once the ROS distribution is released.

### ABI Stability Policy [1.v]

`rclcpp` contains C++ code and therefore must be concerned with ABI stability, and will maintain ABI stability within a ROS distribution.

### ABI and ABI Stability Within a Released ROS Distribution [1.vi]

`rclcpp` will not break API nor ABI within a released ROS distribution, i.e. no major releases once the ROS distribution is released.

## Change Control Process [2]

`rclcpp` follows the recommended guidelines for ROS Core packages in the [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#change-control-process).

### Change Requests [2.i]

All changes will occur through a pull request, check [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#change-control-process) for additional information.

### Contributor Origin [2.ii]

This package uses DCO as its confirmation of contributor origin policy. More information can be found in [CONTRIBUTING](../CONTRIBUTING.md).

### Peer Review Policy [2.iii]

All pull requests will be peer-reviewed, check [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#change-control-process) for additional information.

### Continuous Integration [2.iv]

All pull requests must pass CI on all [tier 1 platforms](https://www.ros.org/reps/rep-2000.html#support-tiers)

Currently nightly results can be seen here:

* [linux-aarch64_release](https://ci.ros2.org/view/nightly/job/nightly_linux-aarch64_release/lastBuild/testReport/rclcpp/)
* [linux_release](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/testReport/rclcpp/)
* [mac_osx_release](https://ci.ros2.org/view/nightly/job/nightly_osx_release/lastBuild/testReport/rclcpp/)
* [windows_release](https://ci.ros2.org/view/nightly/job/nightly_win_rel/lastBuild/testReport/rclcpp/)

###  Documentation Policy [2.v]

All pull requests must resolve related documentation changes before merging.

## Documentation [3]

### Feature Documentation [3.i]

`rclcpp` has a [feature list](http://docs.ros2.org/latest/api/rclcpp/) and each item in the list links to the corresponding feature documentation. There is documentation for all of the features, and new features require documentation before being added.

### Public API Documentation [3.ii]

The API is publicly available in its [ROS 2 API documentation](http://docs.ros2.org/latest/api/rclcpp/).

### License [3.iii]

The license for `rclcpp` is Apache 2.0, and a summary is in each source file, the type is declared in the [`package.xml`](./package.xml) manifest file, and a full copy of the license is in the [`LICENSE`](../LICENSE) file.

There is an automated test which runs a linter that ensures each file has a license statement. [Here](http://build.ros2.org/view/Rpr/job/Rpr__rclcpp__ubuntu_focal_amd64/lastCompletedBuild/testReport/rclcpp/) can be found a list with the latest results of the various linters being run on the package.

### Copyright Statements [3.iv]

The copyright holders each provide a statement of copyright in each source code file in `rclcpp`.

There is an automated test which runs a linter that ensures each file has at least one copyright statement. Latest linter result report can be seen [here](http://build.ros2.org/view/Rpr/job/Rpr__rclcpp__ubuntu_focal_amd64/lastCompletedBuild/testReport/rclcpp/copyright/).

## Testing [4]

### Feature Testing [4.i]

Each feature in `rclcpp` has corresponding tests which simulate typical usage, and they are located in the [`test`](https://github.com/ros2/rclcpp/tree/master/test) directory.
New features are required to have tests before being added.

Currently nightly test results can be seen here:

* [linux-aarch64_release](https://ci.ros2.org/view/nightly/job/nightly_linux-aarch64_release/lastBuild/testReport/rclcpp/)
* [linux_release](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/testReport/rclcpp/)
* [mac_osx_release](https://ci.ros2.org/view/nightly/job/nightly_osx_release/lastBuild/testReport/rclcpp/)
* [windows_release](https://ci.ros2.org/view/nightly/job/nightly_win_rel/lastBuild/testReport/rclcpp/)

### Public API Testing [4.ii]

Each part of the public API has tests, and new additions or changes to the public API require tests before being added.
The tests aim to cover both typical usage and corner cases, but are quantified by contributing to code coverage.

### Coverage [4.iii]

`rclcpp` follows the recommendations for ROS Core packages in the [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#code-coverage), and opts to use line coverage instead of branch coverage.

This includes:

- tracking and reporting line coverage statistics
- achieving and maintaining a reasonable branch line coverage (90-100%)
- no lines are manually skipped in coverage calculations

Changes are required to make a best effort to keep or increase coverage before being accepted, but decreases are allowed if properly justified and accepted by reviewers.

Current coverage statistics can be viewed [here](https://ci.ros2.org/job/nightly_linux_coverage/lastCompletedBuild/cobertura/src_ros2_rclcpp_rclcpp_src_rclcpp/). A description of how coverage statistics are calculated is summarized in this page ["ROS 2 Onboarding Guide"](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#note-on-coverage-runs).

`rclcpp` has a line coverage `>= 95%`, which is calculated over all directories within `rclcpp` with the exception of the `experimental` directory.

### Performance [4.iv]

`rclcpp` follows the recommendations for performance testing of C/C++ code in the [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#performance), and opts to do performance analysis on each release rather than each change.

The performance tests of `rclcpp` are located in the [test/benchmark directory](https://github.com/ros2/rclcpp/tree/master/rclcpp/test/benchmark).

System level performance benchmarks that cover features of `rclcpp` can be found at:
* [Benchmarks](http://build.ros2.org/view/Rci/job/Rci__benchmark_ubuntu_focal_amd64/BenchmarkTable/)
* [Performance](http://build.ros2.org/view/Rci/job/Rci__nightly-performance_ubuntu_focal_amd64/lastCompletedBuild/)

Changes that introduce regressions in performance must be adequately justified in order to be accepted and merged.

### Linters and Static Analysis [4.v]

`rclcpp` uses and passes all the ROS2 standard linters and static analysis tools for a C++ package as described in the [ROS 2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#linters-and-static-analysis). Passing implies there are no linter/static errors when testing against CI of supported platforms.

Currently nightly test results can be seen here:
* [linux-aarch64_release](https://ci.ros2.org/view/nightly/job/nightly_linux-aarch64_release/lastBuild/testReport/rclcpp/)
* [linux_release](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/testReport/rclcpp/)
* [mac_osx_release](https://ci.ros2.org/view/nightly/job/nightly_osx_release/lastBuild/testReport/rclcpp/)
* [windows_release](https://ci.ros2.org/view/nightly/job/nightly_win_rel/lastBuild/testReport/rclcpp/)

## Dependencies [5]

Below are evaluations of each of `rclcpp`'s run-time and build-time dependencies that have been determined to influence the quality.

It has several "buildtool" dependencies, which do not affect the resulting quality of the package, because they do not contribute to the public library API.

It also has several test dependencies, which do not affect the resulting quality of the package, because they are only used to build and run the test code.

### Direct and Optional Runtime ROS Dependencies [5.i]/[5.ii]

`rclcpp` has the following runtime ROS dependencies:

#### `libstatistics_collector`

The `libstatistics_collector` package provides lightweight aggregation utilities to collect statistics and measure message metrics.

It is **Quality Level 1**, see its [Quality Declaration document](https://github.com/ros-tooling/libstatistics_collector/tree/master/QUALITY_DECLARATION.md).

#### `rcl`

`rcl` a library to support implementation of language specific ROS 2 Client Libraries.

It is **Quality Level 1**, see its [Quality Declaration document](https://github.com/ros2/rcl/blob/master/rcl/QUALITY_DECLARATION.md).

#### `rcl_yaml_param_parser`

The `rcl_yaml_param_parser` package provides an API that is used to parse YAML configuration files which may be used to configure ROS and specific nodes.

It is **Quality Level 1**, see its [Quality Declaration document](https://github.com/ros2/rcl/tree/master/rcl_yaml_param_parser/QUALITY_DECLARATION.md).

#### `rcpputils`

The `rcpputils` package provides an API which contains common utilities and data structures useful when programming in C++.

It is **Quality Level 1**, see its [Quality Declaration document](https://github.com/ros2/rcpputils/blob/master/QUALITY_DECLARATION.md).

#### `rcutils`

The `rcutils` package provides an API which contains common utilities and data structures useful when programming in C.

It is **Quality Level 1**, see its [Quality Declaration document](https://github.com/ros2/rcutils/blob/master/QUALITY_DECLARATION.md).

#### `rmw`

`rmw` is the ROS 2 middleware library.

It is **Quality Level 1**, see its [Quality Declaration document](https://github.com/ros2/rmw/blob/master/rmw/QUALITY_DECLARATION.md).

#### `statistics_msgs`

The `statistics_msgs` package contains ROS 2 message definitions for reporting statistics for topics and system resources.

It is **Quality Level 1**, see its [Quality Declaration document](https://github.com/ros2/rcl_interfaces/blob/master/statistics_msgs/QUALITY_DECLARATION.md).

#### `tracetools`

The `tracetools` package provides utilities for instrumenting the code in `rclcpp` so that it may be traced for debugging and performance analysis.

It is **Quality Level 1**, see its [Quality Declaration document](https://gitlab.com/ros-tracing/ros2_tracing/-/blob/master/tracetools/QUALITY_DECLARATION.md).

### Direct Runtime non-ROS Dependency [5.iii]

`rclcpp` has no run-time or build-time dependencies that need to be considered for this declaration.

## Platform Support [6]

`rclcpp` supports all of the tier 1 platforms as described in [REP-2000](https://www.ros.org/reps/rep-2000.html#support-tiers), and tests each change against all of them.

Currently nightly build status can be seen here:
* [linux-aarch64_release](https://ci.ros2.org/view/nightly/job/nightly_linux-aarch64_release/lastBuild/rclcpp/)
* [linux_release](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/rclcpp/)
* [mac_osx_release](https://ci.ros2.org/view/nightly/job/nightly_osx_release/lastBuild/rclcpp/)
* [windows_release](https://ci.ros2.org/view/nightly/job/nightly_win_rel/lastBuild/rclcpp/)

## Security

### Vulnerability Disclosure Policy [7.i]

This package conforms to the Vulnerability Disclosure Policy in [REP-2006](https://www.ros.org/reps/rep-2006.html).
