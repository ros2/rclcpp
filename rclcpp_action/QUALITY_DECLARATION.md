This document is a declaration of software quality for the `rclcpp_action` package, based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html).

# rclcpp_action Quality Declaration

The package `rclcpp_action` claims to be in the **Quality Level 1** category when it is used with a **Quality Level 1** middleware.

Below are the rationales, notes, and caveats for this claim, organized by each requirement listed in the [Package Quality Categories in REP-2004](https://www.ros.org/reps/rep-2004.html) of the ROS2 developer guide.

## Version Policy [1]

### Version Scheme [1.i]

`rclcpp_action` uses `semver` according to the recommendation for ROS Core packages in the [ROS 2 Developer Guide](https://docs.ros.org/en/foxy/Contributing/Developer-Guide.html#versioning).

### Version Stability [1.ii]

`rclcpp_action` is at a stable version, i.e. `>= 1.0.0`.
The current version can be found in its [package.xml](package.xml), and its change history can be found in its [CHANGELOG](CHANGELOG.rst).

### Public API Declaration [1.iii]

All symbols in the installed headers are considered part of the public API.

All installed headers are in the `include` directory of the package, headers in any other folders are not installed and considered private.

### API Stability Policy [1.iv]

`rclcpp_action` will not break public API within a released ROS distribution, i.e. no major releases once the ROS distribution is released.

### ABI Stability Policy [1.v]

`rclcpp_action` contains C++ code and therefore must be concerned with ABI stability, and will maintain ABI stability within a ROS distribution.

### ABI and ABI Stability Within a Released ROS Distribution [1.vi]

`rclcpp_action` will not break API nor ABI within a released ROS distribution, i.e. no major releases once the ROS distribution is released.

## Change Control Process [2]

`rclcpp_action` follows the recommended guidelines for ROS Core packages in the [ROS 2 Developer Guide](https://docs.ros.org/en/foxy/Contributing/Developer-Guide.html#change-control-process).

### Change Requests [2.i]

All changes will occur through a pull request, check [ROS 2 Developer Guide](https://docs.ros.org/en/foxy/Contributing/Developer-Guide.html#change-control-process) for additional information.

### Contributor Origin [2.ii]

This package uses DCO as its confirmation of contributor origin policy. More information can be found in [CONTRIBUTING](../CONTRIBUTING.md).

### Peer Review Policy [2.iii]

All pull requests will be peer-reviewed, check [ROS 2 Developer Guide](https://docs.ros.org/en/foxy/Contributing/Developer-Guide.html#change-control-process) for additional information.

### Continuous Integration [2.iv]

All pull requests must pass CI on all [tier 1 platforms](https://www.ros.org/reps/rep-2000.html#support-tiers)

Though there are no nightly jobs for foxy outside of linux, each change is tested on ci.ros2.org.
* [linux-aarch64](https://ci.ros2.org/job/ci_linux-aarch64)
* [linux](https://ci.ros2.org/job/ci_linux)
* [mac_osx](https://ci.ros2.org/job/ci_osx)
* [windows](https://ci.ros2.org/job/ci_windows)

###  Documentation Policy [2.v]

All pull requests must resolve related documentation changes before merging.

## Documentation [3]

### Feature Documentation [3.i]

`rclcpp_action` has a [feature list](http://docs.ros2.org/foxy/api/rclcpp_action/) and each item in the list links to the corresponding feature documentation. There is documentation for all of the features, and new features require documentation before being added.

### Public API Documentation [3.ii]

The API is publicly available in its [ROS 2 API documentation](http://docs.ros2.org/foxy/api/rclcpp_action/).

### License [3.iii]

The license for `rclcpp_action` is Apache 2.0, and a summary is in each source file, the type is declared in the [`package.xml`](./package.xml) manifest file, and a full copy of the license is in the [`LICENSE`](../LICENSE) file.

There is an automated test which runs a linter that ensures each file has a license statement. [Here](http://build.ros2.org/view/Fpr/job/Fpr__rclcpp__ubuntu_focal_amd64/lastCompletedBuild/testReport/rclcpp_action/) can be found a list with the latest results of the various linters being run on the package.

### Copyright Statements [3.iv]

The copyright holders each provide a statement of copyright in each source code file in `rclcpp_action`.

There is an automated test which runs a linter that ensures each file has at least one copyright statement. Latest linter result report can be seen [here](http://build.ros2.org/view/Fpr/job/Fpr__rclcpp__ubuntu_focal_amd64/lastCompletedBuild/testReport/rclcpp_action/copyright/).

## Testing [4]

### Feature Testing [4.i]

Each feature in `rclcpp_action` has corresponding tests which simulate typical usage, and they are located in the [`test`](https://github.com/ros2/rclcpp_action/tree/foxy/test) directory.
New features are required to have tests before being added.

Though there are no nightly jobs for foxy outside of linux, each change is tested on ci.ros2.org.
* [linux-aarch64](https://ci.ros2.org/job/ci_linux-aarch64)
* [linux](https://ci.ros2.org/job/ci_linux)
* [mac_osx](https://ci.ros2.org/job/ci_osx)
* [windows](https://ci.ros2.org/job/ci_windows)

### Public API Testing [4.ii]

Each part of the public API has tests, and new additions or changes to the public API require tests before being added.
The tests aim to cover both typical usage and corner cases, but are quantified by contributing to code coverage.

### Coverage [4.iii]

`rclcpp_action` follows the recommendations for ROS Core packages in the [ROS 2 Developer Guide](https://docs.ros.org/en/foxy/Contributing/Developer-Guide.html#code-coverage), and opts to use line coverage instead of branch coverage.

This includes:

- tracking and reporting line coverage statistics
- achieving and maintaining a reasonable branch line coverage (90-100%)
- no lines are manually skipped in coverage calculations

Changes are required to make a best effort to keep or increase coverage before being accepted, but decreases are allowed if properly justified and accepted by reviewers.

Current coverage statistics can be viewed [here](https://ci.ros2.org/job/nightly_linux_foxy_coverage/lastCompletedBuild/cobertura/src_ros2_rclcpp_rclcpp_action_src/). A description of how coverage statistics are calculated is summarized in this page ["ROS 2 Onboarding Guide"](https://docs.ros.org/en/foxy/Contributing/Developer-Guide.html#note-on-coverage-runs).

### Performance [4.iv]

`rclcpp_action` follows the recommendations for performance testing of C/C++ code in the [ROS 2 Developer Guide](https://docs.ros.org/en/foxy/Contributing/Developer-Guide.html#performance), and opts to do performance analysis on each release rather than each change.

The performance tests of `rclcpp_action` are located in the [test/benchmark directory](https://github.com/ros2/rclcpp/tree/foxy/rclcpp_action/test/benchmark).

System level performance benchmarks that cover features of `rclcpp_action` can be found at:
* [Benchmarks](http://build.ros2.org/view/Fci/job/Fci__benchmark_ubuntu_focal_amd64/BenchmarkTable/)
* [Performance](http://build.ros2.org/view/Fci/job/Fci__nightly-performance_ubuntu_focal_amd64/lastCompletedBuild/)

Changes that introduce regressions in performance must be adequately justified in order to be accepted and merged.

### Linters and Static Analysis [4.v]

`rclcpp_action` uses and passes all the ROS2 standard linters and static analysis tools for a C++ package as described in the [ROS 2 Developer Guide](https://docs.ros.org/en/foxy/Contributing/Developer-Guide.html#linters-and-static-analysis). Passing implies there are no linter/static errors when testing against CI of supported platforms.

Currently nightly test results can be seen [here](http://build.ros2.org/view/Fpr/job/Fpr__rclcpp__ubuntu_focal_amd64/lastCompletedBuild/testReport/rclcpp_action).

## Dependencies [5]

Below are evaluations of each of `rclcpp_action`'s run-time and build-time dependencies that have been determined to influence the quality.

It has several "buildtool" dependencies, which do not affect the resulting quality of the package, because they do not contribute to the public library API.

It also has several test dependencies, which do not affect the resulting quality of the package, because they are only used to build and run the test code.

### Direct and Optional Runtime ROS Dependencies [5.i]/[5.ii]

`rclcpp_action` has the following runtime ROS dependencies:

#### `action_msgs`

`action_msgs` provides messages and services for ROS 2 actions.

It is **Quality Level 1**, see its [Quality Declaration document](https://github.com/ros2/rcl_interfaces/blob/foxy/action_msgs/QUALITY_DECLARATION.md).

#### `rclcpp`

The `rclcpp` package provides the ROS client library in C++.

It is **Quality Level 1**, see its [Quality Declaration document](https://github.com/ros2/rclcpp/blob/foxy/rclcpp/QUALITY_DECLARATION.md).

#### `rcl_action`

The `rcl_action` package provides C-based ROS action implementation.

It is **Quality Level 1**, see its [Quality Declaration document](https://github.com/ros2/rcl/blob/foxy/rcl_action/QUALITY_DECLARATION.md).

### Direct Runtime non-ROS Dependency [5.iii]

`rclcpp_action` has no run-time or build-time dependencies that need to be considered for this declaration.

## Platform Support [6]

`rclcpp_action` supports all of the tier 1 platforms as described in [REP-2000](https://www.ros.org/reps/rep-2000.html#support-tiers), and tests each change against all of them.

Though there are no nightly jobs for foxy outside of linux, each change is tested on ci.ros2.org.
* [linux-aarch64](https://ci.ros2.org/job/ci_linux-aarch64)
* [linux](https://ci.ros2.org/job/ci_linux)
* [mac_osx](https://ci.ros2.org/job/ci_osx)
* [windows](https://ci.ros2.org/job/ci_windows)

## Security

### Vulnerability Disclosure Policy [7.i]

This package conforms to the Vulnerability Disclosure Policy in [REP-2006](https://www.ros.org/reps/rep-2006.html).
