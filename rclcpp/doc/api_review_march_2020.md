# API Review for `rclcpp` from March 2020

## Notes

### Off-Topic Questions

> [rclcpp_action] There exists a thread-safe and non-thread-safe way to get the goal result from an action client. We probably want to remove the public interface to the non-thread safe call (or consolidate somehow): https://github.com/ros2/rclcpp/issues/955

`rclcpp_action` is out of scope atm.

**Notes from 2020-03-19**: To be handled in separate API review.

## Architecture

### Calling Syntax and Keeping Node-like Class APIs in Sync

> Currently, much of the API is exposed via the `rclcpp::Node` class, and due to the nature of the current architecture there is a lot of repeated code to expose these methods and then call the implementations which are in other classes like `rclcpp::node_interfaces::NodeTopics`, for example.
>
> Also, we have other versions of the class `rclcpp::Node` with different semantics and interfaces, like `rclcpp_lifecycle::LifecycleNode`, and we have been having trouble keeping the interface provided there up to date with how things are done in `rclcpp::Node`. Since `LifecycleNode` has a different API from `Node` in some important cases, it does not just inherit from `Node`.
>
> There are two main proposals (as I see it) to try and address this issue, either (a) break up the functionality in `Node` so that it is in separate classes and make `Node` multiple inherit from those classes, and then `LifecycleNode` could selectively inherit from those as well, or (b) change our calling convention from `node->do_thing(...)` to be `do_thing(node, ...)`.
>
> For (a) which commonly referred to as the [Policy Based Design Pattern](https://en.wikipedia.org/wiki/Modern_C%2B%2B_Design#Policy-based_design), we'd be reversing previous design decisions which we discussed at length where we decided to use composition over inheritance for various reasons.
> One of the reasons was testing, with the theory that having simpler separate interfaces we could more easily mock them as needed for testing.
> The testing goal would still be met, either by keeping the "node_interface" classes as-is or by mocking the classes that node would multiple inherit from, however it's harder to indicate that a function needs a class that multiple inherits from several classes but not others.
> Also having interdependency between the classes which are inherited from is a bit complicated in this design pattern.
>
> For (b), we would be changing how we recommend all code be written (not a trivial thing to do at all), because example code like `auto pub = node->create_publsiher(...);` would be come `auto pub = create_publisher(node, ...);`.
> This has some distinct advantages, however, in that it allows us to write these functions, like `create_publisher(node, ...)`, so that the node argument can be any class that meets the criteria of the function.
> That not only means that when we add a feature it automatically works with `Node` and `LifecycleNode` without adding anything to them, it also means that user defined `Node`-like classes will also work, even if they do not inherit from or provide the complete interface for `rclcpp::Node`.
> Another major downside of this approach is discoverability of the API when using auto-completion in text editors, as `node-><tab>` will often give you a list of methods to explore, but with the new calling convention, there's not way to get an auto complete for code who's first argument is a `Node`-like class.
>
> Both of the above approaches address some of the main concerns, which are: keeping `Node` and `LifecycleNode` in sync, reducing the size of the `Node` class so it is more easily maintained, documented, and so that related functions are grouped more clearly.

- https://github.com/ros2/rclcpp/issues/898
- https://github.com/ros2/rclcpp/issues/509
- https://github.com/ros2/rclcpp/issues/855
- https://github.com/ros2/rclcpp/issues/985
  - subnode feature is in rclcpp::Node only, complicating "node using" API designs
- http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2014/n4174.pdf
- https://en.wikipedia.org/wiki/Uniform_Function_Call_Syntax#C++_proposal
  - "Many programmers are tempted to write member functions to get the benefits of the member function syntax (e.g. "dot-autocomplete" to list member functions);[6] however, this leads to excessive coupling between classes.[7]"

**Suggested Action**: Document the discussion and defer until Foxy.

**Notes from 2020-03-19**:

- Another version of (b) could be to have classes that are constructed with node, e.g. `Publisher(node, ...)` rather than `node->create_publisher(...)`
- (tfoote) interface class? `NodeInterface<NodeLike>::something(node_like)`
  - DRY?
  - `NodeInterface<LifecycleNode>::<tab>` -> only life cycle node methods
- (karsten) use interface classes directly, e.g. node->get_X_interface()->do_thing()
- (dirk) use macros to add methods to class
  - Question: Do we want tab-completable API (specifically list of member functions)?
- Question: Is consistency in calling between core and non-core features more important than tab-completion?
- Add better example of adding new feature and not needing to touch `rclcpp::Node`.
- (dirk) methods and free functions not mutually exclusive.

### Scoped Versus Non-Scoped Entities (e.g. Publishers/Subscriptions)

> Currently, Publisher and Subscription (and similar entities) are scoped, meaning that when they are created they are added to the ROS graph as a side effect, and when they are let out of scope they remove themselves from the graph too.
> Additionally, they necessarily have shared state with the system, for instance when you are spinning on a node, the executor shares ownership of the Subscriptions with the user.
> Therefore, the Subscription only gets removed when both the user and executor are done with it.
>
> This shared ownership is accomplished with the use of shared pointers and weak pointers.
>
> There are a few concerns here, (a) use of shared pointers confuses users, (b) overhead of shared pointers and lack of an ability to use these classes on the stack rather than the heap, and (c) complexity of shutdown of an entity from the users perspective.
>
> For (a), some users are overwhelmed by the need to use a shared pointer.
> In ROS 1 this was avoided by having a class which itself just thinly wraps a shared pointer (see: https://github.com/ros/ros_comm/blob/ac9f88c59a676ca6895e13445fc7d71f398ebe1f/clients/roscpp/include/ros/subscriber.h#L108-L111).
> This could be achieved in ROS 2 either by doing the same with a wrapper class (at the expense of lots of repeated code), or by eliminating the need for using shared ownership.
>
> For (b), for some use cases, especially resource constrained / real-time / safety-critical environments, requiring these classes to be on the heap rather than the stack is at least inconvenient.
> Additionally, there is a cost associated with using shared pointers, in the storage of shared state and in some implementation the use of locks or at least atomics for thread-safety.
>
> For (c), this is the most concerning drawback, because right now when a user lets their shared pointer to a, for example, Subscription go out of scope, a post condition is not that the Subscription is destroyed, nor that it has been removed from the graph.
> In stead, the behavior is more like "at some point in the future the Subscription will be destroyed and removed from the graph, when the system is done with it".
> This isn't a very satisfactory contract, as some users may wish to know when the Subscription has been deleted, but cannot easily know that.
>
> The benefit to the shared state is a safety net for users.
> The alternative would be to document that a Subscription, again for example, cannot be deleted until the system is done with it.
> We'd basically be pushing the responsibility onto the user to ensure the shared ownership is handled properly by the execution of their application, i.e. they create the Subscription, share a reference with the system (adding it by reference to an executor, for example), and they have to make sure the system is done with it before deleting the Subscription.
>
>Separately, from the above points, there is the related concern of forcing the user to keep a copy of their entities in scope, whether it be with a shared pointer or a class wrapping one.
> There is the desire to create it and forget it in some cases.
> The downside to this is that if/when the user wants to destroy the entity, they have no way of doing that as they have no handle or unique way to address the entity.
>
> One proposed solution would be to have a set of "named X" APIs, e.g. `create_named_subscription` rather than just `create_subscription`.
> This would allow the user to address the Subscription in the future in order to obtain a new reference to it or delete it.

- https://github.com/ros2/rclcpp/issues/506
- https://github.com/ros2/rclcpp/issues/726

**Suggested Action**: Consolidate to a single issue, and defer.

**Notes from 2020-03-23**:

- (chris) Putting ownership mechanics on user is hard.
- (dirk) add documentation clearly outlining ownership
- (shane) warn on unused to catch issues with immediately deleted items
- (tfoote) debugging output for destruction so it easy to see when reviewing logs
- (chris) possible to create API that checks for destruction
  - (william) might lead to complex synchronization issues
- (tfoote) could add helper classes to make scoped things non-scoped
  - (shane) concerned that there is no longer "one good way" to do it

### Allow QoS to be configured externally, like we allow remapping of topic names

> Suggestion from @stonier: allow the qos setting on a topic to be changed externally at startup, similar to how we do topic remapping (e.g., do it on the command-line using appropriate syntax).
>
> To keep the syntax manageable, we might just allow profiles to be picked.

- https://github.com/ros2/rclcpp/issues/239

**Suggested Action**: Update issue, defer for now.

**Notes from 2020-03-19**:

- (wjwwood) it depends on the QoS setting, but many don't make sense, mostly because they can change some of the behaviors of underlying API
- (dirk) Should developers expose a parameter instead?
- (multiple) should be a feature that makes configuring them (after opt-in) consistent
- (jacob) customers feedback was that this was expected, surprised it was not allowed
- (karsten) could limit to profiles

## Init/shutdown and Context

### Consider renaming `rclcpp::ok()`

> Old discussion to rename `rclcpp::ok()` to something more specific, like `rclcpp::is_not_shutdown()` or the corollary `rclcpp::is_shutdown()`.

- https://github.com/ros2/rclcpp/issues/3

**Suggested Action**: Defer.

**Notes from 2020-03-19**:

- (shane) preference to not have a negative in the function name

## Executor

### Exposing Scheduling of Tasks in Executor and a Better Default

> Currently there is a hard coded procedure for handling ready tasks in the executor, first timers, then subscriptions, and so on.
> This scheduling is not fair and results in non-deterministic behavior and starvation issues.
>
> We should provide a better default scheduling which is fairer and ideally deterministic, something like round-robin or FIFO.
>
> Additionally, we should make it easier to let the user override the scheduling logic in the executor.

- https://github.com/ros2/rclcpp/pull/614
- https://github.com/ros2/rclcpp/issues/633
- https://github.com/ros2/rclcpp/issues/392

**Suggested Action**: Follow up on proposals to implement FIFO scheduling and refactor the Executor design to more easily expose the scheduling logic.

**Notes from 2020-03-19**:

- No comments.

### Make it possible to wait on entities (e.g. Subscriptions) without an Executor

> Currently, it is only possible to use things like Timers and Subscriptions and Services with an executor.
> It should be possible, however, to either poll these entities or wait on them and then decide which to process as a user.
>
> This is most easily accomplished with a WaitSet-like class.

- https://github.com/ros2/rclcpp/issues/520

**Suggested Action**: implement WaitSet class in rclcpp so that this is possible, and make "waitable" entities such that they can be polled, e.g. `Subscription`s should have a user facing `take()` method, which can fail if no data is available.

**Notes from 2020-03-19**:

- No comments.

### Make it possible to use multiple executors per node

> Currently, you cannot use more than one executor per node, this limits your options when it comes to distributing work within a node across threads.
> You can use a multi-threaded executor, or make your own executor which does this, but it is often convenient to be able to spin part of the node separately from the the rest of the node.

- https://github.com/ros2/rclcpp/issues/519

**Suggested Action**: Make this possible, moving the exclusivity to be between an executor and callback groups rather than nodes.

**Notes from 2020-03-19**:

- No comments.

### Implement a Lock-free Executor

> This would presumably be useful for real-time and safety critical systems where locks and any kind of blocking code is considered undesirable.

- https://github.com/ros2/rclcpp/issues/77

**Suggested Action**: Keep in backlog until someone needs it specifically.

**Notes from 2020-03-19**:

- No comments.

### Add implementation of `spin_some()` to the `MultiThreadedExecutor`

> Currently `spin_some()` is only available in the `SingleThreadedExecutor`.

- https://github.com/ros2/rclcpp/issues/85

**Suggested Action**: Defer.

**Notes from 2020-03-19**:

- No comments.

## Node

### Do argument parsing outside of node constructor

> Things that come from command line arguments should be separately passed into the node's constructor rather than passing in arguments and asking the node to do the parsing.

- https://github.com/ros2/rclcpp/issues/492

**Suggested Action**: Defer until after foxy.

**Notes from 2020-03-23**:

- (dirk) may be related to ROS 1 heritage of argc/argv being passed to node directly
- (shane) impacts rcl API as well, two parts "global options" as well node specific options
- (dirk) what is the recommendation to users that want to add arguments programmatically
  - user should be able to get non-ros argc/argv somehow (seems like you can now)
- (jacob) the argument in NodeOptions are used for application specific argument via component loading as well

## Timer

### Timer based on ROS Time

> `node->create_wall_timer` does exactly what it says; creates a timer that will call the callback when the wall time expires.  But this is almost never what the user wants, since this won’t work properly in simulation.  Suggestion: deprecate `create_wall_timer`, add a new method called `create_timer` that takes the timer to use as one of the arguments, which defaults to ROS_TIME.

- https://github.com/ros2/rclcpp/blob/96ebf59a6045a535730d98fff25e522807c7aa75/rclcpp/include/rclcpp/node.hpp#L219-L230
- https://github.com/ros2/rclcpp/issues/465

**Suggested Action**: Promote `rclcpp::create_timer()` which is templated on a clock type, as suggested, but leave `create_wall_timer` as a convenience.

**Notes from 2020-03-19**:

- (shane) may be a `rclcpp::create_timer()` that can be used to create a non-wall timer already

## Publisher

## Subscription

### Callback Signature

> Is there a reason the subscription callback must have a smart pointer argument instead of accepting a const-reference argument?

- https://github.com/ros2/rclcpp/blob/96ebf59a6045a535730d98fff25e522807c7aa75/rclcpp/include/rclcpp/any_subscription_callback.hpp#L44-L52
- https://github.com/ros2/rclcpp/issues/281

**Suggested Action**: Provide const reference as an option, add documentation as to the implications of one callback signature versus others.

**Notes from 2020-03-19**:

- (dirk) have const reference and document it

## Service Server

### Allow for asynchronous Service Server callbacks

> Currently, the only callback signature for Service Servers takes a request and must return a response.
> This means that all of the activity of the service server has to happen within that function.
> This can cause issues, specifically if you want to call another service within the current service server's callback, as it causes deadlock issues trying to synchronously call the second service within a spin callback.
> More generally, it seems likely that long running service server callbacks may be necessary in the future and requiring them to be synchronous would tie up at least on thread in the spinning executor unnecessarily.

- https://github.com/ros2/rclcpp/issues/491

**Suggested Action**: Defer.

**Notes from 2020-03-23**:

- (dirk) likely new API, so possible to backport

## Service Client

### Callback has SharedFuture rather than const reference to response

> Why does the Client::send_async_request take in a callback that has a SharedFuture argument instead of an argument that is simply a const-reference (or smart pointer) to the service response type? The current API seems to imply that the callback ought to check whether the promise is broken or fulfilled before trying to access it. Is that the case? If so, it should be documented in the header.

- https://github.com/ros2/rclcpp/blob/7c1721a0b390be8242a6b824489d0bc861f6a0ad/rclcpp/include/rclcpp/client.hpp#L134

**Suggested Action**: Update ticket and defer.

**Notes from 2020-03-19**:

- (wjwwood) we wanted the user to handle error cases with the future?
- (dirk) future allows for single callback (rather than one for response and one for error)
- (jacob) actions uses a "wrapped result" object

### rclcpp missing synchronous `send_request` and issues with deadlocks

> This has been reported by several users, but there is only an `async_send_request` currently. `rclpy` has a synchronous `send_request` but  it has issues with deadlock, specifically if you call it without spinning in another thread then it will deadlock. Or if you call it from within a spin callback when using a single threaded executor, it will deadlock.

- https://discourse.ros.org/t/synchronous-request-to-service-in-callback-results-in-deadlock/12767
- https://github.com/ros2/rclcpp/issues/975
- https://github.com/ros2/demos/blob/948b4f4869298f39cfe99d3ae517ad60a72a8909/demo_nodes_cpp/src/services/add_two_ints_client.cpp#L24-L39

**Suggested Action**: Update issue and defer. Also defer decision on reconciling rclpy's send_request.

**Notes from 2020-03-23**:

- (karsten/shane) async spinner helps in rclpy version, rclcpp could emulate
- (chris) sees three options:
  - only async (current case in rclcpp)
  - have sync version, add lots of docs that spinning needs to happen elsewhere (current case for rclpy)
  - reentrant spinning
- (william) you either need async/await from language or ".then" syntax (we have this in async_send_request())
- (chris) more error checking for recursive spinning
- (chris) weird that rclcpp and rclpy have different API
- (shane) thinks it is ok to have different API, but rclpy is not ideal

## Parameters

### Expected vs Unexpected parameters

> Allow node author to define expected parameters and what happens when an unexpected parameter is set.

- https://github.com/ros2/rclcpp/issues/475
- https://github.com/ros2/rclcpp/tree/check_parameters

**Suggested Action**: Defer as nice to have.

**Notes from 2020-03-23**:

- None.

### Implicitly cast integer values for double parameters

> If we try to pass an integer value to a double parameter from the command line or from a parameters YAML file we get a `rclcpp::ParameterTypeException`.
> For example, passing a parameter from the command line:
>
>     ros2 run foo_package foo_node --ros-args -p foo_arg:=1
>
> results in the following error:
>
>     terminate called after throwing an instance of 'rclcpp::ParameterTypeException'
>       what():  expected [double] got [integer]
>
> and we can fix it by explicitly making our value a floating point number:
>
>     ros2 run foo_package foo_node --ros-args -p foo_arg:=1.0
>
> But, it seems reasonable to me that if a user forgets to explicitly provide a floating point value that we should implicitly cast an integer to a float (as is done in many programming languages).

- https://github.com/ros2/rclcpp/issues/979

**Suggested Action**: Continue with issue.

**Notes from 2020-03-23**:

- (shane) says "yes please" :)

### Use `std::variant` instead of custom `ParameterValue` class

> This is only possible if C++17 is available, but it would simplify our code, make our interface more standard, and allow us to use constexpr-if to simply our templated code.

**Suggested Action**: Create an issue for future work.

**Notes from 2020-03-23**:

- (chris) not sure churn is worth
- (ivan) other places for std::variant, like AnySubscriptionCallback

### Cannot set name or value on `Parameter`/`ParameterValue`

> Both `Parameter` and `ParameterValue` are read-only after construction.

- https://github.com/ros2/rclcpp/issues/238

**Suggested Action**: Update issue, possibly close.

**Notes from 2020-03-23**:

- (chris/william) setting values on temporary (local) objects is not reflected in the node, so misleading

## Parameter Clients

### No timeout option with synchronous parameter client calls

> As an example, SyncParametersClient::set_parameters doesn't take a timeout option. So, if anything goes wrong in the service call (e.g. the server goes down), we will get stuck waiting indefinitely.

- https://github.com/ros2/rclcpp/issues/360
- https://github.com/ros2/rclcpp/blob/96ebf59a6045a535730d98fff25e522807c7aa75/rclcpp/src/rclcpp/parameter_client.cpp#L453-L468

**Suggested Action**: Update issue, decide if it can be taken for Foxy or not.

**Notes from 2020-03-23**:

- (tfoote) Seems like adding a timeout is a good idea.

### Name of AsyncParametersClient inconsistent

> AsyncParameter**s**Client uses plural, when filename is singular (and ParameterService is singular):

- https://github.com/ros2/rclcpp/blob/7c1721a0b390be8242a6b824489d0bc861f6a0ad/rclcpp/include/rclcpp/parameter_client.hpp#L44

**Suggested Action**: Reconcile class and file name, switch to singular name?

**Notes from on-line, post 2020-03-23 meeting**:

- (tfoote) +1 for homogenizing to singular

### `SyncParametersClient::get_parameters` doesn't allow you to detect error cases

> E.g. https://github.com/ros2/rclcpp/blob/249b7d80d8f677edcda05052f598de84f4c2181c/rclcpp/src/rclcpp/parameter_client.cpp#L246-L257 returns an empty vector if something goes wrong which is also a valid response.

- https://github.com/ros2/rclcpp/issues/200
- https://github.com/ros2/rclcpp/blob/96ebf59a6045a535730d98fff25e522807c7aa75/rclcpp/src/rclcpp/parameter_client.cpp#L412-L426

**Suggested Action**: Throw an exception to indicate if something went wrong and document other expected conditions of the API.

**Notes from on-line, post 2020-03-23 meeting**:

- (tfoote) An empty list is not a valid response unless you passed in an empty list. The return should have the same length as the request in the same order. Any parameters that are not set should return a ParameterVariant with type PARAMETER_NOT_SET. to indicate that it was polled and determined to not be set. Suggested action improve documentation of the API to clarify a short or incomplete.
- (jacobperron) I think throwing an exception is also a valid action, making it clear that an error occurred.
- (wjwwood) Using exceptions to indicate an exceptional case (something went wrong) seems reasonable to me too.

## Clock

### Clock Jump callbacks on System or Steady time?

> Currently time jump callbacks are registered via Clock::create_jump_handler(). Jump handlers are only invoked by TimeSource::set_clock(). This is only called if the clock type is RCL_ROS_TIME and ROS time is active.

- https://github.com/ros2/rclcpp/issues/528

**Suggested Action**: Document that time jumping is only detected with ROS time, consider a warning.

**Notes from on-line, post 2020-03-23 meeting**:

- (tfoote) There should be no jumps in steady time. If there's a big change in system time, it doesn't necessarily mean that time jumped, just that you might have been sleeping for a long time. Most ntp systems adjust the slew rate these days instead of jumping but still that's an external process and I don't know of any APIs to introspect the state of the clock. I'm not sure that we have a way to detect jumps in time for system or steady time. To that end I think that we should be clear that we only provide callbacks when simulation time starts or stops, or simulation time jumps. We should also strongly recommend that operators not actively adjust their system clocks while running ROS nodes.
- (jacobperron) I agree with Tully, if we don't have a way to detect system time jumps then I think we should just document that this only works with ROS time. In addition to documentation, we could log an info or warning message if the user registers jump callback with steady or system time, but it may be unnecessarily noisy.

