# Architecture of the rclcpp::WaitSet Type

The `rclcpp::WaitSet` type is actually a specific configuration of a the more general `rclcpp::WaitSetTemplate` class.

The `rclcpp::WaitSetTemplate` class is design to have interchangeable implementations for controlling storage of items in the wait set, as well as to control access to the wait set functionality, i.e. whether or not it has thread-safety.

In general the idea behind a wait set in rclcpp is that you gather a set of entities that can be waited on, e.g. subscriptions, timers, guard conditions, and then wait for one or more of them to be ready.
These things are driven by asynchronous events, e.g. subscriptions become ready when new data arrives or timers become ready when their period has elapsed.

At the moment, things that can be waited on are limited by what can be put into an `rcl_wait_set_t`, which includes subscriptions, service clients, service servers, timers, guard conditions, and events.
In this context events are QoS events, as defined in the `rcl` and `rmw` interfaces.

## Design Goals

There are some design goals for the `rclcpp::WaitSet` family of classes:

- minimize inheritance and avoid virtual functions
  - Overhead of the executor and wait set design have been a common complaint and are often blamed for performance issues.
  - Since the wait set is at the core of the wait-do loop of the system, aggressive optimization can actually pay off.
- avoid unnecessary changes to the underlying `rcl_wait_set_t` instances or unnecessarily recreating it
  - This can be expensive, because under the hood that requires changes to the middleware's object, e.g. a DDS wait set or similar.
- minimize memory allocations in the wait() function, avoid them entirely if the set of things to wait on has not changed
- prevent accidentally using an entity with more than wait set at the same time

Several of these surround performance concerns, but others are concerned with safety and convenience for the user.

## Architecture

The `rclcpp::WaitSetTemplate` is the core fixture in the feature, which is using a policy-based design (see https://en.wikipedia.org/wiki/Modern_C%2B%2B_Design#Policy-based_design).

It delegates storage and synchronization to policy template arguments to the `rclcpp::WaitSetTemplate` via the `StoragePolicy` and `SynchronizationPolicy` template arguments respectively.

These policies can be mixed and matched to achieve different results.
There are a few predefined combinations, `rclcpp::WaitSet` which is a non-thread-safe wait set with dynamic storage, `rclcpp::StaticWaitSet` which is a non-thread-safe wait set with fixed storage, and `rclcpp::ThreadSafeWaitSet` which is a thread-safe wait set with dynamic storage.
They are all just aliases to `rclcpp::WaitSetTemplate` with difference combinations of policies.

Users can extend the behavior by implementing their own policies if desired.

This kind of design allows for specialization and code sharing without using standard polymorphism via virtual functions, and the benefit of that is that calls on the wait set class are always non-virtual, avoiding dispatching via the v-table.
It also forces us to use templates, which means a head-only implementation, which will increase compile times, but will avoid some calls to shared libraries which on some systems has a small overhead too.

These are aggressive optimizations, but it can actually be very beneficial in the inner-loop of applications, and the hope is that this approach will offer the best performance we can achieve.

### StoragePolicy

The storage policy is responsible for storing the entities in the wait set and providing accessors and setters (if appropriate).

The storage policy is also responsible for maintaining ownership of the entities it is storing at the right times.
For example, entities are always given to the wait set as `std::shared_ptr`'s and some policies might only maintain ownership for the lifetime of the wait set or until the entity is removed.
Other policies might only hold ownership of the entities while waiting on them, i.e. while they are actively using them, but maintain only weak ownership otherwise.

There are two built-in storage policies available, the `rclcpp::wait_set_policies::DynamicStorage` and the `rclcpp::wait_set_policies::StaticStorage<...>` classes.

The `StaticStorage` policy requires that you fully specify the number of entities of each time that can be stored as template arguments and uses the `std:array` class as backing storage.
Therefore, it also does not allow adding or removing entities after construction.
Because entities cannot be removed and are therefore "always" in use by the wait set, it maintains shared ownership of the entities for the lifetime of the wait set.

The `DynamicStorage` policy provides access to the entities and allows add and removing entities at any time after construction.
It also only maintains shared ownership of the entities while actively waiting on them, which allows the entities to go out of scope while the wait set is idle.
Entities that go out of scope in this manner will be removed from the wait set when it notices their weak pointers no longer lock to the entity's shared pointer.

Both policies allow you to initialize the wait set with entities in the constructor.

### SynchronizationPolicy

The synchronization policy is responsible for synchronizing access to wait set operations that share access to the StoragePolicy.

There are two built-in synchronization policies available, the `rclcpp::wait_set_policies::SequentialSynchronization` and the `rclcpp::wait_set_policies::ThreadSafeSynchronization` policies.

The `rclcpp::wait_set_policies::SequentialSynchronization` policy essentially does nothing but pass through requests from the user-facing `rclcpp::WaitSetTemplate` interface to the `StoragePolicy` that is being used.

The `rclcpp::wait_set_policies::ThreadSafeSynchronization` policy sequences access to the `StoragePolicy` being used so that it is safe to call methods like `add_subscription()` on the wait set while it is doing something else, like waiting with the `wait()` method.
This synchronization comes at the expense of using mutexes and extra logic to organize the access, but provides some thread-safety.

### Entity Intake and Management

Entities, i.e. `rclcpp::SubscriptionBase`, `rclcpp::Waitable`, `rclcpp::ClientBase`, `rclcpp::ServiceBase`, `rclcpp::TimerBase`, `rclcpp::GuardCondition`, etc., may be passed to the wait set during construction.
If the wait set has policies that allow for dynamic storage and supports adding and removing entities after construction, then entities can also be added and removed using method on the wait set.

Either way, the entities are initially given to the wait set as a `std::shared_ptr` in order to maintain shared ownership, but depending on the storage policy it may be stored as a `std::weak_ptr` while the wait set is not actively using the entities.

Also, when taking on new entities the wait set is responsible for ensuring that entity is not being used by another wait set (at least another instance of `rclcpp::WaitSetTemplate`).

One more thing that plays into intake of entities is that a few entity types have some extra requirements, specifically subscriptions and waitables.

Subscriptions (specifically `rclcpp::SubscriptionBase`) are complex objects that contain not only a subscription that can be waited on, but also may have `rclcpp::Waitable`'s inside in order to implement intra-process communication and for QoS events that are also implemented as `rclcpp::Waitable` instances.
Therefore, you need a way to say which parts of the subscription should be added to the wait set, because you don't need to add all the entities that make up a subscription to a single wait set.
The `rclcpp::SubscriptionWaitSetMask` class provides this mapping, and when adding subscriptions to a wait set you need to provide the subscription shared pointer as well as a mask instance.

Waitables are complicated by the fact that they often are part of a larger piece of the API, as we saw with subscriptions, and therefore the waitable may need to keep additional things in scope while being used.
So when adding a waitable to a wait set you may also provide an "associated entity" as a shared pointer to void, which is purely there to keep something in scope along with the waitable.

#### EntityEntry classes

In order to address the need to pair extra information with some entities (e.g. the subscription mask with the subscription entity) and to provide a convenient place to check if the entity is in use by another wait set and claim if not.

There are a series of classes that are a variation of the `EntityEntry` concept, which wrap the incoming entities and their associated data into a single class, and also provide some safety when converting between the shared ownership and weak ownership if needed.

They also provide RAII-style checking about whether or not an entity is already associated with a wait set.
This is important because when ingesting multiple entities at the same time, e.g. via the constructor, because if you claim the first few entities to be associated with the wait set, but then find one that is already associated, then you need to abort and throw an exception.
The RAII-style of these classes addresses this by "dissociating" the first few entities from the wait set when going out of scope due to the thrown exception.

The process from intake to cleanup of the entity entries follows something like this, using waitables as an example:



                                    ┌───────────────┐
   ┌────────────────────────┐       │ WaitSet       │
   │                        ├──────►│               ├───────────────┐
   │ shared_ptr<Waitable>   │       │ constructor   │               │
   │                        │       └───────────────┘  Implicit     │
   │  and optional          │                                       │
   │                        │       ┌───────────────┐  Conversion   │
   │ shared_ptr<void>       │       │ add_waitable()│               │
   │                        ├──────►│               ├───────────────┤
   └────────────────────────┘       │ method        │               │
                                    └───────────────┘               │
                                                                    │
                                                                    │
                                                                    │
          ┌──────────────────────┐        ┌─────────────────┐       │
          │                      │        │                 │       │
    ┌─────┤ ManagedWaitableEntry │◄──┬────┤  WaitableEntry  │◄──────┘
    │     │                      │   │    │                 │
    │     └─┬──────────────────┬─┘   │    └─┬─────────────┬─┘
    │       │ shared_ptr+RAII  │     │      │ shared_ptr  │
    │       └──────────────────┘     │      └─────────────┘
    │                                │
    │                        ┌───────┴───────────┐
    │                        │Check that entity  │
    │ Conversion to          │is not in use, then│
    │ weak ownership         │claim it for this  │
    │ if needed.             │wait set.          │
    │                        └───────────────────┘
    │
    │     ┌───────────────────────────┐
    │     │                           │
    └────►│ WeakManagedWaitableEntry  │
          │                           │
          └─┬───────────────────────┬─┘
            │ weak_ptr + RAII       │
            └───────────────────────┘

Using the diagram as a reference, you can see that the input from the user is converted into the entity entry class that, for now, just stores the entity and any associated data if it exists.
At this point the entity comes in with shared ownership and stays as shared ownership in the basic entry class.
Then this entry class is converted into a "managed" entry class, which will check that the entity is not associated with another wait set and associate it with the current wait set during construction.
This managed entry class will also automatically disassociate the entity with the current wait set during destruction.
The new managed entry class continues to keep shared ownership of the entity.

Optionally, if the storage policy requires it, the managed entry class can be converted into a "weak" version, where in the entity is stored as a weak pointer instead of a shared pointer.
In this case the entity will remain stored as a "weak managed" entry until it is removed or the wait set is destroyed.
In the meantime, if the wait set needs to take shared ownership again, it can do that be getting a copy of the weak pointer(s) in the entry and locking them to get shared pointers.
But the weak managed entry remains responsible for the RAII-style dissociation from the current wait set when destroyed.
Unlike the managed entry that maintains shared ownership, the weak version will need to attempt to lock the weak pointer for the entity before trying to dissociate it from the wait set, because this association is maintained within the entity itself, through methods like `rclcpp::Waitable::exchange_in_use_by_wait_set_state()`.
If the entity cannot be accessed via the weak pointer, then it is not explicitly dissociated by the managed entry, but it doesn't matter because the object has gone out of scope and has been deleted anyway.
So the cleanup for the weak managed entry is best effort, but that does not pose an issue.

#### The EntityEntryTemplate classes

There is a template class called `rclcpp::wait_set_policies::detail::EntityEntryTemplate<EntityT>` (and associated managed and weak-managed versions) which is the foundation for most of the entity types which don't require special handling, like `rclcpp::GuardCondition` and `rclcpp::TimerBase` for example.
Other entities like `rclcpp::SubscriptionBase` and `rclcpp::Waitable` have custom specializations of these classes to handle extra storage, custom constructors, and other extra logic to help them work.
