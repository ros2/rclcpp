======================================
:code:`MultiThreadedExecutor` Redesign
======================================

:code:`MultiThreadedExecutor` may attempt to execute one executable object multiple times.
This occurs because an executable object can be identified as ready in multiple threads before it has been executed, and each of these threads will then try to execute it.
To fix this, a general solution should do the following:

* Work for all the types of executable objects: timers, subscriptions, services, clients, and waitables
* Avoid spurious wake ups (which can be costly)

Summary of discussion so far
----------------------------

Broadly speaking, two approaches were discussed:

1. **Handle no data in the executor**: If more than one thread is trying to execute the same entity and that entity is consumed after it is executed once, check for the executable before trying to execute it:

    .. code-block:: c++

        void execute() {
          if(buffer->has_data()) {
            auto data = buffer->consume_data();
            run_any_callback(data);
          } else {
            // do nothing
          }
        }

   :Advantages:
     * simplicity
     * extends interface in a way that generalizes well for all other types of executable objects (i.e., timers)
   :Disadvantages:
     * spurious wake up calls
     * needs to be documented well so every class that executes :code:`AnyExecutable` objects is thread-safe and doesn't hide a race condition


2. **Take the data**: This approach takes the data when collecting the ready entities, and that process is done in a mutually exclusive fashion. Thus, this approach may avoid spurious wake ups.

    .. code-block:: c++

        // called first
        void take_data(std::shared_ptr<void> & data) {
          data = buffer->consume_data();
        }

        // called second
        void execute(std::shared_ptr<void> & data) {
          run_any_callback(data_);
        }

   :Advantages:
     * fixes spurious wake ups for events where :code:`take_data` has a meaning (i.e., not timers)
   :Disadvantages:
     * more complex
     * extends the interface in a way that doesn't generalize well for all other types of executable objects (i.e., timers)
     * type erasure of data

.. note::

   To avoid erasing the type of the data, :code:`take_data` could be replaced with a :code:`get_handle` method that would return a lambda function.

   .. code-block:: c++

      // called first
      std::function<void()> get_handle() {
        return [data=buffer->consume_data()]() {
          run_any_callback(data);
        };
      }

      // called second
      void execute(std::function<void()> handle) {
        handle();
      }

Ideas
-----------------------

* Add a preparation method to get the event after :code:`take_data`:

  .. code-block:: c++

     std::shared_ptr<void> & data_;

     // called first
     void take_data() {
       data_ = buffer->consume_data();
     }

     // called second
     std::shared_ptr<void> get_prepared() {
       return data_;
     }

     // called third, with the data returned in the previous step
     void execute(std::shared_ptr<void> & data) {
       run_any_callback(data);
     }


* Store conditions in the waitset (like DDS)
