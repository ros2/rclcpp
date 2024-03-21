Architecture of ``rclcpp_components``
=====================================

``rclcpp_components`` provides tools and defines conventions for building ``rclcpp`` nodes into dynamically loadable components.
Building composable nodes makes it easier to compose them with other composable nodes at runtime.
For example, it allows you to build your nodes one time but then at runtime decide whether or not to run them in their own process or combine them into a single process.

It also provides some generic tools for working with composable nodes, including a composable node container, which can load and execute multiple nodes in a variety of scenarios.

Terminology
-----------

- Composable Node:
  - An ``rclcpp`` based node that is setup so that it can be composed into a single process with other composable nodes at either runtime or programming time.
- Node Component:
  - Same thing as Composable Node, can be used interchangeably.
- Composable Node Container:
  - A program that can load and host composable nodes, it is often general purpose and has not a priori knowledge about the nodes it is instantiating and executing.
- pluginlib:
  - ROS package that provides mechanisms to create and register plugins based on user defined interface classes, and then dynamically lookup and load plugins at runtime.
- Stand-alone Node Executable:
  - Convenience executable that can be used to run a composable node as a single process or have it loaded dynamically in a remote composable node container.

Theory of Operation
-------------------

The lowest level series of steps require to make a composable node are as follows:

- Create a shared library in which one or more of your composable nodes are compiled.
  - The nodes should also be registered as a ``pluginlib``-style plugin.
  - The library should be created with CMake's ``add_library``
  - The library should either explicitly be set to ``SHARED``.
- Create or add to a ``plugindescription.xml`` file and register it so that ``pluginlib`` can find the node.
- Create a stand-alone executable for each node.
  - Which can either be used to run the node in its own process or to load it into an existing process and serving as a proxy while it is in the remote process.

There are both C++ preprocessor and CMake macros within this package to assist with each of the above steps.
Generally they can be applied in the following way:

- Write files containing nodes, e.g. ``src/a.cpp``, ``src/b.cpp``, etc., and in each use the ``RCLCPP_COMPONENTS_REGISTER_NODE()`` preprocessor macro in the source code.
- In CMake, create the shared library, e.g. ``add_library(my_library SHARED src/a.cpp src/b.cpp)``.
  - Do any extra CMake steps for the library, e.g. ``target_link_libraries`` or ``target_include_directories``.
- Describe each node in the library with the CMake using ``rclcpp_components_describe_node_component``.
  - e.g. ``rclcpp_components_describe_node_component(my_library NAME MyNodeA TYPE my_cpp_namespace::MyNodeA DESCRIPTION "...")``
- Register the library with ``pluginlib`` by calling ``rclcpp_components_register_node_component_library``.
  - e.g. ``rclcpp_components_register_node_component_library(my_library GENERATE_PLUGIN_DESCRIPTION)``
- Optionally, create a stand-alone executable for each node using ``rclcpp_components_create_executable_for_node``.
  - e.g. ``rclcpp_components_create_executable_for_node(my_node_a_target NODE_COMPONENT_NAME MyNodeA)``
  - An assumption is that package name is equal to ``PROJECT_NAME``, but can be overridden with the ``PACKAGE_NAME <package name>`` option.

Using the ``GENERATE_PLUGIN_DESCRIPTION`` option with the ``rclcpp_components_register_node_component_library()`` macro will cause this function to make a plugin description XML file for you using the information from the ``rclcpp_components_describe_node_component`` calls.
However, you can avoid this macro and the ``rclcpp_components_describe_node_component()`` macro calls by creating your own plugin description XML file and passing it with the ``PLUGIN_DESCRIPTION_FILE <file path>`` option.

So, the required steps are registering your nodes in the source code, creating a shared library, and registering with ``pluginlib``.
However, you have some choice in how the ``pluginlib`` description file is created and how registration occurs, plus creating the node executables is completely optional.

There are more options available to you, and if you do not wish to use the convenience macros in CMake you can see in the details that follow how each are implemented and do any of the steps manually yourself.

Writing the Code for a Node Component
-------------------------------------

Consider this trivial example:

.. code-block:: cpp

    #include <rclcpp/rclcpp.hpp>

    class MyNode : public rclcpp::Node
    {
    public:
      MyNode(rclcpp::NodeOptions options) : rclcpp::Node("my_node", options) {}
    };

    #include <rclcpp_components/register_node_macro.hpp>

    RCLCPP_COMPONENTS_REGISTER_NODE(MyNode)

The contract for the class passed to ``RCLCPP_COMPONENTS_REGISTER_NODE`` is that it:

- has a constructor which takes a single argument which is an instance of ``rclcpp::NodeOptions``
- has a method of the signature ``rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()``

Note that it does not *need* to inherit from ``rclcpp::Node``, but that is the easiest way to do it and to get the ``get_node_base_interface()`` method for "free".

The RCLCPP_COMPONENTS_REGISTER_NODE Macro
+++++++++++++++++++++++++++++++++++++++++

One possible (psuedo-code) implementation for the ``RCLCPP_COMPONENTS_REGISTER_NODE`` macro might be:

.. code-block:: cpp

    #define RCLCPP_COMPONENTS_REGISTER_NODE(NodeClass) \
      CLASS_LOADER_REGISTER_CLASS(NodeFactoryTemplate<NodeClass>, rclcpp_components::NodeFactory)

Where the base class ``rclcpp_components::NodeFactory`` is the actual plugin interface and the ``rclcpp_components::NodeFactoryTemplate`` class is a templated "adapter" class which will provide the ``rclcpp_components::NodeFactory`` interface given a sub-class of ``rclcpp::Node``.

The NodeFactory and Related Interfaces
++++++++++++++++++++++++++++++++++++++

The ``rclcpp_components::NodeFactory`` interface is dual-purposed, first it works around a limitation of instantiating classes via a general purpose class loading interface which is that you cannot call constructors on derived classes because you only know the base class and are unaware of any different signatures for constructors in the derived class.
Having the factory class allows us to instantiate a trivially constructible C++ class from the plugin and then call a method on that factory class instance which can create an instance of the actual node, passing any kind of arguments we would like.

The second purpose of the interface is to allow for composable nodes which are not based on the ``rclcpp_components::Node`` class, but instead a derivative based on composition of the "node interfaces" rather than inheritance of the ``rclcpp::Node`` class.
An example of this is the ``rclcpp_lifecycle::LifecycleNode`` class.

The interface of the ``rclcpp_components::NodeFactory`` class is something like this:

.. code-block:: cpp

    class NodeFactory
    {
    public:
      NodeFactory() = default;
      virtual ~NodeFactory() = default;

      virtual
      NodeInstanceWrapper
      create_node_instance(rclcpp::NodeOptions options) = 0;
    };

The ``NodeInstanceWrapper`` type would just encapsulate the instance of the node as a type-erased shared pointer, and a method for accessing the ``rclcpp::node_interfaces::NodeBaseInterface`` pointer from that node instance.
It might look like this:

.. code-block:: cpp

    class NodeInstanceWrapper
    {
    public:
      using NodeBaseInterfaceGetter = std::function<
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr (const std::shared_ptr<void> &)
      >;

      NodeInstanceWrapper(
        std::shared_ptr<void> node_instance,
        NodeBaseInterfaceGetter node_base_interface_getter)
      : node_instance_(node_instance), node_base_interface_getter_(node_base_interface_getter)
      {}

      const std::shared_ptr<void>
      get_node_instance() const
      {
        return node_instance_;
      }

      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
      get_node_base_interface()
      {
        return node_base_interface_getter_(node_instance_);
      }

    private:
      std::shared_ptr<void> node_instance_;
      NodeBaseInterfaceGetter node_base_interface_getter_;
    };

This interface only requires that you can give access to the ``rclcpp::node_interfaces::NodeBaseInterface`` pointer, as that's the only thing required to compose (and then execute) composable nodes.
The ability to get access to the original Node instance shared pointer is just for debugging and special cases.
Most of the time the program loading the instance of the node will not be aware of the original type of the node instance, whether it be a ``rclcpp::Node`` or something like ``my_ns::MyNode`` which inherits from ``rclcpp::Node`` or even something else completely.

Finally, the ``rclcpp_components::NodeFactoryTemplate<NodeT>`` convenience class will create an implementation of ``NodeFactory`` based on the ``NodeT`` type.
This template class will work so long as the class given for ``NodeT`` provides the ``get_node_base_interface()`` method that exists on the ``rclcpp::Node`` class.

Building the Shared Library which Contains Composable Nodes
-----------------------------------------------------------

Typically this is done by just calling the standard ``add_library`` macro in CMake, but any thing that results in generating a CMake target which represents a shared library is sufficient.

Once you've created the target you may manipulate the target with standard CMake macros like ``target_include_directories`` and ``target_link_libraries``.

Creating the Plugin Description File
------------------------------------

The plugin description file is specific to ``pluginlib`` and is used by ``pluginlib`` to associate shared libraries with plugins that exist within them, as well to capture additional meta about each plugin if desired.

The description file is a hierarchy of information, roughly structured like this:

- The library name (which implies location)
  - Zero to many classes (name, C++ type name, C++ base class name)
    - unstructured XML meta data, e.g. description, supported features, etc.

If you use the ``rclcpp_components_describe_node_component()`` macro in conjunction with the ``rclcpp_components_register_node_component_library()`` and its ``GENERATE_PLUGIN_DESCRIPTION`` option, the plugin description XML file will be generated for you.
The generated plugin description file will be named based on the target name in CMake for the library, i.e. ``<target_name>_plugindescriptions.xml`` and will be installed automatically.

However, you may manually create the plugin description file yourself if you wish.

The library name will be based on the CMake target for the shared library, ultimately using some part of the final file name for the shared library.

The class name will be an arbitrary name that you get to pick.
The class name may contain any characters and can have a made up structure if you would like.
For example, ``rviz`` prefixes all of its plugins with ``rviz/``, but that's not required.
It just needs to be unique among other composable node plugins, so the node's name is probably the most appropriate value, e.g. ``talker``, ``image_view``, ``rviz2``, etc.

The C++ type name is the C++ symbol that represents the class you're using as the plugin.
Getting this value right is harder, because it requires that you provide the name of the *factory* class and not the Node class.
If you're using the ``RCLCPP_COMPONENTS_REGISTER_NODE()`` macro the first argument to ``CLASS_LOADER_REGISTER_CLASS()`` will end up being ``NodeFactoryTemplate<NodeClass>`` where ``NodeClass`` is what ever you passed to ``RCLCPP_COMPONENTS_REGISTER_NODE()``.
For example, if you did ``RCLCPP_COMPONENTS_REGISTER_NODE(MyComposableNode)`` then you'd want to use ``NodeFactoryTemplate<MyComposableNode>`` as the type name.
If you use the ``rclcpp_components_describe_node_component()`` macro and the ``rclcpp_components_register_node_component_library()`` macro with the ``GENERATE_PLUGIN_DESCRIPTION`` option, this will be taken care of for you and you should instead provide the type as the same thing you gave to the ``RCLCPP_COMPONENTS_REGISTER_NODE()``, e.g. ``MyComposableNode``.

The C++ base type name will always be ``rclcpp_components::NodeFactory``.

Registering with ``pluginlib``
------------------------------

To register with ``pluginlib`` you need to provide the "plugin category" and the plugin description file.
The plugin category is just the name which will be used when looking for plugins of a certain at runtime.
By convention and specifically for composable nodes based on ``rclcpp``, it should be ``rclcpp_components``.

This is done with a CMake macro from ``pluginlib`` itself called ``pluginlib_export_plugin_description_file()``, see:

https://github.com/ros/pluginlib/blob/c0bbfaf8b22f3800bc3bd20414b9b15ea0aa52de/pluginlib/cmake/pluginlib_export_plugin_description_file.cmake#L22-L81

The ``rclcpp_components_register_node_component_library()`` will do this step for you.

Creating a Stand-alone Node Executable
--------------------------------------

This package also provides some "boilerplate" source code that can be compiled into a main executable for each of your nodes, if you desire that.
This generic main function is templated to take the salient information on how to locate your node (name from the plugin description, and your package which provides that composable node), and it uses that information at runtime to either load and instantiate your node by itself in the executable or contact an already running composable node container process and have that process load and execute your node.

Composable Node Container
-------------------------

This package provides one or more "composable node containers", which exist to assist with loading and executing multiple composable nodes in a single process without the need to write your own main function to do this.

The containers could receive the information about which nodes to load and execute in various ways, but only two are discussed here: a configuration file or via a ROS interface.

In both cases, the information related to loading and executing a node consists the following information (some required and some optional):

- package name
  - name of the package in which the node is located
- node plugin name
  - the name for the plugin that was given in the ``rclcpp_components_describe_node_component()`` or the plugin description XML
- node name
  - overrides the node name hard coded into source code (i.e. like ``__node:=<new name>`` on the command line)
- node namespace
  - overrides the node namespace hard coded into the source code (i.e. like ``__ns:=<new ns>`` on the command line)
- remappings
  - remappings that apply to this node for things like topics and services
- initial parameter values
  - initial values for required parameters or to be used instead of the defaults for optional parameters

Essentially, where to find the node (package name plus node name) and anything that can be influenced with command line arguments to nodes normally.

Compose Nodes with a Configuration File
+++++++++++++++++++++++++++++++++++++++

The composable node container can be given a configuration file to load nodes on startup.
This file is YAML and contains a structure with a list of dictionaries where the key/values of the dictionary are the information described above.
For package name, node name, and node namespace the values are just strings.
The remappings and initial parameter values are themselves dictionaries.

Compose Nodes with the ROS Interface
++++++++++++++++++++++++++++++++++++

The composable node container is itself a node which provides a ROS service to allow runtime composition of nodes.
The service is defined as ``rcl_interfaces/srv/LoadComposableNode`` which has a similar structure to a single entry in the configuration file described above.

Composition at Compile Time
---------------------------

Right now this package does not provide a way to dynamically compose nodes at compile time (without using ``pluginlib`` or ``dlopen`` like features).
If you need to do this, then you should create a header for each node you'd like to compose and then create a main file which explicitly includes the headers for each node to be composed, link to their libraries, and then manually instantiate and pass the appropriate arguments in the main executable.
This can be done without any prior coordination or tools provided by this package, but it does require the nodes to have a header file (and therefore on Windows to have dll visibility done correctly) whereas the dynamic approach allows nodes to be completely defined in a source file, avoiding the need for a class header.
