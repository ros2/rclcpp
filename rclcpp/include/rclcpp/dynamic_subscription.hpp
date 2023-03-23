// Copyright 2022 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RCLCPP__DYNAMIC_SUBSCRIPTION_HPP_
#define RCLCPP__DYNAMIC_SUBSCRIPTION_HPP_

#include <functional>
#include <memory>
#include <string>

#include "rcpputils/shared_library.hpp"

#include "rclcpp/callback_group.hpp"
#include "rclcpp/dynamic_typesupport/dynamic_message_type_support.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/typesupport_helpers.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/// %Subscription for messages whose type descriptions are obtained at runtime.
/**
 * Since the type is not known at compile time, this is not a template, and the dynamic library
 * containing type support information has to be identified and loaded based on the type name.
 *
 * NOTE(methylDragon): No considerations for intra-process handling are made.
 */
class DynamicSubscription : public rclcpp::SubscriptionBase
{
public:
  // cppcheck-suppress unknownMacro
  RCLCPP_SMART_PTR_DEFINITIONS(DynamicSubscription)

  template<typename AllocatorT = std::allocator<void>>
  DynamicSubscription(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    rclcpp::dynamic_typesupport::DynamicMessageTypeSupport::SharedPtr type_support,
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    std::function<void(rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr)> callback,
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options,
    bool use_take_dynamic_message = true)
  : SubscriptionBase(
      node_base,
      *(type_support->get_rosidl_message_type_support()),
      topic_name,
      options.to_rcl_subscription_options(qos),
      options.event_callbacks,
      options.use_default_callbacks,
      false,
      true,
      use_take_dynamic_message),
    ts_(type_support),
    callback_(callback),
    serialization_support_(nullptr),
    dynamic_message_(nullptr),
    dynamic_message_type_(nullptr)
  {
    if (!type_support) {
      throw std::runtime_error("DynamicMessageTypeSupport cannot be nullptr!");
    }

    if (type_support->get_rosidl_message_type_support()->typesupport_identifier !=
      rmw_dynamic_typesupport_c__identifier)
    {
      throw std::runtime_error(
              "DynamicSubscription must use dynamic type introspection type support!");
    }

    serialization_support_ = type_support->get_shared_dynamic_serialization_support();
    dynamic_message_type_ = type_support->get_shared_dynamic_message_type()->clone_shared();
    dynamic_message_ = type_support->get_shared_dynamic_message()->clone_shared();
  }

  // TODO(methylDragon):
  /// Deferred type description constructor, only usable if the middleware implementation supports
  /// type discovery
  // template<typename AllocatorT = std::allocator<void>>
  // DynamicSubscription(
  //   rclcpp::node_interfaces::NodeBaseInterface * node_base,
  //   const std::string & topic_name,
  //   const rclcpp::QoS & qos,
  //   // TODO(methylDragons): Eventually roll out an rclcpp::DynamicData that encompasses the serialization_support
  //   //                      support and DynamicData, and pass that to the callback
  //   std::function<void(
  //     rclcpp::dynamic_typesupport::DynamicSerializationSupport::SharedPtr,
  //     rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr
  //   )> callback,
  //   const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options,
  //   const char * serialization_lib_name = nullptr)
  // : SubscriptionBase(
  //     node_base,
  //     // NOTE(methylDragon): Since the typesupport is deferred, it needs to be modified post-hoc
  //     //                     which means it technically isn't const correct...
  //     *rmw_get_dynamic_message_typesupport_handle(serialization_lib_name),
  //     topic_name,
  //     options.to_rcl_subscription_options(qos),
  //     options.event_callbacks,
  //     options.use_default_callbacks,
  //     false,
  //     true),
  //   callback_(callback)
  // {}

  RCLCPP_PUBLIC
  virtual ~DynamicSubscription() = default;

  // Same as create_serialized_message() as the subscription is to serialized_messages only
  RCLCPP_PUBLIC
  std::shared_ptr<void> create_message() override;

  RCLCPP_PUBLIC
  std::shared_ptr<rclcpp::SerializedMessage> create_serialized_message() override;

  /// Cast the message to a rclcpp::SerializedMessage and call the callback.
  RCLCPP_PUBLIC
  void handle_message(
    std::shared_ptr<void> & message, const rclcpp::MessageInfo & message_info) override;

  /// Handle dispatching rclcpp::SerializedMessage to user callback.
  RCLCPP_PUBLIC
  void
  handle_serialized_message(
    const std::shared_ptr<rclcpp::SerializedMessage> & serialized_message,
    const rclcpp::MessageInfo & message_info) override;

  /// This function is currently not implemented.
  RCLCPP_PUBLIC
  void handle_loaned_message(
    void * loaned_message, const rclcpp::MessageInfo & message_info) override;

  // Same as return_serialized_message() as the subscription is to serialized_messages only
  RCLCPP_PUBLIC
  void return_message(std::shared_ptr<void> & message) override;

  RCLCPP_PUBLIC
  void return_serialized_message(std::shared_ptr<rclcpp::SerializedMessage> & message) override;


  // DYNAMIC TYPE ==================================================================================
  // TODO(methylDragon): Reorder later
  RCLCPP_PUBLIC
  rclcpp::dynamic_typesupport::DynamicMessageType::SharedPtr
  get_shared_dynamic_message_type() override;

  RCLCPP_PUBLIC
  rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr get_shared_dynamic_message() override;

  RCLCPP_PUBLIC
  rclcpp::dynamic_typesupport::DynamicSerializationSupport::SharedPtr
  get_shared_dynamic_serialization_support() override;

  RCLCPP_PUBLIC
  rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr create_dynamic_message() override;

  RCLCPP_PUBLIC
  void return_dynamic_message(
    rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr & message) override;

  RCLCPP_PUBLIC
  void handle_dynamic_message(
    const rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr & message,
    const rclcpp::MessageInfo & message_info) override;

private:
  RCLCPP_DISABLE_COPY(DynamicSubscription)

  rclcpp::dynamic_typesupport::DynamicMessageTypeSupport::SharedPtr ts_;
  std::function<void(rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr)> callback_;

  rclcpp::dynamic_typesupport::DynamicSerializationSupport::SharedPtr serialization_support_;
  rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr dynamic_message_;
  rclcpp::dynamic_typesupport::DynamicMessageType::SharedPtr dynamic_message_type_;
};

}  // namespace rclcpp

#endif  // RCLCPP__DYNAMIC_SUBSCRIPTION_HPP_
