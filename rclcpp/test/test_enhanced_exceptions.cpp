// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/exceptions/enhanced_exceptions.hpp"
#include "rclcpp/exceptions/error_codes.hpp"
#include "rclcpp/exceptions/error_macros.hpp"

using rclcpp::exceptions::ErrorCategory;
using rclcpp::exceptions::ErrorCode;
using rclcpp::exceptions::ErrorContext;
using rclcpp::exceptions::ErrorSeverity;
using rclcpp::exceptions::ErrorSuggestion;
using rclcpp::exceptions::RclcppException;
using rclcpp::exceptions::NodeException;
using rclcpp::exceptions::TopicException;
using rclcpp::exceptions::ServiceException;
using rclcpp::exceptions::ParameterException;
using rclcpp::exceptions::ExecutorException;
using rclcpp::exceptions::QoSException;
using rclcpp::exceptions::InvalidNodeNameError;
using rclcpp::exceptions::InvalidNamespaceError;
using rclcpp::exceptions::NodeInitializationError;
using rclcpp::exceptions::InvalidTopicNameError;
using rclcpp::exceptions::InvalidServiceNameError;
using rclcpp::exceptions::ServiceNotAvailableError;
using rclcpp::exceptions::ParameterNotDeclaredException;
using rclcpp::exceptions::ParameterTypeMismatchException;
using rclcpp::exceptions::InvalidParameterValueException;
using rclcpp::exceptions::QoSIncompatibleError;
using rclcpp::exceptions::CallbackGroupError;
using rclcpp::exceptions::get_error_category;
using rclcpp::exceptions::get_error_severity;
using rclcpp::exceptions::get_error_code_name;
using rclcpp::exceptions::get_error_code_description;
using rclcpp::exceptions::get_category_name;
using rclcpp::exceptions::get_severity_name;
using rclcpp::exceptions::is_error_in_category;
using rclcpp::exceptions::make_context;
using rclcpp::exceptions::context_for_node;
using rclcpp::exceptions::context_for_topic;
using rclcpp::exceptions::context_for_service;
using rclcpp::exceptions::context_for_parameter;
using rclcpp::exceptions::make_suggestion;
using rclcpp::exceptions::error_code_to_string;
using rclcpp::exceptions::is_exception_category;
using rclcpp::exceptions::is_exception_code;

// ============================================================================
// Error Code Tests
// ============================================================================

class ErrorCodeTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(ErrorCodeTest, GetErrorCategoryForNodeErrors)
{
  EXPECT_EQ(ErrorCategory::NODE, get_error_category(ErrorCode::NODE_NAME_INVALID));
  EXPECT_EQ(ErrorCategory::NODE, get_error_category(ErrorCode::NODE_NAMESPACE_INVALID));
  EXPECT_EQ(ErrorCategory::NODE, get_error_category(ErrorCode::NODE_INIT_FAILED));
  EXPECT_EQ(ErrorCategory::NODE, get_error_category(ErrorCode::NODE_ALREADY_EXISTS));
  EXPECT_EQ(ErrorCategory::NODE, get_error_category(ErrorCode::NODE_NOT_FOUND));
}

TEST_F(ErrorCodeTest, GetErrorCategoryForTopicErrors)
{
  EXPECT_EQ(ErrorCategory::TOPIC, get_error_category(ErrorCode::TOPIC_NAME_INVALID));
  EXPECT_EQ(ErrorCategory::TOPIC, get_error_category(ErrorCode::TOPIC_NOT_AVAILABLE));
  EXPECT_EQ(ErrorCategory::TOPIC, get_error_category(ErrorCode::PUBLISHER_CREATION_FAILED));
  EXPECT_EQ(ErrorCategory::TOPIC, get_error_category(ErrorCode::SUBSCRIBER_CREATION_FAILED));
}

TEST_F(ErrorCodeTest, GetErrorCategoryForServiceErrors)
{
  EXPECT_EQ(ErrorCategory::SERVICE, get_error_category(ErrorCode::SERVICE_NAME_INVALID));
  EXPECT_EQ(ErrorCategory::SERVICE, get_error_category(ErrorCode::SERVICE_NOT_AVAILABLE));
  EXPECT_EQ(ErrorCategory::SERVICE, get_error_category(ErrorCode::SERVICE_TIMEOUT));
}

TEST_F(ErrorCodeTest, GetErrorCategoryForParameterErrors)
{
  EXPECT_EQ(ErrorCategory::PARAMETER, get_error_category(ErrorCode::PARAMETER_NOT_DECLARED));
  EXPECT_EQ(ErrorCategory::PARAMETER, get_error_category(ErrorCode::PARAMETER_TYPE_MISMATCH));
  EXPECT_EQ(ErrorCategory::PARAMETER, get_error_category(ErrorCode::PARAMETER_READ_ONLY));
}

TEST_F(ErrorCodeTest, GetErrorCategoryForQoSErrors)
{
  EXPECT_EQ(ErrorCategory::QOS, get_error_category(ErrorCode::QOS_INCOMPATIBLE));
  EXPECT_EQ(ErrorCategory::QOS, get_error_category(ErrorCode::QOS_RELIABILITY_MISMATCH));
  EXPECT_EQ(ErrorCategory::QOS, get_error_category(ErrorCode::QOS_DEADLINE_MISSED));
}

TEST_F(ErrorCodeTest, GetErrorCategoryForExecutorErrors)
{
  EXPECT_EQ(ErrorCategory::EXECUTOR, get_error_category(ErrorCode::EXECUTOR_NODE_ALREADY_ADDED));
  EXPECT_EQ(ErrorCategory::EXECUTOR, get_error_category(ErrorCode::EXECUTOR_CALLBACK_ERROR));
  EXPECT_EQ(ErrorCategory::EXECUTOR, get_error_category(ErrorCode::CALLBACK_GROUP_ERROR));
}

TEST_F(ErrorCodeTest, GetErrorCategoryForUnknown)
{
  EXPECT_EQ(ErrorCategory::UNKNOWN, get_error_category(ErrorCode::UNKNOWN));
}

TEST_F(ErrorCodeTest, GetErrorSeverity)
{
  EXPECT_EQ(ErrorSeverity::UNKNOWN, get_error_severity(ErrorCode::UNKNOWN));
  EXPECT_EQ(ErrorSeverity::ERROR, get_error_severity(ErrorCode::NODE_NAME_INVALID));
  EXPECT_EQ(ErrorSeverity::FATAL, get_error_severity(ErrorCode::ALLOCATION_FAILED));
  EXPECT_EQ(ErrorSeverity::FATAL, get_error_severity(ErrorCode::INTERNAL_ERROR));
}

TEST_F(ErrorCodeTest, GetErrorCodeName)
{
  EXPECT_EQ("NODE_NAME_INVALID", get_error_code_name(ErrorCode::NODE_NAME_INVALID));
  EXPECT_EQ("TOPIC_NOT_AVAILABLE", get_error_code_name(ErrorCode::TOPIC_NOT_AVAILABLE));
  EXPECT_EQ("PARAMETER_NOT_DECLARED", get_error_code_name(ErrorCode::PARAMETER_NOT_DECLARED));
  EXPECT_EQ("UNKNOWN", get_error_code_name(ErrorCode::UNKNOWN));
}

TEST_F(ErrorCodeTest, GetErrorCodeDescription)
{
  std::string desc = get_error_code_description(ErrorCode::NODE_NAME_INVALID);
  EXPECT_FALSE(desc.empty());
  EXPECT_NE(std::string::npos, desc.find("naming"));

  desc = get_error_code_description(ErrorCode::PARAMETER_NOT_DECLARED);
  EXPECT_FALSE(desc.empty());
  EXPECT_NE(std::string::npos, desc.find("declared"));
}

TEST_F(ErrorCodeTest, IsErrorInCategory)
{
  EXPECT_TRUE(is_error_in_category(ErrorCode::NODE_NAME_INVALID, ErrorCategory::NODE));
  EXPECT_FALSE(is_error_in_category(ErrorCode::NODE_NAME_INVALID, ErrorCategory::TOPIC));
  EXPECT_TRUE(is_error_in_category(ErrorCode::TOPIC_NAME_INVALID, ErrorCategory::TOPIC));
  EXPECT_FALSE(is_error_in_category(ErrorCode::TOPIC_NAME_INVALID, ErrorCategory::NODE));
}

TEST_F(ErrorCodeTest, GetCategoryName)
{
  EXPECT_EQ("Node", get_category_name(ErrorCategory::NODE));
  EXPECT_EQ("Topic", get_category_name(ErrorCategory::TOPIC));
  EXPECT_EQ("Service", get_category_name(ErrorCategory::SERVICE));
  EXPECT_EQ("Parameter", get_category_name(ErrorCategory::PARAMETER));
  EXPECT_EQ("QoS", get_category_name(ErrorCategory::QOS));
  EXPECT_EQ("Unknown", get_category_name(ErrorCategory::UNKNOWN));
}

TEST_F(ErrorCodeTest, GetSeverityName)
{
  EXPECT_EQ("UNKNOWN", get_severity_name(ErrorSeverity::UNKNOWN));
  EXPECT_EQ("INFO", get_severity_name(ErrorSeverity::INFO));
  EXPECT_EQ("WARNING", get_severity_name(ErrorSeverity::WARNING));
  EXPECT_EQ("ERROR", get_severity_name(ErrorSeverity::ERROR));
  EXPECT_EQ("FATAL", get_severity_name(ErrorSeverity::FATAL));
}

// ============================================================================
// Error Suggestion Tests
// ============================================================================

class ErrorSuggestionTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(ErrorSuggestionTest, DefaultConstruction)
{
  ErrorSuggestion suggestion;
  EXPECT_TRUE(suggestion.empty());
  EXPECT_TRUE(suggestion.primary.empty());
  EXPECT_TRUE(suggestion.alternatives.empty());
  EXPECT_TRUE(suggestion.documentation_url.empty());
}

TEST_F(ErrorSuggestionTest, ConstructWithPrimary)
{
  ErrorSuggestion suggestion("Try this instead");
  EXPECT_FALSE(suggestion.empty());
  EXPECT_EQ("Try this instead", suggestion.primary);
  EXPECT_TRUE(suggestion.alternatives.empty());
}

TEST_F(ErrorSuggestionTest, ConstructWithAlternatives)
{
  ErrorSuggestion suggestion("Primary", {"Alt 1", "Alt 2"});
  EXPECT_FALSE(suggestion.empty());
  EXPECT_EQ("Primary", suggestion.primary);
  EXPECT_EQ(2u, suggestion.alternatives.size());
  EXPECT_EQ("Alt 1", suggestion.alternatives[0]);
  EXPECT_EQ("Alt 2", suggestion.alternatives[1]);
}

TEST_F(ErrorSuggestionTest, ConstructWithDocUrl)
{
  ErrorSuggestion suggestion("Primary", {"Alt 1"}, "https://docs.ros.org");
  EXPECT_EQ("https://docs.ros.org", suggestion.documentation_url);
}

TEST_F(ErrorSuggestionTest, Format)
{
  ErrorSuggestion suggestion("Fix this", {"Option A", "Option B"}, "https://docs.ros.org");
  std::string formatted = suggestion.format();

  EXPECT_NE(std::string::npos, formatted.find("Fix this"));
  EXPECT_NE(std::string::npos, formatted.find("Option A"));
  EXPECT_NE(std::string::npos, formatted.find("Option B"));
  EXPECT_NE(std::string::npos, formatted.find("https://docs.ros.org"));
}

TEST_F(ErrorSuggestionTest, FormatEmpty)
{
  ErrorSuggestion suggestion;
  EXPECT_EQ("", suggestion.format());
}

// ============================================================================
// Error Context Tests
// ============================================================================

class ErrorContextTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(ErrorContextTest, DefaultConstruction)
{
  ErrorContext context;
  EXPECT_TRUE(context.empty());
  EXPECT_TRUE(context.as_map().empty());
}

TEST_F(ErrorContextTest, WithNodeName)
{
  ErrorContext context;
  context.with_node_name("my_node");

  EXPECT_FALSE(context.empty());
  EXPECT_EQ(1u, context.as_map().size());
  EXPECT_EQ("my_node", context.as_map().at("node_name"));
}

TEST_F(ErrorContextTest, WithTopicName)
{
  ErrorContext context;
  context.with_topic_name("/chatter");

  EXPECT_EQ("/chatter", context.as_map().at("topic_name"));
}

TEST_F(ErrorContextTest, WithServiceName)
{
  ErrorContext context;
  context.with_service_name("/add_two_ints");

  EXPECT_EQ("/add_two_ints", context.as_map().at("service_name"));
}

TEST_F(ErrorContextTest, WithParameterName)
{
  ErrorContext context;
  context.with_parameter_name("max_speed");

  EXPECT_EQ("max_speed", context.as_map().at("parameter_name"));
}

TEST_F(ErrorContextTest, WithTypes)
{
  ErrorContext context;
  context.with_expected_type("int").with_actual_type("string");

  EXPECT_EQ("int", context.as_map().at("expected_type"));
  EXPECT_EQ("string", context.as_map().at("actual_type"));
}

TEST_F(ErrorContextTest, WithValues)
{
  ErrorContext context;
  context.with_expected_value("42").with_actual_value("-1");

  EXPECT_EQ("42", context.as_map().at("expected_value"));
  EXPECT_EQ("-1", context.as_map().at("actual_value"));
}

TEST_F(ErrorContextTest, WithCustom)
{
  ErrorContext context;
  context.with("custom_key", "custom_value");

  EXPECT_EQ("custom_value", context.as_map().at("custom_key"));
}

TEST_F(ErrorContextTest, FluentInterface)
{
  ErrorContext context;
  context
  .with_node_name("node1")
  .with_topic_name("/topic1")
  .with_namespace("/ns");

  EXPECT_EQ(3u, context.as_map().size());
  EXPECT_EQ("node1", context.as_map().at("node_name"));
  EXPECT_EQ("/topic1", context.as_map().at("topic_name"));
  EXPECT_EQ("/ns", context.as_map().at("namespace"));
}

TEST_F(ErrorContextTest, Format)
{
  ErrorContext context;
  context.with_node_name("my_node").with_topic_name("/chatter");

  std::string formatted = context.format();
  EXPECT_NE(std::string::npos, formatted.find("node_name"));
  EXPECT_NE(std::string::npos, formatted.find("my_node"));
  EXPECT_NE(std::string::npos, formatted.find("topic_name"));
  EXPECT_NE(std::string::npos, formatted.find("/chatter"));
}

TEST_F(ErrorContextTest, CopyConstruction)
{
  ErrorContext original;
  original.with_node_name("test_node");

  ErrorContext copy(original);
  EXPECT_EQ("test_node", copy.as_map().at("node_name"));

  // Verify independence
  copy.with_topic_name("/new_topic");
  EXPECT_EQ(1u, original.as_map().size());
  EXPECT_EQ(2u, copy.as_map().size());
}

TEST_F(ErrorContextTest, MoveConstruction)
{
  ErrorContext original;
  original.with_node_name("test_node");

  ErrorContext moved(std::move(original));
  EXPECT_EQ("test_node", moved.as_map().at("node_name"));
}

// ============================================================================
// RclcppException Tests
// ============================================================================

class RclcppExceptionTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(RclcppExceptionTest, BasicConstruction)
{
  RclcppException ex(ErrorCode::NODE_NAME_INVALID, "Test message");

  EXPECT_EQ(ErrorCode::NODE_NAME_INVALID, ex.error_code());
  EXPECT_EQ(ErrorCategory::NODE, ex.error_category());
  EXPECT_EQ(ErrorSeverity::ERROR, ex.error_severity());
  EXPECT_EQ("Test message", ex.message());
}

TEST_F(RclcppExceptionTest, ConstructionWithSuggestion)
{
  RclcppException ex(
    ErrorCode::PARAMETER_NOT_DECLARED,
    "Parameter not found",
    "Declare the parameter first");

  EXPECT_EQ("Declare the parameter first", ex.suggestion());
  EXPECT_NE(nullptr, std::strstr(ex.what(), "Declare the parameter first"));
}

TEST_F(RclcppExceptionTest, ConstructionWithFullSuggestion)
{
  ErrorSuggestion suggestion("Primary fix", {"Alternative 1", "Alternative 2"});
  RclcppException ex(ErrorCode::QOS_INCOMPATIBLE, "QoS error", suggestion);

  EXPECT_EQ("Primary fix", ex.suggestion());
  EXPECT_EQ(2u, ex.full_suggestion().alternatives.size());
}

TEST_F(RclcppExceptionTest, ConstructionWithContext)
{
  ErrorContext context;
  context.with_node_name("test_node").with_topic_name("/test_topic");

  RclcppException ex(
    ErrorCode::TOPIC_NAME_INVALID,
    "Invalid topic",
    ErrorSuggestion("Fix the name"),
    context);

  EXPECT_EQ(2u, ex.context_map().size());
  EXPECT_EQ("test_node", ex.context_map().at("node_name"));
  EXPECT_EQ("/test_topic", ex.context_map().at("topic_name"));
}

TEST_F(RclcppExceptionTest, WhatMessageContainsErrorCode)
{
  RclcppException ex(ErrorCode::NODE_NAME_INVALID, "Test");
  std::string what_msg = ex.what();

  // Should contain the error code number
  EXPECT_NE(std::string::npos, what_msg.find("1001"));
}

TEST_F(RclcppExceptionTest, WhatMessageContainsMessage)
{
  RclcppException ex(ErrorCode::NODE_NAME_INVALID, "My custom message");
  std::string what_msg = ex.what();

  EXPECT_NE(std::string::npos, what_msg.find("My custom message"));
}

TEST_F(RclcppExceptionTest, WhatMessageContainsSuggestion)
{
  RclcppException ex(
    ErrorCode::NODE_NAME_INVALID,
    "Error occurred",
    "Try this fix");

  std::string what_msg = ex.what();
  EXPECT_NE(std::string::npos, what_msg.find("Try this fix"));
}

TEST_F(RclcppExceptionTest, SourceLocation)
{
  RclcppException ex(ErrorCode::NODE_NAME_INVALID, "Test");

  // Source location should be captured
  EXPECT_NE(nullptr, ex.file_name());
  EXPECT_NE(0u, ex.line());

  std::string loc = ex.location_string();
  EXPECT_FALSE(loc.empty());
}

TEST_F(RclcppExceptionTest, DetailedReport)
{
  ErrorContext context;
  context.with_node_name("my_node");

  RclcppException ex(
    ErrorCode::PARAMETER_NOT_DECLARED,
    "Parameter error",
    ErrorSuggestion("Declare parameter", {"Use declare_parameter()"}, "https://docs.ros.org"),
    context);

  std::string report = ex.detailed_report();

  EXPECT_NE(std::string::npos, report.find("4001"));
  EXPECT_NE(std::string::npos, report.find("PARAMETER_NOT_DECLARED"));
  EXPECT_NE(std::string::npos, report.find("Parameter error"));
  EXPECT_NE(std::string::npos, report.find("my_node"));
  EXPECT_NE(std::string::npos, report.find("Declare parameter"));
  EXPECT_NE(std::string::npos, report.find("https://docs.ros.org"));
}

TEST_F(RclcppExceptionTest, WithContextMethod)
{
  RclcppException ex(ErrorCode::NODE_NAME_INVALID, "Test");
  ex.with_context("key1", "value1").with_context("key2", "value2");

  EXPECT_EQ(2u, ex.context_map().size());
  EXPECT_EQ("value1", ex.context_map().at("key1"));
  EXPECT_EQ("value2", ex.context_map().at("key2"));
}

TEST_F(RclcppExceptionTest, CopyConstruction)
{
  RclcppException original(ErrorCode::NODE_NAME_INVALID, "Original message");
  original.with_context("key", "value");

  RclcppException copy(original);

  EXPECT_EQ(original.error_code(), copy.error_code());
  EXPECT_EQ(original.message(), copy.message());
  EXPECT_EQ(original.context_map().at("key"), copy.context_map().at("key"));
}

TEST_F(RclcppExceptionTest, MoveConstruction)
{
  RclcppException original(ErrorCode::NODE_NAME_INVALID, "Original message");
  RclcppException moved(std::move(original));

  EXPECT_EQ(ErrorCode::NODE_NAME_INVALID, moved.error_code());
  EXPECT_EQ("Original message", moved.message());
}

TEST_F(RclcppExceptionTest, CatchAsStdException)
{
  bool caught = false;
  try {
    throw RclcppException(ErrorCode::NODE_NAME_INVALID, "Test");
  } catch (const std::exception & e) {
    caught = true;
    EXPECT_NE(nullptr, e.what());
  }
  EXPECT_TRUE(caught);
}

// ============================================================================
// Specialized Exception Tests
// ============================================================================

class SpecializedExceptionTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(SpecializedExceptionTest, InvalidNodeNameError)
{
  InvalidNodeNameError ex("123_invalid");

  EXPECT_EQ(ErrorCode::NODE_NAME_INVALID, ex.error_code());
  EXPECT_NE(std::string::npos, std::string(ex.what()).find("123_invalid"));
  EXPECT_EQ("123_invalid", ex.context_map().at("node_name"));
  EXPECT_FALSE(ex.suggestion().empty());
}

TEST_F(SpecializedExceptionTest, InvalidNodeNameErrorWithReason)
{
  InvalidNodeNameError ex("bad_name", "starts with reserved character");

  std::string what_msg = ex.what();
  EXPECT_NE(std::string::npos, what_msg.find("reserved character"));
}

TEST_F(SpecializedExceptionTest, InvalidNamespaceError)
{
  InvalidNamespaceError ex("bad/namespace");

  EXPECT_EQ(ErrorCode::NODE_NAMESPACE_INVALID, ex.error_code());
  EXPECT_EQ("bad/namespace", ex.context_map().at("namespace"));
}

TEST_F(SpecializedExceptionTest, NodeInitializationError)
{
  NodeInitializationError ex("my_node", "RMW error");

  EXPECT_EQ(ErrorCode::NODE_INIT_FAILED, ex.error_code());
  EXPECT_NE(std::string::npos, std::string(ex.what()).find("RMW error"));
}

TEST_F(SpecializedExceptionTest, InvalidTopicNameError)
{
  InvalidTopicNameError ex("/bad topic");

  EXPECT_EQ(ErrorCode::TOPIC_NAME_INVALID, ex.error_code());
  EXPECT_EQ("/bad topic", ex.context_map().at("topic_name"));
}

TEST_F(SpecializedExceptionTest, InvalidServiceNameError)
{
  InvalidServiceNameError ex("/bad service");

  EXPECT_EQ(ErrorCode::SERVICE_NAME_INVALID, ex.error_code());
  EXPECT_EQ("/bad service", ex.context_map().at("service_name"));
}

TEST_F(SpecializedExceptionTest, ServiceNotAvailableError)
{
  ServiceNotAvailableError ex("/my_service");

  EXPECT_EQ(ErrorCode::SERVICE_NOT_AVAILABLE, ex.error_code());
  EXPECT_NE(std::string::npos, std::string(ex.what()).find("not available"));
}

TEST_F(SpecializedExceptionTest, ParameterNotDeclaredException)
{
  ParameterNotDeclaredException ex("my_param");

  EXPECT_EQ(ErrorCode::PARAMETER_NOT_DECLARED, ex.error_code());
  EXPECT_EQ("my_param", ex.context_map().at("parameter_name"));
  EXPECT_NE(std::string::npos, std::string(ex.what()).find("declare"));
}

TEST_F(SpecializedExceptionTest, ParameterTypeMismatchException)
{
  ParameterTypeMismatchException ex("speed", "double", "string");

  EXPECT_EQ(ErrorCode::PARAMETER_TYPE_MISMATCH, ex.error_code());
  EXPECT_EQ("speed", ex.context_map().at("parameter_name"));
  EXPECT_EQ("double", ex.context_map().at("expected_type"));
  EXPECT_EQ("string", ex.context_map().at("actual_type"));
}

TEST_F(SpecializedExceptionTest, InvalidParameterValueException)
{
  InvalidParameterValueException ex("count", "must be positive");

  EXPECT_EQ(ErrorCode::PARAMETER_INVALID_VALUE, ex.error_code());
  EXPECT_NE(std::string::npos, std::string(ex.what()).find("must be positive"));
}

TEST_F(SpecializedExceptionTest, QoSIncompatibleError)
{
  QoSIncompatibleError ex("/topic", "reliable", "best_effort");

  EXPECT_EQ(ErrorCode::QOS_INCOMPATIBLE, ex.error_code());
  EXPECT_EQ("/topic", ex.context_map().at("topic_name"));
  EXPECT_EQ("reliable", ex.context_map().at("publisher_qos"));
  EXPECT_EQ("best_effort", ex.context_map().at("subscriber_qos"));
}

TEST_F(SpecializedExceptionTest, CallbackGroupError)
{
  CallbackGroupError ex("mutex violation");

  EXPECT_EQ(ErrorCode::CALLBACK_GROUP_ERROR, ex.error_code());
  EXPECT_NE(std::string::npos, std::string(ex.what()).find("mutex violation"));
}

TEST_F(SpecializedExceptionTest, CatchAsBaseClass)
{
  bool caught_as_node_exception = false;
  bool caught_as_rclcpp_exception = false;

  try {
    throw InvalidNodeNameError("test");
  } catch (const NodeException &) {
    caught_as_node_exception = true;
  } catch (const RclcppException &) {
    caught_as_rclcpp_exception = true;
  }

  EXPECT_TRUE(caught_as_node_exception);
  EXPECT_FALSE(caught_as_rclcpp_exception);
}

TEST_F(SpecializedExceptionTest, CatchParameterExceptionHierarchy)
{
  bool caught = false;

  try {
    throw ParameterNotDeclaredException("test_param");
  } catch (const ParameterException &) {
    caught = true;
  }

  EXPECT_TRUE(caught);
}

// ============================================================================
// Error Macro Tests
// ============================================================================

class ErrorMacroTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(ErrorMacroTest, RclcppThrow)
{
  EXPECT_THROW(
    RCLCPP_THROW(ErrorCode::NODE_NAME_INVALID, "Test error"),
    RclcppException);
}

TEST_F(ErrorMacroTest, RclcppThrowWithSuggestion)
{
  try {
    RCLCPP_THROW_WITH_SUGGESTION(
      ErrorCode::PARAMETER_NOT_DECLARED,
      "Parameter missing",
      "Use declare_parameter()");
    FAIL() << "Expected exception";
  } catch (const RclcppException & ex) {
    EXPECT_EQ("Use declare_parameter()", ex.suggestion());
  }
}

TEST_F(ErrorMacroTest, RclcppThrowIf)
{
  bool condition = true;
  EXPECT_THROW(
    RCLCPP_THROW_IF(condition, ErrorCode::NODE_NAME_INVALID, "Condition was true"),
    RclcppException);

  condition = false;
  EXPECT_NO_THROW(
    RCLCPP_THROW_IF(condition, ErrorCode::NODE_NAME_INVALID, "Should not throw"));
}

TEST_F(ErrorMacroTest, RclcppThrowUnless)
{
  bool condition = false;
  EXPECT_THROW(
    RCLCPP_THROW_UNLESS(condition, ErrorCode::NODE_NAME_INVALID, "Condition was false"),
    RclcppException);

  condition = true;
  EXPECT_NO_THROW(
    RCLCPP_THROW_UNLESS(condition, ErrorCode::NODE_NAME_INVALID, "Should not throw"));
}

TEST_F(ErrorMacroTest, RclcppThrowIfNull)
{
  int * ptr = nullptr;
  EXPECT_THROW(
    RCLCPP_THROW_IF_NULL(ptr, ErrorCode::INTERNAL_ERROR, "Pointer is null"),
    RclcppException);

  int value = 42;
  ptr = &value;
  EXPECT_NO_THROW(
    RCLCPP_THROW_IF_NULL(ptr, ErrorCode::INTERNAL_ERROR, "Should not throw"));
}

TEST_F(ErrorMacroTest, RclcppThrowInvalidNodeName)
{
  EXPECT_THROW(
    RCLCPP_THROW_INVALID_NODE_NAME("123bad"),
    InvalidNodeNameError);
}

TEST_F(ErrorMacroTest, RclcppThrowInvalidNodeNameWithReason)
{
  try {
    RCLCPP_THROW_INVALID_NODE_NAME("bad", "starts with reserved");
    FAIL() << "Expected exception";
  } catch (const InvalidNodeNameError & ex) {
    EXPECT_NE(std::string::npos, std::string(ex.what()).find("reserved"));
  }
}

TEST_F(ErrorMacroTest, RclcppThrowParameterNotDeclared)
{
  try {
    RCLCPP_THROW_PARAMETER_NOT_DECLARED("my_param");
    FAIL() << "Expected exception";
  } catch (const ParameterNotDeclaredException & ex) {
    EXPECT_EQ("my_param", ex.context_map().at("parameter_name"));
  }
}

TEST_F(ErrorMacroTest, RclcppThrowParameterTypeMismatch)
{
  try {
    RCLCPP_THROW_PARAMETER_TYPE_MISMATCH("speed", "double", "int");
    FAIL() << "Expected exception";
  } catch (const ParameterTypeMismatchException & ex) {
    EXPECT_EQ("speed", ex.context_map().at("parameter_name"));
    EXPECT_EQ("double", ex.context_map().at("expected_type"));
    EXPECT_EQ("int", ex.context_map().at("actual_type"));
  }
}

TEST_F(ErrorMacroTest, RclcppThrowQoSIncompatible)
{
  try {
    RCLCPP_THROW_QOS_INCOMPATIBLE("/topic", "reliable", "best_effort");
    FAIL() << "Expected exception";
  } catch (const QoSIncompatibleError & ex) {
    EXPECT_EQ(ErrorCode::QOS_INCOMPATIBLE, ex.error_code());
  }
}

TEST_F(ErrorMacroTest, RclcppErrorMsg)
{
  std::string msg = RCLCPP_ERROR_MSG("Value " << 42 << " is invalid");
  EXPECT_EQ("Value 42 is invalid", msg);
}

TEST_F(ErrorMacroTest, RclcppThrowFmt)
{
  try {
    int value = 42;
    RCLCPP_THROW_FMT(ErrorCode::PARAMETER_INVALID_VALUE, "Value " << value << " is invalid");
    FAIL() << "Expected exception";
  } catch (const RclcppException & ex) {
    EXPECT_NE(std::string::npos, std::string(ex.what()).find("Value 42 is invalid"));
  }
}

// ============================================================================
// Context Builder Helper Tests
// ============================================================================

class ContextBuilderHelperTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(ContextBuilderHelperTest, MakeContext)
{
  auto context = make_context().with_node_name("test").with_topic_name("/topic");
  EXPECT_EQ(2u, context.as_map().size());
}

TEST_F(ContextBuilderHelperTest, ContextForNode)
{
  auto context = context_for_node("my_node");
  EXPECT_EQ("my_node", context.as_map().at("node_name"));
}

TEST_F(ContextBuilderHelperTest, ContextForTopic)
{
  auto context = context_for_topic("/my_topic");
  EXPECT_EQ("/my_topic", context.as_map().at("topic_name"));
}

TEST_F(ContextBuilderHelperTest, ContextForService)
{
  auto context = context_for_service("/my_service");
  EXPECT_EQ("/my_service", context.as_map().at("service_name"));
}

TEST_F(ContextBuilderHelperTest, ContextForParameter)
{
  auto context = context_for_parameter("my_param");
  EXPECT_EQ("my_param", context.as_map().at("parameter_name"));
}

TEST_F(ContextBuilderHelperTest, MakeSuggestion)
{
  auto suggestion = make_suggestion("Primary fix");
  EXPECT_EQ("Primary fix", suggestion.primary);
  EXPECT_TRUE(suggestion.alternatives.empty());
}

TEST_F(ContextBuilderHelperTest, MakeSuggestionWithAlternatives)
{
  auto suggestion = make_suggestion("Primary", {"Alt 1", "Alt 2"});
  EXPECT_EQ(2u, suggestion.alternatives.size());
}

TEST_F(ContextBuilderHelperTest, MakeSuggestionWithDocUrl)
{
  auto suggestion = make_suggestion("Primary", {"Alt"}, "https://docs.ros.org");
  EXPECT_EQ("https://docs.ros.org", suggestion.documentation_url);
}

TEST_F(ContextBuilderHelperTest, ErrorCodeToString)
{
  std::string str = error_code_to_string(ErrorCode::NODE_NAME_INVALID);
  EXPECT_NE(std::string::npos, str.find("1001"));
  EXPECT_NE(std::string::npos, str.find("NODE_NAME_INVALID"));
}

TEST_F(ContextBuilderHelperTest, IsExceptionCategory)
{
  InvalidNodeNameError ex("test");
  EXPECT_TRUE(is_exception_category(ex, ErrorCategory::NODE));
  EXPECT_FALSE(is_exception_category(ex, ErrorCategory::TOPIC));
}

TEST_F(ContextBuilderHelperTest, IsExceptionCode)
{
  ParameterNotDeclaredException ex("param");
  EXPECT_TRUE(is_exception_code(ex, ErrorCode::PARAMETER_NOT_DECLARED));
  EXPECT_FALSE(is_exception_code(ex, ErrorCode::PARAMETER_TYPE_MISMATCH));
}

// ============================================================================
// Thread Safety Tests
// ============================================================================

class ThreadSafetyTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(ThreadSafetyTest, ConcurrentWhatCalls)
{
  RclcppException ex(ErrorCode::NODE_NAME_INVALID, "Test message", "Suggestion");

  std::vector<std::thread> threads;
  std::vector<std::string> results(10);

  for (size_t i = 0; i < 10; ++i) {
    threads.emplace_back([&ex, &results, i]() {
        for (int j = 0; j < 100; ++j) {
          results[i] = ex.what();
        }
      });
  }

  for (auto & t : threads) {
    t.join();
  }

  // All threads should get the same result
  for (const auto & result : results) {
    EXPECT_EQ(results[0], result);
  }
}

TEST_F(ThreadSafetyTest, ExceptionCopyInThreads)
{
  RclcppException original(ErrorCode::NODE_NAME_INVALID, "Original");
  original.with_context("key", "value");

  std::vector<std::thread> threads;
  std::atomic<int> success_count{0};

  for (size_t i = 0; i < 10; ++i) {
    threads.emplace_back([&original, &success_count]() {
        try {
          RclcppException copy(original);
          if (copy.error_code() == ErrorCode::NODE_NAME_INVALID &&
          copy.context_map().at("key") == "value")
          {
            success_count++;
          }
        } catch (...) {
          // Should not happen
        }
      });
  }

  for (auto & t : threads) {
    t.join();
  }

  EXPECT_EQ(10, success_count.load());
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
