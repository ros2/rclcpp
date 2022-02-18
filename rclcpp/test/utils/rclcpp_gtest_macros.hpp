// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef UTILS__RCLCPP_GTEST_MACROS_HPP_
#define UTILS__RCLCPP_GTEST_MACROS_HPP_

#include <gtest/gtest.h>

#include <cstring>
#include <type_traits>

#include "rclcpp/exceptions/exceptions.hpp"

namespace rclcpp
{
namespace testing
{
namespace details
{

/**
 * \brief Check if two thrown objects are equals.
 *
 * For generic thrown objects, probably is unlikely to be used. This type must
 * overload the == and << operators.
 */
template<typename T,
  typename = typename std::enable_if_t<
    !std::is_convertible<T, std::exception>::value>>
::testing::AssertionResult AreThrowableContentsEqual(
  const T & expected, const T & actual, const char * expected_exception_expression,
  const char * throwing_expression)
{
  if (expected == actual) {
    return ::testing::AssertionSuccess() <<
           "'\nThe value of the non-standard throwable thrown by the expression\n'" <<
           throwing_expression << "'\n\nmatches the value of the expected thrown object\n'" <<
           expected_exception_expression << "'\n\t(" << expected << " == " << actual << ")\n";
  }

  return ::testing::AssertionFailure() <<
         "\nThe value of the non-standard throwable thrown by the expression\n'" <<
         throwing_expression << "'\n\ndoes not match the value of the expected thrown object\n'" <<
         expected_exception_expression << "'\n\t(" << expected << " != " << actual << ")\n";
}

/**
 * \brief Check if two std::exceptions are equal according to their message.
 *
 * If the exception type also derives from rclcpp::Exception, then the next overload is called
 * instead
 */
template<typename T,
  typename = typename std::enable_if_t<
    !std::is_convertible<T, rclcpp::exceptions::RCLErrorBase>::value>>
::testing::AssertionResult AreThrowableContentsEqual(
  const std::exception & expected, const std::exception & actual,
  const char * expected_exception_expression,
  const char * throwing_expression)
{
  if (std::strcmp(expected.what(), actual.what()) == 0) {
    return ::testing::AssertionSuccess() <<
           "'\nThe contents of the std::exception thrown by the expression\n'" <<
           throwing_expression << "':\n\te.what(): '" << actual.what() <<
           "'\n\nmatch the contents of the expected std::exception\n'" <<
           expected_exception_expression << "'\n\te.what(): '" << expected.what() << "'\n";
  }

  return ::testing::AssertionFailure() <<
         "\nThe contents of the std::exception thrown by the expression\n'" <<
         throwing_expression << "':\n\te.what(): '" << actual.what() <<
         "'\n\ndo not match the contents of the expected std::exception\n'" <<
         expected_exception_expression << "'\n\te.what(): '" << expected.what() << "'\n";
}

/**
 * \brief Check if two exceptions that derive from rclcpp::RCLErrorBase are equal.
 *
 * This checks equality based on their return and message. It does not check the formatted
 * message, which is what is reported by std::exception::what() for RCLErrors.
 */
template<typename T,
  typename = typename std::enable_if_t<
    std::is_convertible<T, rclcpp::exceptions::RCLErrorBase>::value>>
::testing::AssertionResult AreThrowableContentsEqual(
  const rclcpp::exceptions::RCLErrorBase & expected,
  const rclcpp::exceptions::RCLErrorBase & actual,
  const char * expected_exception_expression,
  const char * throwing_expression)
{
  if ((expected.ret == actual.ret) && (expected.message == actual.message)) {
    return ::testing::AssertionSuccess() <<
           "'\nThe contents of the RCLError thrown by the expression\n'" << throwing_expression <<
           "':\n\trcl_ret_t: " << actual.ret << "\n\tmessage: '" << actual.message <<
           "'\n\nmatch the contents of the expected RCLError\n'" <<
           expected_exception_expression << "'\n\trcl_ret_t: " << expected.ret <<
           "\n\tmessage: '" << expected.message << "'\n";
  }

  return ::testing::AssertionFailure() <<
         "'\nThe contents of the RCLError thrown by the expression\n'" << throwing_expression <<
         "':\n\trcl_ret_t: " << actual.ret << "\n\tmessage: '" << actual.message <<
         "'\n\ndo not match the contents of the expected RCLError\n'" <<
         expected_exception_expression << "'\n\trcl_ret_t: " << expected.ret << "\n\tmessage: '" <<
         expected.message << "'\n";
}

}  // namespace details
}  // namespace testing
}  // namespace rclcpp

/**
 * \def CHECK_THROW_EQ_IMPL
 * \brief Implemented check if statement throws expected exception. don't use directly, use
 * RCLCPP_EXPECT_THROW_EQ or RCLCPP_ASSERT_THROW_EQ instead.
 */
#define CHECK_THROW_EQ_IMPL(throwing_statement, expected_exception, assertion_result) \
  do { \
    using ExceptionT = decltype(expected_exception); \
    try { \
      throwing_statement; \
      assertion_result = ::testing::AssertionFailure() << \
        "\nExpected the expression:\n\t'" #throwing_statement "'\nto throw: \n\t'" << \
        #expected_exception "'\nbut it did not throw.\n"; \
    } catch (const ExceptionT & e) { \
      assertion_result = \
        rclcpp::testing::details::AreThrowableContentsEqual<ExceptionT>( \
        expected_exception, e, #expected_exception, #throwing_statement); \
    } catch (const std::exception & e) { \
      assertion_result = ::testing::AssertionFailure() << \
        "\nExpected the expression:\n\t'" #throwing_statement "'\nto throw: \n\t'" << \
        #expected_exception "'\nbut it threw:\n\tType: " << typeid(e).name() << \
        "\n\te.what(): '" << e.what() << "'\n"; \
    } catch (...) { \
      assertion_result = ::testing::AssertionFailure() << \
        "\nExpected the expression:\n\t'" #throwing_statement "'\nto throw: \n\t'" << \
        #expected_exception "'\nbut it threw an unrecognized throwable type.\n"; \
    } \
  } while (0)

/**
 * \def RCLCPP_EXPECT_THROW_EQ
 * \brief Check if a statement throws the expected exception type and that the exceptions matches
 *   the expected exception.
 *
 * Like other gtest EXPECT_ macros, this doesn't halt a test and return on failure. Instead it
 * just adds a failure to the current test.
 *
 * See test_gtest_macros.cpp for examples
 */
#define RCLCPP_EXPECT_THROW_EQ(throwing_statement, expected_exception) \
  do { \
    ::testing::AssertionResult \
      is_the_result_of_the_throwing_expression_equal_to_the_expected_throwable = \
      ::testing::AssertionSuccess(); \
    CHECK_THROW_EQ_IMPL( \
      throwing_statement, \
      expected_exception, \
      is_the_result_of_the_throwing_expression_equal_to_the_expected_throwable); \
    EXPECT_TRUE(is_the_result_of_the_throwing_expression_equal_to_the_expected_throwable); \
  } while (0)

/**
 * \def RCLCPP_ASSERT_THROW_EQ
 * \brief Assert that a statement throws the expected exception type and that the exceptions
 * matches the expected exception.
 *
 * See test_gtest_macros.cpp for examples
 *
 * Like other gtest ASSERT_ macros, this will halt the test on failure and return.
 */
#define RCLCPP_ASSERT_THROW_EQ(throwing_statement, expected_exception) \
  do { \
    ::testing::AssertionResult \
      is_the_result_of_the_throwing_expression_equal_to_the_expected_throwable = \
      ::testing::AssertionSuccess(); \
    CHECK_THROW_EQ_IMPL( \
      throwing_statement, \
      expected_exception, \
      is_the_result_of_the_throwing_expression_equal_to_the_expected_throwable); \
    ASSERT_TRUE(is_the_result_of_the_throwing_expression_equal_to_the_expected_throwable); \
  } while (0)

#endif  // UTILS__RCLCPP_GTEST_MACROS_HPP_
