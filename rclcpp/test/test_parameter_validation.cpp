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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/parameter_validation.hpp"
#include "rclcpp/parameter_validators.hpp"
#include "rclcpp/parameter_builder.hpp"
#include "rclcpp/rclcpp.hpp"

// =============================================================================
// ValidationResult Tests
// =============================================================================

class ValidationResultTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(ValidationResultTest, SuccessResult)
{
  auto result = rclcpp::ValidationResult::success();
  EXPECT_TRUE(result.successful());
  EXPECT_TRUE(static_cast<bool>(result));
  EXPECT_TRUE(result.reason().empty());
}

TEST_F(ValidationResultTest, FailureResult)
{
  auto result = rclcpp::ValidationResult::failure("Something went wrong");
  EXPECT_FALSE(result.successful());
  EXPECT_FALSE(static_cast<bool>(result));
  EXPECT_EQ(result.reason(), "Something went wrong");
}

TEST_F(ValidationResultTest, ToMsg)
{
  auto success = rclcpp::ValidationResult::success();
  auto success_msg = success.to_msg();
  EXPECT_TRUE(success_msg.successful);
  EXPECT_TRUE(success_msg.reason.empty());

  auto failure = rclcpp::ValidationResult::failure("Error message");
  auto failure_msg = failure.to_msg();
  EXPECT_FALSE(failure_msg.successful);
  EXPECT_EQ(failure_msg.reason, "Error message");
}

// =============================================================================
// RangeValidator Tests
// =============================================================================

class RangeValidatorTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(RangeValidatorTest, DoubleRangeInclusive)
{
  rclcpp::RangeValidator<double> validator(0.0, 100.0);

  // Valid values
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", 0.0)).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", 50.0)).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", 100.0)).successful());

  // Invalid values
  EXPECT_FALSE(validator.validate(rclcpp::Parameter("p", -0.001)).successful());
  EXPECT_FALSE(validator.validate(rclcpp::Parameter("p", 100.001)).successful());
  EXPECT_FALSE(validator.validate(rclcpp::Parameter("p", -1000.0)).successful());
}

TEST_F(RangeValidatorTest, DoubleRangeExclusive)
{
  auto validator = rclcpp::RangeValidator<double>(0.0, 100.0).exclusive();

  // Boundaries should fail
  EXPECT_FALSE(validator.validate(rclcpp::Parameter("p", 0.0)).successful());
  EXPECT_FALSE(validator.validate(rclcpp::Parameter("p", 100.0)).successful());

  // Values inside should pass
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", 0.001)).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", 99.999)).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", 50.0)).successful());
}

TEST_F(RangeValidatorTest, DoubleRangeExclusiveMin)
{
  auto validator = rclcpp::RangeValidator<double>(0.0, 100.0).exclusive_min();

  EXPECT_FALSE(validator.validate(rclcpp::Parameter("p", 0.0)).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", 100.0)).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", 0.001)).successful());
}

TEST_F(RangeValidatorTest, DoubleRangeExclusiveMax)
{
  auto validator = rclcpp::RangeValidator<double>(0.0, 100.0).exclusive_max();

  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", 0.0)).successful());
  EXPECT_FALSE(validator.validate(rclcpp::Parameter("p", 100.0)).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", 99.999)).successful());
}

TEST_F(RangeValidatorTest, DoubleMinOnly)
{
  auto validator = rclcpp::RangeValidator<double>::min(0.0);

  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", 0.0)).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", 1000000.0)).successful());
  EXPECT_FALSE(validator.validate(rclcpp::Parameter("p", -0.001)).successful());
}

TEST_F(RangeValidatorTest, DoubleMaxOnly)
{
  auto validator = rclcpp::RangeValidator<double>::max(100.0);

  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", 100.0)).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", -1000000.0)).successful());
  EXPECT_FALSE(validator.validate(rclcpp::Parameter("p", 100.001)).successful());
}

TEST_F(RangeValidatorTest, IntegerRange)
{
  rclcpp::RangeValidator<int64_t> validator(0, 100);

  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", 0)).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", 50)).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", 100)).successful());
  EXPECT_FALSE(validator.validate(rclcpp::Parameter("p", -1)).successful());
  EXPECT_FALSE(validator.validate(rclcpp::Parameter("p", 101)).successful());
}

TEST_F(RangeValidatorTest, IntegerFromDouble)
{
  // Double validator should accept integer values
  rclcpp::RangeValidator<double> validator(0.0, 100.0);

  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", 50)).successful());
}

TEST_F(RangeValidatorTest, TypeMismatch)
{
  rclcpp::RangeValidator<int64_t> int_validator(0, 100);
  rclcpp::RangeValidator<double> double_validator(0.0, 100.0);

  // Integer validator should reject strings
  EXPECT_FALSE(int_validator.validate(rclcpp::Parameter("p", "50")).successful());

  // Double validator should reject strings
  EXPECT_FALSE(double_validator.validate(rclcpp::Parameter("p", "50.0")).successful());
}

TEST_F(RangeValidatorTest, Description)
{
  rclcpp::RangeValidator<double> both(0.0, 100.0);
  EXPECT_NE(both.description().find("[0"), std::string::npos);
  EXPECT_NE(both.description().find("100]"), std::string::npos);

  auto min_only = rclcpp::RangeValidator<double>::min(0.0);
  EXPECT_NE(min_only.description().find("[0"), std::string::npos);
  EXPECT_NE(min_only.description().find("+inf"), std::string::npos);

  auto exclusive = rclcpp::RangeValidator<double>(0.0, 100.0).exclusive();
  EXPECT_NE(exclusive.description().find("(0"), std::string::npos);
  EXPECT_NE(exclusive.description().find("100)"), std::string::npos);
}

TEST_F(RangeValidatorTest, ErrorMessageContent)
{
  rclcpp::RangeValidator<double> validator(0.0, 100.0);
  auto result = validator.validate(rclcpp::Parameter("max_velocity", 150.0));

  EXPECT_FALSE(result.successful());
  EXPECT_NE(result.reason().find("max_velocity"), std::string::npos);
  EXPECT_NE(result.reason().find("150"), std::string::npos);
}

// =============================================================================
// OneOfValidator Tests
// =============================================================================

class OneOfValidatorTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(OneOfValidatorTest, StringValues)
{
  rclcpp::OneOfValidator<std::string> validator({"slow", "normal", "fast"});

  EXPECT_TRUE(validator.validate(rclcpp::Parameter("mode", "slow")).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("mode", "normal")).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("mode", "fast")).successful());

  EXPECT_FALSE(validator.validate(rclcpp::Parameter("mode", "turbo")).successful());
  EXPECT_FALSE(validator.validate(rclcpp::Parameter("mode", "")).successful());
  EXPECT_FALSE(validator.validate(rclcpp::Parameter("mode", "SLOW")).successful());  // Case sensitive
}

TEST_F(OneOfValidatorTest, IntegerValues)
{
  rclcpp::OneOfValidator<int64_t> validator({1, 2, 4, 8, 16});

  EXPECT_TRUE(validator.validate(rclcpp::Parameter("power", 1)).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("power", 8)).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("power", 16)).successful());

  EXPECT_FALSE(validator.validate(rclcpp::Parameter("power", 3)).successful());
  EXPECT_FALSE(validator.validate(rclcpp::Parameter("power", 0)).successful());
}

TEST_F(OneOfValidatorTest, DoubleValues)
{
  rclcpp::OneOfValidator<double> validator({0.0, 0.5, 1.0});

  EXPECT_TRUE(validator.validate(rclcpp::Parameter("factor", 0.0)).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("factor", 0.5)).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("factor", 1.0)).successful());

  EXPECT_FALSE(validator.validate(rclcpp::Parameter("factor", 0.25)).successful());
}

TEST_F(OneOfValidatorTest, BoolValues)
{
  rclcpp::OneOfValidator<bool> validator({true});

  EXPECT_TRUE(validator.validate(rclcpp::Parameter("enabled", true)).successful());
  EXPECT_FALSE(validator.validate(rclcpp::Parameter("enabled", false)).successful());
}

TEST_F(OneOfValidatorTest, TypeMismatch)
{
  rclcpp::OneOfValidator<std::string> validator({"a", "b", "c"});

  EXPECT_FALSE(validator.validate(rclcpp::Parameter("p", 123)).successful());
  EXPECT_FALSE(validator.validate(rclcpp::Parameter("p", 1.5)).successful());
}

TEST_F(OneOfValidatorTest, Description)
{
  rclcpp::OneOfValidator<std::string> validator({"slow", "fast"});
  auto desc = validator.description();

  EXPECT_NE(desc.find("one of"), std::string::npos);
  EXPECT_NE(desc.find("slow"), std::string::npos);
  EXPECT_NE(desc.find("fast"), std::string::npos);
}

TEST_F(OneOfValidatorTest, AllowedValues)
{
  rclcpp::OneOfValidator<std::string> validator({"a", "b", "c"});
  auto values = validator.allowed_values();

  EXPECT_EQ(values.size(), 3u);
  EXPECT_EQ(values[0], "a");
  EXPECT_EQ(values[1], "b");
  EXPECT_EQ(values[2], "c");
}

// =============================================================================
// RegexValidator Tests
// =============================================================================

class RegexValidatorTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(RegexValidatorTest, SimplePattern)
{
  rclcpp::RegexValidator validator("^[a-z]+$");

  EXPECT_TRUE(validator.validate(rclcpp::Parameter("name", "hello")).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("name", "world")).successful());

  EXPECT_FALSE(validator.validate(rclcpp::Parameter("name", "Hello")).successful());
  EXPECT_FALSE(validator.validate(rclcpp::Parameter("name", "hello123")).successful());
  EXPECT_FALSE(validator.validate(rclcpp::Parameter("name", "")).successful());
}

TEST_F(RegexValidatorTest, IdentifierPattern)
{
  rclcpp::RegexValidator validator("^[a-zA-Z_][a-zA-Z0-9_]*$", "valid identifier");

  EXPECT_TRUE(validator.validate(rclcpp::Parameter("id", "my_var")).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("id", "_private")).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("id", "CamelCase123")).successful());

  EXPECT_FALSE(validator.validate(rclcpp::Parameter("id", "123start")).successful());
  EXPECT_FALSE(validator.validate(rclcpp::Parameter("id", "has-dash")).successful());
}

TEST_F(RegexValidatorTest, IPAddressPattern)
{
  rclcpp::RegexValidator validator(R"(^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}$)", "IPv4 address");

  EXPECT_TRUE(validator.validate(rclcpp::Parameter("ip", "192.168.1.1")).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("ip", "0.0.0.0")).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("ip", "255.255.255.255")).successful());

  EXPECT_FALSE(validator.validate(rclcpp::Parameter("ip", "1234.1.1.1")).successful());
  EXPECT_FALSE(validator.validate(rclcpp::Parameter("ip", "192.168.1")).successful());
}

TEST_F(RegexValidatorTest, TypeMismatch)
{
  rclcpp::RegexValidator validator(".*");

  EXPECT_FALSE(validator.validate(rclcpp::Parameter("p", 123)).successful());
  EXPECT_FALSE(validator.validate(rclcpp::Parameter("p", 1.5)).successful());
}

TEST_F(RegexValidatorTest, Description)
{
  rclcpp::RegexValidator validator("^test$", "test pattern");
  auto desc = validator.description();

  EXPECT_NE(desc.find("^test$"), std::string::npos);
  EXPECT_NE(desc.find("test pattern"), std::string::npos);
}

TEST_F(RegexValidatorTest, PatternString)
{
  rclcpp::RegexValidator validator("^[a-z]+$");
  EXPECT_EQ(validator.pattern_string(), "^[a-z]+$");
}

// =============================================================================
// NotEmptyValidator Tests
// =============================================================================

class NotEmptyValidatorTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(NotEmptyValidatorTest, StringValidation)
{
  rclcpp::NotEmptyValidator validator;

  EXPECT_TRUE(validator.validate(rclcpp::Parameter("s", "hello")).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("s", " ")).successful());  // Whitespace is not empty

  EXPECT_FALSE(validator.validate(rclcpp::Parameter("s", "")).successful());
}

TEST_F(NotEmptyValidatorTest, ArrayValidation)
{
  rclcpp::NotEmptyValidator validator;

  // Non-empty arrays
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("arr", std::vector<int64_t>{1})).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("arr", std::vector<double>{1.0, 2.0})).successful());
  EXPECT_TRUE(validator.validate(
    rclcpp::Parameter("arr", std::vector<std::string>{"a"})).successful());

  // Empty arrays
  EXPECT_FALSE(validator.validate(rclcpp::Parameter("arr", std::vector<int64_t>{})).successful());
  EXPECT_FALSE(validator.validate(rclcpp::Parameter("arr", std::vector<double>{})).successful());
  EXPECT_FALSE(validator.validate(rclcpp::Parameter("arr", std::vector<std::string>{})).successful());
}

TEST_F(NotEmptyValidatorTest, ScalarTypesAlwaysPass)
{
  rclcpp::NotEmptyValidator validator;

  // Scalar types are never "empty"
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", true)).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", false)).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", 0)).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", 0.0)).successful());
}

TEST_F(NotEmptyValidatorTest, Description)
{
  rclcpp::NotEmptyValidator validator;
  EXPECT_NE(validator.description().find("non-empty"), std::string::npos);
}

// =============================================================================
// StringLengthValidator Tests
// =============================================================================

class StringLengthValidatorTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(StringLengthValidatorTest, BothBounds)
{
  rclcpp::StringLengthValidator validator(2, 10);

  EXPECT_TRUE(validator.validate(rclcpp::Parameter("s", "ab")).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("s", "hello")).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("s", "1234567890")).successful());

  EXPECT_FALSE(validator.validate(rclcpp::Parameter("s", "a")).successful());
  EXPECT_FALSE(validator.validate(rclcpp::Parameter("s", "12345678901")).successful());
}

TEST_F(StringLengthValidatorTest, MinLengthOnly)
{
  auto validator = rclcpp::StringLengthValidator::min_length(3);

  EXPECT_TRUE(validator.validate(rclcpp::Parameter("s", "abc")).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("s", "very long string")).successful());

  EXPECT_FALSE(validator.validate(rclcpp::Parameter("s", "ab")).successful());
  EXPECT_FALSE(validator.validate(rclcpp::Parameter("s", "")).successful());
}

TEST_F(StringLengthValidatorTest, MaxLengthOnly)
{
  auto validator = rclcpp::StringLengthValidator::max_length(5);

  EXPECT_TRUE(validator.validate(rclcpp::Parameter("s", "")).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("s", "12345")).successful());

  EXPECT_FALSE(validator.validate(rclcpp::Parameter("s", "123456")).successful());
}

TEST_F(StringLengthValidatorTest, TypeMismatch)
{
  rclcpp::StringLengthValidator validator(1, 10);

  EXPECT_FALSE(validator.validate(rclcpp::Parameter("p", 123)).successful());
  EXPECT_FALSE(validator.validate(rclcpp::Parameter("p", 1.5)).successful());
}

// =============================================================================
// TypeValidator Tests
// =============================================================================

class TypeValidatorTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(TypeValidatorTest, SingleType)
{
  rclcpp::TypeValidator validator(rclcpp::ParameterType::PARAMETER_DOUBLE);

  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", 1.5)).successful());
  EXPECT_FALSE(validator.validate(rclcpp::Parameter("p", 1)).successful());
  EXPECT_FALSE(validator.validate(rclcpp::Parameter("p", "1.5")).successful());
}

TEST_F(TypeValidatorTest, MultipleTypes)
{
  rclcpp::TypeValidator validator({
    rclcpp::ParameterType::PARAMETER_INTEGER,
    rclcpp::ParameterType::PARAMETER_DOUBLE
  });

  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", 1)).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", 1.5)).successful());
  EXPECT_FALSE(validator.validate(rclcpp::Parameter("p", "1")).successful());
}

TEST_F(TypeValidatorTest, Description)
{
  rclcpp::TypeValidator single(rclcpp::ParameterType::PARAMETER_DOUBLE);
  EXPECT_NE(single.description().find("double"), std::string::npos);

  rclcpp::TypeValidator multiple({
    rclcpp::ParameterType::PARAMETER_INTEGER,
    rclcpp::ParameterType::PARAMETER_DOUBLE
  });
  EXPECT_NE(multiple.description().find("integer"), std::string::npos);
  EXPECT_NE(multiple.description().find("double"), std::string::npos);
}

// =============================================================================
// CompositeValidator Tests
// =============================================================================

class CompositeValidatorTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(CompositeValidatorTest, AllMode)
{
  rclcpp::CompositeValidator validator(rclcpp::CompositeValidator::Mode::ALL);
  validator.add(std::make_shared<rclcpp::RangeValidator<double>>(0.0, 100.0));
  validator.add(std::make_shared<rclcpp::RangeValidator<double>>(50.0, 150.0));

  // Must pass both: [0, 100] AND [50, 150] = [50, 100]
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", 50.0)).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", 100.0)).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", 75.0)).successful());

  EXPECT_FALSE(validator.validate(rclcpp::Parameter("p", 25.0)).successful());  // Fails second
  EXPECT_FALSE(validator.validate(rclcpp::Parameter("p", 125.0)).successful());  // Fails first
}

TEST_F(CompositeValidatorTest, AnyMode)
{
  rclcpp::CompositeValidator validator(rclcpp::CompositeValidator::Mode::ANY);
  validator.add(std::make_shared<rclcpp::RangeValidator<double>>(0.0, 10.0));
  validator.add(std::make_shared<rclcpp::RangeValidator<double>>(90.0, 100.0));

  // Must pass either: [0, 10] OR [90, 100]
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", 5.0)).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", 95.0)).successful());

  EXPECT_FALSE(validator.validate(rclcpp::Parameter("p", 50.0)).successful());  // Fails both
}

TEST_F(CompositeValidatorTest, FluentAPI)
{
  rclcpp::CompositeValidator validator;
  validator
    .and_also(std::make_shared<rclcpp::RangeValidator<double>>(0.0, 100.0))
    .and_also(std::make_shared<rclcpp::TypeValidator>(rclcpp::ParameterType::PARAMETER_DOUBLE));

  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", 50.0)).successful());
  EXPECT_FALSE(validator.validate(rclcpp::Parameter("p", 50)).successful());  // Wrong type
}

TEST_F(CompositeValidatorTest, OrElseChangesMode)
{
  rclcpp::CompositeValidator validator;
  validator
    .add(std::make_shared<rclcpp::RangeValidator<double>>(0.0, 10.0))
    .or_else(std::make_shared<rclcpp::RangeValidator<double>>(90.0, 100.0));

  EXPECT_EQ(validator.mode(), rclcpp::CompositeValidator::Mode::ANY);
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", 5.0)).successful());
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", 95.0)).successful());
}

TEST_F(CompositeValidatorTest, EmptyComposite)
{
  rclcpp::CompositeValidator validator;
  EXPECT_TRUE(validator.validate(rclcpp::Parameter("p", "anything")).successful());
}

TEST_F(CompositeValidatorTest, Description)
{
  rclcpp::CompositeValidator all_validator(rclcpp::CompositeValidator::Mode::ALL);
  all_validator.add(std::make_shared<rclcpp::NotEmptyValidator>());
  all_validator.add(std::make_shared<rclcpp::StringLengthValidator>(1, 10));

  auto desc = all_validator.description();
  EXPECT_NE(desc.find("AND"), std::string::npos);

  rclcpp::CompositeValidator any_validator(rclcpp::CompositeValidator::Mode::ANY);
  any_validator.add(std::make_shared<rclcpp::NotEmptyValidator>());
  any_validator.add(std::make_shared<rclcpp::StringLengthValidator>(1, 10));

  desc = any_validator.description();
  EXPECT_NE(desc.find("OR"), std::string::npos);
}

// =============================================================================
// Factory Functions Tests
// =============================================================================

class FactoryFunctionsTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(FactoryFunctionsTest, RangeFactory)
{
  auto validator = rclcpp::validators::range(0.0, 100.0);
  EXPECT_TRUE(validator->validate(rclcpp::Parameter("p", 50.0)).successful());
  EXPECT_FALSE(validator->validate(rclcpp::Parameter("p", 150.0)).successful());
}

TEST_F(FactoryFunctionsTest, MinMaxFactory)
{
  auto min_validator = rclcpp::validators::min(0.0);
  EXPECT_TRUE(min_validator->validate(rclcpp::Parameter("p", 0.0)).successful());
  EXPECT_FALSE(min_validator->validate(rclcpp::Parameter("p", -1.0)).successful());

  auto max_validator = rclcpp::validators::max(100.0);
  EXPECT_TRUE(max_validator->validate(rclcpp::Parameter("p", 100.0)).successful());
  EXPECT_FALSE(max_validator->validate(rclcpp::Parameter("p", 101.0)).successful());
}

TEST_F(FactoryFunctionsTest, OneOfFactory)
{
  auto validator = rclcpp::validators::one_of<std::string>({"a", "b", "c"});
  EXPECT_TRUE(validator->validate(rclcpp::Parameter("p", "a")).successful());
  EXPECT_FALSE(validator->validate(rclcpp::Parameter("p", "d")).successful());
}

TEST_F(FactoryFunctionsTest, MatchesFactory)
{
  auto validator = rclcpp::validators::matches("^[a-z]+$");
  EXPECT_TRUE(validator->validate(rclcpp::Parameter("p", "hello")).successful());
  EXPECT_FALSE(validator->validate(rclcpp::Parameter("p", "Hello")).successful());
}

TEST_F(FactoryFunctionsTest, NotEmptyFactory)
{
  auto validator = rclcpp::validators::not_empty();
  EXPECT_TRUE(validator->validate(rclcpp::Parameter("p", "hello")).successful());
  EXPECT_FALSE(validator->validate(rclcpp::Parameter("p", "")).successful());
}

TEST_F(FactoryFunctionsTest, LengthFactory)
{
  auto validator = rclcpp::validators::length(1, 10);
  EXPECT_TRUE(validator->validate(rclcpp::Parameter("p", "hello")).successful());
  EXPECT_FALSE(validator->validate(rclcpp::Parameter("p", "")).successful());
}

TEST_F(FactoryFunctionsTest, TypeFactory)
{
  auto validator = rclcpp::validators::type(rclcpp::ParameterType::PARAMETER_DOUBLE);
  EXPECT_TRUE(validator->validate(rclcpp::Parameter("p", 1.5)).successful());
  EXPECT_FALSE(validator->validate(rclcpp::Parameter("p", 1)).successful());
}

TEST_F(FactoryFunctionsTest, AllOfFactory)
{
  auto validator = rclcpp::validators::all_of({
    rclcpp::validators::range(0.0, 100.0),
    rclcpp::validators::type(rclcpp::ParameterType::PARAMETER_DOUBLE)
  });

  EXPECT_TRUE(validator->validate(rclcpp::Parameter("p", 50.0)).successful());
  EXPECT_FALSE(validator->validate(rclcpp::Parameter("p", 50)).successful());  // Wrong type
  EXPECT_FALSE(validator->validate(rclcpp::Parameter("p", 150.0)).successful());  // Out of range
}

TEST_F(FactoryFunctionsTest, AnyOfFactory)
{
  auto validator = rclcpp::validators::any_of({
    rclcpp::validators::one_of<std::string>({"yes"}),
    rclcpp::validators::one_of<std::string>({"no"})
  });

  EXPECT_TRUE(validator->validate(rclcpp::Parameter("p", "yes")).successful());
  EXPECT_TRUE(validator->validate(rclcpp::Parameter("p", "no")).successful());
  EXPECT_FALSE(validator->validate(rclcpp::Parameter("p", "maybe")).successful());
}

// =============================================================================
// ParameterBuilder Tests
// =============================================================================

class ParameterBuilderTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(ParameterBuilderTest, BasicConstruction)
{
  auto builder = rclcpp::ParameterBuilder()
    .name("test_param")
    .description("A test parameter")
    .default_value(42.0)
    .read_only(true);

  EXPECT_EQ(builder.get_name(), "test_param");
  EXPECT_EQ(builder.get_description(), "A test parameter");
  EXPECT_TRUE(builder.has_default_value());
  EXPECT_TRUE(builder.is_read_only());
}

TEST_F(ParameterBuilderTest, ConstructWithName)
{
  rclcpp::ParameterBuilder builder("my_param");
  EXPECT_EQ(builder.get_name(), "my_param");
}

TEST_F(ParameterBuilderTest, BuildParameter)
{
  auto param = rclcpp::ParameterBuilder()
    .name("velocity")
    .default_value(1.5)
    .build();

  EXPECT_EQ(param.get_name(), "velocity");
  EXPECT_EQ(param.as_double(), 1.5);
}

TEST_F(ParameterBuilderTest, BuildDescriptor)
{
  auto builder = rclcpp::ParameterBuilder()
    .name("velocity")
    .description("Robot velocity")
    .read_only(true)
    .range(0.0, 10.0);

  auto descriptor = builder.build_descriptor();

  EXPECT_EQ(descriptor.description, "Robot velocity");
  EXPECT_TRUE(descriptor.read_only);
  EXPECT_FALSE(descriptor.additional_constraints.empty());
  EXPECT_EQ(descriptor.floating_point_range.size(), 1u);
  EXPECT_EQ(descriptor.floating_point_range[0].from_value, 0.0);
  EXPECT_EQ(descriptor.floating_point_range[0].to_value, 10.0);
}

TEST_F(ParameterBuilderTest, IntegerRangeDescriptor)
{
  auto builder = rclcpp::ParameterBuilder()
    .name("count")
    .range<int64_t>(0, 100);

  auto descriptor = builder.build_descriptor();

  EXPECT_EQ(descriptor.integer_range.size(), 1u);
  EXPECT_EQ(descriptor.integer_range[0].from_value, 0);
  EXPECT_EQ(descriptor.integer_range[0].to_value, 100);
}

TEST_F(ParameterBuilderTest, FluentValidators)
{
  auto builder = rclcpp::ParameterBuilder()
    .name("config")
    .default_value("default")
    .not_empty()
    .matches("^[a-z]+$")
    .length(1, 20);

  auto validators = builder.get_validators();
  EXPECT_EQ(validators.size(), 3u);
}

TEST_F(ParameterBuilderTest, Validation)
{
  auto builder = rclcpp::ParameterBuilder()
    .name("velocity")
    .range(0.0, 10.0);

  EXPECT_TRUE(builder.validate(rclcpp::Parameter("velocity", 5.0)).successful());
  EXPECT_FALSE(builder.validate(rclcpp::Parameter("velocity", 15.0)).successful());
}

TEST_F(ParameterBuilderTest, CustomValidator)
{
  class EvenValidator : public rclcpp::ParameterValidator
  {
  public:
    rclcpp::ValidationResult validate(const rclcpp::Parameter & param) const override
    {
      if (param.as_int() % 2 == 0) {
        return rclcpp::ValidationResult::success();
      }
      return rclcpp::ValidationResult::failure("Value must be even");
    }
    std::string description() const override {return "even integer";}
  };

  auto builder = rclcpp::ParameterBuilder()
    .name("even_number")
    .validator(std::make_shared<EvenValidator>());

  EXPECT_TRUE(builder.validate(rclcpp::Parameter("even_number", 4)).successful());
  EXPECT_FALSE(builder.validate(rclcpp::Parameter("even_number", 3)).successful());
}

TEST_F(ParameterBuilderTest, BuildWithoutName)
{
  rclcpp::ParameterBuilder builder;
  EXPECT_THROW(builder.build(), std::runtime_error);
}

TEST_F(ParameterBuilderTest, GetDefaultValueWithoutSetting)
{
  rclcpp::ParameterBuilder builder;
  builder.name("test");
  EXPECT_THROW(builder.get_default_value(), std::runtime_error);
}

// =============================================================================
// ParameterValidatorRegistry Tests
// =============================================================================

class ParameterValidatorRegistryTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(ParameterValidatorRegistryTest, RegisterAndValidate)
{
  rclcpp::ParameterValidatorRegistry registry;

  registry.register_validators("velocity", {
    std::make_shared<rclcpp::RangeValidator<double>>(0.0, 10.0)
  });

  EXPECT_TRUE(registry.has_validators("velocity"));
  EXPECT_FALSE(registry.has_validators("other"));

  EXPECT_TRUE(registry.validate(rclcpp::Parameter("velocity", 5.0)).successful());
  EXPECT_FALSE(registry.validate(rclcpp::Parameter("velocity", 15.0)).successful());

  // Unregistered parameter always passes
  EXPECT_TRUE(registry.validate(rclcpp::Parameter("other", 100.0)).successful());
}

TEST_F(ParameterValidatorRegistryTest, AddValidator)
{
  rclcpp::ParameterValidatorRegistry registry;

  registry.add_validator("param", std::make_shared<rclcpp::RangeValidator<double>>(0.0, 100.0));
  registry.add_validator("param", std::make_shared<rclcpp::RangeValidator<double>>(50.0, 150.0));

  // Both must pass
  EXPECT_TRUE(registry.validate(rclcpp::Parameter("param", 75.0)).successful());
  EXPECT_FALSE(registry.validate(rclcpp::Parameter("param", 25.0)).successful());
}

TEST_F(ParameterValidatorRegistryTest, ClearValidators)
{
  rclcpp::ParameterValidatorRegistry registry;

  registry.register_validators("param", {
    std::make_shared<rclcpp::RangeValidator<double>>(0.0, 10.0)
  });

  EXPECT_TRUE(registry.has_validators("param"));

  registry.clear_validators("param");

  EXPECT_FALSE(registry.has_validators("param"));
}

TEST_F(ParameterValidatorRegistryTest, ValidateMultiple)
{
  rclcpp::ParameterValidatorRegistry registry;

  registry.register_validators("a", {
    std::make_shared<rclcpp::RangeValidator<double>>(0.0, 10.0)
  });
  registry.register_validators("b", {
    std::make_shared<rclcpp::RangeValidator<double>>(0.0, 100.0)
  });

  std::vector<rclcpp::Parameter> valid_params = {
    rclcpp::Parameter("a", 5.0),
    rclcpp::Parameter("b", 50.0)
  };
  EXPECT_TRUE(registry.validate(valid_params).successful());

  std::vector<rclcpp::Parameter> invalid_params = {
    rclcpp::Parameter("a", 15.0),  // Invalid
    rclcpp::Parameter("b", 50.0)
  };
  EXPECT_FALSE(registry.validate(invalid_params).successful());
}

TEST_F(ParameterValidatorRegistryTest, CreateCallback)
{
  rclcpp::ParameterValidatorRegistry registry;

  registry.register_validators("velocity", {
    std::make_shared<rclcpp::RangeValidator<double>>(0.0, 10.0)
  });

  auto callback = registry.create_callback();

  auto result = callback({rclcpp::Parameter("velocity", 5.0)});
  EXPECT_TRUE(result.successful);

  result = callback({rclcpp::Parameter("velocity", 15.0)});
  EXPECT_FALSE(result.successful);
}

// =============================================================================
// Utility Functions Tests
// =============================================================================

class UtilityFunctionsTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(UtilityFunctionsTest, FormatValidationError)
{
  auto error = rclcpp::format_validation_error(
    "max_velocity",
    "value out of range",
    "value in [0.0, 10.0]",
    "15.5");

  EXPECT_NE(error.find("max_velocity"), std::string::npos);
  EXPECT_NE(error.find("value out of range"), std::string::npos);
  EXPECT_NE(error.find("[0.0, 10.0]"), std::string::npos);
  EXPECT_NE(error.find("15.5"), std::string::npos);
}

TEST_F(UtilityFunctionsTest, ParameterValueToString)
{
  EXPECT_NE(
    rclcpp::parameter_value_to_string(rclcpp::Parameter("p", true)).find("true"),
    std::string::npos);
  EXPECT_NE(
    rclcpp::parameter_value_to_string(rclcpp::Parameter("p", 42)).find("42"),
    std::string::npos);
  EXPECT_NE(
    rclcpp::parameter_value_to_string(rclcpp::Parameter("p", 3.14)).find("3.14"),
    std::string::npos);
  EXPECT_NE(
    rclcpp::parameter_value_to_string(rclcpp::Parameter("p", "hello")).find("hello"),
    std::string::npos);
}

TEST_F(UtilityFunctionsTest, ParameterTypeToString)
{
  EXPECT_EQ(rclcpp::parameter_type_to_string(rclcpp::ParameterType::PARAMETER_BOOL), "bool");
  EXPECT_EQ(rclcpp::parameter_type_to_string(rclcpp::ParameterType::PARAMETER_INTEGER), "integer");
  EXPECT_EQ(rclcpp::parameter_type_to_string(rclcpp::ParameterType::PARAMETER_DOUBLE), "double");
  EXPECT_EQ(rclcpp::parameter_type_to_string(rclcpp::ParameterType::PARAMETER_STRING), "string");
  EXPECT_EQ(rclcpp::parameter_type_to_string(rclcpp::ParameterType::PARAMETER_NOT_SET), "not_set");
}

TEST_F(UtilityFunctionsTest, ValidateParameterFunction)
{
  std::vector<rclcpp::ParameterValidatorPtr> validators = {
    std::make_shared<rclcpp::RangeValidator<double>>(0.0, 100.0),
    std::make_shared<rclcpp::RangeValidator<double>>(50.0, 150.0)
  };

  EXPECT_TRUE(rclcpp::validate_parameter(rclcpp::Parameter("p", 75.0), validators).successful());
  EXPECT_FALSE(rclcpp::validate_parameter(rclcpp::Parameter("p", 25.0), validators).successful());
}

// =============================================================================
// Node Integration Tests
// =============================================================================

class NodeIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_node");
  }

  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::Node> node_;
};

TEST_F(NodeIntegrationTest, DeclareValidatedParameter)
{
  auto param = rclcpp::declare_validated_parameter(
    *node_,
    rclcpp::ParameterBuilder()
      .name("velocity")
      .description("Robot velocity")
      .default_value(1.0)
      .range(0.0, 10.0)
  );

  EXPECT_EQ(param.get_name(), "velocity");
  EXPECT_EQ(param.as_double(), 1.0);

  // Try to set valid value
  auto result = node_->set_parameter(rclcpp::Parameter("velocity", 5.0));
  EXPECT_TRUE(result.successful);

  // Try to set invalid value
  result = node_->set_parameter(rclcpp::Parameter("velocity", 15.0));
  EXPECT_FALSE(result.successful);
}

TEST_F(NodeIntegrationTest, DeclareValidatedParameterInvalidDefault)
{
  EXPECT_THROW(
    rclcpp::declare_validated_parameter(
      *node_,
      rclcpp::ParameterBuilder()
        .name("invalid_default")
        .default_value(100.0)  // Outside range
        .range(0.0, 10.0)
    ),
    rclcpp::exceptions::InvalidParameterValueException
  );
}

TEST_F(NodeIntegrationTest, DeclareMultipleValidatedParameters)
{
  std::vector<rclcpp::ParameterBuilder> builders = {
    rclcpp::ParameterBuilder()
      .name("velocity")
      .default_value(1.0)
      .range(0.0, 10.0),
    rclcpp::ParameterBuilder()
      .name("mode")
      .default_value("normal")
      .one_of<std::string>({"slow", "normal", "fast"})
  };

  auto params = rclcpp::declare_validated_parameters(*node_, builders);

  EXPECT_EQ(params.size(), 2u);
  EXPECT_EQ(params[0].get_name(), "velocity");
  EXPECT_EQ(params[1].get_name(), "mode");

  // Test validation
  EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("velocity", 5.0)).successful);
  EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("velocity", 15.0)).successful);

  EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("mode", "fast")).successful);
  EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("mode", "turbo")).successful);
}

TEST_F(NodeIntegrationTest, CreateValidationCallback)
{
  // Declare parameters normally
  node_->declare_parameter("legacy_param", 5.0);

  // Add validation later
  std::map<std::string, std::vector<rclcpp::ParameterValidatorPtr>> validators;
  validators["legacy_param"] = {
    std::make_shared<rclcpp::RangeValidator<double>>(0.0, 10.0)
  };

  auto handle = rclcpp::create_validation_callback(*node_, validators);

  // Test validation
  EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("legacy_param", 5.0)).successful);
  EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("legacy_param", 15.0)).successful);
}

TEST_F(NodeIntegrationTest, ReadOnlyParameter)
{
  rclcpp::declare_validated_parameter(
    *node_,
    rclcpp::ParameterBuilder()
      .name("constant")
      .default_value(42)
      .read_only(true)
  );

  // Should fail to change read-only parameter
  auto result = node_->set_parameter(rclcpp::Parameter("constant", 100));
  EXPECT_FALSE(result.successful);
}

TEST_F(NodeIntegrationTest, ParameterDescriptorPopulated)
{
  rclcpp::declare_validated_parameter(
    *node_,
    rclcpp::ParameterBuilder()
      .name("documented_param")
      .description("A well documented parameter")
      .default_value(5.0)
      .range(0.0, 10.0)
  );

  auto descriptor = node_->describe_parameter("documented_param");
  EXPECT_EQ(descriptor.description, "A well documented parameter");
  EXPECT_FALSE(descriptor.additional_constraints.empty());
  EXPECT_EQ(descriptor.floating_point_range.size(), 1u);
}

// =============================================================================
// Error Message Tests
// =============================================================================

class ErrorMessageTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(ErrorMessageTest, RangeErrorMessage)
{
  rclcpp::RangeValidator<double> validator(0.0, 100.0);
  auto result = validator.validate(rclcpp::Parameter("max_speed", 150.0));

  EXPECT_FALSE(result.successful());
  EXPECT_NE(result.reason().find("max_speed"), std::string::npos);
  EXPECT_NE(result.reason().find("150"), std::string::npos);
  EXPECT_NE(result.reason().find("above maximum"), std::string::npos);
}

TEST_F(ErrorMessageTest, OneOfErrorMessage)
{
  rclcpp::OneOfValidator<std::string> validator({"a", "b", "c"});
  auto result = validator.validate(rclcpp::Parameter("choice", "invalid"));

  EXPECT_FALSE(result.successful());
  EXPECT_NE(result.reason().find("choice"), std::string::npos);
  EXPECT_NE(result.reason().find("invalid"), std::string::npos);
}

TEST_F(ErrorMessageTest, RegexErrorMessage)
{
  rclcpp::RegexValidator validator("^[a-z]+$", "lowercase letters");
  auto result = validator.validate(rclcpp::Parameter("identifier", "Invalid123"));

  EXPECT_FALSE(result.successful());
  EXPECT_NE(result.reason().find("identifier"), std::string::npos);
  EXPECT_NE(result.reason().find("Invalid123"), std::string::npos);
  EXPECT_NE(result.reason().find("pattern"), std::string::npos);
}

TEST_F(ErrorMessageTest, CompositeAnyModeErrorMessage)
{
  rclcpp::CompositeValidator validator(rclcpp::CompositeValidator::Mode::ANY);
  validator.add(std::make_shared<rclcpp::RangeValidator<double>>(0.0, 10.0));
  validator.add(std::make_shared<rclcpp::RangeValidator<double>>(90.0, 100.0));

  auto result = validator.validate(rclcpp::Parameter("value", 50.0));

  EXPECT_FALSE(result.successful());
  EXPECT_NE(result.reason().find("none of the validators"), std::string::npos);
}

// =============================================================================
// Main
// =============================================================================

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
