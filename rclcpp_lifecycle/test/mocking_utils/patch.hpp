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

// Original file taken from:
// https://github.com/ros2/rcutils/blob/master/test/mocking_utils/patch.hpp

#ifndef MOCKING_UTILS__PATCH_HPP_
#define MOCKING_UTILS__PATCH_HPP_

#define MOCKING_UTILS_SUPPORT_VA_LIST
#if (defined(__aarch64__) || defined(__arm__) || defined(_M_ARM) || defined(__thumb__))
// In ARM machines, va_list does not define comparison operators
// nor the compiler allows defining them via operator overloads.
// Thus, Mimick argument matching code will not compile.
#undef MOCKING_UTILS_SUPPORT_VA_LIST
#endif

#ifdef MOCKING_UTILS_SUPPORT_VA_LIST
#include <cstdarg>
#endif

#include <functional>
#include <string>
#include <type_traits>
#include <utility>

#include "mimick/mimick.h"

#include "rcutils/error_handling.h"
#include "rcutils/macros.h"

namespace mocking_utils
{

/// Mimick specific traits for each mocking_utils::Patch instance.
/**
 * \tparam ID Numerical identifier of the patch. Ought to be unique.
 * \tparam SignatureT Type of the symbol to be patched.
*/
template<size_t ID, typename SignatureT>
struct PatchTraits;

/// Traits specialization for ReturnT(void) free functions.
/**
 * \tparam ID Numerical identifier of the patch. Ought to be unique.
 * \tparam ReturnT Return value type.
 */
template<size_t ID, typename ReturnT>
struct PatchTraits<ID, ReturnT(void)>
{
  mmk_mock_define(mock_type, ReturnT);
};

/// Traits specialization for void(void) free functions.
/**
 * Necessary for Mimick macros to adjust accordingly when the return
 * type is `void`.
 *
 * \tparam ID Numerical identifier of the patch. Ought to be unique.
 */
template<size_t ID>
struct PatchTraits<ID, void(void)>
{
  mmk_mock_define(mock_type, void);
};

/// Traits specialization for ReturnT(ArgT0) free functions.
/**
 * \tparam ID Numerical identifier of the patch. Ought to be unique.
 * \tparam ReturnT Return value type.
 * \tparam ArgT0 Argument type.
 */
template<size_t ID, typename ReturnT, typename ArgT0>
struct PatchTraits<ID, ReturnT(ArgT0)>
{
  mmk_mock_define(mock_type, ReturnT, ArgT0);
};

/// Traits specialization for void(ArgT0) free functions.
/**
 * Necessary for Mimick macros to adjust accordingly when the return
 * type is `void`.
 *
 * \tparam ID Numerical identifier of the patch. Ought to be unique.
 * \tparam ArgT0 Argument type.
 */
template<size_t ID, typename ArgT0>
struct PatchTraits<ID, void(ArgT0)>
{
  mmk_mock_define(mock_type, void, ArgT0);
};

/// Traits specialization for ReturnT(ArgT0, ArgT1) free functions.
/**
 * \tparam ID Numerical identifier of the patch. Ought to be unique.
 * \tparam ReturnT Return value type.
 * \tparam ArgTx Argument types.
 */
template<size_t ID, typename ReturnT,
  typename ArgT0, typename ArgT1>
struct PatchTraits<ID, ReturnT(ArgT0, ArgT1)>
{
  mmk_mock_define(mock_type, ReturnT, ArgT0, ArgT1);
};

/// Traits specialization for void(ArgT0, ArgT1) free functions.
/**
 * Necessary for Mimick macros to adjust accordingly when the return
 * type is `void`.
 *
 * \tparam ID Numerical identifier of the patch. Ought to be unique.
 * \tparam ArgTx Argument types.
 */
template<size_t ID, typename ArgT0, typename ArgT1>
struct PatchTraits<ID, void(ArgT0, ArgT1)>
{
  mmk_mock_define(mock_type, void, ArgT0, ArgT1);
};

/// Traits specialization for ReturnT(ArgT0, ArgT1, ArgT2) free functions.
/**
 * \tparam ID Numerical identifier of the patch. Ought to be unique.
 * \tparam ReturnT Return value type.
 * \tparam ArgTx Argument types.
 */
template<size_t ID, typename ReturnT,
  typename ArgT0, typename ArgT1, typename ArgT2>
struct PatchTraits<ID, ReturnT(ArgT0, ArgT1, ArgT2)>
{
  mmk_mock_define(mock_type, ReturnT, ArgT0, ArgT1, ArgT2);
};

/// Traits specialization for void(ArgT0, ArgT1, ArgT2) free functions.
/**
 * Necessary for Mimick macros to adjust accordingly when the return
 * type is `void`.
 *
 * \tparam ID Numerical identifier of the patch. Ought to be unique.
 * \tparam ArgTx Argument types.
 */
template<size_t ID, typename ArgT0, typename ArgT1, typename ArgT2>
struct PatchTraits<ID, void(ArgT0, ArgT1, ArgT2)>
{
  mmk_mock_define(mock_type, void, ArgT0, ArgT1, ArgT2);
};

/// Traits specialization for ReturnT(ArgT0, ArgT1, ArgT2, ArgT3) free functions.
/**
 * \tparam ID Numerical identifier of the patch. Ought to be unique.
 * \tparam ReturnT Return value type.
 * \tparam ArgTx Argument types.
 */
template<size_t ID, typename ReturnT,
  typename ArgT0, typename ArgT1,
  typename ArgT2, typename ArgT3>
struct PatchTraits<ID, ReturnT(ArgT0, ArgT1, ArgT2, ArgT3)>
{
  mmk_mock_define(mock_type, ReturnT, ArgT0, ArgT1, ArgT2, ArgT3);
};

/// Traits specialization for void(ArgT0, ArgT1, ArgT2, ArgT3) free functions.
/**
 * Necessary for Mimick macros to adjust accordingly when the return
 * type is `void`.
 *
 * \tparam ID Numerical identifier of the patch. Ought to be unique.
 * \tparam ArgTx Argument types.
 */
template<size_t ID,
  typename ArgT0, typename ArgT1,
  typename ArgT2, typename ArgT3>
struct PatchTraits<ID, void(ArgT0, ArgT1, ArgT2, ArgT3)>
{
  mmk_mock_define(mock_type, void, ArgT0, ArgT1, ArgT2, ArgT3);
};

/// Traits specialization for ReturnT(ArgT0, ArgT1, ArgT2, ArgT3, ArgT4)
/// free functions.
/**
 * \tparam ID Numerical identifier of the patch. Ought to be unique.
 * \tparam ReturnT Return value type.
 * \tparam ArgTx Argument types.
 */
template<size_t ID, typename ReturnT,
  typename ArgT0, typename ArgT1,
  typename ArgT2, typename ArgT3, typename ArgT4>
struct PatchTraits<ID, ReturnT(ArgT0, ArgT1, ArgT2, ArgT3, ArgT4)>
{
  mmk_mock_define(mock_type, ReturnT, ArgT0, ArgT1, ArgT2, ArgT3, ArgT4);
};

/// Traits specialization for void(ArgT0, ArgT1, ArgT2, ArgT3, ArgT4)
/// free functions.
/**
 * Necessary for Mimick macros to adjust accordingly when the return
 * type is `void`.
 *
 * \tparam ID Numerical identifier of the patch. Ought to be unique.
 * \tparam ArgTx Argument types.
 */
template<size_t ID,
  typename ArgT0, typename ArgT1,
  typename ArgT2, typename ArgT3, typename ArgT4>
struct PatchTraits<ID, void(ArgT0, ArgT1, ArgT2, ArgT3, ArgT4)>
{
  mmk_mock_define(mock_type, void, ArgT0, ArgT1, ArgT2, ArgT3, ArgT4);
};

/// Traits specialization for ReturnT(ArgT0, ArgT1, ArgT2, ArgT3, ArgT4, ArgT5)
/// free functions.
/**
 * \tparam ID Numerical identifier of the patch. Ought to be unique.
 * \tparam ReturnT Return value type.
 * \tparam ArgTx Argument types.
 */
template<size_t ID, typename ReturnT,
  typename ArgT0, typename ArgT1,
  typename ArgT2, typename ArgT3,
  typename ArgT4, typename ArgT5>
struct PatchTraits<ID, ReturnT(ArgT0, ArgT1, ArgT2, ArgT3, ArgT4, ArgT5)>
{
  mmk_mock_define(
    mock_type, ReturnT, ArgT0, ArgT1, ArgT2, ArgT3, ArgT4, ArgT5);
};

/// Traits specialization for void(ArgT0, ArgT1, ArgT2, ArgT3, ArgT4, ArgT5)
/// free functions.
/**
 * Necessary for Mimick macros to adjust accordingly when the return
 * type is `void`.
 *
 * \tparam ID Numerical identifier of the patch. Ought to be unique.
 * \tparam ArgTx Argument types.
 */
template<size_t ID,
  typename ArgT0, typename ArgT1,
  typename ArgT2, typename ArgT3,
  typename ArgT4, typename ArgT5>
struct PatchTraits<ID, void(ArgT0, ArgT1, ArgT2, ArgT3, ArgT4, ArgT5)>
{
  mmk_mock_define(
    mock_type, void, ArgT0, ArgT1, ArgT2, ArgT3, ArgT4, ArgT5);
};

/// Traits specialization for
/// ReturnT(ArgT0, ArgT1, ArgT2, ArgT3, ArgT4, ArgT5, ArgT6, ArgT7, ArgT8)
/// free functions.
/**
 * Necessary for Mimick macros to adjust accordingly when the return
 * type is `void`.
 *
 * \tparam ID Numerical identifier of the patch. Ought to be unique.
 * \tparam ArgTx Argument types.
 */
template<size_t ID, typename ReturnT,
  typename ArgT0, typename ArgT1,
  typename ArgT2, typename ArgT3,
  typename ArgT4, typename ArgT5,
  typename ArgT6, typename ArgT7,
  typename ArgT8>
struct PatchTraits<ID, ReturnT(
    ArgT0, ArgT1, ArgT2, ArgT3, ArgT4, ArgT5, ArgT6, ArgT7, ArgT8)>
{
  mmk_mock_define(
    mock_type, ReturnT, ArgT0, ArgT1, ArgT2, ArgT3, ArgT4, ArgT5, ArgT6, ArgT7, ArgT8);
};

/// Traits specialization for void(ArgT0, ArgT1, ArgT2, ArgT3, ArgT4, ArgT5, ArgT6, ArgT7)
/// free functions.
/**
 * Necessary for Mimick macros to adjust accordingly when the return
 * type is `void`.
 *
 * \tparam ID Numerical identifier of the patch. Ought to be unique.
 * \tparam ArgTx Argument types.
 */
template<size_t ID, typename ReturnT,
  typename ArgT0, typename ArgT1,
  typename ArgT2, typename ArgT3,
  typename ArgT4, typename ArgT5,
  typename ArgT6, typename ArgT7,
  typename ArgT8, typename ArgT9>
struct PatchTraits<ID, ReturnT(
    ArgT0, ArgT1, ArgT2, ArgT3, ArgT4, ArgT5, ArgT6, ArgT7, ArgT8, ArgT9)>
{
  mmk_mock_define(
    mock_type, ReturnT, ArgT0, ArgT1, ArgT2, ArgT3, ArgT4, ArgT5, ArgT6, ArgT7, ArgT8, ArgT9);
};

/// Generic trampoline to wrap generalized callables in plain functions.
/**
 * \tparam ID Numerical identifier of this trampoline. Ought to be unique.
 * \tparam SignatureT Type of the symbol this trampoline replaces.
 */
template<size_t ID, typename SignatureT>
struct Trampoline;

/// Trampoline specialization for free functions.
template<size_t ID, typename ReturnT, typename ... ArgTs>
struct Trampoline<ID, ReturnT(ArgTs...)>
{
  static ReturnT base(ArgTs... args)
  {
    return target(std::forward<ArgTs>(args)...);
  }

  static std::function<ReturnT(ArgTs...)> target;
};

template<size_t ID, typename ReturnT, typename ... ArgTs>
std::function<ReturnT(ArgTs...)>
Trampoline<ID, ReturnT(ArgTs...)>::target;

/// Setup trampoline with the given @p target.
/**
 * \param[in] target Callable that this trampoline will target.
 * \return the plain base function of this trampoline.
 *
 * \tparam ID Numerical identifier of this trampoline. Ought to be unique.
 * \tparam SignatureT Type of the symbol this trampoline replaces.
 */
template<size_t ID, typename SignatureT>
auto prepare_trampoline(std::function<SignatureT> target)
{
  Trampoline<ID, SignatureT>::target = target;
  return Trampoline<ID, SignatureT>::base;
}

/// Patch class for binary API mocking
/**
 * Built on top of Mimick, to enable symbol mocking on a per dynamically
 * linked binary object basis.
 *
 * \tparam ID Numerical identifier for this patch. Ought to be unique.
 * \tparam SignatureT Type of the symbol to be patched.
 */
template<size_t ID, typename SignatureT>
class Patch;

/// Patch specialization for ReturnT(ArgTs...) free functions.
/**
 * \tparam ID Numerical identifier for this patch. Ought to be unique.
 * \tparam ReturnT Return value type.
 * \tparam ArgTs Argument types.
 */
template<size_t ID, typename ReturnT, typename ... ArgTs>
class Patch<ID, ReturnT(ArgTs...)>
{
public:
  using mock_type = typename PatchTraits<ID, ReturnT(ArgTs...)>::mock_type;

  /// Construct a patch.
  /**
   * \param[in] target Symbol target string, using Mimick syntax
   *   i.e. "symbol(@scope)?", where scope may be "self" to target the current
   *   binary, "lib:library_name" to target a given library, "file:path/to/library"
   *   to target a given file, or "sym:other_symbol" to target the first library
   *   that defines said symbol.
   * \param[in] proxy An indirection to call the target function.
   *   This indirection must ensure this call goes through the function's
   *   trampoline, as setup by the dynamic linker.
   * \return a mocking_utils::Patch instance.
   */
  explicit Patch(const std::string & target, std::function<ReturnT(ArgTs...)> proxy)
  : target_(target), proxy_(proxy)
  {
  }

  // Copy construction and assignment are disabled.
  Patch(const Patch &) = delete;
  Patch & operator=(const Patch &) = delete;

  Patch(Patch && other)
  {
    mock_ = other.mock_;
    other.mock_ = nullptr;
  }

  Patch & operator=(Patch && other)
  {
    if (mock_) {
      mmk_reset(mock_);
    }
    mock_ = other.mock_;
    other.mock_ = nullptr;
  }

  ~Patch()
  {
    if (mock_) {
      mmk_reset(mock_);
    }
  }

  /// Inject a @p replacement for the patched function.
  Patch & then_call(std::function<ReturnT(ArgTs...)> replacement) &
  {
    replace_with(replacement);
    return *this;
  }

  /// Inject a @p replacement for the patched function.
  Patch && then_call(std::function<ReturnT(ArgTs...)> replacement) &&
  {
    replace_with(replacement);
    return std::move(*this);
  }

private:
  // Helper for template parameter pack expansion using `mmk_any`
  // macro as pattern.
  template<typename T>
  T any() {return mmk_any(T);}

  void replace_with(std::function<ReturnT(ArgTs...)> replacement)
  {
    if (mock_) {
      throw std::logic_error("Cannot configure patch more than once");
    }
    auto type_erased_trampoline =
      reinterpret_cast<mmk_fn>(prepare_trampoline<ID>(replacement));
    auto MMK_MANGLE(mock_type, create) =
      PatchTraits<ID, ReturnT(ArgTs...)>::MMK_MANGLE(mock_type, create);
    mock_ = mmk_mock(target_.c_str(), mock_type);
    mmk_when(proxy_(any<ArgTs>()...), .then_call = type_erased_trampoline);
  }

  mock_type mock_{nullptr};
  std::string target_;
  std::function<ReturnT(ArgTs...)> proxy_;
};

/// Make a patch for a `target` function.
/**
 * Useful for type deduction during \ref mocking_utils::Patch construction.
 *
 * \param[in] target Symbol target string, using Mimick syntax.
 * \param[in] proxy An indirection to call the target function.
 * \return a mocking_utils::Patch instance.
 *
 * \tparam ID Numerical identifier for this patch. Ought to be unique.
 * \tparam SignatureT Type of the function to be patched.
 *
 * \sa mocking_utils::Patch for further reference.
 */
template<size_t ID, typename SignatureT>
auto make_patch(const std::string & target, std::function<SignatureT> proxy)
{
  return Patch<ID, SignatureT>(target, proxy);
}

/// Define a dummy operator `op` for a given `type`.
/**
 * Useful to enable patching functions that take arguments whose types
 * do not define basic comparison operators, as required by Mimick.
*/
#define MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(type_, op) \
  template<typename T> \
  typename std::enable_if<std::is_same<T, type_>::value, bool>::type \
  operator op(const T &, const T &) { \
    return false; \
  }

/// Get the exact \ref mocking_utils::Patch type for a given `id` and `function`.
/**
 * Useful to avoid ignored attribute warnings when using the \b decltype operator.
 */
#define MOCKING_UTILS_PATCH_TYPE(id, function) \
  decltype(mocking_utils::make_patch<id, decltype(function)>("", nullptr))

/// A transparent forwarding proxy to a given `function`.
/**
 * Useful to ensure a call to `function` goes through its trampoline.
 */
#define MOCKING_UTILS_PATCH_PROXY(function) \
  [] (auto && ... args)->decltype(auto) { \
    return function(std::forward<decltype(args)>(args)...); \
  }

/// Compute a Mimick symbol target string based on which `function` is to be patched
/// in which `scope`.
#define MOCKING_UTILS_PATCH_TARGET(scope, function) \
  (std::string(RCUTILS_STRINGIFY(function)) + "@" + (scope))

/// Prepare a mocking_utils::Patch for patching a `function` in a given `scope`
/// but defer applying any changes.
#define prepare_patch(scope, function) \
  make_patch<__COUNTER__, decltype(function)>( \
    MOCKING_UTILS_PATCH_TARGET(scope, function), MOCKING_UTILS_PATCH_PROXY(function) \
  )

/// Patch a `function` with a used-provided `replacement` in a given `scope`.
#define patch(scope, function, replacement) \
  prepare_patch(scope, function).then_call(replacement)

/// Patch a `function` to always yield a given `return_code` in a given `scope`.
#define patch_and_return(scope, function, return_code) \
  patch(scope, function, [&](auto && ...) {return return_code;})

/// Patch a `function` to always yield a given `return_code` in a given `scope`.
#define patch_to_fail(scope, function, error_message, return_code) \
  patch( \
    scope, function, [&](auto && ...) { \
      RCUTILS_SET_ERROR_MSG(error_message); \
      return return_code; \
    })

/// Patch a `function` to execute normally but always yield a given `return_code`
/// in a given `scope`.
/**
 * \warning On some Linux distributions (e.g. CentOS), pointers to function
 *   reference their PLT trampolines. In such cases, it is not possible to
 *   call `function` from within the mock.
 */
#define inject_on_return(scope, function, return_code) \
  patch( \
    scope, function, ([&, base = function](auto && ... __args) { \
      if (base != function) { \
        static_cast<void>(base(std::forward<decltype(__args)>(__args)...)); \
      } else { \
        RCUTILS_SAFE_FWRITE_TO_STDERR( \
          "[WARNING] mocking_utils::inject_on_return() cannot forward call to " \
          "original '" RCUTILS_STRINGIFY(function) "' function before injection\n" \
          "    at " __FILE__ ":" RCUTILS_STRINGIFY(__LINE__) "\n"); \
      } \
      return return_code; \
    }))

}  // namespace mocking_utils

#ifdef MOCKING_UTILS_SUPPORT_VA_LIST
// Define dummy comparison operators for C standard va_list type
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(va_list, ==)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(va_list, !=)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(va_list, <)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(va_list, >)
#endif

#endif  // MOCKING_UTILS__PATCH_HPP_
