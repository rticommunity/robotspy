// (c) 2023 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.
#include <string>
#include <regex>

#include "robotspy/typesupport.hpp"

#include "ament_index_cpp/get_resources.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"
#include "rcutils/error_handling.h"
#include "rcutils/env.h"
#include "rcpputils/shared_library.hpp"

#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"
#include "rosidl_typesupport_introspection_c/visibility_control.h"

namespace robotspy
{
std::string
normalize_dds_type_name(const std::string & type_fqname)
{
  const static std::regex double_underscore_re("__");
  const static std::regex trail_underscore_re("_$");
  const static std::regex dds_ns_re("::dds_::");
  const static std::regex dds_ns_rm_re("::dds::");
  if (type_fqname.size() == 0) {
    throw std::runtime_error("empty type name");
  }
  return
    std::regex_replace(
      std::regex_replace(
        std::regex_replace(
          std::regex_replace(
            type_fqname,
            double_underscore_re, "::"),
          trail_underscore_re, ""),
        dds_ns_re, "::dds::"),
      dds_ns_rm_re, "::");
}

const std::pair<bool, bool>
is_type_requestreply(const std::string & type_fqname)
{
  size_t sfx_len = 8;
  auto sfx_pos = type_fqname.rfind("Request_");
  if (sfx_pos == std::string::npos) {
    sfx_len = 7;
    sfx_pos = type_fqname.rfind("Request");
  }
  if (sfx_pos == type_fqname.size() - sfx_len) {
    return {true, true};
  }

  sfx_len = 9;
  sfx_pos = type_fqname.rfind("Response_");
  if (sfx_pos == std::string::npos) {
    sfx_len = 8;
    sfx_pos = type_fqname.rfind("Response");
  }
  if (sfx_pos == type_fqname.size() - sfx_len) {
    return {true, false};
  }

  return {false, false};
}

const std::tuple<std::string, std::string, std::string>
parse_ros_type_name(const std::string & type_fqname)
{
  const static std::regex double_underscore_re("__");
  const static std::regex trail_underscore_re("_$");
  std::string norm_fqname =
    std::regex_replace(
    std::regex_replace(type_fqname, double_underscore_re, "/"),
    trail_underscore_re, "");
  static const char type_separator = '/';
  auto sep_position_back = norm_fqname.find_last_of(type_separator);
  auto sep_position_front = norm_fqname.find_first_of(type_separator);
  if (sep_position_back == std::string::npos ||
    sep_position_back == 0 ||
    sep_position_back == norm_fqname.length() - 1)
  {
    throw InvalidTopicNameException(
      std::string("invalid ROS 2 type name: ") + type_fqname);
  }

  std::string package_name = type_fqname.substr(0, sep_position_front);
  std::string middle_module = "";
  if (sep_position_back - sep_position_front > 0) {
    middle_module =
      type_fqname.substr(sep_position_front + 1, sep_position_back - sep_position_front - 1);
  }
  std::string type_name = type_fqname.substr(sep_position_back + 1);

  return std::make_tuple(package_name, middle_module, type_name);
}

void
get_library_path(std::vector<std::string> & library_path)
{
#ifdef _WIN32
  static const std::string env_var = "PATH";
  static const std::string path_sep = ";";
#elif __APPLE__
  static const std::string env_var = "DYLD_LIBRARY_PATH";
  static const std::string path_sep = ":";
#else
  static const std::string env_var = "LD_LIBRARY_PATH";
  static const std::string path_sep = ":";
#endif
  const char * env_value_c = nullptr;
  const char * lookup_rc = rcutils_get_env(env_var.c_str(), &env_value_c);
  if (nullptr != lookup_rc || nullptr == env_value_c) {
    throw std::runtime_error("failed to lookup library path from environment");
  }
  if (env_value_c[0] == '\0') {
    return;
  }
  size_t consumed = 0;
  std::string env_value = env_value_c;
  while (consumed < env_value.size()) {
    auto next_sep_pos = env_value.find(path_sep, consumed);
    if (next_sep_pos == std::string::npos) {
      next_sep_pos = env_value.size();
    }
    auto next_len = next_sep_pos - consumed;
    library_path.emplace_back(env_value.substr(consumed, next_len));
    consumed += next_len + 1;
  }
}

const rosidl_message_type_support_t *
lookup_introspection_typesupport(
  const std::string & package_name,
  const std::string & middle_module,
  const std::string & type_name,
  rcpputils::SharedLibrary & typesupport_lib,
  const bool cpp_version)
{
  std::string typesupport_identifier = (cpp_version) ?
    "rosidl_typesupport_introspection_cpp" :
    "rosidl_typesupport_introspection_c";

  auto symbol_name = typesupport_identifier + "__get_message_type_support_handle__" +
    package_name + "__" + (middle_module.empty() ? "msg" : middle_module) + "__" + type_name;

  if (!typesupport_lib.get_symbol(symbol_name)) {
    throw std::runtime_error("symbol not found");
  }

  const rosidl_message_type_support_t * (* get_ts)() = nullptr;
  get_ts = (decltype(get_ts))typesupport_lib.get_symbol(symbol_name);

  if (!get_ts) {
    throw std::runtime_error("failed to cast typesupport function");
  }
  auto type_support = get_ts();
  return type_support;
}

std::tuple<
  bool,
  std::shared_ptr<rcpputils::SharedLibrary>,
  const rosidl_message_type_support_t *>
load_instrospection_typesupport_library(
  const std::string & package_name,
  const std::string & middle_module,
  const std::string & type_name,
  const std::vector<std::string> & library_path)
{
#ifdef _WIN32
  static const std::string dynamic_library_folder = "/bin/";
  static const std::string filename_prefix = "";
  static const std::string filename_extension = ".dll";
#elif __APPLE__
  static const std::string filename_prefix = "lib";
  static const std::string filename_extension = ".dylib";
  static const std::string dynamic_library_folder = "/lib/";
#else
  static const std::string filename_prefix = "lib";
  static const std::string filename_extension = ".so";
  static const std::string dynamic_library_folder = "/lib/";
#endif
  static const std::vector<std::string> intro_langs = {"c", "cpp"};

  std::vector<std::string> package_prefixes;

  std::ostringstream ss;
  try {
    package_prefixes.emplace_back(
      ament_index_cpp::get_package_prefix(package_name));
  } catch (ament_index_cpp::PackageNotFoundError & e) {
    // TODO log ?
  }
  package_prefixes.insert(
    std::end(package_prefixes), std::begin(library_path), std::end(library_path));
  if (package_prefixes.size() == 0) {
    throw std::runtime_error("no directory in library search path");
  }
  for (const auto & pkg_prefix : package_prefixes) {
    for (const auto & intro_lang: intro_langs) {
      ss.clear();
      ss << pkg_prefix << dynamic_library_folder <<
        filename_prefix << package_name << "__" <<
        "rosidl_typesupport_introspection" << "_" << intro_lang <<
        filename_extension;
      try {
        std::shared_ptr<rcpputils::SharedLibrary> shared_lib =
          std::make_shared<rcpputils::SharedLibrary>(ss.str());
        bool shared_lib_cpp = intro_lang == "cpp";
        auto type_support = lookup_introspection_typesupport(
          package_name, middle_module, type_name, *shared_lib, shared_lib_cpp);
        return std::make_tuple(shared_lib_cpp, shared_lib, type_support);
      } catch (std::exception & e) {
        // TODO(asorbini) log exception ?
      }
    }
  }
  throw std::runtime_error("failed to load typesupport");
}

std::pair<bool, const rosidl_message_type_support_t *>
get_nested_introspection_typesupport(
  const rosidl_message_type_support_t * const input_typesupport)
{
  bool cpp_version = false;
  const rosidl_message_type_support_t * type_support =
    get_message_typesupport_handle(
    input_typesupport, rosidl_typesupport_introspection_c__identifier);
  if (nullptr == type_support) {
    // Reset error string since this is not (yet) an error
    // (see https://github.com/ros2/rosidl_typesupport/pull/102)
    rcutils_error_string_t prev_error_string = rcutils_get_error_string();
    (void)prev_error_string;
    rcutils_reset_error();

    type_support =
      get_message_typesupport_handle(
      input_typesupport,
      rosidl_typesupport_introspection_cpp::typesupport_identifier);
    if (nullptr != type_support) {
      cpp_version = true;
    } else {
      rcutils_error_string_t error_string = rcutils_get_error_string();
      rcutils_reset_error();
      (void)error_string;
      throw std::runtime_error("failed to look up introspection type support");
    }
  } else {
    cpp_version = false;
  }
  return {cpp_version, type_support};
}
}  // namespace robotspy
