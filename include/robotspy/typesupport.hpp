// (c) 2023 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.
#ifndef ROBOTSPY__TYPESUPPORT_HPP_
#define ROBOTSPY__TYPESUPPORT_HPP_

#include <string>
#include <tuple>
#include <memory>
#include <sstream>
#include <regex>

#include "rcpputils/shared_library.hpp"
#include "rosidl_runtime_c/message_type_support_struct.h"

namespace robotspy
{
class InvalidTopicNameException : public std::exception
{
public:
  InvalidTopicNameException(const std::string & msg)
  : msg_(msg) {}
  const char * what()
  {
    return msg_.c_str();
  }

private:
  std::string msg_;
};

std::string
normalize_dds_type_name(const std::string & type_fqname);

const std::pair<bool, bool>
is_type_requestreply(const std::string & type_fqname);

const std::tuple<std::string, std::string, std::string>
parse_ros_type_name(const std::string & type_fqname);

void
get_library_path(std::vector<std::string> & library_path);

std::tuple<
  bool,
  std::shared_ptr<rcpputils::SharedLibrary>,
  const rosidl_message_type_support_t *>
load_instrospection_typesupport_library(
  const std::string & package_name,
  const std::string & middle_module,
  const std::string & type_name,
  const std::vector<std::string> & library_path = std::vector<std::string>());

const rosidl_message_type_support_t *
lookup_introspection_typesupport(
  const std::string & package_name,
  const std::string & middle_module,
  const std::string & type_name,
  rcpputils::SharedLibrary & typesupport_lib,
  const bool cpp_version = false);

std::pair<bool, const rosidl_message_type_support_t *>
get_nested_introspection_typesupport(
  const rosidl_message_type_support_t * const input_typesupport);

inline
std::string
create_dds_type_name(
  const char * const message_namespace,
  const char * const message_name,
  const char * const message_suffix,
  const bool mangle_prefix)
{
  const char * prefix_sfx = "";
  if (mangle_prefix) {
    prefix_sfx = "_";
  }

  std::ostringstream ss;
  std::string msg_namespace =
    std::regex_replace(message_namespace, std::regex("__"), "::");
  if (!msg_namespace.empty()) {
    ss << msg_namespace << "::";
  }
  ss << "dds" << prefix_sfx << "::" << message_name << message_suffix;
  return ss.str();
}

template<typename TypeMembers>
std::string
create_dds_type_name_from_members(
  TypeMembers * const members,
  const bool mangle_names)
{
  const char * msg_prefix = "";
  const bool mangle_prefix = mangle_names;
  if (mangle_names) {
    msg_prefix = "_";
  }
  return create_dds_type_name(
    members->message_namespace_,
    members->message_name_,
    msg_prefix,
    mangle_prefix);
}

inline
std::string
demangle_dds_type_name(const std::string & dds_type_name)
{
  // Check if the name is already in "canonical" ROS 2 form
  // i.e. <package>::(msg|srv)::<type>
  size_t ns_count = 0;
  size_t ns_sep_pos = dds_type_name.find("::");
  size_t first_sep_pos = std::string::npos;
  while (ns_sep_pos != std::string::npos) {
    if (ns_count == 0) {
      first_sep_pos = ns_sep_pos;
    }
    ns_count += 1;
    ns_sep_pos = dds_type_name.find("::", ns_sep_pos + 2);
  }
  if (ns_count == 2
      && (0 == strncmp(dds_type_name.c_str() + first_sep_pos, "::msg::", 7)
      || 0 == strncmp(dds_type_name.c_str() + first_sep_pos, "::srv::", 7)))
  {
    return std::regex_replace(dds_type_name, std::regex("::"), "/");
  }

  size_t dds_prefix_len = 8;
  auto dds_prefix_pos = dds_type_name.rfind("::dds_::");
  if (dds_prefix_pos == std::string::npos) {
    dds_prefix_len -= 1;
    dds_prefix_pos = dds_type_name.rfind("::dds::");
    if (dds_prefix_pos == std::string::npos) {
      throw InvalidTopicNameException(
        std::string("invalid ROS 2 DDS type name 1: ") + dds_type_name);
    }
  }
  // Check that there aren't more "::" after "dds[_]::"
  if (dds_type_name.rfind("::") != (dds_prefix_pos + dds_prefix_len - 2)) {
    throw InvalidTopicNameException(
        std::string("invalid ROS 2 DDS type name 2: ") + dds_type_name);
  }

  auto type_name = dds_type_name.substr(dds_prefix_pos + dds_prefix_len);
  if (type_name[type_name.size() - 1] == '_') {
    type_name = type_name.substr(0, type_name.size() - 1);
  }

  std::ostringstream ss;
  size_t search_start = 0;
  size_t sep_pos;
  do {
    sep_pos = dds_type_name.find("::", search_start);
    if (search_start > 0) {
      ss << "/";
    }
    auto el = dds_type_name.substr(search_start, sep_pos - search_start);
    ss << el;
    search_start += el.size() + 2;
    sep_pos = dds_type_name.find("::", search_start);
  } while (sep_pos <= dds_prefix_pos);
  ss << "/" << type_name;
  return ss.str();
}
}  // namespace robotspy

#endif  // ROBOTSPY__TYPESUPPORT_HPP_
