// (c) 2023 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.
#ifndef ROBOTSPY__TYPECACHE_HPP_
#define ROBOTSPY__TYPECACHE_HPP_

#include <string>
#include <map>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>
#include <ostream>

#include "ndds/ndds_c.h"

#include "rcutils/types.h"
#include "rcutils/logging_macros.h"
#include "rcpputils/scope_exit.hpp"

#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"

#include "rcpputils/shared_library.hpp"

#include "robotspy/typecodes.hpp"
#include "robotspy/typesupport.hpp"

namespace robotspy
{
enum class RequestReplyMapping
{
  Basic,
  Extended
};

inline
RequestReplyMapping
request_reply_mapping_from_string(const std::string & mapping)
{
  std::string lowercase = mapping;
  std::transform(
    lowercase.begin(), lowercase.end(), lowercase.begin(),
    [](const unsigned char c) {return std::tolower(c);});
  if (lowercase == "basic" || lowercase == "b") {
    return RequestReplyMapping::Basic;
  } else if (lowercase == "extended" || lowercase == "e") {
    return RequestReplyMapping::Extended;
  } else {
    throw std::runtime_error("invalid request/reply mapping");
  }
}

struct TypeCacheOptions
{
  bool demangle_ros_names{true};
  bool cyclone_compatible{false};
  bool legacy_rmw_compatible{false};
  RequestReplyMapping request_reply_mapping{RequestReplyMapping::Extended};
};

typedef std::string (*TypeCodeMakeNameFn)(const std::string & base_name);

class TypeCache
{
public:
  TypeCache()
  : TypeCache(TypeCacheOptions())
  {}

  explicit TypeCache(const TypeCacheOptions & options);

  virtual ~TypeCache();

  std::tuple<bool, std::vector<const DDS_TypeCode *>, std::vector<const DDS_TypeCode *>>
  assert_dds_type(
    const DDS_TypeCode * const tc,
    const bool ros_type = true,
    const std::string & demangled_ros_type = std::string());

  std::tuple<bool, std::vector<const DDS_TypeCode *>, std::vector<const DDS_TypeCode *>>
  assert_ros_type(const std::string & type_fqname);

  std::tuple<bool, bool, std::vector<const DDS_TypeCode *>, std::vector<const DDS_TypeCode *>>
  assert_dds_topic(
    const std::string & topic_name,
    const DDS_TypeCode * const tc,
    const bool ros_type = true,
    const std::string & demangled_ros_type = std::string());

  std::tuple<bool, bool, std::vector<const DDS_TypeCode *>, std::vector<const DDS_TypeCode *>>
  assert_ros_topic(
    const std::string & topic_name,
    const std::string & type_fqname);

  std::string
  to_idl();

  const std::vector<DDS_TypeCode *> &
  cache() const
  {
    return tc_cache_;
  }

protected:
  static const DDS_Long LENGTH_UNBOUND = RTIXCdrLong_MAX;

  bool
  insert_topic(const std::string & topic_name, const std::string & type_fqname);

  const DDS_TypeCode *
  find(const std::string & type_fqname, const bool ros_type);

  void
  insert(const std::string & type_fqname, DDS_TypeCode * const typecode, const bool ros_type);

  void
  insert(DDS_TypeCode * const typecode);

  void
  clear(const bool nothrow = false);

  void
  unload();

  std::pair<bool, const rosidl_message_type_support_t *>
  load_typesupport(const std::string & type_fqname);

  std::tuple<bool, std::vector<const DDS_TypeCode *>, std::vector<const DDS_TypeCode *>>
  assert_typecode(
    const DDS_TypeCode * const tc,
    const bool ros_type = true,
    const std::string & demangled_ros_type = std::string());

  bool
  assert_typecode(
    const std::string & type_fqname,
    const bool request_reply,
    const bool is_request,
    const bool cpp_version,
    const rosidl_message_type_support_t * const type_support_intro,
    std::vector<const DDS_TypeCode *> & new_asserted,
    std::vector<const DDS_TypeCode *> & already_asserted,
    const bool root = true);

  template<typename MembersType>
  DDS_StructMemberSeq
  convert_typesupport_members(
    const MembersType * const members,
    const bool request_reply,
    const bool is_request,
    std::vector<const DDS_TypeCode *> & new_asserted,
    std::vector<const DDS_TypeCode *> & already_asserted,
    const bool root = false)
  {
    DDS_StructMemberSeq tc_members_stack = DDS_SEQUENCE_INITIALIZER;
    DDS_StructMemberSeq * const tc_members = &tc_members_stack;
    auto scope_exit_tc_members_delete = rcpputils::make_scope_exit(
      [tc_members]()
      {
        const DDS_Long seq_len =
        DDS_StructMemberSeq_get_length(tc_members);
        for (DDS_Long i = 0; i < seq_len; i++) {
          DDS_StructMember * const tc_member =
          DDS_StructMemberSeq_get_reference(tc_members, i);
          DDS_String_free(tc_member->name);
          tc_member->name = nullptr;
        }

        DDS_StructMemberSeq_finalize(tc_members);
      });
    const DDS_TypeCode * tc_header = nullptr;
    if (root && request_reply) {
      const bool basic_mapping =
        RequestReplyMapping::Basic == options_.request_reply_mapping;
      if (options_.cyclone_compatible) {
        tc_header = typecodes::CycloneRequestHeader();
      } else if (basic_mapping) {
        if (is_request) {
          tc_header = typecodes::RequestHeader();
        } else {
          tc_header = typecodes::ReplyHeader();
        }
      }
      if (nullptr != tc_header) {
        bool newly_cached = false;
        std::vector<const DDS_TypeCode *> header_new;
        std::vector<const DDS_TypeCode *> header_already;
        std::tie(newly_cached, header_new, header_already) = assert_typecode(tc_header);
        tc_header = (newly_cached) ? header_new.back() : header_already.back();
        new_asserted.insert(new_asserted.end(), header_new.begin(), header_new.end());
        already_asserted.insert(
          already_asserted.end(), header_already.begin(),
          header_already.end());
      }
    }
    // assert(members->member_count >= 0)
    const DDS_Long member_count =
      static_cast<DDS_Long>(members->member_count_) +
      ((nullptr != tc_header) ? 1 : 0);
    const uint32_t member_i_start = (nullptr != tc_header) ? 1 : 0;
    if (!DDS_StructMemberSeq_ensure_length(
        tc_members, member_count, member_count))
    {
      throw std::runtime_error("failed to ensure sequence length");
    }

    if (nullptr != tc_header) {
      DDS_StructMember * const tc_member =
        DDS_StructMemberSeq_get_reference(tc_members, 0);
      tc_member->name = DDS_String_dup("_header");
      if (nullptr == tc_member->name) {
        throw std::runtime_error("failed to duplicate string");
      }
      tc_member->type = tc_header;
    }

    for (uint32_t i = 0, j = member_i_start; i < members->member_count_; ++i, ++j) {
      DDS_StructMember * const tc_member =
        DDS_StructMemberSeq_get_reference(tc_members, j);
      const auto * member = members->members_ + i;

      if (nullptr == member->name_) {
        throw std::runtime_error("unexpected empty member name");
      }
      /* Check that member has a non-empty name */
      size_t member_name_len = strlen(member->name_);
      if (member_name_len == 0 || (member_name_len == 1 && member->name_[0] == '_')) {
        throw std::runtime_error("unexpected empty member name");
      }

      /* Names in the introspection plugin don't actually end with "_" */
      if (options_.legacy_rmw_compatible) {
        std::ostringstream ss;
        ss << member->name_ << "_";
        tc_member->name = DDS_String_dup(ss.str().c_str());
      } else {
        tc_member->name = DDS_String_dup(member->name_);
      }
      if (nullptr == tc_member->name) {
        throw std::runtime_error("failed to duplicate member name");
      }

      tc_member->type =
        convert_typesupport_member(
        member, request_reply, is_request, new_asserted, already_asserted);
    }

    scope_exit_tc_members_delete.cancel();
    return *tc_members;
  }

  template<typename MemberType>
  DDS_TypeCode *
  convert_typesupport_member(
    const MemberType * const member,
    const bool request_reply,
    const bool is_request,
    std::vector<const DDS_TypeCode *> & new_asserted,
    std::vector<const DDS_TypeCode *> & already_asserted)
  {
    DDS_TypeCode * el_tc = nullptr;

    switch (member->type_id_) {
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL:
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE:
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT32:
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64:
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
        {
          DDS_TCKind dds_type_id = type_id_ros_to_dds(member->type_id_);

          el_tc =
            /* TODO(asorbini): refactor out this cast */
            const_cast<DDS_TypeCode *>(
            DDS_TypeCodeFactory_get_primitive_tc(tc_factory_, dds_type_id));
          break;
        }
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
        {
          DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
          el_tc = DDS_TypeCodeFactory_create_string_tc(
            tc_factory_,
            (member->string_upper_bound_ > 0) ?
            // TODO(asorbini) checked conversion of member->string_upper_bound_
            static_cast<DDS_UnsignedLong>(member->string_upper_bound_) : LENGTH_UNBOUND,
            &ex);
          if (nullptr == el_tc) {
            throw std::runtime_error("failed to create string typecode");
          }
          insert(el_tc);
          break;
        }
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
        {
          DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
          el_tc = DDS_TypeCodeFactory_create_wstring_tc(
            tc_factory_,
            (member->string_upper_bound_ > 0) ?
            // TODO(asorbini) checked conversion of member->string_upper_bound_
            static_cast<DDS_UnsignedLong>(member->string_upper_bound_) : LENGTH_UNBOUND,
            &ex);
          if (nullptr == el_tc) {
            throw std::runtime_error("failed to create wide string typecode");
          }
          insert(el_tc);
          break;
        }
      case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE:
        {
          bool cpp_version = false;
          const rosidl_message_type_support_t * type_support_intro = nullptr;

          std::tie(cpp_version, type_support_intro) =
            get_nested_introspection_typesupport(member->members_);

          std::string type_name;

          if (cpp_version) {
            type_name = create_dds_type_name_from_members(
              reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
                type_support_intro->data), !options_.demangle_ros_names /* mangle_names */);
          } else {
            type_name = create_dds_type_name_from_members(
              reinterpret_cast<const rosidl_typesupport_introspection_c__MessageMembers *>(
                type_support_intro->data), !options_.demangle_ros_names /* mangle_names */);
          }
          const bool new_type = assert_typecode(
            type_name,
            request_reply,
            is_request,
            cpp_version,
            type_support_intro,
            new_asserted,
            already_asserted,
            false /* root */);
          if (new_type) {
            el_tc = const_cast<DDS_TypeCode *>(new_asserted.back());
          } else {
            el_tc = const_cast<DDS_TypeCode *>(already_asserted.back());
          }
          break;
        }
      default: {
          break;
        }
    }

    if (nullptr == el_tc) {
      throw std::runtime_error("failed to create member type code");
    }

    if (member->is_array_) {
      DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
      if (member->array_size_ > 0 && !member->is_upper_bound_) {
        if (member->array_size_ > static_cast<size_t>(INT32_MAX)) {
          throw std::runtime_error("unrepresentable array length");
        }
        struct DDS_UnsignedLongSeq dimensions = DDS_SEQUENCE_INITIALIZER;
        if (!DDS_UnsignedLongSeq_ensure_length(&dimensions, 1, 1)) {
          throw std::runtime_error("failed to ensure sequence length");
        }
        *DDS_UnsignedLongSeq_get_reference(&dimensions, 0) =
          static_cast<DDS_UnsignedLong>(member->array_size_);
        el_tc = DDS_TypeCodeFactory_create_array_tc(
          tc_factory_, &dimensions, el_tc, &ex);
        DDS_UnsignedLongSeq_finalize(&dimensions);
        if (nullptr == el_tc) {
          throw std::runtime_error("failed to create array typecode");
        }
        insert(el_tc);
      } else {
        DDS_Long tc_seq_len = LENGTH_UNBOUND;
        if (member->is_upper_bound_) {
          if (member->array_size_ > static_cast<size_t>(INT32_MAX)) {
            throw std::runtime_error("unrepresentable sequence length");
          }
          tc_seq_len = static_cast<DDS_Long>(member->array_size_);
        }
        el_tc = DDS_TypeCodeFactory_create_sequence_tc(
          tc_factory_, tc_seq_len, el_tc, &ex);
        if (nullptr == el_tc) {
          throw std::runtime_error("failed to create sequence typecode");
        }
        insert(el_tc);
      }
    }

    return el_tc;
  }

  std::vector<DDS_TypeCode *>
  collect_nested_typecodes(const DDS_TypeCode * const tc, const bool ros_type = true);

  DDS_TypeCode *
  resolve_collection_typecode(const DDS_TypeCode * const tc);

  DDS_TypeCode *
  mangle_typecode(const DDS_TypeCode * const tc);

  DDS_TypeCode *
  demangle_typecode(const DDS_TypeCode * const tc);

  std::vector<const DDS_TypeCode *>
  extract_nested_typecodes(
    const DDS_TypeCode * const tc,
    std::vector<const DDS_TypeCode *> * tc_cache = nullptr);

  DDS_TypeCode*
  mangle_typecode_recur(
    const DDS_TypeCode * const tc,
    TypeCodeMakeNameFn make_name_fn,
    TypeCodeMakeNameFn make_member_name_fn);


private:
  const TypeCacheOptions options_;
  DDS_TypeCodeFactory * tc_factory_{nullptr};
  std::vector<DDS_TypeCode *> tc_cache_;
  std::map<std::string, const DDS_TypeCode *> tc_named_cache_;
  std::map<std::string, std::shared_ptr<rcpputils::SharedLibrary>> typesupports_cpp_;
  std::map<std::string, std::shared_ptr<rcpputils::SharedLibrary>> typesupports_c_;
  std::vector<std::string> lib_path_;
  std::map<std::string, std::string> topics_cache_;
  std::mutex cache_mutex_;
};

}  // namespace robotspy

inline
std::ostream & operator<<(std::ostream & os, const robotspy::RequestReplyMapping & mapping)
{
  if (mapping == robotspy::RequestReplyMapping::Basic) {
    os << "basic";
  } else {
    os << "extended";
  }
  return os;
}

#endif  // ROBOTSPY__TYPECACHE_HPP_
