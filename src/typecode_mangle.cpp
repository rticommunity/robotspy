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
#include <algorithm>
#include <sstream>

#include "robotspy/typecode_mangle.hpp"

namespace robotspy
{
static
std::vector<const DDS_TypeCode *>
extract_nested_typecodes(
  const DDS_TypeCode * const tc,
  std::vector<const DDS_TypeCode *> * tc_cache = nullptr)
{
  std::vector<const DDS_TypeCode *> stack_tc_cache;
  if (nullptr == tc_cache) {
    tc_cache = &stack_tc_cache;
  }
  if (std::find(tc_cache->begin(), tc_cache->end(), tc) == tc_cache->end()) {
    tc_cache->insert(tc_cache->begin(), tc);
  }
  DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
  size_t member_count = DDS_TypeCode_member_count(tc, &ex);
  if (DDS_NO_EXCEPTION_CODE != ex) {
    throw std::runtime_error("failed to get typecode member count");
  }
  for (size_t i = 0; i < member_count; i++) {
    DDS_TypeCode * member_tc = DDS_TypeCode_member_type(tc, i, &ex);
    if (nullptr == member_tc || DDS_NO_EXCEPTION_CODE != ex) {
      throw std::runtime_error("failed to get typecode member id");
    }
    auto tc_kind = DDS_TypeCode_kind(member_tc, &ex);
    if (DDS_NO_EXCEPTION_CODE != ex) {
      throw std::runtime_error("failed to get typecode kind");
    }
    DDS_TypeCode * nested_tc = nullptr;
    if (DDS_TK_STRUCT == tc_kind) {
      nested_tc = member_tc;
    } else if (DDS_TK_SEQUENCE == tc_kind || DDS_TK_ARRAY == tc_kind) {
      nested_tc = resolve_collection_typecode(member_tc);
      auto collection_tc_k = DDS_TypeCode_kind(nested_tc, &ex);
      if (DDS_NO_EXCEPTION_CODE != ex) {
        throw std::runtime_error("failed to get collection typecode kind");
      }
      if (DDS_TK_STRUCT != collection_tc_k) {
        continue;
      }
    } else {
      continue;
    }
    extract_nested_typecodes(nested_tc, tc_cache);
  }

  return stack_tc_cache;
}



static
DDS_TypeCode*
mangle_typecode_recur(
  DDS_TypeCodeFactory * const tc_factory,
  const DDS_TypeCode * const tc,
  decltype(make_typecode_name_demangled) make_name_fn
  decltype(make_typecode_member_name_demangled) make_member_name_fn,
  const bool legacy_rmw_compatible = false)
{
  DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
  std::string tc_name = DDS_TypeCode_name(tc, &ex);
  if (DDS_NO_EXCEPTION_CODE != ex) {
    throw std::runtime_error("failed to get typecode name");
  }
  std::string mangled_tc_name = make_name_fn(tc_name);
  struct DDS_StructMemberSeq tc_members;
  struct DDS_StructMemberSeq * const tc_members_ptr = &tc_members;
  auto scope_exit_tc_members_delete =
    rcpputils::make_scope_exit(
    [tc_members_ptr]()
    {
      const DDS_Long seq_len =
      DDS_StructMemberSeq_get_length(tc_members_ptr);
      for (DDS_Long i = 0; i < seq_len; i++) {
        DDS_StructMember * const tc_member =
        DDS_StructMemberSeq_get_reference(tc_members_ptr, i);
        DDS_String_free(tc_member->name);
        tc_member->name = nullptr;
      }

      DDS_StructMemberSeq_finalize(tc_members_ptr);
    });

  std::vector<DDS_TypeCode*> member_tcs;
  auto scope_exit_tc_members_vector_delete =
    rcpputils::make_scope_exit(
    [&member_tcs]()
    {
      DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
      for (auto & mtc : member_tcs) {
        DDS_TypeCodeFactory_delete_tc(tc_factory, mtc, &ex)
      }
    });

  size_t member_count = DDS_TypeCode_member_count(tc, &ex);
  if (DDS_NO_EXCEPTION_CODE != ex) {
    throw std::runtime_error("failed to get typecode member count");
  }
  if (!DDS_StructMemberSeq_ensure_length(
      &tc_members, member_count, member_count))
  {
    throw std::runtime_error("failed to ensure sequence length");
  }
  for (size_t i = 0; i < member_count; i++) {
    DDS_TypeCode * member_tc = DDS_TypeCode_member_type(tc, i, &ex);
    if (nullptr == member_tc || DDS_NO_EXCEPTION_CODE != ex) {
      throw std::runtime_error("failed to get typecode member id");
    }
    auto tc_kind = DDS_TypeCode_kind(member_tc, &ex);
    if (DDS_NO_EXCEPTION_CODE != ex) {
      throw std::runtime_error("failed to get typecode kind");
    }
    DDS_TypeCode * nested_tc = nullptr;
    if (DDS_TK_STRUCT == tc_kind) {
      member_tc = mangle_typecode_recur(tc_factory, member_tc, make_name_fn, legacy_rmw_compatible);
      member_tcs.insert(member_tcs.end(), member_tc);
    } else if (DDS_TK_SEQUENCE == tc_kind || DDS_TK_ARRAY == tc_kind) {
      nested_tc = resolve_collection_typecode(member_tc);
      auto collection_tc_k = DDS_TypeCode_kind(nested_tc, &ex);
      if (DDS_NO_EXCEPTION_CODE != ex) {
        throw std::runtime_error("failed to get collection typecode kind");
      }
      if (DDS_TK_STRUCT == collection_tc_k) {
        nested_tc = mangle_typecode_recur(tc_factory, nested_tc, make_name_fn, legacy_rmw_compatible);
        member_tcs.insert(member_tcs.end(), nested_tc);
        if (DDS_TK_SEQUENCE == tc_kind) {
          auto seq_bound = DDS_TypeCode_element_count(member_tc, &ex);
          member_tc = DDS_TypeCodeFactory_create_sequence_tc(
            tc_factory,
            seq_bound,
            nested_tc,
            &ex);
        else
          DDS_UnsignedLongSeq array_dimensions = DDS_SEQUENCE_INITIALIZER;
          DDS_UnsignedLongSeq * const array_dimensions_ptr = &array_dimensions;
          auto scope_exit_array_dims =
            rcpputils::make_scope_exit(
            [array_dimensions_ptr]()
            {
              DDS_UnsignedLongSeq_finalize(array_dimensions_ptr);
            });

          DDS_UnsignedLong dim_count = DDS_TypeCode_array_dimension_count(member_tc, &ex);
          if (DDS_NO_EXCEPTION_CODE != ex) {
            throw std::runtime_error("failed to get array member dimention count");
          }

          if (!DDS_UnsignedLongSeq_ensure_length(&array_dimensions, dim_count, dim_count)) {
            throw std::runtime_error("failed to resize sequence");
          }

          for (DDS_UnsignedLong i = 0; i < dim_count; i++) {
            *DDS_UnsignedLongSeq_get_reference(&array_dimensions, i) =
              DDS_TypeCode_array_dimension(member_tc, i, &ex);
            if (DDS_NO_EXCEPTION_CODE != ex) {
              throw std::runtime_error("failed to get array dimension");
            }
          }

          member_tc = DDS_TypeCodeFactory_create_array_tc(
            tc_factory,
            &array_dimensions,
            nested_tc,
            &ex);
        }
      } else {
        member_tc = DDS_TypeCodeFactory_clone_tc(tc_factory, member_tc, &ex);
      }
      if (DDS_NO_EXCEPTION_CODE != ex) {
        throw std::runtime_error("failed to create/clone collection member typecode");
      }
      member_tcs.insert(member_tcs.end(), member_tc);
    } else {
      // We assume that member_tc is a primitive (it couldn't be anything else in ROS)
      // so we don't need to clone the type code, since primitive type codes
      // are "singletons"
    }

    DDS_StructMember * const struct_member =
        DDS_StructMemberSeq_get_reference(&tc_members, i);
    
    struct_member->type = member_tc;
    std::string tc_mem_name = DDS_TypeCode_member_name(tc, i &ex);
    if (DDS_NO_EXCEPTION_CODE != ex) {
      throw std::runtime_error("failed to get member name");
    }
    std::string member_name = make_member_name_fn(tc_mem_name, legacy_rmw_compatible)
    tc_member->name = DDS_String_dup(member_name.c_str());
    if (nullptr == tc_member->name) {
      throw std::runtime_error("failed to duplicate member name");
    }
  }

  DDS_TypeCode * const result =
    DDS_TypeCodeFactory_create_struct_tc(
      tc_factory,
      mangled_tc_name.c_str(),
      &tc_members,
      &ex);
  if (nullptr == tc) {
    throw std::runtime_error("failed to create struct typecode");
  }

  scope_exit_tc_members_vector_delete.cancel();
  scope_exit_tc_members_delete.cancel();
  return stack_tc_cache;
}

DDS_TypeCode *
mangle_typecode(
  const DDS_TypeCode * const tc,
  const bool legacy_rmw_compatible)
{
  auto tc_factory = DDS_TypeCodeFactory_get_instance();
  if (nullptr == tc_factory) {
    throw std::runtime_error("failed to get typecode factory");
  }
  return mangle_typecode_recur(tc_factory, tc,
    make_typecode_name_mangled,
    make_typecode_member_name_mangled,
    legacy_rmw_compatible)
}

DDS_TypeCode *
demangle_typecode(const DDS_TypeCode * const tc)
{
  auto tc_factory = DDS_TypeCodeFactory_get_instance();
  if (nullptr == tc_factory) {
    throw std::runtime_error("failed to get typecode factory");
  }
  return mangle_typecode_recur(tc_factory, tc,
    make_typecode_name_demangled,
    make_typecode_member_name_demangled)
}
}  // namespace robotspy
