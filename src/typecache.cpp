// (c) 2023 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.
#include "robotspy/typecache.hpp"

namespace robotspy
{
static
std::string
make_typecode_name_mangled(const std::string & tc_name) {
  auto norm_tc_name = normalize_dds_type_name(tc_name);
  if (norm_tc_name == tc_name) {
    std::string package_name;
    std::string middle_module;
    std::string type_name;
    std::tie(package_name, middle_module, type_name) =
      parse_ros_type_name(demangle_dds_type_name(tc_name));
    std::stringstream ss;
    ss << package_name
      << "::" << middle_module
      << "::" << "dds_"
      << "::" << type_name << "_";
    return ss.str();
  } else {
    // Assume that the name is already mangled
    // TODO(asorbini) verify that the name is actually mangled
    return tc_name;
  }
}

static
std::string
make_typecode_name_demangled(const std::string & tc_name) {
  auto norm_tc_name = normalize_dds_type_name(tc_name);
  if (norm_tc_name == tc_name) {
    return tc_name;
  } else {
    return norm_tc_name;
  }
}


static
std::string
make_typecode_member_name_mangled(
    const std::string & member_name,
    const bool legacy_rmw_compatible) {
  if (legacy_rmw_compatible) {
    if (member_name[member_name.size() - 1] != '_') {
      return member_name + "_";
    } else {
      return member_name;
    }
  } else {
    return member_name;
  }
}

static
std::string
make_typecode_member_name_demangled(
  const std::string & member_name)
{
  if (member_name[member_name.size() - 1] == '_') {
    return member_name.substr(0, member_name.size() - 1);
  } else {
    return member_name;
  }
}

static
bool
is_typecode_complex(const DDS_TCKind tckind)
{
  return (DDS_TK_ENUM == tckind
    || DDS_TK_STRUCT == tckind
    || DDS_TK_VALUE == tckind
    || DDS_TK_UNION == tckind)
}

static
bool
is_typecode_collection(const DDS_TCKind tckind)
{
  return (DDS_TK_ARRAY == tckind
    || DDS_TK_SEQUENCE == tckind)
}

TypeCache::TypeCache(const TypeCacheOptions & options)
: options_(options),
  tc_factory_(DDS_TypeCodeFactory_get_instance())
{
  if (options.cyclone_compatible && options.legacy_rmw_compatible) {
    throw std::runtime_error("multiple compatibility modes enabled");
  }
  if (options.cyclone_compatible &&
    options.request_reply_mapping != RequestReplyMapping::Basic)
  {
    throw std::runtime_error("compatibility mode requires basic mapping");
  } else if (options.legacy_rmw_compatible &&
    options.request_reply_mapping != RequestReplyMapping::Extended)
  {
    throw std::runtime_error("compatibility mode required extended mapping");
  }
  if (nullptr == tc_factory_) {
    throw std::runtime_error("failed to access DDS_TypeCodeFactory");
  }

  get_library_path(lib_path_);
}
TypeCache::~TypeCache()
{
  clear(true);
}
void
TypeCache::clear(const bool nothrow)
{
  for (auto & tc : tc_cache_) {
    DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
    DDS_TypeCodeFactory_delete_tc(tc_factory_, tc, &ex);
    if (!nothrow && DDS_NO_EXCEPTION_CODE != ex) {
      throw std::runtime_error("failed to delete typecode");
    }
  }
  if (!nothrow) {
    tc_named_cache_.clear();
  } else {
    try {
      tc_named_cache_.clear();
    } catch (std::exception & e) {
      // suppress exceptions
      (void)e;
    }
  }
}

void
TypeCache::unload()
{
  typesupports_cpp_.clear();
  typesupports_c_.clear();
}

const DDS_TypeCode *
TypeCache::find(const std::string & type_fqname, const bool ros_type)
{
  auto cached = tc_named_cache_.find(
    (ros_type ? normalize_dds_type_name(type_fqname) : type_fqname));
  if (tc_named_cache_.end() != cached) {
    return cached->second;
  }
  return nullptr;
}

void
TypeCache::insert(
  const std::string & type_fqname, DDS_TypeCode * const typecode,
  const bool ros_type)
{
  tc_named_cache_.emplace(
    (ros_type ? normalize_dds_type_name(type_fqname) : type_fqname), typecode);
  tc_cache_.insert(tc_cache_.end(), typecode);
}

void
TypeCache::insert(DDS_TypeCode * const typecode)
{
  tc_cache_.insert(tc_cache_.end(), typecode);
}

std::tuple<bool, bool, std::vector<const DDS_TypeCode *>, std::vector<const DDS_TypeCode *>>
TypeCache::assert_dds_topic(
  const std::string & topic_name,
  const DDS_TypeCode * const tc,
  const bool ros_type,
  const std::string & demangled_ros_type)
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  bool new_type;
  std::vector<const DDS_TypeCode *> new_types;
  std::vector<const DDS_TypeCode *> existing_types;
  std::tie(new_type, new_types, existing_types) = assert_typecode(tc, ros_type, demangled_ros_type);
  auto topic_tc = (new_type) ? new_types.back() : existing_types.back();
  DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
  std::string tc_name = DDS_TypeCode_name(topic_tc, &ex);
  if (DDS_NO_EXCEPTION_CODE != ex) {
    throw std::runtime_error("failed to get typecode name");
  }
  const bool new_topic = insert_topic(topic_name, tc_name);
  return std::make_tuple(new_topic, new_type, new_types, existing_types);
}

std::tuple<bool, bool, std::vector<const DDS_TypeCode *>, std::vector<const DDS_TypeCode *>>
TypeCache::assert_ros_topic(
  const std::string & topic_name,
  const std::string & type_fqname)
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  bool request_reply;
  bool is_request;
  std::tie(request_reply, is_request) = is_type_requestreply(type_fqname);
  const rosidl_message_type_support_t * intro_typesupport;
  bool cpp_version;

  std::tie(cpp_version, intro_typesupport) = load_typesupport(type_fqname);

  std::vector<const DDS_TypeCode *> new_asserted;
  std::vector<const DDS_TypeCode *> already_asserted;
  const bool new_type = assert_typecode(
    type_fqname,
    request_reply,
    is_request,
    cpp_version,
    intro_typesupport,
    new_asserted,
    already_asserted);
  auto topic_tc = (new_type) ? new_asserted.back() : already_asserted.back();
  DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
  std::string tc_name = DDS_TypeCode_name(topic_tc, &ex);
  if (DDS_NO_EXCEPTION_CODE != ex) {
    throw std::runtime_error("failed to get typecode name");
  }
  const bool new_topic = insert_topic(topic_name, tc_name);
  return std::make_tuple(new_topic, new_type, new_asserted, already_asserted);
}

bool
TypeCache::insert_topic(
  const std::string & topic_name,
  const std::string & type_fqname)
{
  std::string norm_fqname = normalize_dds_type_name(type_fqname);
  auto cached = topics_cache_.find(topic_name);
  if (cached != topics_cache_.end()) {
    if (norm_fqname != cached->second) {
      throw std::runtime_error("topic already asserted with a different type");
    }
    return false;
  }
  topics_cache_.emplace(topic_name, norm_fqname);
  return true;
}

std::tuple<bool, std::vector<const DDS_TypeCode *>, std::vector<const DDS_TypeCode *>>
TypeCache::assert_dds_type(
  const DDS_TypeCode * const tc,
  const bool ros_type,
  const std::string & demangled_ros_type)
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  return assert_typecode(tc, ros_type, demangled_ros_type);
}

std::tuple<bool, std::vector<const DDS_TypeCode *>, std::vector<const DDS_TypeCode *>>
TypeCache::assert_typecode(
  const DDS_TypeCode * const tc,
  const bool ros_type,
  const std::string & demangled_ros_type)
{
  std::vector<const DDS_TypeCode *> new_asserted;
  std::vector<const DDS_TypeCode *> already_asserted;
  DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
  std::string type_fqname = DDS_TypeCode_name(tc, &ex);
  if (DDS_NO_EXCEPTION_CODE != ex) {
    throw std::runtime_error("failed to get typecode name");
  }
  DDS_TypeCode * assert_tc = nullptr;
  auto scope_exit_assert_tc =
    rcpputils::make_scope_exit(
    [this, &assert_tc]() {
      if (nullptr != assert_tc) {
        DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
        DDS_TypeCodeFactory_delete_tc(tc_factory_, assert_tc, &ex);
      }
    });

  if (ros_type && !options_.demangle_ros_names && type_fqname == demangled_ros_type) {
    assert_tc = mangle_typecode(tc);
    type_fqname = DDS_TypeCode_name(assert_tc, &ex);
    if (DDS_NO_EXCEPTION_CODE != ex) {
      throw std::runtime_error("failed to get typecode name");
    }
  } else if (ros_type && options_.demangle_ros_names && type_fqname != demangled_ros_type) {
    assert_tc = demangle_typecode(tc);
    type_fqname = DDS_TypeCode_name(assert_tc, &ex);
    if (DDS_NO_EXCEPTION_CODE != ex) {
      throw std::runtime_error("failed to get typecode name");
    }
  } else {
    assert_tc = DDS_TypeCodeFactory_clone_tc(tc_factory_, tc, &ex);
    if (nullptr == assert_tc || DDS_NO_EXCEPTION_CODE != ex) {
      throw std::runtime_error("failed to get clone typecode");
    }
  }

  auto cached = find(type_fqname, ros_type);
  if (nullptr != cached) {
    if (!DDS_TypeCode_equal(cached, assert_tc, &ex)) {
      // DDS_TypeCode_print_IDL(cached, 0, &ex);
      // DDS_TypeCode_print_IDL(assert_tc, 0, &ex);
      std::string msg = "conflict detected for asserted typecode: ";
      msg += type_fqname;
      throw std::runtime_error(msg);
    }
    already_asserted.insert(already_asserted.end(), cached);
    return std::make_tuple(false, new_asserted, already_asserted);
  }
  DDS_TypeCode * clone_tc = DDS_TypeCodeFactory_clone_tc(tc_factory_, assert_tc, &ex);
  auto scope_exit_clone_tc =
    rcpputils::make_scope_exit(
    [this, clone_tc]() {
      DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
      if (nullptr != clone_tc) {
        DDS_TypeCodeFactory_delete_tc(tc_factory_, clone_tc, &ex);
      }
    });
  if (nullptr == clone_tc || DDS_NO_EXCEPTION_CODE != ex) {
    throw std::runtime_error("failed to get typecode name");
  }

  std::vector<DDS_TypeCode *> nested = collect_nested_typecodes(clone_tc, ros_type);
  auto scope_exit_nested =
    rcpputils::make_scope_exit(
    [this, &nested]() {
      for (auto & n : nested) {
        DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
        DDS_TypeCodeFactory_delete_tc(tc_factory_, n, &ex);
      }
    });
  for (auto & n : nested) {
    std::string n_name = DDS_TypeCode_name(n, &ex);
    if (DDS_NO_EXCEPTION_CODE != ex) {
      throw std::runtime_error("failed to get typecode name");
    }
    auto cached = find(n_name, ros_type);
    if (nullptr == cached) {
      insert(n_name, n, ros_type);
      new_asserted.insert(new_asserted.end(), n);
    } else {
      if (!DDS_TypeCode_equal(cached, n, &ex)) {
        // DDS_TypeCode_print_IDL(cached, 0, &ex);
        // DDS_TypeCode_print_IDL(n, 0, &ex);
        std::string msg = "conflict detected for asserted nested typecode: ";
        msg += n_name;
        throw std::runtime_error(msg);
      }
      already_asserted.insert(already_asserted.end(), n);
    }
  }
  insert(type_fqname, clone_tc, ros_type);
  new_asserted.insert(new_asserted.end(), clone_tc);

  scope_exit_nested.cancel();
  scope_exit_clone_tc.cancel();
  return std::make_tuple(true, new_asserted, already_asserted);
}

DDS_TypeCode *
TypeCache::resolve_collection_typecode(const DDS_TypeCode * const tc)
{
  DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
  DDS_TypeCode * content_tc = DDS_TypeCode_content_type(tc, &ex);
  if (DDS_NO_EXCEPTION_CODE != ex) {
    throw std::runtime_error("failed to get collection typecode");
  }
  auto tc_kind = DDS_TypeCode_kind(content_tc, &ex);
  if (DDS_NO_EXCEPTION_CODE != ex) {
    throw std::runtime_error("failed to get typecode kind");
  }
  if (DDS_TK_SEQUENCE == tc_kind || DDS_TK_ARRAY == tc_kind) {
    return resolve_collection_typecode(content_tc);
  } else {
    return content_tc;
  }
}

std::vector<DDS_TypeCode *>
TypeCache::collect_nested_typecodes(const DDS_TypeCode * const tc, const bool ros_type)
{
  DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
  std::vector<DDS_TypeCode *> result;
  auto scope_exit_result =
    rcpputils::make_scope_exit(
    [this, &result]() {
      for (auto & n : result) {
        DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
        DDS_TypeCodeFactory_delete_tc(tc_factory_, n, &ex);
      }
    });
  // Check if the type has a concrete base type, and if so, collect it
  DDS_TypeCode *concrete_tc = DDS_TypeCode_concrete_base_type(tc, &ex)
  if (DDS_NO_EXCEPTION_CODE != ex) {
    throw std::runtime_error("failed to get concrete base type");
  }
  std::vector<DDS_TypeCode *> inspect_nested;
  if (nullptr != concrete_tc) {
    inspect_nested.insert(inspect_nested.end(), concrete_tc)
  }
  // Inspect the type codes of all nested members
  size_t member_count = DDS_TypeCode_member_count(tc, &ex);
  if (DDS_NO_EXCEPTION_CODE != ex) {
    throw std::runtime_error("failed to get typecode member count");
  }
  for (size_t i = 0; i < member_count; i++) {
    DDS_TypeCode * member_tc = DDS_TypeCode_member_type(tc, i, &ex);
    if (nullptr == member_tc || DDS_NO_EXCEPTION_CODE != ex) {
      throw std::runtime_error("failed to get typecode member id");
    }
    inspect_nested.insert(inspect_nested.end(), member_tc)
  }
  if (DDS_TK_UNION == )

  for (auto & ntc : inspect_nested) {
    auto tc_kind = DDS_TypeCode_kind(ntc, &ex);
    if (DDS_NO_EXCEPTION_CODE != ex) {
      throw std::runtime_error("failed to get typecode kind");
    }
    DDS_TypeCode * nested_tc = nullptr;
    if (is_typecode_complex(tc_kind)) {
      nested_tc = ntc;
    } else if (is_typecode_collection(tc_kind)) {
      nested_tc = resolve_collection_typecode(ntc);
      auto collection_tc_k = DDS_TypeCode_kind(nested_tc, &ex);
      if (DDS_NO_EXCEPTION_CODE != ex) {
        throw std::runtime_error("failed to get collection typecode kind");
      }
      if (!is_typecode_complex(collection_tc_k)) {
        nested_tc = nullptr;
      }
    }
    if (nullptr == nested_tc) {
      continue;
    }
    std::vector<DDS_TypeCode *> nested = collect_nested_typecodes(nested_tc, ros_type);
    auto scope_exit_nested =
      rcpputils::make_scope_exit(
      [this, &nested]() {
        for (auto & n : nested) {
          DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
          DDS_TypeCodeFactory_delete_tc(tc_factory_, n, &ex);
        }
      });
    result.insert(result.end(), nested.begin(), nested.end());
    scope_exit_nested.cancel();
    std::string nested_name = DDS_TypeCode_name(nested_tc, &ex);
    if (DDS_NO_EXCEPTION_CODE != ex) {
      throw std::runtime_error("failed to get typecode name");
    }
    auto already_cached = find(nested_name, ros_type);
    if (nullptr == already_cached) {
      DDS_TypeCode * cloned = DDS_TypeCodeFactory_clone_tc(tc_factory_, nested_tc, &ex);
      if (nullptr == cloned) {
        throw std::runtime_error("failed to clone typecode");
      }
      auto scope_exit_clone_tc =
        rcpputils::make_scope_exit(
        [this, cloned]() {
          DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
          if (nullptr != cloned) {
            DDS_TypeCodeFactory_delete_tc(tc_factory_, cloned, &ex);
          }
        });
      result.insert(result.end(), cloned);
      scope_exit_clone_tc.cancel();
    }
  }
  scope_exit_result.cancel();
  return result;
}

std::pair<bool, const rosidl_message_type_support_t *>
TypeCache::load_typesupport(const std::string & type_fqname)
{
  std::string demangled_type_fqname = demangle_dds_type_name(type_fqname);
  std::string package_name;
  std::string middle_module;
  std::string type_name;
  std::tie(package_name, middle_module, type_name) =
    parse_ros_type_name(demangled_type_fqname);
  bool cpp_version = false;
  std::shared_ptr<rcpputils::SharedLibrary> typesupport_lib;
  const rosidl_message_type_support_t * typesupport = nullptr;
  auto cached_lib = typesupports_c_.find(package_name);
  bool already_cached = cached_lib != typesupports_c_.end();
  if (!already_cached) {
    cpp_version = true;
    cached_lib = typesupports_cpp_.find(package_name);
    already_cached = cached_lib != typesupports_cpp_.end();
  }
  if (already_cached) {
    try {
      std::tie(std::ignore, typesupport_lib) = *cached_lib;
      typesupport = lookup_introspection_typesupport(
        package_name, middle_module, type_name, *typesupport_lib, cpp_version);
    } catch (std::exception & e) {
      already_cached = false;
    }
  }
  if (!already_cached) {
    std::tie(cpp_version, typesupport_lib, typesupport) =
      load_instrospection_typesupport_library(
      package_name, middle_module, type_name, lib_path_);
    auto & typesupport_cache = (cpp_version) ? typesupports_cpp_ : typesupports_c_;
    bool duplicate =
      typesupport_cache.find(package_name) != typesupport_cache.end();
    if (duplicate) {
      throw std::runtime_error("multiple copies of the same shared library");
    }
    typesupport_cache.emplace(package_name, typesupport_lib);
  }
  if (nullptr == typesupport) {
    throw std::runtime_error("failed to load type support");
  }

  return {cpp_version, typesupport};
}

std::tuple<bool, std::vector<const DDS_TypeCode *>, std::vector<const DDS_TypeCode *>>
TypeCache::assert_ros_type(const std::string & type_fqname)
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  bool request_reply;
  bool is_request;
  std::tie(request_reply, is_request) = is_type_requestreply(type_fqname);
  const rosidl_message_type_support_t * intro_typesupport;
  bool cpp_version;

  std::tie(cpp_version, intro_typesupport) = load_typesupport(type_fqname);

  std::vector<const DDS_TypeCode *> new_asserted;
  std::vector<const DDS_TypeCode *> already_asserted;
  const bool new_type = assert_typecode(
    type_fqname,
    request_reply,
    is_request,
    cpp_version,
    intro_typesupport,
    new_asserted,
    already_asserted);
  return std::make_tuple(new_type, new_asserted, already_asserted);
}

bool
TypeCache::assert_typecode(
  const std::string & type_fqname,
  const bool request_reply,
  const bool is_request,
  const bool cpp_version,
  const rosidl_message_type_support_t * const type_support_intro,
  std::vector<const DDS_TypeCode *> & new_asserted,
  std::vector<const DDS_TypeCode *> & already_asserted,
  const bool root)
{
  std::string assert_type_fqname;
  // type_fqname is assumed to be a "demangled" type name, i.e. in the form
  // "<package>::<middle>::<type>". Check if we are caching types using
  // mangled names and if so, tranform it.
  if (!options_.demangle_ros_names) {
    assert_type_fqname = make_typecode_name_mangled(type_fqname);
  } else {
    assert_type_fqname = normalize_dds_type_name(type_fqname);
  }

  auto cached = find(assert_type_fqname, true);
  if (nullptr != cached) {
    already_asserted.insert(already_asserted.end(), cached);
    return false;
  }

  struct DDS_StructMemberSeq tc_members;
  if (cpp_version) {
    tc_members = convert_typesupport_members(
      reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
        type_support_intro->data),
      request_reply,
      is_request,
      new_asserted,
      already_asserted,
      root);
  } else {
    tc_members = convert_typesupport_members(
      reinterpret_cast<const rosidl_typesupport_introspection_c__MessageMembers *>(
        type_support_intro->data),
      request_reply,
      is_request,
      new_asserted,
      already_asserted,
      root);
  }
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

  DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
  DDS_TypeCode * tc =
    DDS_TypeCodeFactory_create_struct_tc(
    tc_factory_,
    assert_type_fqname.c_str(),
    &tc_members,
    &ex);
  if (nullptr == tc) {
    throw std::runtime_error("failed to create struct typecode");
  }
  auto scope_exit_tc =
    rcpputils::make_scope_exit(
    [this, tc]()
    {
      DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
      DDS_TypeCodeFactory_delete_tc(tc_factory_, tc, &ex);
    });

  insert(assert_type_fqname, tc, true);
  new_asserted.insert(new_asserted.end(), tc);

  scope_exit_tc.cancel();
  scope_exit_tc_members_delete.cancel();
  return true;
}


DDS_TypeCode *
TypeCache::mangle_typecode(const DDS_TypeCode * const tc)
{
  return mangle_typecode_recur(
    tc,
    make_typecode_name_mangled,
    static_cast<TypeCodeMakeNameFn>(
      ((options_.legacy_rmw_compatible)?
        [](const std::string & member_name) {
          return make_typecode_member_name_mangled(
            member_name, true /* legacy_rmw_compatible */);
        } :
        [](const std::string & member_name) {
          return make_typecode_member_name_mangled(
            member_name, false /* legacy_rmw_compatible */);
        })
      )
    );
}

DDS_TypeCode *
TypeCache::demangle_typecode(const DDS_TypeCode * const tc)
{
  return mangle_typecode_recur(
    tc,
    make_typecode_name_demangled,
    make_typecode_member_name_demangled);
}

DDS_TypeCode*
TypeCache::mangle_typecode_recur(
  const DDS_TypeCode * const tc,
  TypeCodeMakeNameFn make_name_fn,
  TypeCodeMakeNameFn make_member_name_fn)
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
    [this, &member_tcs]()
    {
      DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
      for (auto & mtc : member_tcs) {
        DDS_TypeCodeFactory_delete_tc(tc_factory_, mtc, &ex);
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
      member_tc = mangle_typecode_recur(member_tc, make_name_fn, make_member_name_fn);
      member_tcs.insert(member_tcs.end(), member_tc);
    } else if (DDS_TK_SEQUENCE == tc_kind || DDS_TK_ARRAY == tc_kind) {
      nested_tc = resolve_collection_typecode(member_tc);
      auto collection_tc_k = DDS_TypeCode_kind(nested_tc, &ex);
      if (DDS_NO_EXCEPTION_CODE != ex) {
        throw std::runtime_error("failed to get collection typecode kind");
      }
      if (DDS_TK_STRUCT == collection_tc_k) {
        nested_tc = mangle_typecode_recur(nested_tc, make_name_fn, make_member_name_fn);
        member_tcs.insert(member_tcs.end(), nested_tc);
        if (DDS_TK_SEQUENCE == tc_kind) {
          auto seq_bound = DDS_TypeCode_length(member_tc, &ex);
          member_tc = DDS_TypeCodeFactory_create_sequence_tc(
            tc_factory_,
            seq_bound,
            nested_tc,
            &ex);
        } else {
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
            tc_factory_,
            &array_dimensions,
            nested_tc,
            &ex);
        }
      } else {
        member_tc = DDS_TypeCodeFactory_clone_tc(tc_factory_, member_tc, &ex);
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
    std::string tc_mem_name = DDS_TypeCode_member_name(tc, i, &ex);
    if (DDS_NO_EXCEPTION_CODE != ex) {
      throw std::runtime_error("failed to get member name");
    }
    std::string member_name = make_member_name_fn(tc_mem_name);
    struct_member->name = DDS_String_dup(member_name.c_str());
    if (nullptr == struct_member->name) {
      throw std::runtime_error("failed to duplicate member name");
    }
  }

  DDS_TypeCode * const result =
    DDS_TypeCodeFactory_create_struct_tc(
      tc_factory_,
      mangled_tc_name.c_str(),
      &tc_members,
      &ex);
  if (nullptr == tc) {
    throw std::runtime_error("failed to create struct typecode");
  }

  scope_exit_tc_members_vector_delete.cancel();
  scope_exit_tc_members_delete.cancel();
  return result;
}
}  // namespace robotspy
