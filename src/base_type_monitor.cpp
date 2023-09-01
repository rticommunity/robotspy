// (c) 2023 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.
#include <unistd.h>
#include <string.h>

#include <sstream>
#include <string>

#include "robotspy/base_type_monitor.hpp"
#include "robotspy/log.hpp"

namespace robotspy
{
BaseTypeMonitor::BaseTypeMonitor(
    std::shared_ptr<InputEmitter> input,
    std::shared_ptr<OutputEmitter> output,
    const BaseTypeMonitorOptions & options)
: options_(options),
  input_(input),
  output_(output),
  type_cache_(options_.cache),
  type_filter_(options.type_filter),
  raw_type_filter_(options.raw_type_filter)
{
  LOG(DEBUG) <<
    "type filter: " << options_.type_filter << std::endl;
  LOG(DEBUG) <<
    "raw_type filter: " << options_.raw_type_filter << std::endl;
  LOG(DEBUG) <<
    "cache: { " << options_.cache.cyclone_compatible << ", " <<
    options_.cache.legacy_rmw_compatible << ", " <<
    options_.cache.request_reply_mapping << " }" << std::endl;
}

BaseTypeMonitor::~BaseTypeMonitor()
{

}

void
BaseTypeMonitor::start()
{
  // LOG(INFO) << "starting monitoring..." << std::endl;
  std::lock_guard<std::mutex> lock(active_mutex_);
  output_->open();
  input_->open();
}

void
BaseTypeMonitor::stop()
{
  // LOG(INFO) << "stopping monitoring..." << std::endl;
  std::lock_guard<std::mutex> lock(active_mutex_);
  output_->close();
  input_->close();
}

void
BaseTypeMonitor::consume_input()
{
  try {
    LOG(INFO) << "consuming input..." << std::endl;
    while (input_->is_active()) {
      std::string next_topic;
      std::string next_type;
      DDS_TypeCode * next_tc = nullptr;
      auto tc_factory = DDS_TypeCodeFactory_get_instance();
      if (nullptr == tc_factory) {
        throw std::runtime_error("failed to get typecode factory");
      }
      auto scope_exit_tc = rcpputils::make_scope_exit(
        [tc_factory, &next_tc]() {
          if (nullptr != next_tc) {
            DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
            DDS_TypeCodeFactory_delete_tc(tc_factory, next_tc, &ex);
          }
        });
      LOG(TRACE) << "waiting for next input..." << std::endl;
      std::tie(next_topic, next_type, next_tc) = input_->next();
      LOG(DEBUG) << ">>> input   : "
        "topic='" << next_topic << "', type='" << next_type << "', "
        << "tc=" << next_tc << std::endl;
      try {
        if (next_topic.size() > 0) {
          if (nullptr == next_tc) {
            if (next_type.size() == 0) {
              LOG(ERROR) << "xxx no type : " << next_topic << std::endl;
              continue;
            }
            on_topic_detected(next_topic, next_type);
          } else {
            on_topic_detected(next_topic, next_tc);
          }
        } else {
          if (nullptr == next_tc) {
            if (next_type.size() == 0) {
              LOG(DEBUG) << "xxx empty input received" << std::endl;
              continue;
            }
            on_type_detected(next_type);
          } else {
            on_type_detected(next_tc);
          }
        }
        if (nullptr != next_tc) {
          DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
          DDS_TypeCodeFactory_delete_tc(tc_factory, next_tc, &ex);
          next_tc = nullptr;
        }
      } catch (InvalidTopicNameException & e) {
        LOG(DEBUG) << "xxx invalid : "
          "topic='" << next_topic << "', type='" << next_type << "', "
          << "tc=" << next_tc << " (" << e.what() << ")"  << std::endl;
      } 
    }
  } catch (NoInputException & e) {
    LOG(DEBUG) << "received EOF: " << e.what() << std::endl;
  }
  LOG(DEBUG) << "consumed all input" << std::endl;
}

bool
BaseTypeMonitor::filter_type_name(const std::string & type_fqname)
{
  std::string ros_type_name = type_fqname;
  std::string failed = options_.raw_type_filter;
  bool detected = std::regex_match(type_fqname, raw_type_filter_);

  if (detected) {
    try {
      LOG(DEBUG) << "??? inspect : " << type_fqname << std::endl;
      ros_type_name = demangle_dds_type_name(type_fqname);
      ros_type_name = normalize_dds_type_name(type_fqname);
      LOG(TRACE) << "??? demangled: " << ros_type_name << std::endl;
      detected = std::regex_match(ros_type_name, type_filter_);
      if (!detected) {
        failed = options_.type_filter;
      }
    } catch (InvalidTopicNameException & e) {
      LOG(DEBUG) << "--- not ros : " << type_fqname << std::endl;
      detected = options_.include_non_ros;
    }
  }

  if (detected) {
    LOG(DEBUG) << "vvv detected: " << ros_type_name << std::endl;
  } else {
    LOG(DEBUG) << "xxx filtered: " << ros_type_name << " (" << failed << ")" << std::endl;
  }

  return detected;
}

void
BaseTypeMonitor::on_type_detected(
  const std::string & topic_name,
  const std::string & type_fqname,
  const DDS_TypeCode * const type_tc)
{
  if (type_fqname.size() == 0) {
    throw InvalidTopicNameException("empty type name");
  }
  if (!filter_type_name(type_fqname)) {
    return;
  }
  std::vector<const DDS_TypeCode *> new_asserted;
  std::vector<const DDS_TypeCode *> already_asserted;
  bool new_topic = false;
  bool new_type = false;
  if (nullptr != type_tc) {
    bool ros_type = true;
    std::string demangled_ros_type;
    try {
      demangled_ros_type = demangle_dds_type_name(normalize_dds_type_name(type_fqname));
    } catch (InvalidTopicNameException & e) {
      ros_type = false;
    }
    if (topic_name.size() > 0) {
      {
        DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
        LOG(TRACE) << "+++ assert DDS type: topic_name=" << topic_name << ", type_name=" << DDS_TypeCode_name(type_tc, &ex) << ", ros_type=" << ros_type << std::endl;
      }
      std::tie(new_topic, new_type, new_asserted, already_asserted) =
        type_cache_.assert_dds_topic(topic_name, type_tc, ros_type, demangled_ros_type);
    } else {
      {
        DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
        LOG(TRACE) << "+++ assert DDS type: name=" << DDS_TypeCode_name(type_tc, &ex) << ", ros_type=" << ros_type << std::endl;
      }
      std::tie(new_type, new_asserted, already_asserted) =
        type_cache_.assert_dds_type(type_tc, ros_type, demangled_ros_type);
    }
  } else {
    if (topic_name.size() > 0) {
      LOG(TRACE) << "+++ assert ROS topic: topic_name=" << topic_name << ", type_name=" << type_fqname << std::endl;
      std::tie(new_topic, new_type, new_asserted, already_asserted) =
        type_cache_.assert_ros_topic(topic_name, type_fqname);
    } else {
      LOG(TRACE) << "+++ assert ROS type: name=" << type_fqname << std::endl;
      std::tie(new_type, new_asserted, already_asserted) =
        type_cache_.assert_ros_type(type_fqname);
    }
  }
  DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
  for (const auto & new_t : new_asserted) {
    LOG(INFO) << "+++ asserted: " << DDS_TypeCode_name(new_t, &ex) << std::endl;
    output_->emit_type(new_t);
  }
  for (const auto & old_t : already_asserted) {
    LOG(DEBUG) << "--- cached  : " << DDS_TypeCode_name(old_t, &ex) << std::endl;
  }
  if (topic_name.size() > 0) {
    auto topic_tc = (new_type) ? new_asserted.back() : already_asserted.back();
    std::string tc_name = DDS_TypeCode_name(topic_tc, &ex);
    if (DDS_NO_EXCEPTION_CODE != ex) {
      throw std::runtime_error("failed to get typecode name");
    }
    if (new_topic) {
      LOG(INFO) << "+++ asserted: " << tc_name << "@" << topic_name << std::endl;
      output_->emit_topic(topic_name, topic_tc);
    } else {
      LOG(DEBUG) << "--- cached  : " << tc_name << "@" << topic_name << std::endl;
    }
  }
}

void
BaseTypeMonitor::on_type_detected(const std::string & type_fqname)
{
  on_type_detected("", type_fqname, nullptr);
}

void
BaseTypeMonitor::on_type_detected(const dds::core::xtypes::DynamicType & dyn_type)
{
  DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
  const DDS_TypeCode * type_tc = &dyn_type.native();
  std::string type_fqname = DDS_TypeCode_name(type_tc, &ex);
  if (DDS_NO_EXCEPTION_CODE != ex) {
    throw std::runtime_error("failed to get typecode name");
  }
  on_type_detected("", type_fqname, type_tc);
}

void
BaseTypeMonitor::on_type_detected(const DDS_TypeCode * const tc)
{
  DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
  std::string type_key = DDS_TypeCode_name(tc, &ex);
  if (DDS_NO_EXCEPTION_CODE != ex) {
    throw std::runtime_error("failed to get typecode name");
  }
  return on_type_detected("", type_key, tc);
}

void
BaseTypeMonitor::on_topic_detected(
  const std::string & topic_name,
  const std::string & type_name)
{
  on_type_detected(topic_name, type_name, nullptr);
}

void
BaseTypeMonitor::on_topic_detected(
  const std::string & topic_name,
  const dds::core::xtypes::DynamicType & dyn_type)
{
  DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
  const DDS_TypeCode * type_tc = &dyn_type.native();
  std::string type_fqname = DDS_TypeCode_name(type_tc, &ex);
  if (DDS_NO_EXCEPTION_CODE != ex) {
    throw std::runtime_error("failed to get typecode name");
  }
  on_type_detected(topic_name, type_fqname, type_tc);
}

void
BaseTypeMonitor::on_topic_detected(
  const std::string & topic_name,
  const DDS_TypeCode * const tc)
{
  DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
  std::string type_key = DDS_TypeCode_name(tc, &ex);
  if (DDS_NO_EXCEPTION_CODE != ex) {
    throw std::runtime_error("failed to get typecode name");
  }
  return on_type_detected(topic_name, type_key, tc);
}
}  // namespace robotspy
