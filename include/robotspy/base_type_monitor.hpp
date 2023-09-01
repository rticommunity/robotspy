// (c) 2023 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.
#ifndef ROBOTSPY__BASE_TYPE_MONITOR_HPP_
#define ROBOTSPY__BASE_TYPE_MONITOR_HPP_

#include <regex>
#include <atomic>

#include "dds/dds.hpp"

#include "robotspy/typecache.hpp"
#include "robotspy/output_emitter.hpp"
#include "robotspy/input_emitter.hpp"

namespace robotspy
{
struct BaseTypeMonitorOptions
{
  bool include_non_ros{true};
  std::string type_filter{{".*"}};
  std::string raw_type_filter{{".*"}};
  TypeCacheOptions cache;
};

class BaseTypeMonitor
{
public:
  explicit BaseTypeMonitor(
    std::shared_ptr<InputEmitter> input,
    std::shared_ptr<OutputEmitter> output,
    const BaseTypeMonitorOptions & options);

  virtual ~BaseTypeMonitor();

  virtual void start();

  virtual void stop();

  virtual void consume_input();

  // virtual void wait_for_exit();

  void
  on_type_detected(const std::string & type_name);

  void
  on_type_detected(const dds::core::xtypes::DynamicType & dyn_type);

  void
  on_type_detected(const DDS_TypeCode * const tc);

  void
  on_topic_detected(const std::string & topic_name, const std::string & type_name);

  void
  on_topic_detected(
    const std::string & topic_name,
    const dds::core::xtypes::DynamicType & dyn_type);

  void
  on_topic_detected(const std::string & topic_name, const DDS_TypeCode * const tc);

protected:
  void
  on_type_detected(
    const std::string & topic_name,
    const std::string & type_fqname,
    const DDS_TypeCode * const tc);

  bool
  filter_type_name(const std::string & type_name);

private:
  const BaseTypeMonitorOptions options_;

protected:
  std::shared_ptr<InputEmitter> input_;
  std::shared_ptr<OutputEmitter> output_;
  TypeCache type_cache_;
  std::regex type_filter_;
  std::regex raw_type_filter_;
  std::mutex active_mutex_;
  std::atomic_bool active_{true};
};
}  // namespace robotspy


#endif  // ROBOTSPY__BASE_TYPE_MONITOR_HPP_
