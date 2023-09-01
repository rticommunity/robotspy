// (c) 2023 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.
#ifndef ROBOTSPY__DDS_INPUT_EMITTER_HPP_
#define ROBOTSPY__DDS_INPUT_EMITTER_HPP_

#include <iostream>
#include <fstream>
#include <string>
#include <mutex>
#include <regex>
#include <atomic>
#include <condition_variable>
#include <chrono>
#include <deque>
#include <thread>

#include "dds/dds.hpp"
#include "rti/core/cond/AsyncWaitSet.hpp"
#include "robotspy/base_input_emitter.hpp"
#include "robotspy/log.hpp"

namespace robotspy
{

struct DDSInputEmitterOptions : public BaseInputEmitterOptions
{
  std::vector<dds::domain::DomainParticipant> participants;
};

class DDSInputEmitter : public BaseInputEmitter
{
public:
  explicit DDSInputEmitter(const DDSInputEmitterOptions & options);

  virtual ~DDSInputEmitter();

  virtual void open();

  virtual void close();

  virtual bool is_active() const;
protected:
  virtual void
  reader_thread_complete();

  virtual void
  monitor_participant(dds::domain::DomainParticipant participant);

  template<typename T>
  void
  on_reader_data(dds::sub::DataReader<T> & reader)
  {
    dds::sub::LoanedSamples<T> samples = reader.take();
    for (const auto & sample : samples) {
      if (not sample.info().valid()) {
        continue;
      }
      auto data = sample.data();
      auto opt_dyn_type = data->get_type_no_copy();
      if (opt_dyn_type.is_set()) {
        LOG(DEBUG) << "--- topic++ : " << data.topic_name()
          << " (" << opt_dyn_type.value().name() << ")"  << std::endl;
        queue_input(data.topic_name(), "", &opt_dyn_type.value().native());
      } else {
        LOG(DEBUG) << "--- topic   : " << data.topic_name()
          << " (" << data.type_name() << ")" << std::endl;
        queue_input(data.topic_name(), data.type_name(), nullptr);
      }
    }
  }

private:
  const DDSInputEmitterOptions options_;

protected:
  std::vector<dds::sub::cond::ReadCondition> reader_conditions_;
  std::vector<dds::sub::DataReader<dds::topic::SubscriptionBuiltinTopicData>> readers_sub_;
  std::vector<dds::sub::DataReader<dds::topic::PublicationBuiltinTopicData>> readers_pub_;
  rti::core::cond::AsyncWaitSet waitset_{nullptr};
};
}  // namespace robotspy
#endif  // ROBOTSPY__DDS_INPUT_EMITTER_HPP_
