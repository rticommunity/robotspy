// (c) 2023 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.
#include "rcpputils/scope_exit.hpp"
#include "robotspy/dds_input_emitter.hpp"
#include "robotspy/log.hpp"

namespace robotspy
{
DDSInputEmitter::DDSInputEmitter(const DDSInputEmitterOptions & options)
: BaseInputEmitter(options),
  options_(options)
{
  for (auto & participant : options_.participants) {
    monitor_participant(participant);
  }
}

DDSInputEmitter::~DDSInputEmitter()
{

}


void
DDSInputEmitter::monitor_participant(
  dds::domain::DomainParticipant participant)
{
  LOG(INFO) << "+++ dds monitor: domain=" << participant.domain_id() << std::endl;
  auto sub = dds::sub::builtin_subscriber(participant);

  int32_t found =
    dds::sub::find<dds::sub::DataReader<dds::topic::SubscriptionBuiltinTopicData>>(
    sub,
    dds::topic::subscription_topic_name(),
    std::back_inserter(readers_sub_));
  if (found != 1) {
    throw std::runtime_error("failed to lookup built-in subscriptions DataReader");
  }
  const size_t reader_sub_i = readers_sub_.size() - 1;
  reader_conditions_.emplace_back(
    dds::sub::cond::ReadCondition(
      readers_sub_.back(),
      dds::sub::status::DataState::any_data(),
      [this, reader_sub_i]() {
        on_reader_data(readers_sub_[reader_sub_i]);
      }
  ));

  found = dds::sub::find<dds::sub::DataReader<dds::topic::PublicationBuiltinTopicData>>(
    sub,
    dds::topic::publication_topic_name(),
    std::back_inserter(readers_pub_));
  if (found != 1) {
    throw std::runtime_error("failed to lookup built-in publication DataReader");
  }
  const size_t reader_pub_i = readers_pub_.size() - 1;
  reader_conditions_.emplace_back(
    dds::sub::cond::ReadCondition(
      readers_pub_.back(),
      dds::sub::status::DataState::any_data(),
      [this, reader_pub_i]() {
        on_reader_data(readers_pub_[reader_pub_i]);
      }
  ));
}

void
DDSInputEmitter::open()
{
  const rti::core::cond::AsyncWaitSetProperty waitset_props;
  LOG(DEBUG) << "creating async-waitset..." << std::endl;
  waitset_ = rti::core::cond::AsyncWaitSet(waitset_props);
  for (auto & condition : reader_conditions_) {
    LOG(DEBUG) << "attaching reader condition: "
      << condition.data_reader().topic_name() << std::endl;
    waitset_ += condition;
  }
  waitset_->start();
  BaseInputEmitter::open();
  reader_thread_active_ = reader_thread_active_ || options_.participants.size() > 0;
}

void
DDSInputEmitter::close()
{
  if (nullptr != waitset_) {
    LOG(DEBUG) << "stopping async-waitset.." << std::endl;
    waitset_->stop();
    for (auto & condition : reader_conditions_) {
      waitset_ -= condition;
    }
    waitset_ = nullptr;
    LOG(DEBUG) << "async-waitset stopped." << std::endl;
  }
  BaseInputEmitter::close();
}

bool
DDSInputEmitter::is_active() const
{
  return BaseInputEmitter::is_active() ||
    (active_ && options_.participants.size() > 0);
}

void
DDSInputEmitter::reader_thread_complete()
{
  LOG(DEBUG) << "reader thread complete" << std::endl;
  // active_ = false;
  reader_thread_active_ = options_.participants.size() > 0;
  input_queue_ready_.notify_all();
}
}  // namespace robotspy
