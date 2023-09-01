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
#include "robotspy/base_input_emitter.hpp"
#include "robotspy/log.hpp"

namespace robotspy
{
BaseInputEmitter::BaseInputEmitter(const BaseInputEmitterOptions & options)
: options_(options)
{
  LOG(INFO) << options_.input_files.size() << " input files" << std::endl;
  for (const auto & input_file : options_.input_files) {
    LOG(INFO) << "input file: "
      << ((input_file == "-")? "stdin" : input_file)  << std::endl;
  }
}

BaseInputEmitter::~BaseInputEmitter()
{

}

void
BaseInputEmitter::open()
{
  bool read_stdin = false;
  input_streams_.clear();
  input_files_.clear();
  for (const auto & input_file : options_.input_files) {
    const bool is_stdin = (input_file == "-");
    read_stdin = read_stdin || is_stdin;
    if (is_stdin) {
      continue;
    }
    LOG(INFO) << "ooo input file: " << input_file << std::endl;
    input_files_.emplace_back(std::ifstream(input_file));
    input_streams_.emplace(input_file, &input_files_.back());
  }
  // Always add stdin as the last stream so that it will be consumed last
  if (read_stdin) {
    LOG(INFO) << "ooo input file: stdin" << std::endl;
    input_streams_.emplace("stdin", &std::cin);
  }
  if (input_streams_.size() == 0) {
    reader_thread_active_ = false;
    return;
  }
  reader_thread_ = std::thread(BaseInputEmitter::reader_thread, this);
}

void
BaseInputEmitter::close()
{
  active_ = false;
  reader_thread_active_ = false;
  {
    std::unique_lock<std::mutex> lock(input_queue_mutex_);
    input_queue_ready_.notify_all();
  }
  if (reader_thread_.joinable()) {
    reader_thread_.join();
  }
  input_streams_.clear();
  input_files_.clear();
}

bool
BaseInputEmitter::is_active() const
{
  return active_;
}


void
BaseInputEmitter::queue_input(
  const std::string & topic_name,
  const std::string & type_name,
  const DDS_TypeCode * const type_tc)
{
  DDS_TypeCode * cloned_tc = nullptr;
  auto tc_factory = DDS_TypeCodeFactory_get_instance();
  if (nullptr == tc_factory) {
    throw std::runtime_error("failed to get typecode factory");
  }
  if (nullptr != type_tc) {
    DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
    cloned_tc = DDS_TypeCodeFactory_clone_tc(tc_factory, type_tc, &ex);
    if (nullptr == cloned_tc) {
      throw std::runtime_error("failed to clone typecode");
    }
  }
  auto scope_exit_cloned = rcpputils::make_scope_exit(
    [tc_factory, cloned_tc]() {
      if (nullptr != cloned_tc) {
        DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
        DDS_TypeCodeFactory_delete_tc(tc_factory, cloned_tc, &ex);
      }
    });
  std::unique_lock<std::mutex> lock(input_queue_mutex_);
  input_queue_.push_back(
    std::make_tuple(topic_name, type_name, cloned_tc));
  LOG(TRACE) << "queued input (" << input_queue_.size() << ")" << std::endl;
  scope_exit_cloned.cancel();
  input_queue_ready_.notify_all();
}

void
BaseInputEmitter::reader_thread(BaseInputEmitter * const self)
{
  for (const auto& [input_file, input_stream]: self->input_streams_) {
    if (!self->is_active()) {
      break;
    }
    LOG(DEBUG) << "consuming input: " << input_file << std::endl;
    while (self->is_active() && input_stream->peek() != EOF) {
      std::string line;
      std::getline(*input_stream, line);
      if (line.size() == 0) {
        // ignore empty lines
        continue;
      }
      auto at_pos = line.find("@");
      std::string next_topic;
      std::string next_type = line;
      if (at_pos != std::string::npos) {
        next_type = line.substr(0, at_pos);
        next_topic = line.substr(at_pos + 1);
      }
      self->queue_input(next_topic, next_type, nullptr);
    }
    LOG(DEBUG) << "consumed input: " << input_file << std::endl;
  }
  self->reader_thread_complete();
}

void
BaseInputEmitter::reader_thread_complete()
{
  LOG(DEBUG) << "reader thread complete" << std::endl;
  // active_ = false;
  reader_thread_active_ = false;
  input_queue_ready_.notify_all();
}

std::tuple<std::string, std::string, DDS_TypeCode *>
BaseInputEmitter::next(const std::chrono::milliseconds & timeout, const bool block)
{
  std::unique_lock<std::mutex> lock(input_queue_mutex_);
  auto now = std::chrono::system_clock::now();
  if (is_active() && reader_thread_active_ && input_queue_.size() == 0) {
    auto predicate = [this](){ 
      return !is_active() || !reader_thread_active_ || input_queue_.size() > 0;
    };
    if (block) {
      input_queue_ready_.wait(lock, predicate);
    } else if (timeout.count() > 0) {
      input_queue_ready_.wait_until(lock, now + timeout, predicate);
    }
  }
  if (is_active() && input_queue_.size() > 0) {
    auto next = input_queue_.front();
    input_queue_.pop_front();
    LOG(TRACE) << "popped input (" << input_queue_.size() << ")" << std::endl;
    return next;
  }
  throw NoInputException();
}

std::tuple<std::string, std::string, DDS_TypeCode *>
BaseInputEmitter::next()
{
  return next(std::chrono::milliseconds(0), true);
}
}  // namespace robotspy
