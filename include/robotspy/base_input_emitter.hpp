// (c) 2023 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.
#ifndef ROBOTSPY__BASE_INPUT_EMITTER_HPP_
#define ROBOTSPY__BASE_INPUT_EMITTER_HPP_

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
#include <tuple>

#include "dds/dds.hpp"

#include "robotspy/input_emitter.hpp"

namespace robotspy
{

struct BaseInputEmitterOptions
{
  std::vector<std::string> input_files;
};

class BaseInputEmitter : public InputEmitter
{
public:
  explicit BaseInputEmitter(const BaseInputEmitterOptions & options);

  virtual ~BaseInputEmitter();

  virtual void open();

  virtual void close();

  virtual bool is_active() const;

  virtual
  std::tuple<std::string, std::string, DDS_TypeCode *>
  next(const std::chrono::milliseconds & timeout, const bool block = false);

  virtual
  std::tuple<std::string, std::string, DDS_TypeCode *>
  next();
protected:
  static
  void
  reader_thread(BaseInputEmitter * const self);  

  virtual void reader_thread_complete();

  virtual void queue_input(
    const std::string & topic_name,
    const std::string & type_name,
    const DDS_TypeCode * const type_tc = nullptr);

private:
  const BaseInputEmitterOptions options_;

protected:
  std::vector<std::ifstream> input_files_;
  std::map<std::string, std::istream *> input_streams_;
  std::thread reader_thread_;
  std::atomic_bool active_{true};
  std::atomic_bool reader_thread_active_{true};
  std::deque<std::tuple<std::string, std::string, DDS_TypeCode *>> input_queue_;
  std::mutex input_queue_mutex_;
  std::condition_variable input_queue_ready_;
};
}  // namespace robotspy
#endif  // ROBOTSPY__BASE_INPUT_EMITTER_HPP_
