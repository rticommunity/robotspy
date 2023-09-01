// (c) 2023 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.
#ifndef ROBOTSPY__INPUT_EMITTER_HPP_
#define ROBOTSPY__INPUT_EMITTER_HPP_

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

namespace robotspy
{

class NoInputException : public std::exception
{
public:
  NoInputException()
  : msg_("no more input available") {}
  const char * what()
  {
    return msg_.c_str();
  }

private:
  std::string msg_;
};

class InputEmitter
{
public:
  virtual ~InputEmitter() = default;

  virtual void open() = 0;

  virtual void close() = 0;

  virtual bool is_active() const = 0;

  virtual
  std::tuple<std::string, std::string, DDS_TypeCode *>
  next(const std::chrono::milliseconds & timeout, const bool block = false) = 0;

  virtual
  std::tuple<std::string, std::string, DDS_TypeCode *>
  next() = 0;
};
}  // namespace robotspy
#endif  // ROBOTSPY__INPUT_EMITTER_HPP_
