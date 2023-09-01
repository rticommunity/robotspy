// (c) 2023 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.
#ifndef ROBOTSPY__BASE_OUTPUT_EMITTER_HPP_
#define ROBOTSPY__BASE_OUTPUT_EMITTER_HPP_

#include <iostream>
#include <fstream>
#include <mutex>
#include <regex>

#include "robotspy/output_emitter.hpp"

namespace robotspy
{
struct BaseOutputEmitterOptions
{
  std::string output_file;
  bool overwrite{false};
  bool append{false};
  bool swap_outputs{false};
};

class BaseOutputEmitter: public OutputEmitter
{
public:
  explicit BaseOutputEmitter(const BaseOutputEmitterOptions & options);

  virtual ~BaseOutputEmitter();

  virtual void open();

  virtual void close();

  virtual void emit_type(const DDS_TypeCode * const type);

  virtual void emit_topic(
    const std::string & topic_name,
    const DDS_TypeCode * const topic_type);

  std::ostream &
  stdout()
  {
    if (options_.output_file.size() > 0) {
      return output_stream_;
    } else {
      if (options_.swap_outputs) {
        return std::cerr;
      } else {
        return std::cout;
      }
    }
  }

protected:
  std::string
  format_type(
    const std::string type_fqname, const DDS_TypeCode * const type);

  std::string
  format_topic(
    const std::string topic_name,
    const std::string topic_type_name,
    const DDS_TypeCode * const topic_type);

private:
  const BaseOutputEmitterOptions options_;

protected:
  std::ofstream output_stream_;
  std::mutex output_mutex_;
};
}  // namespace robotspy
#endif  // ROBOTSPY__BASE_OUTPUT_EMITTER_HPP_
