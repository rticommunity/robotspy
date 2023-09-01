// (c) 2023 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.
#ifndef ROBOTSPY__OUTPUT_EMITTER_HPP_
#define ROBOTSPY__OUTPUT_EMITTER_HPP_

#include <string>
#include "dds/dds.hpp"

namespace robotspy
{
class OutputEmitter
{
public:
  virtual ~OutputEmitter() = default;

  virtual void open() = 0;

  virtual void close() = 0;

  virtual void emit_type(const DDS_TypeCode * const type) = 0;

  virtual void emit_topic(
    const std::string & topic_name,
    const DDS_TypeCode * const topic_type) = 0;
};
}  // namespace robotspy
#endif  // ROBOTSPY__OUTPUT_EMITTER_HPP_
