// (c) 2023 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.
#ifndef ROBOTSPY__LOG_HPP_
#define ROBOTSPY__LOG_HPP_

#include <ostream>
#include <memory>

namespace robotspy
{
class Logger {
public:
  static const uint8_t LEVEL_TRACE = 5;
  static const uint8_t LEVEL_DEBUG = 4;
  static const uint8_t LEVEL_INFO = 3;
  static const uint8_t LEVEL_WARNING = 2;
  static const uint8_t LEVEL_ERROR = 1;

  virtual ~Logger() = default;

  virtual bool enabled(const int32_t level) const = 0;

  virtual std::ostream & stream() const = 0;
};

Logger& logger();

void log_init(std::shared_ptr<Logger> logger);

#define _robotspy_xconcat(a_, b_)  a_ ## b_
#define _robotspy_concat(a_, b_)  _robotspy_xconcat(a_, b_)
#define LOG(LVL_) \
  if (!robotspy::logger().enabled(_robotspy_concat(robotspy::Logger::LEVEL_, LVL_))) {} \
  else robotspy::logger().stream()

}  // namespace robotspy
#endif  // ROBOTSPY__LOG_HPP_