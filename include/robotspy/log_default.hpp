// (c) 2023 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.
#ifndef ROBOTSPY__LOG_DEFAULT_HPP_
#define ROBOTSPY__LOG_DEFAULT_HPP_

#include "robotspy/log.hpp"

namespace robotspy
{
struct DefaultLoggerOptions
{
  int32_t verbosity{3};
  bool swap_outputs{false};
};
class DefaultLogger : public Logger {
public:
  explicit DefaultLogger(const DefaultLoggerOptions & options)
  : options_(options)
  {}

  virtual ~DefaultLogger() = default;

  virtual bool enabled(const int32_t level) const {
    return options_.verbosity >= level;
  }

  virtual
  std::ostream &
  stream() const
  {
    if (options_.swap_outputs) {
      return std::cout;
    } else {
      return std::cerr;
    }
  }
private:
  DefaultLoggerOptions options_;
};

inline
void log_init_default(const DefaultLoggerOptions & options) {
  auto logger = std::make_shared<DefaultLogger>(options);
  log_init(logger);
}

inline
void log_init_default() {
  const DefaultLoggerOptions options;
  log_init_default(options);
}


}  // namespace robotspy
#endif  // ROBOTSPY__LOG_DEFAULT_HPP_