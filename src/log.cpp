// (c) 2023 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.
#include "robotspy/log.hpp"

namespace robotspy
{

std::shared_ptr<Logger> gv_logger = nullptr;

Logger& logger()
{
  if (nullptr == gv_logger) {
    throw std::runtime_error("logger not initialized");
  }
  return *gv_logger;
}

void log_init(std::shared_ptr<Logger> logger) {
  if (nullptr != gv_logger) {
    throw std::runtime_error("logger already initialized");
  }
  gv_logger = logger;
}

}  // namespace robotspy