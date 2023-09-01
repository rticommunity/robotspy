// (c) 2023 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.
#ifndef ROBOTSPY__CLI_HPP_
#define ROBOTSPY__CLI_HPP_

#include "dds/dds.hpp"

namespace robotspy
{

typedef void (*exit_handler_fn)();

int
register_exit_handlers(exit_handler_fn handler);

void force_exit();

void wait_for_exit();

dds::domain::DomainParticipant
create_participant(const int32_t domain_id, const std::string & qos_profile);

}  // namespace robotspy
#endif  // ROBOTSPY__CLI_HPP_
