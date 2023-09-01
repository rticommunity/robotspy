// (c) 2023 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.
#ifndef ROBOTSPY__TYPECODE_MANGLE_HPP_
#define ROBOTSPY__TYPECODE_MANGLE_HPP_

#include "ndds/ndds_c.h"

#include "robotspy/typesupport.hpp"

namespace robotspy
{
class TypeCache;

DDS_TypeCode *
mangle_typecode(const DDS_TypeCode * const tc);

DDS_TypeCode *
demangle_typecode(const DDS_TypeCode * const tc);

}  // namespace robotspy

#endif  // ROBOTSPY__TYPECODE_HPP_
