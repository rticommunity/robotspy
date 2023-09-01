// (c) 2023 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.
#ifndef ROBOTSPY__TYPECODES_HPP_
#define ROBOTSPY__TYPECODES_HPP_

extern "C" {
#include "robotspy/dds_request_reply.h"
}  // extern "C"

namespace robotspy
{
inline
DDS_TCKind
type_id_ros_to_dds(const uint8_t ros_type_id)
{
  switch (ros_type_id) {
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL:
      {
        return DDS_TK_BOOLEAN;
      }
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE:
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
      {
        return DDS_TK_OCTET;
      }
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
      {
        return DDS_TK_CHAR;
      }
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT32:
      {
        return DDS_TK_FLOAT;
      }
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64:
      {
        return DDS_TK_DOUBLE;
      }
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
      {
        return DDS_TK_SHORT;
      }
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
      {
        return DDS_TK_USHORT;
      }
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
      {
        return DDS_TK_LONG;
      }
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
      {
        return DDS_TK_ULONG;
      }
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
      {
        return DDS_TK_LONGLONG;
      }
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
      {
        return DDS_TK_ULONGLONG;
      }
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
      {
        return DDS_TK_STRING;
      }
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
      {
        return DDS_TK_WSTRING;
      }
    case ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE:
      {
        return DDS_TK_STRUCT;
      }
    default:
      {
        return DDS_TK_NULL;
      }
  }
}

namespace typecodes
{
inline DDS_TypeCode *
GUID()
{
  return dds_msg_GUID_get_typecode();
}

inline DDS_TypeCode *
ReplyHeader()
{
  return dds_msg_ReplyHeader_get_typecode();
}

inline DDS_TypeCode *
RequestHeader()
{
  return dds_msg_RequestHeader_get_typecode();
}

inline DDS_TypeCode *
CycloneRequestHeader()
{
  return dds_msg_CycloneRequestHeader_get_typecode();
}

inline DDS_TypeCode *
SampleIdentity()
{
  return dds_msg_SampleIdentity_get_typecode();
}

inline DDS_TypeCode *
SequenceNumber()
{
  return dds_msg_SequenceNumber_get_typecode();
}
}  // namespace typecodes
}  // namespace robotspy

#endif  // ROBOTSPY__TYPECODES_HPP_
