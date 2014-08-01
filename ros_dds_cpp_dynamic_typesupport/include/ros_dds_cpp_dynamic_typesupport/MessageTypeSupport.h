
#ifndef __ros_dds_cpp_dynamic_typesupport__MessageTypeSupport__h__
#define __ros_dds_cpp_dynamic_typesupport__MessageTypeSupport__h__

namespace ros_dds_cpp_dynamic_typesupport
{

const char * _dynamic_identifier = "dynamic";

typedef struct MessageTypeSupportMember {
  const char * _name;
  const char * _type;
  unsigned long _offset;
} MessageTypeSupportMember;

typedef struct MessageTypeSupportMembers {
  const char * _package_name;
  const char * _message_name;
  unsigned long _size;
  const MessageTypeSupportMember * _members;
} MessageTypeSupportMembers;

}  // namespace ros_dds_cpp_dynamic_typesupport

#endif  // __ros_dds_cpp_dynamic_typesupport__MessageTypeSupport__h__
