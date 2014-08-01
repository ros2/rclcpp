
#ifndef __ros_dds_connext_static__MessageTypeSupport__h__
#define __ros_dds_connext_static__MessageTypeSupport__h__

class DDSDomainParticipant;
class DDSDataWriter;

namespace ros_dds_connext_static
{

const char * _connext_static_identifier = "connext_static";

typedef struct MessageTypeSupportCallbacks {
  const char * _package_name;
  const char * _message_name;
  void (*_register_type)(DDSDomainParticipant * participant, const char * type_name);
  void (*_publish)(DDSDataWriter * topic_writer, const void * ros_message);
} MessageTypeSupportCallbacks;

}  // namespace ros_dds_connext_static

#endif  // __ros_dds_connext_static__MessageTypeSupport__h__
