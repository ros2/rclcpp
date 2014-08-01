#ifndef __ros_middleware_interface__handles__h__
#define __ros_middleware_interface__handles__h__

namespace ros_middleware_interface
{

typedef struct NodeHandle {
  const char * _implementation_identifier;
  void * _data;
} NodeHandle;

typedef struct PublisherHandle {
  const char * _implementation_identifier;
  void * _data;
} PublisherHandle;

}

#endif  // __ros_middleware_interface__handles__h__
