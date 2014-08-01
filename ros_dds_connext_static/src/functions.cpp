#include <iostream>
#include <stdexcept>

#include "ndds/ndds_cpp.h"

#include "rosidl_generator_cpp/MessageTypeSupport.h"
#include "ros_middleware_interface/handles.h"
#include "ros_dds_connext_static/MessageTypeSupport.h"

namespace ros_middleware_interface
{

const char * _rti_connext_identifier = "connext_static";

ros_middleware_interface::NodeHandle create_node()
{
    std::cout << "create_node()" << std::endl;

    std::cout << "  create_node() get_instance" << std::endl;
    DDSDomainParticipantFactory* dpf_ = DDSDomainParticipantFactory::get_instance();
    if (!dpf_) {
        printf("  create_node() could not get participant factory\n");
        throw std::runtime_error("could not get participant factory");
    };

    DDS_DomainId_t domain = 23;

    std::cout << "  create_node() create_participant" << std::endl;
    DDSDomainParticipant* participant = dpf_->create_participant(
        domain, DDS_PARTICIPANT_QOS_DEFAULT, NULL,
        DDS_STATUS_MASK_NONE);
    if (!participant) {
        printf("  create_node() could not create participant\n");
        throw std::runtime_error("could not create participant");
    };

    std::cout << "  create_node() pass opaque node handle" << std::endl;

    ros_middleware_interface::NodeHandle node_handle = {
        _rti_connext_identifier,
        participant
    };
    return node_handle;
}

struct CustomPublisherInfo {
  DDSDataWriter * topic_writer_;
  ros_dds_connext_static::MessageTypeSupportCallbacks * callbacks_;
};

ros_middleware_interface::PublisherHandle create_publisher(const ros_middleware_interface::NodeHandle& node_handle, const rosidl_generator_cpp::MessageTypeSupportHandle & type_support_handle, const char * topic_name)
{
    std::cout << "create_publisher()" << std::endl;

    if (node_handle._implementation_identifier != _rti_connext_identifier)
    {
        printf("node handle not from this implementation\n");
        printf("but from: %s\n", node_handle._implementation_identifier);
        throw std::runtime_error("node handle not from this implementation");
    }

    std::cout << "  create_publisher() extract participant from opaque node handle" << std::endl;
    DDSDomainParticipant* participant = (DDSDomainParticipant*)node_handle._data;

    ros_dds_connext_static::MessageTypeSupportCallbacks * callbacks = (ros_dds_connext_static::MessageTypeSupportCallbacks*)type_support_handle._data;
    std::string type_name = std::string(callbacks->_package_name) + "/" + callbacks->_message_name;


    std::cout << "  create_publisher() invoke register callback" << std::endl;
    callbacks->_register_type(participant, type_name.c_str());


    DDS_PublisherQos publisher_qos;
    DDS_ReturnCode_t status = participant->get_default_publisher_qos(publisher_qos);
    if (status != DDS_RETCODE_OK) {
        printf("get_default_publisher_qos() failed. Status = %d\n", status);
        throw std::runtime_error("get default publisher qos failed");
    };

    std::cout << "  create_publisher() create dds publisher" << std::endl;
    DDSPublisher* dds_publisher = participant->create_publisher(
        publisher_qos, NULL, DDS_STATUS_MASK_NONE);
    if (!dds_publisher) {
        printf("  create_publisher() could not create publisher\n");
        throw std::runtime_error("could not create publisher");
    };


    DDS_TopicQos default_topic_qos;
    status = participant->get_default_topic_qos(default_topic_qos);
    if (status != DDS_RETCODE_OK) {
        printf("get_default_topic_qos() failed. Status = %d\n", status);
        throw std::runtime_error("get default topic qos failed");
    };

    std::cout << "  create_publisher() create topic" << std::endl;
    DDSTopic* topic = participant->create_topic(
        topic_name, type_name.c_str(), default_topic_qos, NULL,
        DDS_STATUS_MASK_NONE
    );
    if (!topic) {
        printf("  create_topic() could not create topic\n");
        throw std::runtime_error("could not create topic");
    };


    DDS_DataWriterQos default_datawriter_qos;
    status = participant->get_default_datawriter_qos(default_datawriter_qos);
    if (status != DDS_RETCODE_OK) {
        printf("get_default_datawriter_qos() failed. Status = %d\n", status);
        throw std::runtime_error("get default datawriter qos failed");
    };

    std::cout << "  create_publisher() create data writer" << std::endl;
    DDSDataWriter* topic_writer = dds_publisher->create_datawriter(
        topic, default_datawriter_qos,
        NULL, DDS_STATUS_MASK_NONE);


    std::cout << "  create_publisher() build opaque publisher handle" << std::endl;
    CustomPublisherInfo* custom_publisher_info = new CustomPublisherInfo();
    custom_publisher_info->topic_writer_ = topic_writer;
    custom_publisher_info->callbacks_ = callbacks;

    ros_middleware_interface::PublisherHandle publisher_handle = {
        _rti_connext_identifier,
        custom_publisher_info
    };
    return publisher_handle;
}

void publish(const ros_middleware_interface::PublisherHandle& publisher_handle, const void * ros_message)
{
    std::cout << "publish()" << std::endl;

    if (publisher_handle._implementation_identifier != _rti_connext_identifier)
    {
        printf("publisher handle not from this implementation\n");
        printf("but from: %s\n", publisher_handle._implementation_identifier);
        throw std::runtime_error("publisher handle not from this implementation");
    }

    std::cout << "  publish() extract data writer and type code from opaque publisher handle" << std::endl;
    CustomPublisherInfo * custom_publisher_info = (CustomPublisherInfo*)publisher_handle._data;
    DDSDataWriter * topic_writer = custom_publisher_info->topic_writer_;
    const ros_dds_connext_static::MessageTypeSupportCallbacks * callbacks = custom_publisher_info->callbacks_;


    std::cout << "  publish() invoke publish callback" << std::endl;
    callbacks->_publish(topic_writer, ros_message);
}

}
