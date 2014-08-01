#include <iostream>
#include <stdexcept>

#include "ndds/ndds_cpp.h"

#include "rosidl_generator_cpp/MessageTypeSupport.h"
#include "ros_middleware_interface/handles.h"
#include "ros_dds_cpp_dynamic_typesupport/MessageTypeSupport.h"

namespace ros_middleware_interface
{

const char * _rti_connext_identifier = "connext";

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

struct TypeCodeAndDataWriter {
  DDSDynamicDataWriter * dynamic_writer_;
  DDS_TypeCode * type_code_;
  ros_dds_cpp_dynamic_typesupport::MessageTypeSupportMembers * members_;
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

    ros_dds_cpp_dynamic_typesupport::MessageTypeSupportMembers * members = (ros_dds_cpp_dynamic_typesupport::MessageTypeSupportMembers*)type_support_handle._data;
    std::string type_name = std::string(members->_package_name) + "/" + members->_message_name;

    std::cout << "  create_publisher() create type code for " << type_name.c_str() << std::endl;
    DDS_TypeCode * type_code;
    {
        DDS_TypeCodeFactory * factory = NULL;
        DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
        DDS_StructMemberSeq struct_members;
        factory = DDS_TypeCodeFactory::get_instance();
        if (!factory) {
            printf("  create_publisher() could not get typecode factory\n");
            throw std::runtime_error("could not get typecode factory");
        };

        type_code = factory->create_struct_tc(type_name.c_str(), struct_members, ex);
        for(unsigned long i = 0; i < members->_size; ++i)
        {
            const ros_dds_cpp_dynamic_typesupport::MessageTypeSupportMember * member = members->_members + i * sizeof(ros_dds_cpp_dynamic_typesupport::MessageTypeSupportMember);
            std::cout << "  create_publisher() create type code - add member " << i << ": " << member->_name << "" << std::endl;
            const DDS_TypeCode * member_type_code;
            if (strcmp(member->_type, "int32") == 0)
            {
                member_type_code = factory->get_primitive_tc(DDS_TK_LONG);
            }
            else
            {
                printf("unknown type %s\n", member->_type);
                throw std::runtime_error("unknown type");
            }
            type_code->add_member(member->_name, DDS_TYPECODE_MEMBER_ID_INVALID, member_type_code,
                        DDS_TYPECODE_NONKEY_REQUIRED_MEMBER, ex);
        }
        DDS_StructMemberSeq_finalize(&struct_members);
    }


    std::cout << "  create_publisher() register type code" << std::endl;
    DDSDynamicDataTypeSupport* ddts = new DDSDynamicDataTypeSupport(
        type_code, DDS_DYNAMIC_DATA_TYPE_PROPERTY_DEFAULT);
    DDS_ReturnCode_t status = ddts->register_type(participant, type_name.c_str());
    if (status != DDS_RETCODE_OK) {
        printf("register_type() failed. Status = %d\n", status);
        throw std::runtime_error("register type failed");
    };


    DDS_PublisherQos publisher_qos;
    status = participant->get_default_publisher_qos(publisher_qos);
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

    DDSDynamicDataWriter* dynamic_writer = DDSDynamicDataWriter::narrow(topic_writer);
    if (!dynamic_writer) {
        printf("narrow() failed.\n");
        throw std::runtime_error("narrow datawriter failed");
    };


    std::cout << "  create_publisher() build opaque publisher handle" << std::endl;
    TypeCodeAndDataWriter* type_code_and_data_writer = new TypeCodeAndDataWriter();
    type_code_and_data_writer->dynamic_writer_ = dynamic_writer;
    type_code_and_data_writer->type_code_ = type_code;
    type_code_and_data_writer->members_ = members;

    ros_middleware_interface::PublisherHandle publisher_handle = {
        _rti_connext_identifier,
        type_code_and_data_writer
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
    TypeCodeAndDataWriter * type_code_and_data_writer = (TypeCodeAndDataWriter*)publisher_handle._data;
    DDSDynamicDataWriter * dynamic_writer = type_code_and_data_writer->dynamic_writer_;
    DDS_TypeCode * type_code = type_code_and_data_writer->type_code_;
    const ros_dds_cpp_dynamic_typesupport::MessageTypeSupportMembers * members = type_code_and_data_writer->members_;

    std::cout << "  publish() create " << members->_package_name << "/" << members->_message_name << " and populate dynamic data" << std::endl;
    DDS_DynamicData dynamic_data(type_code, DDS_DYNAMIC_DATA_PROPERTY_DEFAULT);
    for(unsigned long i = 0; i < members->_size; ++i)
    {
        const ros_dds_cpp_dynamic_typesupport::MessageTypeSupportMember * member = members->_members + i * sizeof(ros_dds_cpp_dynamic_typesupport::MessageTypeSupportMember);
        std::cout << "  publish() set member " << i << ": " << member->_name << "" << std::endl;
        DDS_TypeCode * member_type_code;
        if (strcmp(member->_type, "int32") == 0)
        {
            long value = *((long*)((char*)ros_message + member->_offset));
            std::cout << "  publish() set member " << i << ": " << member->_name << " = " << value << std::endl;
            DDS_ReturnCode_t status = dynamic_data.set_long(member->_name, DDS_DYNAMIC_DATA_MEMBER_ID_UNSPECIFIED, value);
            if (status != DDS_RETCODE_OK) {
                printf("set_long() failed. Status = %d\n", status);
                throw std::runtime_error("set member failed");
            };
        }
        else
        {
            printf("unknown type %s\n", member->_type);
            throw std::runtime_error("unknown type");
        }
    }

    std::cout << "  publish() write dynamic data" << std::endl;
    DDS_ReturnCode_t status = dynamic_writer->write(dynamic_data, DDS_HANDLE_NIL);
    if (status != DDS_RETCODE_OK) {
        printf("write() failed. Status = %d\n", status);
        throw std::runtime_error("write failed");
    };
}

}
