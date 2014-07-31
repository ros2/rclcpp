#include <iostream>

#include "ndds/ndds_cpp.h"

#include "rosidl_generator_cpp/MessageTypeSupport.h"

namespace ros_middleware_interface
{

void * create_node()
{
    std::cout << "create_node()" << std::endl;

    std::cout << "  create_node() get_instance" << std::endl;
    DDSDomainParticipantFactory* dpf_ = DDSDomainParticipantFactory::get_instance();
    DDS_DomainId_t domain = 23;

    std::cout << "  create_node() create_participant" << std::endl;
    DDSDomainParticipant* participant = dpf_->create_participant(
        domain, DDS_PARTICIPANT_QOS_DEFAULT, NULL,
        DDS_STATUS_MASK_NONE);

    std::cout << "  create_node() pass opaque node handle" << std::endl;

    return participant;
}

struct TypeCodeAndDataWriter {
  DDSDynamicDataWriter* dynamic_writer_;
  DDS_TypeCode* type_code_;
  rosidl_generator_cpp::MessageTypeSupportMembers members_;
};

void * create_publisher(void * node, const rosidl_generator_cpp::MessageTypeSupportMembers & members, const char * topic_name)
{
    std::cout << "create_publisher()" << std::endl;

    std::cout << "  create_publisher() extract participant from opaque node handle" << std::endl;
    DDSDomainParticipant* participant = (DDSDomainParticipant*)node;

    std::cout << "  create_publisher() create type code" << std::endl;
    DDS_TypeCode * type_code;
    {
        DDS_TypeCodeFactory * factory = NULL;
        DDS_ExceptionCode_t ex = DDS_NO_EXCEPTION_CODE;
        DDS_StructMemberSeq struct_members;
        factory = DDS_TypeCodeFactory::get_instance();

        type_code = factory->create_struct_tc("test_type", struct_members, ex);
        for(unsigned long i = 0; i < members._size; ++i)
        {
            const rosidl_generator_cpp::MessageTypeSupportMember * member = members._members + i * sizeof(rosidl_generator_cpp::MessageTypeSupportMember);
            std::cout << "  create_publisher() create type code - add member " << i << ": " << member->_name << "" << std::endl;
            const DDS_TypeCode * member_type_code;
            if (strcmp(member->_type, "int32") == 0)
            {
                member_type_code = factory->get_primitive_tc(DDS_TK_LONG);
            }
            else
            {
                printf("unknown type %s\n", member->_type);
                return 0;
            }
            type_code->add_member(member->_name, DDS_TYPECODE_MEMBER_ID_INVALID, member_type_code,
                        DDS_TYPECODE_NONKEY_REQUIRED_MEMBER, ex);
        }
        DDS_StructMemberSeq_finalize(&struct_members);
    }


    std::cout << "  create_publisher() register type code" << std::endl;
    DDSDynamicDataTypeSupport* ddts = new DDSDynamicDataTypeSupport(
        type_code, DDS_DYNAMIC_DATA_TYPE_PROPERTY_DEFAULT);
    DDS_ReturnCode_t status = ddts->register_type(participant, "my_type");
    if (status != DDS_RETCODE_OK) {
        printf("register_type() failed. Status = %d\n", status);
        return 0;
    };

    DDS_PublisherQos publisher_qos;
    status = participant->get_default_publisher_qos(publisher_qos);
    if (status != DDS_RETCODE_OK) {
        printf("register_type() failed. Status = %d\n", status);
        return 0;
    };

    std::cout << "  create_publisher() create dds publisher" << std::endl;
    DDSPublisher* dds_publisher = participant->create_publisher(
        publisher_qos, NULL, DDS_STATUS_MASK_NONE);


    DDS_TopicQos default_topic_qos;
    status = participant->get_default_topic_qos(default_topic_qos);
    if (status != DDS_RETCODE_OK) {
        printf("register_type() failed. Status = %d\n", status);
        return 0;
    };

    std::cout << "  create_publisher() create topic" << std::endl;
    DDSTopic* topic = participant->create_topic(
        topic_name, "my_type", default_topic_qos, NULL,
        DDS_STATUS_MASK_NONE
    );


    DDS_DataWriterQos default_datawriter_qos;
    status = participant->get_default_datawriter_qos(default_datawriter_qos);
    if (status != DDS_RETCODE_OK) {
        printf("register_type() failed. Status = %d\n", status);
        return 0;
    };

    std::cout << "  create_publisher() create data writer" << std::endl;
    DDSDataWriter* topic_writer = dds_publisher->create_datawriter(
        topic, default_datawriter_qos,
        NULL, DDS_STATUS_MASK_NONE);

    DDSDynamicDataWriter* dynamic_writer = DDSDynamicDataWriter::narrow(topic_writer);
    if (!dynamic_writer) {
        printf("narrow() failed.\n");
        return 0;
    };


    std::cout << "  create_publisher() build opaque publisher handle" << std::endl;
    TypeCodeAndDataWriter* type_code_and_data_writer = new TypeCodeAndDataWriter();
    type_code_and_data_writer->dynamic_writer_ = dynamic_writer;
    type_code_and_data_writer->type_code_ = type_code;
    type_code_and_data_writer->members_ = members;

    return type_code_and_data_writer;
}

void publish(void * publisher, const void * ros_message)
{
    std::cout << "publish()" << std::endl;

    std::cout << "  publish() extract data writer and type code from opaque publisher handle" << std::endl;
    TypeCodeAndDataWriter * type_code_and_data_writer = (TypeCodeAndDataWriter*)publisher;
    DDSDynamicDataWriter * dynamic_writer = type_code_and_data_writer->dynamic_writer_;
    DDS_TypeCode * type_code = type_code_and_data_writer->type_code_;
    const rosidl_generator_cpp::MessageTypeSupportMembers & members = type_code_and_data_writer->members_;

    std::cout << "  publish() create and populate dynamic data" << std::endl;
    DDS_DynamicData dynamic_data(type_code, DDS_DYNAMIC_DATA_PROPERTY_DEFAULT);
    for(unsigned long i = 0; i < members._size; ++i)
    {
        const rosidl_generator_cpp::MessageTypeSupportMember * member = members._members + i * sizeof(rosidl_generator_cpp::MessageTypeSupportMember);
        std::cout << "  create_publisher() set member " << i << ": " << member->_name << "" << std::endl;
        DDS_TypeCode * member_type_code;
        if (strcmp(member->_type, "int32") == 0)
        {
            long value = *((long*)((char*)ros_message + member->_offset));
            std::cout << "  create_publisher() set member " << i << ": " << member->_name << " = " << value << std::endl;
            DDS_ReturnCode_t status = dynamic_data.set_long(member->_name, DDS_DYNAMIC_DATA_MEMBER_ID_UNSPECIFIED, value);
            if (status != DDS_RETCODE_OK) {
                printf("set_long() failed. Status = %d\n", status);
            };
        }
        else
        {
            printf("unknown type %s\n", member->_type);
            return;
        }
    }

    std::cout << "  publish() write dynamic data" << std::endl;
    DDS_ReturnCode_t status = dynamic_writer->write(dynamic_data, DDS_HANDLE_NIL);
    if (status != DDS_RETCODE_OK) {
        printf("write() failed. Status = %d\n", status);
    };
}

}
