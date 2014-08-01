import em
import os

from rosidl_parser import parse_message_file


def generate_cpp(pkg_name, interface_files, deps, output_dir, template_dir):
    mapping = {
        os.path.join(template_dir, 'msg_TypeSupport.h.template'): '%s_TypeSupport.h',
    }
    for template_file in mapping.keys():
        assert(os.path.exists(template_file))

    try:
        os.makedirs(output_dir)
    except FileExistsError:
        pass

    for idl_file in interface_files:
        spec = parse_message_file(pkg_name, idl_file)
        for template_file, generated_filename in mapping.items():
            generated_file = os.path.join(output_dir, generated_filename % spec.base_type.type)
            print('Generating: %s' % generated_file)

            try:
                # TODO only touch generated file if its content actually changes
                ofile = open(generated_file, 'w')
                # TODO reuse interpreter
                interpreter = em.Interpreter(
                    output=ofile,
                    options={
                        em.RAW_OPT: True,
                        em.BUFFERED_OPT: True,
                    },
                    globals={'spec': spec},
                )
                interpreter.file(open(template_file))
                interpreter.shutdown()
            except Exception:
                os.remove(generated_file)
                raise

    return 0


MSG_TYPE_TO_CPP = {
    'bool': 'uint8_t',
    'float32': 'float',
    'float64': 'double',
    'uint8': 'uint8_t',
    'int8': 'int8_t',
    'uint16': 'uint16_t',
    'int16': 'int16_t',
    'uint32': 'uint32_t',
    'int32': 'int32_t',
    'uint64': 'uint64_t',
    'int64': 'int64_t',
    'string': 'std::basic_string<char, std::char_traits<char>, ' +
              'typename ContainerAllocator::template rebind<char>::other>',
    'byte': 'int8_t',
    'char': 'uint8_t',
}


def msg_type_to_cpp(type_):
    """
    Convert a message type into the C++ declaration.

    Example input: uint32, std_msgs/String
    Example output: uint32_t, std_msgs::String_<ContainerAllocator>

    @param type: The message type
    @type type: rosidl_parser.Type
    """
    cpp_type = None
    if type_.is_primitive_type():
        cpp_type = MSG_TYPE_TO_CPP[type_.type]
    else:
        cpp_type = '::%s::%s_<ContainerAllocator> ' % \
            (type_.pkg_name, type_.type)

    if type_.is_array:
        if type_.array_size is None:
            return 'std::vector<%s, typename ContainerAllocator::template ' + \
                'rebind<%s>::other > ' % (cpp_type, cpp_type)
        else:
            return 'boost::array<%s, %s> ' % (cpp_type, type_.array_size)
    else:
        return cpp_type


def escape_string(s):
    s = s.replace('\\', '\\\\')
    s = s.replace('"', '\\"')
    return s
