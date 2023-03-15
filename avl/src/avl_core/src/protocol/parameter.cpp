//==============================================================================
// Autonomous Vehicle Library
//
// Description: A parameter of variable type. Consists of a name, a type, and
//              a value. Can be converted to/from a set of packet fields. The
//              parameter name, type, and value are each represented by a field.
//==============================================================================

#include "protocol/parameter.h"

namespace avl
{

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        type_to_string
// Description: Converts a parameter type to a string.
// Arguments:   - type: Type to get string representation of.
// Returns:     String representing type.
//------------------------------------------------------------------------------
std::string Parameter::type_to_string(Type type)
{
    switch (type)
    {
        case TYPE_BOOL:
            return "bool";
        case TYPE_INT:
            return "int";
        case TYPE_DOUBLE:
            return "double";
        case TYPE_STRING:
            return "string";
        case TYPE_BOOL_VECTOR:
            return "bool vector";
        case TYPE_INT_VECTOR:
            return "int vector";
        case TYPE_DOUBLE_VECTOR:
            return "double vector";
    }
    return "unknown";
}

//------------------------------------------------------------------------------
// Name:        Parameter constructor
// Description: Default constructor.
//------------------------------------------------------------------------------
Parameter::Parameter()
{

}

//------------------------------------------------------------------------------
// Name:        Parameter constructor
// Description: Constructs a parameter from name, type, and value.
// Arguments:   - name: Parameter name.
//              - type: Parameter value data type.
//              - value: Vector of bytes representing the parameter value.
//------------------------------------------------------------------------------
Parameter::Parameter(std::string name, Type type, std::vector<uint8_t> value) :
    name(name), type(type), value(value)
{

}

//------------------------------------------------------------------------------
// Name:        Parameter constructor
// Description: Constructs a boolean parameter from name and value.
// Arguments:   - name: Parameter name.
//              - value: Parameter value.
//------------------------------------------------------------------------------
Parameter::Parameter(std::string name, bool value) : name(name), type(TYPE_BOOL),
    value(avl::to_bytes(value))
{

}

//------------------------------------------------------------------------------
// Name:        Parameter constructor
// Description: Constructs an int parameter from name and value.
// Arguments:   - name: Parameter name.
//              - value: Parameter value.
//------------------------------------------------------------------------------
Parameter::Parameter(std::string name, int value) : name(name), type(TYPE_INT),
    value(avl::to_bytes(value))
{

}

//------------------------------------------------------------------------------
// Name:        Parameter constructor
// Description: Constructs a double parameter from name and value.
// Arguments:   - name: Parameter name.
//              - value: Parameter value.
//------------------------------------------------------------------------------
Parameter::Parameter(std::string name, double value) : name(name),
    type(TYPE_DOUBLE), value(avl::to_bytes(value))
{

}

//------------------------------------------------------------------------------
// Name:        Parameter constructor
// Description: Constructs a string parameter from name and value.
// Arguments:   - name: Parameter name.
//              - value: Parameter value.
//------------------------------------------------------------------------------
Parameter::Parameter(std::string name, std::string value) : name(name),
    type(TYPE_STRING), value(avl::to_bytes(value))
{

}

//------------------------------------------------------------------------------
// Name:        Parameter constructor
// Description: Constructs a string parameter from name and value. Value is
//              a string literal.
// Arguments:   - name: Parameter name.
//              - value: Parameter value.
//------------------------------------------------------------------------------
Parameter::Parameter(std::string name, const char* value) : name(name),
    type(TYPE_STRING)
{
    this->value = avl::to_bytes(std::string(value, std::strlen(value)));
}

//------------------------------------------------------------------------------
// Name:        Parameter constructor
// Description: Constructs a boolean vector parameter from name and value.
// Arguments:   - name: Parameter name.
//              - value: Parameter value.
//------------------------------------------------------------------------------
Parameter::Parameter(std::string name, std::vector<bool> value) : name(name),
    type(TYPE_BOOL_VECTOR), value(avl::to_bytes(value))
{

}

//------------------------------------------------------------------------------
// Name:        Parameter constructor
// Description: Constructs an int vector parameter from name and value.
// Arguments:   - name: Parameter name.
//              - value: Parameter value.
//------------------------------------------------------------------------------
Parameter::Parameter(std::string name, std::vector<int> value) : name(name),
    type(TYPE_INT_VECTOR), value(avl::to_bytes(value))
{

}

//------------------------------------------------------------------------------
// Name:        Parameter constructor
// Description: Constructs a double vector parameter from name and value.
// Arguments:   - name: Parameter name.
//              - value: Parameter value.
//------------------------------------------------------------------------------
Parameter::Parameter(std::string name, std::vector<double> value) : name(name),
    type(TYPE_DOUBLE_VECTOR), value(avl::to_bytes(value))
{

}

//------------------------------------------------------------------------------
// Name:        Parameter constructor
// Description: Constructs a parameter from a vector of three fields
//              representing name, type, and value in that order.
// Arguments:   - fields: Parameter name, type, and value fields.
//------------------------------------------------------------------------------
Parameter::Parameter(std::vector<Field> fields)
{
    from_fields(fields);
}

//------------------------------------------------------------------------------
// Name:        Parameter destructor
// Description: Default virtual destructor.
//------------------------------------------------------------------------------
Parameter::~Parameter()
{

}

//------------------------------------------------------------------------------
// Name:        from_field
// Description: Configures the parameter from a vector of three fields
//              containing name, type, and value in that order.
// Arguments:   - fields: Parameter name, type, and value fields.
//------------------------------------------------------------------------------
void Parameter::from_fields(std::vector<Field> fields)
{

    // The vector of fields must contain exactly three fields for name,
    // type, and value
    if (fields.size() != 3)
        throw std::runtime_error("from_fields: a parameter must be "
            "constructed from exactly three fields");

    try
    {

        // The first field contains the name as a string
        std::vector<uint8_t> name_data = fields.at(0).get_data();
        name = std::string(name_data.begin(), name_data.end());

        // The second field contains the parameter data type as one byte
        type = static_cast<Type>(fields.at(1).get_data().at(0));

        // Check that the value fied contains the correct number of bytes
        // for the specified type
        size_t num_value_bytes = fields.at(2).get_data().size();
        if (type == TYPE_BOOL && num_value_bytes != 1)
                throw std::runtime_error("incorrect number of value bytes "
                    "for a BOOL parameter");
        else if (type == TYPE_INT && num_value_bytes != 4)
                throw std::runtime_error("incorrect number of value bytes "
                    "for an INT parameter");
        else if (type == TYPE_DOUBLE && num_value_bytes != 8)
                throw std::runtime_error("incorrect number of value bytes "
                    "for an DOUBLE parameter");
        else if (type == TYPE_INT_VECTOR && (num_value_bytes % 4) != 0)
                throw std::runtime_error("int vector must have multiple of"
                    " 4 number of bytes");
        else if (type == TYPE_DOUBLE_VECTOR && (num_value_bytes % 8) != 0)
                throw std::runtime_error("double vector must have multiple"
                    " of 8 number of bytes");

        // The third field contains the parameter value bytes
        value = fields.at(2).get_data();

    }
    catch (const std::exception& ex)
    {
        std::string msg = std::string("from_fields: unable to parse fields "
            "into parameter (") + ex.what() + ")";
        throw std::runtime_error(msg);
    }

}

//------------------------------------------------------------------------------
// Name:        to_fields
// Description: Converts the parameter to a vector of three packet fields
//              containing the parameter name, type, and value.
// Arguments:   - start_desc: Field descriptor of the first field (name).
//                The following type and value fields will have descriptors
//                of start_desc+1 and start_desc+2 respectively.
// Returns:     Vector of fields containing the parameter name, type,
//              and value.
//------------------------------------------------------------------------------
std::vector<Field> Parameter::to_fields(uint8_t start_desc)
{
    Field name_field( start_desc,     avl::to_bytes(name));
    Field type_field( start_desc + 1, {static_cast<uint8_t>(type)});
    Field value_field(start_desc + 2, value);
    return {name_field, type_field, value_field};
}

//------------------------------------------------------------------------------
// Name:        to_bool
// Description: Gets the parameter value as a bool. Throws an exception if
//              the parameter type is incorrect.
// Returns:     Parameter value as a bool.
//------------------------------------------------------------------------------
bool Parameter::to_bool()
{
    if (type != TYPE_BOOL)
        throw std::runtime_error("to_bool: parameter is not bool type");
    return avl::from_bytes<bool>(value);
}

//------------------------------------------------------------------------------
// Name:        to_int
// Description: Gets the parameter value as an int. Throws an exception if
//              the parameter type is incorrect.
// Returns:     Parameter value as an int.
//------------------------------------------------------------------------------
int Parameter::to_int()
{
    if (type != TYPE_INT)
        throw std::runtime_error("to_int: parameter is not int type");
    return avl::from_bytes<int>(value);
}

//------------------------------------------------------------------------------
// Name:        to_double
// Description: Gets the parameter value as a double. Throws an exception if
//              the parameter type is incorrect.
// Returns:     Parameter value as a double.
//------------------------------------------------------------------------------
double Parameter::to_double()
{
    if (type != TYPE_DOUBLE)
        throw std::runtime_error("to_double: parameter is not double type");
    return avl::from_bytes<double>(value);
}

//------------------------------------------------------------------------------
// Name:        to_string
// Description: Gets the parameter value as a string. Throws an exception if
//              the parameter type is incorrect.
// Returns:     Parameter value as a string.
//------------------------------------------------------------------------------
std::string Parameter::to_string()
{
    if (type != TYPE_STRING)
        throw std::runtime_error("to_string: parameter is not string type");
    return std::string(value.begin(), value.end());
}

//------------------------------------------------------------------------------
// Name:        to_bool_vector
// Description: Gets the parameter value as a bool vector. Throws an
//              exception if the parameter type is incorrect.
// Returns:     Parameter value as a bool vector.
//------------------------------------------------------------------------------
std::vector<bool> Parameter::to_bool_vector()
{
    if (type != TYPE_BOOL_VECTOR)
        throw std::runtime_error("to_bool_vector: parameter is not bool "
            "vector type");
    return avl::vector_from_bytes<bool>(value);
}

//------------------------------------------------------------------------------
// Name:        to_int_vector
// Description: Gets the parameter value as an int vector. Throws an
//              exception if the parameter type is incorrect.
// Returns:     Parameter value as an int vector.
//------------------------------------------------------------------------------
std::vector<int> Parameter::to_int_vector()
{
    if (type != TYPE_INT_VECTOR)
        throw std::runtime_error("to_int_vector: parameter is not int "
            "vector type");
    return avl::vector_from_bytes<int>(value);
}

//------------------------------------------------------------------------------
// Name:        to_double
// Description: Gets the parameter value as a double vector. Throws an
//              exception if the parameter type is incorrect.
// Returns:     Parameter value as a double vector.
//------------------------------------------------------------------------------
std::vector<double> Parameter::to_double_vector()
{
    if (type != TYPE_DOUBLE_VECTOR)
        throw std::runtime_error("to_double_vector: parameter is not "
            "double vector type");
    return avl::vector_from_bytes<double>(value);
}

}
