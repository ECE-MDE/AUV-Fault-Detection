//==============================================================================
// Autonomous Vehicle Library
//
// Description: A parameter of variable type. Consists of a name, a type, and
//              a value. Can be converted to/from a set of packet fields. The
//              parameter name, type, and value are each represented by a field.
//==============================================================================

#ifndef PARAMETER_H
#define PARAMETER_H

// Field class
#include "field.h"

namespace avl
{

//==============================================================================
//                            STRUCT DEFINITION
//==============================================================================

// Parameter value data type enum
enum Type
{
    TYPE_BOOL,
    TYPE_INT,
    TYPE_DOUBLE,
    TYPE_STRING,
    TYPE_BOOL_VECTOR,
    TYPE_INT_VECTOR,
    TYPE_DOUBLE_VECTOR
};

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class Parameter
{

public:

    // Parameter name
    std::string name;

    // Data type of the parameter value
    Type type;

    // Parameter value as a vector of bytes. Can be converted to the parameter's
    // type by the to
    std::vector<uint8_t> value;

public:

    //--------------------------------------------------------------------------
    // Name:        type_to_string
    // Description: Converts a parameter type to a string.
    // Arguments:   - type: Type to get string representation of.
    // Returns:     String representing type.
    //--------------------------------------------------------------------------
    static std::string type_to_string(Type type);

public:

    //--------------------------------------------------------------------------
    // Name:        Parameter constructor
    // Description: Default constructor.
    //--------------------------------------------------------------------------
    Parameter();

    //--------------------------------------------------------------------------
    // Name:        Parameter constructor
    // Description: Constructs a parameter from name, type, and value.
    // Arguments:   - name: Parameter name.
    //              - type: Parameter value data type.
    //              - value: Vector of bytes representing the parameter value.
    //--------------------------------------------------------------------------
    Parameter(std::string name, Type type, std::vector<uint8_t> value) ;

    //--------------------------------------------------------------------------
    // Name:        Parameter constructor
    // Description: Constructs a boolean parameter from name and value.
    // Arguments:   - name: Parameter name.
    //              - value: Parameter value.
    //--------------------------------------------------------------------------
    Parameter(std::string name, bool value);

    //--------------------------------------------------------------------------
    // Name:        Parameter constructor
    // Description: Constructs an int parameter from name and value.
    // Arguments:   - name: Parameter name.
    //              - value: Parameter value.
    //--------------------------------------------------------------------------
    Parameter(std::string name, int value);
    //--------------------------------------------------------------------------
    // Name:        Parameter constructor
    // Description: Constructs a double parameter from name and value.
    // Arguments:   - name: Parameter name.
    //              - value: Parameter value.
    //--------------------------------------------------------------------------
    Parameter(std::string name, double value);

    //--------------------------------------------------------------------------
    // Name:        Parameter constructor
    // Description: Constructs a string parameter from name and value.
    // Arguments:   - name: Parameter name.
    //              - value: Parameter value.
    //--------------------------------------------------------------------------
    Parameter(std::string name, std::string value);

    //--------------------------------------------------------------------------
    // Name:        Parameter constructor
    // Description: Constructs a string parameter from name and value. Value is
    //              a string literal.
    // Arguments:   - name: Parameter name.
    //              - value: Parameter value.
    //--------------------------------------------------------------------------
    Parameter(std::string name, const char* value);
    //--------------------------------------------------------------------------
    // Name:        Parameter constructor
    // Description: Constructs a boolean vector parameter from name and value.
    // Arguments:   - name: Parameter name.
    //              - value: Parameter value.
    //--------------------------------------------------------------------------
    Parameter(std::string name, std::vector<bool> value);

    //--------------------------------------------------------------------------
    // Name:        Parameter constructor
    // Description: Constructs an int vector parameter from name and value.
    // Arguments:   - name: Parameter name.
    //              - value: Parameter value.
    //--------------------------------------------------------------------------
    Parameter(std::string name, std::vector<int> value);

    //--------------------------------------------------------------------------
    // Name:        Parameter constructor
    // Description: Constructs a double vector parameter from name and value.
    // Arguments:   - name: Parameter name.
    //              - value: Parameter value.
    //--------------------------------------------------------------------------
    Parameter(std::string name, std::vector<double> value);

    //--------------------------------------------------------------------------
    // Name:        Parameter constructor
    // Description: Constructs a parameter from a vector of three fields
    //              representing name, type, and value in that order.
    // Arguments:   - fields: Parameter name, type, and value fields.
    //--------------------------------------------------------------------------
    Parameter(std::vector<Field> fields);

    //--------------------------------------------------------------------------
    // Name:        Parameter destructor
    // Description: Default virtual destructor.
    //--------------------------------------------------------------------------
    virtual ~Parameter();

    //--------------------------------------------------------------------------
    // Name:        from_field
    // Description: Configures the parameter from a vector of three fields
    //              containing name, type, and value in that order.
    // Arguments:   - fields: Parameter name, type, and value fields.
    //--------------------------------------------------------------------------
    void from_fields(std::vector<Field> fields);

    //--------------------------------------------------------------------------
    // Name:        to_fields
    // Description: Converts the parameter to a vector of three packet fields
    //              containing the parameter name, type, and value.
    // Arguments:   - start_desc: Field descriptor of the first field (name).
    //                The following type and value fields will have descriptors
    //                of start_desc+1 and start_desc+2 respectively.
    // Returns:     Vector of fields containing the parameter name, type,
    //              and value.
    //--------------------------------------------------------------------------
    std::vector<Field> to_fields(uint8_t start_desc=0x00);

    //--------------------------------------------------------------------------
    // Name:        to_bool
    // Description: Gets the parameter value as a bool. Throws an exception if
    //              the parameter type is incorrect.
    // Returns:     Parameter value as a bool.
    //--------------------------------------------------------------------------
    bool to_bool();

    //--------------------------------------------------------------------------
    // Name:        to_int
    // Description: Gets the parameter value as an int. Throws an exception if
    //              the parameter type is incorrect.
    // Returns:     Parameter value as an int.
    //--------------------------------------------------------------------------
    int to_int();

    //--------------------------------------------------------------------------
    // Name:        to_enum
    // Description: Gets the parameter value as an enum. Throws an exception if
    //              the parameter type is incorrect. Must be an int to be
    //              converted to an enum.
    // Returns:     Parameter value as an enum.
    //--------------------------------------------------------------------------
    template <typename T>
    T to_enum()
    {
        if (type != TYPE_INT)
            throw std::runtime_error("to_enum: parameter is not int type");
        return static_cast<T>(avl::from_bytes<int>(value));
    }

    //--------------------------------------------------------------------------
    // Name:        to_double
    // Description: Gets the parameter value as a double. Throws an exception if
    //              the parameter type is incorrect.
    // Returns:     Parameter value as a double.
    //--------------------------------------------------------------------------
    double to_double();

    //--------------------------------------------------------------------------
    // Name:        to_string
    // Description: Gets the parameter value as a string. Throws an exception if
    //              the parameter type is incorrect.
    // Returns:     Parameter value as a string.
    //--------------------------------------------------------------------------
    std::string to_string();

    //--------------------------------------------------------------------------
    // Name:        to_bool_vector
    // Description: Gets the parameter value as a bool vector. Throws an
    //              exception if the parameter type is incorrect.
    // Returns:     Parameter value as a bool vector.
    //--------------------------------------------------------------------------
    std::vector<bool> to_bool_vector();

    //--------------------------------------------------------------------------
    // Name:        to_int_vector
    // Description: Gets the parameter value as an int vector. Throws an
    //              exception if the parameter type is incorrect.
    // Returns:     Parameter value as an int vector.
    //--------------------------------------------------------------------------
    std::vector<int> to_int_vector();

    //--------------------------------------------------------------------------
    // Name:        to_double
    // Description: Gets the parameter value as a double vector. Throws an
    //              exception if the parameter type is incorrect.
    // Returns:     Parameter value as a double vector.
    //--------------------------------------------------------------------------
    std::vector<double> to_double_vector();

};

}

#endif // PARAMETER_H
