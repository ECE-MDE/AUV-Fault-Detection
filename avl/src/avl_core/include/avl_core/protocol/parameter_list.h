//==============================================================================
// Autonomous Vehicle Library
//
// Description: A class consisting of a list of parameters and functions to
//              configure the list and access parameters.
//==============================================================================

#ifndef PARAMETER_LIST_H
#define PARAMETER_LIST_H

// Parameter class
#include "parameter.h"

namespace avl
{

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class ParameterList
{

public:

    // List of parameters
    std::vector<Parameter> parameters;

public:

    //--------------------------------------------------------------------------
    // Name:        ParameterList constructor
    // Description: Default constructor.
    //--------------------------------------------------------------------------
    ParameterList();

    //--------------------------------------------------------------------------
    // Name:        ParameterList constructor
    // Description: Constructs a parameter list from a vector of fields
    //              representing name, type, and value in that order for each
    //              parameter.
    // Arguments:   - fields: Parameter name, type, and value fields for each
    //                parameter.
    //--------------------------------------------------------------------------
    ParameterList(std::vector<Field> fields);

    //--------------------------------------------------------------------------
    // Name:        ParameterList destructor
    // Description: Default virtual destructor.
    //--------------------------------------------------------------------------
    virtual ~ParameterList();

    //--------------------------------------------------------------------------
    // Name:        begin
    // Description: Parameter list begin iterator.
    // Returns:     Parameter list begin iterator.
    //--------------------------------------------------------------------------
    std::vector<Parameter>::iterator begin();

    //--------------------------------------------------------------------------
    // Name:        end
    // Description: Parameter list end iterator.
    // Returns:     Parameter list end iterator.
    //--------------------------------------------------------------------------
    std::vector<Parameter>::iterator end();

    //--------------------------------------------------------------------------
    // Name:        from_fields
    // Description: Configures the parameter list from a vector of fields.
    //              The vector of fields must contain fields for name, type, and
    //              value for each parameter.
    // Arguments:   - fields: Vector of fields representing the parameters.
    //--------------------------------------------------------------------------
    void from_fields(std::vector<Field> fields);

    //--------------------------------------------------------------------------
    // Name:        to_fields
    // Description: Converts the parameter list to a vector of Fields. Each
    //              parameter in the list is converted into three fields (name,
    //              type, and value).
    // Arguments:   - start_desc: Field descriptor of the first field's name
    //                field. The following fields will have field descriptors
    //                incremented after this start value.
    // Returns:     Vector of fields representing the parameters.
    //--------------------------------------------------------------------------
    std::vector<Field> to_fields(uint8_t start_desc);

    //--------------------------------------------------------------------------
    // Name:        from_bytes
    // Description: Configures the parameter list from a vector of bytes.
    //              The vector of bytes must contain fields for name, type, and
    //              value for each parameter.
    // Arguments:   - data: Vector of bytes representing the parameters.
    //--------------------------------------------------------------------------
    void from_bytes(std::vector<uint8_t> data);

    //--------------------------------------------------------------------------
    // Name:        to_bytes
    // Description: Converts the parameter list to a vector of bytes.
    // Returns:     Vector of bytes representing the parameters.
    //--------------------------------------------------------------------------
    std::vector<uint8_t> to_bytes();

    //--------------------------------------------------------------------------
    // Name:        add
    // Description: Adds a parameter to the list.
    // Arguments:   - param: Parameter to be added.
    //--------------------------------------------------------------------------
    void add(Parameter param);

    //--------------------------------------------------------------------------
    // Name:        size
    // Description: Gets the number of parameters in the list.
    // Returns:     Number of parameters in the list.
    //--------------------------------------------------------------------------
    size_t size();

    //--------------------------------------------------------------------------
    // Name:        has
    // Description: Checks if a parameter with the given name is in the list.
    // Arguments:   - name: Parameter name.
    // Returns:     True if a parameter with the name is in the list, false
    //              otherwise.
    //--------------------------------------------------------------------------
    bool has(std::string name);

    //--------------------------------------------------------------------------
    // Name:        get
    // Description: Gets a parameter by parameter number.
    // Arguments:   - num: Parameter number.
    // Returns:     Parameter at given umber in parameter list.
    //--------------------------------------------------------------------------
    Parameter get(size_t num);

    //--------------------------------------------------------------------------
    // Name:        get
    // Description: Gets a parameter from the list by name.
    // Arguments:   - name: Parameter name.
    // Returns:     Parameter with given name.
    //--------------------------------------------------------------------------
    Parameter get(std::string name);

    //--------------------------------------------------------------------------
    // Name:        get_names
    // Description: Gets a vector of parameter names in the parameter list.
    // Returns:     Vector of parameter names in the parameter list.
    //--------------------------------------------------------------------------
    std::vector<std::string> get_names();

};

}

#endif // PARAMETER_LIST_H
