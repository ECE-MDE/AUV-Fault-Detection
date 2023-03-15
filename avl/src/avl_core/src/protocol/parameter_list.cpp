//==============================================================================
// Autonomous Vehicle Library
//
// Description: A class consisting of a list of parameters and functions to
//              configure the list and access parameters.
//==============================================================================

#include "protocol/parameter_list.h"

namespace avl
{

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        ParameterList constructor
// Description: Default constructor.
//------------------------------------------------------------------------------
ParameterList::ParameterList()
{

}

//------------------------------------------------------------------------------
// Name:        ParameterList constructor
// Description: Constructs a parameter list from a vector of fields
//              representing name, type, and value in that order for each
//              parameter.
// Arguments:   - fields: Parameter name, type, and value fields for each
//                parameter.
//------------------------------------------------------------------------------
ParameterList::ParameterList(std::vector<Field> fields)
{
    from_fields(fields);
}

//------------------------------------------------------------------------------
// Name:        ParameterList destructor
// Description: Default virtual destructor.
//------------------------------------------------------------------------------
ParameterList::~ParameterList()
{

}

//------------------------------------------------------------------------------
// Name:        begin
// Description: Parameter list begin iterator.
// Returns:     Parameter list begin iterator.
//------------------------------------------------------------------------------
std::vector<Parameter>::iterator ParameterList::begin()
{
    return parameters.begin();
}

//------------------------------------------------------------------------------
// Name:        end
// Description: Parameter list end iterator.
// Returns:     Parameter list end iterator.
//------------------------------------------------------------------------------
std::vector<Parameter>::iterator ParameterList::end()
{
    return parameters.end();
}

//------------------------------------------------------------------------------
// Name:        from_fields
// Description: Configures the parameter list from a vector of fields.
//              The vector of fields must contain fields for name, type, and
//              value for each parameter.
// Arguments:   - fields: Vector of fields representing the parameters.
//------------------------------------------------------------------------------
void ParameterList::from_fields(std::vector<Field> fields)
{

    // The vector of fields must contain exactly three fields for each
    // parameter
    if (fields.size() % 3 != 0)
        throw std::runtime_error("from_fields: parameter list must have "
            "exactly three fields per parameter");

    try
    {
        parameters.clear();
        for (size_t i = 0; i < fields.size(); i+=3)
            parameters.push_back(Parameter(avl::subvector(fields, i, 3)));
    }
    catch (const std::exception& ex)
    {
        std::string msg = std::string("from_fields: unable to parse fields "
            "into parameter list (") + ex.what() + ")";
        throw std::runtime_error(msg);
    }

}

//------------------------------------------------------------------------------
// Name:        to_fields
// Description: Converts the parameter list to a vector of Fields. Each
//              parameter in the list is converted into three fields (name,
//              type, and value).
// Arguments:   - start_desc: Field descriptor of the first field's name
//                field. The following fields will have field descriptors
//                incremented after this start value.
// Returns:     Vector of fields representing the parameters.
//------------------------------------------------------------------------------
std::vector<Field> ParameterList::to_fields(uint8_t start_desc)
{
    std::vector<Field> fields;
    for (size_t i = 0; i < parameters.size(); i++)
        avl::append(fields, parameters.at(i).to_fields(3*i+start_desc));
    return fields;
}

//------------------------------------------------------------------------------
// Name:        from_bytes
// Description: Configures the parameter list from a vector of bytes.
//              The vector of bytes must contain fields for name, type, and
//              value for each parameter.
// Arguments:   - data: Vector of bytes representing the parameters.
//------------------------------------------------------------------------------
void ParameterList::from_bytes(std::vector<uint8_t> data)
{
    std::vector<Field> fields = Field::parse_multiple(data);
    from_fields(fields);
}

//------------------------------------------------------------------------------
// Name:        to_bytes
// Description: Converts the parameter list to a vector of bytes.
// Returns:     Vector of bytes representing the parameters.
//------------------------------------------------------------------------------
std::vector<uint8_t> ParameterList::to_bytes()
{
    std::vector<Field> fields = to_fields(0x00);
    std::vector<uint8_t> data;

    for (size_t i = 0; i < fields.size(); i++)
        avl::append(data, fields.at(i).get_bytes());
    return data;
}

//------------------------------------------------------------------------------
// Name:        add
// Description: Adds a parameter to the list.
// Arguments:   - param: Parameter to be added.
//------------------------------------------------------------------------------
void ParameterList::add(Parameter param)
{
    parameters.push_back(param);
}


//------------------------------------------------------------------------------
// Name:        size
// Description: Gets the number of parameters in the list.
// Returns:     Number of parameters in the list.
//------------------------------------------------------------------------------
size_t ParameterList::size()
{
    return parameters.size();
}

//------------------------------------------------------------------------------
// Name:        has
// Description: Checks if a parameter with the given name is in the list.
// Arguments:   - name: Parameter name.
// Returns:     True if a parameter with the name is in the list, false
//              otherwise.
//------------------------------------------------------------------------------
bool ParameterList::has(std::string name)
{
    for (const auto& param : parameters)
        if (param.name == name)
            return true;
    return false;
}

//------------------------------------------------------------------------------
// Name:        get
// Description: Gets a parameter by parameter number.
// Arguments:   - num: Parameter number.
// Returns:     Parameter at given umber in parameter list.
//------------------------------------------------------------------------------
Parameter ParameterList::get(size_t num)
{
    return parameters.at(num);
}

//------------------------------------------------------------------------------
// Name:        get
// Description: Gets a parameter from the list by name.
// Arguments:   - name: Parameter name.
// Returns:     Parameter with given name.
//------------------------------------------------------------------------------
Parameter ParameterList::get(std::string name)
{
    for (const auto& param : parameters)
        if (param.name == name)
            return param;
    throw std::runtime_error("get_parameter: no parameter with name " +
        name);
}

//------------------------------------------------------------------------------
// Name:        get_names
// Description: Gets a vector of parameter names in the parameter list.
// Returns:     Vector of parameter names in the parameter list.
//------------------------------------------------------------------------------
std::vector<std::string> ParameterList::get_names()
{
    std::vector<std::string> param_names;
    for (const auto& param : parameters)
        param_names.push_back(param.name);
    return param_names;
}

}
