//==============================================================================
// Autonomous Vehicle Library
//
// Description: Class for validating packets according to JSON files containing
//              valid packets and their parameters.
//==============================================================================

#ifndef PACKET_VALIDATOR_H
#define PACKET_VALIDATOR_H

// AVL binary protocol
#include "avl.h"

// JSON parsing
#include "util/json.hpp"
using json = nlohmann::json;

// C++ includes
#include <fstream>

namespace avl
{

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class PacketValidator
{

public:

    //--------------------------------------------------------------------------
    // Name:        PacketValidator constructor
    // Description: Default constructor
    //--------------------------------------------------------------------------
    PacketValidator();

    //--------------------------------------------------------------------------
    // Name:        PacketValidator destructor
    // Description: Default virtual destructor.
    //--------------------------------------------------------------------------
    virtual ~PacketValidator();

    //--------------------------------------------------------------------------
    // Name:        load
    // Description: Loads the packet JSON files containing packet definitions
    //              and their parameters.
    // Arguments:   - commands_path: Path to the command definitions JSON file.
    //              - actions_path: Path to the action definitions JSON file.
    //--------------------------------------------------------------------------
    void load(std::string commands_path, std::string actions_path);

    //--------------------------------------------------------------------------
    // Name:        validate_packet
    // Description: Validates the contents of a packet. Throws an exception if
    //              packet validation fails.
    // Arguments:   - packet: Packet to be validated.
    //              - bsd: True if packet is from BSD, false if it is from FSD.
    //--------------------------------------------------------------------------
    void validate(Packet packet, bool bsd);

private:

    // Flag indicating if the command and action definition files are loaded
    bool loaded = false;

    // JSON file contents for command and action definitions
    json command_defs;
    json action_defs;

private:

    //--------------------------------------------------------------------------
    // Name:        validate_action
    // Description: Validates an action against the action definitions file.
    //              Throws an exception with a reason if the validation fails.
    // Arguments:   - action: Action to validate.
    //              - bsd: True if action is from BSD, false if it is from FSD.
    //--------------------------------------------------------------------------
    void validate_action(Action action, bool bsd);

    //--------------------------------------------------------------------------
    // Name:        validate_command
    // Description: Validates a command against the command definitions file.
    //              Throws an exception with a reason if the validation fails.
    // Arguments:   - command: Command to validate.
    //              - bsd: True if command is from BSD, false if it is from FSD.
    //--------------------------------------------------------------------------
    void validate_command(Command command, bool bsd);

    //--------------------------------------------------------------------------
    // Name:        validate_conflicts
    // Description: Checks whether a set of parameter names has conflicting
    //              entries according to the JSON array listing sets of
    //              conflicting parameters. Throws an exception if there is a
    //              conflict.
    // Arguments:   - params: List of parameter parameters to validate.
    //              - conflicts_def: JSON array listing sets of conflicting
    //                parameters.
    //--------------------------------------------------------------------------
    void validate_conflicts(ParameterList params, json conflicts_def);

    //--------------------------------------------------------------------------
    // Name:        get_by_name
    // Description: Searches a JSON array for an object that has a "name" tag
    //              with the given value.
    // Arguments:   - array: Array to search in for object with given name.
    //              - name: Name value to search for.
    //              - object: Object found with matching name.
    // Returns:     True if the object with the name was found, false if it was
    //              not found.
    //--------------------------------------------------------------------------
    template <class T>
    size_t count_subset(std::vector<T> vec, std::vector<T> subset)
    {
        return std::count_if(vec.begin(), vec.end(),
            [subset](T val) {
                return std::count(subset.begin(), subset.end(), val);
            }
        );
    }

    //--------------------------------------------------------------------------
    // Name:        get_by_name
    // Description: Searches a JSON array for an object that has a "name" tag
    //              with the given value.
    // Arguments:   - array: Array to search in for object with given name.
    //              - name: Name value to search for.
    //              - object: Object found with matching name.
    // Returns:     True if the object with the name was found, false if it was
    //              not found.
    //--------------------------------------------------------------------------
    json get_by_name(json array, std::string name, std::string source);

};

}

#endif // PACKET_VALIDATOR_H
