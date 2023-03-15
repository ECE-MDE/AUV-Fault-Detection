//==============================================================================
// Autonomous Vehicle Library
//
// Description: Class for validating packets according to JSON files containing
//              valid packets and their parameters.
//==============================================================================

#include "protocol/packet_validator.h"

namespace avl
{

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        PacketValidator constructor
// Description: Default constructor
//------------------------------------------------------------------------------
PacketValidator::PacketValidator()
{

}

//------------------------------------------------------------------------------
// Name:        PacketValidator destructor
// Description: Default virtual destructor.
//------------------------------------------------------------------------------
PacketValidator::~PacketValidator()
{

}

//------------------------------------------------------------------------------
// Name:        load
// Description: Loads the packet JSON files containing packet definitions
//              and their parameters.
// Arguments:   - commands_path: Path to the command definitions JSON file.
//              - actions_path: Path to the action definitions JSON file.
//------------------------------------------------------------------------------
void PacketValidator::load(std::string commands_path, std::string actions_path)
{

    // Load the commands definitions JSON file
    try
    {
        std::ifstream command_ifs(commands_path);
        command_defs = json::parse(command_ifs)["command"];
    }
    catch (const std::exception& ex)
    {
        throw std::runtime_error("unable to load file " + commands_path +
            " (" + ex.what() + ")");
    }

    // Load the action definitions JSON file
    try
    {
        std::ifstream action_ifs(actions_path);
        action_defs = json::parse(action_ifs)["mission"];
    }
    catch (const std::exception& ex)
    {
        throw std::runtime_error("unable to load file " + actions_path +
            " (" + ex.what() + ")");
    }

    loaded = true;

}

//------------------------------------------------------------------------------
// Name:        validate_packet
// Description: Validates the contents of a packet. Throws an exception if
//              packet validation fails.
// Arguments:   - packet: Packet to be validated.
//              - bsd: True if packet is from BSD, false if it is from FSD.
//------------------------------------------------------------------------------
void PacketValidator::validate(Packet packet, bool bsd)
{

    // Ensure the definition files are loaded
    if (!loaded)
        throw std::runtime_error("must call load() before validating "
            "packets");

    PacketDescriptor desc = packet.get_descriptor();

    // Validate ACTION packets
    if (desc == ACTION_PACKET)
    {
        ActionPacket action_packet(packet);
        Action action = action_packet.get_action();
        validate_action(action, bsd);
    }

    // Validate COMMAND packets
    else if (desc == COMMAND_PACKET)
    {
        CommandPacket command_packet(packet);
        Command command = command_packet.get_command();
        validate_command(command, bsd);
    }

}

//------------------------------------------------------------------------------
// Name:        validate_action
// Description: Validates an action against the action definitions file.
//              Throws an exception with a reason if the validation fails.
// Arguments:   - action: Action to validate.
//              - bsd: True if action is from BSD, false if it is from FSD.
//------------------------------------------------------------------------------
void PacketValidator::validate_action(Action action, bool bsd)
{

    // If the action in a command action, turn its parameters into a
    // command and validate it as a command
    if (action.name == "COMMAND")
    {

        Command command;
        command.name = action.parameters.get(0).to_string();
        for (size_t i = 1; i < action.parameters.size(); i++)
            command.parameters.add(action.parameters.get(i));
        validate_command(command, bsd);
        return;

    }

    // Check that the action name is in the list of actions
    json action_def = get_by_name(action_defs, action.name, "action");
    json param_defs = action_def["parameters"];

    // Check if the action is allowed from the BSD
    if (bsd && !action_def["bsd"].get<bool>())
        throw std::runtime_error(action.name + " action not allowed from "
            "BSD");

    // Check that all required parameters are provided
    for (const auto& param : param_defs)
    {
        std::string name = param["name"];
        bool required = param["req"];
        bool has_parameter = action.parameters.has(name);
        if (required && !has_parameter)
            throw std::runtime_error("missing required " + name +
                " parameter");
    }

    // Validate each parameter in the action
    for (const auto& param : action.parameters)
    {

        // Check that the param name is in the list of params
        json param_def = get_by_name(param_defs, param.name, "parameter");

        // Check if the parameter is allowed from the BSD
        if (bsd && !param_def["bsd"].get<bool>())
            throw std::runtime_error(action.name + " parameter " +
                param.name + " not allowed from BSD");

        // TODO: Check parameter type matches

    }

    // Check for parameter conflicts in the action if there is a conflict
    // definition
    if (action_def.find("conflicts") != action_def.end())
        validate_conflicts(action.parameters, action_def["conflicts"]);

}

//------------------------------------------------------------------------------
// Name:        validate_command
// Description: Validates a command against the command definitions file.
//              Throws an exception with a reason if the validation fails.
// Arguments:   - command: Command to validate.
//              - bsd: True if command is from BSD, false if it is from FSD.
//------------------------------------------------------------------------------
void PacketValidator::validate_command(Command command, bool bsd)
{

    // Check that the command name is in the list of commands
    json command_def = get_by_name(command_defs, command.name, "command");
    json param_defs = command_def["parameters"];

    // Check if the command is allowed from the BSD
    if (bsd && !command_def["bsd"].get<bool>())
        throw std::runtime_error(command.name + " command not allowed from "
            "BSD");

    // Check that all required parameters are provided
    for (const auto& param : param_defs)
    {
        std::string name = param["name"];
        bool required = param["req"];
        bool has_parameter = command.parameters.has(name);
        if (required && !has_parameter)
            throw std::runtime_error("missing required " + name +
                " parameter");
    }

    // Validate each parameter in the command
    for (const auto& param : command.parameters)
    {

        // Check that the param name is in the list of params
        json param_def = get_by_name(param_defs, param.name, "parameter");

        // Check if the parameter is allowed from the BSD
        if (bsd && !param_def["bsd"].get<bool>())
            throw std::runtime_error(command.name + " parameter " +
                param.name + " not allowed from BSD");

        // TODO: Check parameter type matches

    }

    // Check for parameter conflicts in the command if there is a conflict
    // definition
    if (command_def.find("conflicts") != command_def.end())
        validate_conflicts(command.parameters, command_def["conflicts"]);

}

//------------------------------------------------------------------------------
// Name:        validate_conflicts
// Description: Checks whether a set of parameter names has conflicting
//              entries according to the JSON array listing sets of
//              conflicting parameters. Throws an exception if there is a
//              conflict.
// Arguments:   - params: List of parameter parameters to validate.
//              - conflicts_def: JSON array listing sets of conflicting
//                parameters.
//------------------------------------------------------------------------------
void PacketValidator::validate_conflicts(ParameterList params,
    json conflicts_def)
{

    // Get the list of parameter names from the vector of parameters
    std::vector<std::string> param_names = params.get_names();

    // Turn the JSON object into a vector of sets of conflicting names
    using namespace std;
    vector<vector<string>> conflict_sets = conflicts_def;

    // Check for conflicts in the parameters. If there is more than one
    // element from a conflict set in the list of parameter names, then
    // there is a conflict
    for (const auto& conflict_set : conflict_sets)
         if (count_subset(param_names, conflict_set) > 1)
             throw std::runtime_error("contains multiple conflicting "
                "parameters");

}

//------------------------------------------------------------------------------
// Name:        get_by_name
// Description: Searches a JSON array for an object that has a "name" tag
//              with the given value.
// Arguments:   - array: Array to search in for object with given name.
//              - name: Name value to search for.
//              - object: Object found with matching name.
// Returns:     True if the object with the name was found, false if it was
//              not found.
//------------------------------------------------------------------------------
json PacketValidator::get_by_name(json array, std::string name,
    std::string source)
{
    for (const json& obj : array)
        if (obj["name"].get<std::string>() == name)
            return obj;
    throw std::runtime_error("invalid " + source + " name " + name);
}

}
