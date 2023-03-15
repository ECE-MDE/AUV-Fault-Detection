//==============================================================================
// Autonomous Vehicle Library
//
// Description: Class to handle log messages written to the console and to a
//              file. Files are logged to the console or file in the following
//              format:
//                  [timestamp] [level] [label] <message>
//              The timestamp is given in seconds since epoch. The log level
//              is one of the following levels:
//                  DATA, DEBUG, INFO, WARNING, ERROR
//              The log label us a user defined tag describing the source of the
//              log message. Finally, <message> is the log message logged by the
//              user.
//==============================================================================

#ifndef LOGGER_H
#define LOGGER_H

// Console color styles
#include <avl_core/console_style.h>

// C++ includes
#include <string>
#include <vector>
#include <memory>
#include <time.h>
#include <iostream>
#include <cstdarg>
#include <map>
#include <algorithm>

// Eigen
#include <Eigen/Core>

//==============================================================================
//                              STRUCT DECLARATION
//==============================================================================

// Structure containing logger configuration settings
struct LoggerConfig
{
    bool enable = true;
    bool log_node = true;
    bool log_data = true;
    bool log_debug = true;
    bool log_info = true;
    bool log_warning = true;
    bool log_error = true;
};

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class Logger
{

public:

    //--------------------------------------------------------------------------
    // Name:        Logger constructor
    // Description: Default constructor.
    //--------------------------------------------------------------------------
    Logger();

    //--------------------------------------------------------------------------
    // Name:        Logger destructor
    // Description: Default virtual destructor.
    //--------------------------------------------------------------------------
    virtual ~Logger();

    //--------------------------------------------------------------------------
    // Name:        set_log_name
    // Description: Sets the log name that appears in log messages and the file
    //              name that will be written to if file logging is enabled.
    //              The log filepath is specified with the set_log_filepath
    //              function. The file will be created if it does not exist, or
    //              will be appended to if it does exist.
    // Arguments:   - name: Log name.
    //--------------------------------------------------------------------------
    void set_log_name(std::string name);

    //--------------------------------------------------------------------------
    // Name:        set_log_filepath
    // Description: Sets the log file path that will be written to if file
    //              logging is enabled. This file path should not contain the
    //              file name. The file name is set with the set_log_name
    //              function. The file will be created if it does not exist, or
    //              will be appended to if it does exist.
    // Arguments:   - filepath: Log filepath.
    //--------------------------------------------------------------------------
    void set_log_filepath(std::string filepath);

    //--------------------------------------------------------------------------
    // Name:        configure_file_log
    // Description: Configures which log messages will be logged to file.
    // Arguments:   - config: Logger configuration struct.
    //--------------------------------------------------------------------------
    void configure_file_log(LoggerConfig config);

    //--------------------------------------------------------------------------
    // Name:        configure_console_log
    // Description: Configures which log messages will be logged to the console.
    // Arguments:   - config: Logger configuration struct.
    //--------------------------------------------------------------------------
    void configure_console_log(LoggerConfig config);

    //--------------------------------------------------------------------------
    // Name:        add_data_header
    // Description: Adds a header to the list of data headers to be logged
    //              when the log file is created.
    // Arguments:   - format: Printf style format string.
    //              - ...: Printf style list of variables to be formatted.
    //--------------------------------------------------------------------------
    void add_data_header(const char *format, ...);

    //--------------------------------------------------------------------------
    // Name:        add_data_header
    // Description: Adds a header to the list of data headers to be logged
    //              when the log file is created.
    // Arguments:   - header: Header to add to the list of data headers.
    //--------------------------------------------------------------------------
    void add_data_header(std::string header);

    //--------------------------------------------------------------------------
    // Name:        log_node
    // Description: Logs a launch message using printf style formatting.
    // Arguments:   - format: Printf style format string.
    //              - ...: Printf style list of variables to be formatted.
    //--------------------------------------------------------------------------
    void log_node(const char *format, ...);

    //--------------------------------------------------------------------------
    // Name:        log_debug
    // Description: Logs a debug message using printf style formatting.
    // Arguments:   - format: Printf style format string.
    //              - ...: Printf style list of variables to be formatted.
    //--------------------------------------------------------------------------
    void log_debug(const char *format, ...);

    //--------------------------------------------------------------------------
    // Name:        log_data
    // Description: Logs a data message using printf style formatting.
    // Arguments:   - format: Printf style format string.
    //              - ...: Printf style list of variables to be formatted.
    //--------------------------------------------------------------------------
    void log_data(const char *format, ...);

    //--------------------------------------------------------------------------
    // Name:        log_info
    // Description: Logs an info message using printf style formatting.
    // Arguments:   - format: Printf style format string.
    //              - ...: Printf style list of variables to be formatted.
    //--------------------------------------------------------------------------
    void log_info(const char *format, ...);

    //--------------------------------------------------------------------------
    // Name:        log_warning
    // Description: Logs a warning message using printf style formatting.
    // Arguments:   - format: Printf style format string.
    //              - ...: Printf style list of variables to be formatted.
    //--------------------------------------------------------------------------
    void log_warning(const char *format, ...);

    //--------------------------------------------------------------------------
    // Name:        log_error
    // Description: Logs an error message using printf style formatting.
    // Arguments:   - format: Printf style format string.
    //              - ...: Printf style list of variables to be formatted.
    //--------------------------------------------------------------------------
    void log_error(const char *format, ...);

    //--------------------------------------------------------------------------
    // Name:        log_node
    // Description: Logs a string node message.
    // Arguments:   - message: Message to log.
    //--------------------------------------------------------------------------
    void log_node(std::string message);

    //--------------------------------------------------------------------------
    // Name:        log_debug
    // Description: Logs a string debug message.
    // Arguments:   - message: Message to log.
    //--------------------------------------------------------------------------
    void log_debug(std::string message);

    //--------------------------------------------------------------------------
    // Name:        log_data
    // Description: Logs a string data message.
    // Arguments:   - message: Message to log.
    //--------------------------------------------------------------------------
    void log_data(std::string message);

    //--------------------------------------------------------------------------
    // Name:        log_info
    // Description: Logs a string info message.
    // Arguments:   - message: Message to log.
    //--------------------------------------------------------------------------
    void log_info(std::string message);

    //--------------------------------------------------------------------------
    // Name:        log_warning
    // Description: Logs a string warning message.
    // Arguments:   - message: Message to log.
    //--------------------------------------------------------------------------
    void log_warning(std::string message);

    //--------------------------------------------------------------------------
    // Name:        log_error
    // Description: Logs a string error message.
    // Arguments:   - message: Message to log.
    //--------------------------------------------------------------------------
    void log_error(std::string message);

    //--------------------------------------------------------------------------
    // Name:        log_vector
    // Description: Logs every element from a vector from Eigen as data.
    // Arguments:   - tag: Data tag to log.
    //              - vec: Vector to be logged.
    //              - precision: Number of decimal points for each element.
    //--------------------------------------------------------------------------
    void log_vector(std::string tag, Eigen::VectorXd vec, int precision);

private:

    // Logging level enumeration
    enum class LEVEL : unsigned char
    {

        NODE   = 0,
        DEBUG    = 1,
        DATA     = 2,
        INFO     = 3,
        WARNING  = 4,
        ERROR    = 5

    };

    // Map to tie a string description of the logging level to the LEVEL enum,
    // defined above, that the set_log_level function needs. This makes it
    // easier to set the logging level from a configuration file via log level
    // name
    std::map<std::string, LEVEL> logging_level_map =
        {{"NODE",    LEVEL::NODE},
         {"DEBUG",   LEVEL::DEBUG},
         {"DATA",    LEVEL::DATA},
         {"INFO",    LEVEL::INFO},
         {"WARNING", LEVEL::WARNING},
         {"ERROR",   LEVEL::ERROR}};

    // Logging level labels that are printed on each message. Labels are chosen
    // to be the same number of characters for alignment
    const std::vector<std::string> level_name = {"NDE", "DBG", "DAT", "INF", "WRN", "ERR"};

    // Log message styling used for each logging level when printing to the
    // console. There is an array entry for each log level ordered as:
    //     [style, foreground, background]
    // NOTE: This is a static constant that cannot be changed at runtime. This
    // is so all loggers have the same look-and-feel.
    constexpr static unsigned char colors[6][3] =
    {

        {CONSOLE_STYLE::style_normal, CONSOLE_STYLE::fg_magenta, CONSOLE_STYLE::bg_default}, // NODE
        {CONSOLE_STYLE::style_normal, CONSOLE_STYLE::fg_blue,    CONSOLE_STYLE::bg_default}, // DEBUG
        {CONSOLE_STYLE::style_normal, CONSOLE_STYLE::fg_default, CONSOLE_STYLE::bg_default}, // DATA
        {CONSOLE_STYLE::style_normal, CONSOLE_STYLE::fg_green,   CONSOLE_STYLE::bg_default}, // INFO
        {CONSOLE_STYLE::style_bold,   CONSOLE_STYLE::fg_yellow,  CONSOLE_STYLE::bg_default}, // WARNING
        {CONSOLE_STYLE::style_bold,   CONSOLE_STYLE::fg_red,     CONSOLE_STYLE::bg_default}  // ERROR

    };

private:

    // Log name that is displayed on log messages, and is used as the file name
    // if logging to file is enabled
    std::string log_name;

    // Filepath that will be written to if file logging is enabled
    std::string log_filepath;

    // Current logging level
    LEVEL log_level;

    // Flags to enable or disable console and file logging
    bool log_to_console;
    bool log_to_file;

    // Log level configuration for the file log. If the level is set to true, it
    // will be logged to file. If it is false, it will not.
    std::map<LEVEL, bool> file_log_config =
        {{LEVEL::NODE,    true},
         {LEVEL::DEBUG,   true},
         {LEVEL::DATA,    true},
         {LEVEL::INFO,    true},
         {LEVEL::WARNING, true},
         {LEVEL::ERROR,   true}};

     // Log level configuration for the console log. If the level is set to
     // true, it will be logged to the console. If it is false, it will not.
     std::map<LEVEL, bool> console_log_config =
         {{LEVEL::NODE,    true},
          {LEVEL::DEBUG,   true},
          {LEVEL::DATA,    true},
          {LEVEL::INFO,    true},
          {LEVEL::WARNING, true},
          {LEVEL::ERROR,   true}};

    // Vector of data headers. Must be logged when a new file is created
    std::vector<std::string> data_headers;

private:

    //--------------------------------------------------------------------------
    // Name:        style_seq
    // Description: Returns the appropriate VT100 control sequence string for
    //              the specified style. NOTE: We're using a flag of 255 to
    //              indicate "no style" in which case an empty string is
    //              returned.
    // Arguments:   - style_code: Character representing the style.
    // Returns:     Control sequence string that styles characters after it.
    //--------------------------------------------------------------------------
    std::string style_seq(unsigned char style_code);

    //--------------------------------------------------------------------------
    // Name:        log_message
    // Description: Writes the log message to the console and/or the log file.
    //              Throws std::runtime_error if writing to file fails.
    // Arguments:   - level: Logging level.
    //              - message: Message to be logged.
    // See also:    log_error, log_data, et. al.
    //--------------------------------------------------------------------------
    void log_message(LEVEL level, std::string message);

};

#endif // LOGGER_H
