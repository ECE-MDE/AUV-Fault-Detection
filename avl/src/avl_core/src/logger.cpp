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

#include <avl_core/logger.h>

// Console color styles
#include <avl_core/console_style.h>

// Util functions
#include <avl_core/util/string.h>
#include <avl_core/util/time.h>
#include <avl_core/util/file.h>

// C++ includes
#include <string>
#include <vector>
#include <memory>
#include <time.h>
#include <iostream>
#include <iomanip>
#include <cstdarg>
#include <map>
#include <algorithm>

// Ugh...  It seems that C++ requires the (apparent) redefinition of this
// array.  The definition is in the header file but it is not actually
// created until here (need both declaration and definition of static member)
constexpr unsigned char Logger::colors[6][3];

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        Logger constructor
// Description: Default constructor.
//------------------------------------------------------------------------------
Logger::Logger()
{

    // Default logger settings. Will be changed by calling configuration
    // functions
    log_name      = "unlabeled_log";
    log_level      = LEVEL::DEBUG;
    log_to_console = true;
    log_to_file    = false;
    log_filepath   = "/dev/null";

}

//------------------------------------------------------------------------------
// Name:        Logger destructor
// Description: Default virtual destructor.
//------------------------------------------------------------------------------
Logger::~Logger()
{

}

//------------------------------------------------------------------------------
// Name:        set_log_name
// Description: Sets the log name that appears in log messages and the file
//              name that will be written to if file logging is enabled.
//              The log filepath is specified with the set_log_filepath
//              function. The file will be created if it does not exist, or
//              will be appended to if it does exist.
// Arguments:   - name: Log name.
//------------------------------------------------------------------------------
void Logger::set_log_name(std::string name)
{
    // If the name starts with a slash, remove it
    if (name.at(0) == '/')
        name.erase(0,1);
    log_name = name;
}

//------------------------------------------------------------------------------
// Name:        set_log_filepath
// Description: Sets the log file path that will be written to if file
//              logging is enabled. This file path should not contain the
//              file name. The file name is set with the set_log_name
//              function. The file will be created if it does not exist, or
//              will be appended to if it does exist.
// Arguments:   - filepath: Log filepath.
//------------------------------------------------------------------------------
void Logger::set_log_filepath(std::string filepath)
{
    // If the filepath ends with a slash, remove it
    if (filepath.back() == '/')
        filepath.pop_back();
    log_filepath = filepath;
}

//------------------------------------------------------------------------------
// Name:        configure_file_log
// Description: Configures which log messages will be logged to file.
// Arguments:   - config: Logger configuration struct.
//------------------------------------------------------------------------------
void Logger::configure_file_log(LoggerConfig config)
{

    log_to_file = config.enable;

    // Modify the configuration map for each level
    file_log_config[LEVEL::NODE] =    config.log_node;
    file_log_config[LEVEL::DATA] =    config.log_data;
    file_log_config[LEVEL::DEBUG] =   config.log_debug;
    file_log_config[LEVEL::INFO] =    config.log_info;
    file_log_config[LEVEL::WARNING] = config.log_warning;
    file_log_config[LEVEL::ERROR] =   config.log_error;

}

//------------------------------------------------------------------------------
// Name:        configure_console_log
// Description: Configures which log messages will be logged to the console.
// Arguments:   - config: Logger configuration struct.
//------------------------------------------------------------------------------
void Logger::configure_console_log(LoggerConfig config)
{

    log_to_console = config.enable;

    // Modify the configuration map for each level
    console_log_config[LEVEL::NODE] =    config.log_node;
    console_log_config[LEVEL::DATA] =    config.log_data;
    console_log_config[LEVEL::DEBUG] =   config.log_debug;
    console_log_config[LEVEL::INFO] =    config.log_info;
    console_log_config[LEVEL::WARNING] = config.log_warning;
    console_log_config[LEVEL::ERROR] =   config.log_error;

}

//------------------------------------------------------------------------------
// Name:        add_data_header
// Description: Adds a header to the list of data headers to be logged
//              when the log file is created.
// Arguments:   - format: Printf style format string.
//              - ...: Printf style list of variables to be formatted.
//------------------------------------------------------------------------------
void Logger::add_data_header(const char *format, ...)
{
    va_list args;
    va_start (args, format);
    add_data_header(avl::format_string(format, args));
    va_end (args);
}

//------------------------------------------------------------------------------
// Name:        add_data_header
// Description: Adds a header to the list of data headers to be logged
//              when the log file is created.
// Arguments:   - header: Header to add to the list of data headers.
//------------------------------------------------------------------------------
void Logger::add_data_header(std::string header)
{
    data_headers.push_back(header);
    log_data(header);
}

//------------------------------------------------------------------------------
// Name:        log_node
// Description: Logs a launch message using printf style formatting.
// Arguments:   - format: Printf style format string.
//              - ...: Printf style list of variables to be formatted.
//------------------------------------------------------------------------------
void Logger::log_node(const char *format, ...)
{
    va_list args;
    va_start (args, format);
    log_node(avl::format_string(format, args));
    va_end (args);
}

//------------------------------------------------------------------------------
// Name:        log_debug
// Description: Logs a debug message using printf style formatting.
// Arguments:   - format: Printf style format string.
//              - ...: Printf style list of variables to be formatted.
//------------------------------------------------------------------------------
void Logger::log_debug(const char *format, ...)
{
    va_list args;
    va_start (args, format);
    log_debug(avl::format_string(format, args));
    va_end (args);
}

//------------------------------------------------------------------------------
// Name:        log_data
// Description: Logs a data message using printf style formatting.
// Arguments:   - format: Printf style format string.
//              - ...: Printf style list of variables to be formatted.
//------------------------------------------------------------------------------
void Logger::log_data(const char *format, ...)
{
    va_list args;
    va_start (args, format);
    log_data(avl::format_string(format, args));
    va_end (args);
}

//------------------------------------------------------------------------------
// Name:        log_info
// Description: Logs an info message using printf style formatting.
// Arguments:   - format: Printf style format string.
//              - ...: Printf style list of variables to be formatted.
//------------------------------------------------------------------------------
void Logger::log_info(const char *format, ...)
{
    va_list args;
    va_start (args, format);
    log_info(avl::format_string(format, args));
    va_end (args);
}

//------------------------------------------------------------------------------
// Name:        log_warning
// Description: Logs a warning message using printf style formatting.
// Arguments:   - format: Printf style format string.
//              - ...: Printf style list of variables to be formatted.
//------------------------------------------------------------------------------
void Logger::log_warning(const char *format, ...)
{
    va_list args;
    va_start (args, format);
    log_warning(avl::format_string(format, args));
    va_end (args);
}

//------------------------------------------------------------------------------
// Name:        log_error
// Description: Logs an error message using printf style formatting.
// Arguments:   - format: Printf style format string.
//              - ...: Printf style list of variables to be formatted.
//------------------------------------------------------------------------------
void Logger::log_error(const char *format, ...)
{
    va_list args;
    va_start (args, format);
    log_error(avl::format_string(format, args));
    va_end (args);
}

//------------------------------------------------------------------------------
// Name:        log_node
// Description: Logs a string node message.
// Arguments:   - message: Message to log.
//------------------------------------------------------------------------------
void Logger::log_node(std::string message)
{
    log_message(LEVEL::NODE, message);
}

//------------------------------------------------------------------------------
// Name:        log_debug
// Description: Logs a string debug message.
// Arguments:   - message: Message to log.
//------------------------------------------------------------------------------
void Logger::log_debug(std::string message)
{
    log_message(LEVEL::DEBUG, message);
}

//------------------------------------------------------------------------------
// Name:        log_data
// Description: Logs a string data message.
// Arguments:   - message: Message to log.
//------------------------------------------------------------------------------
void Logger::log_data(std::string message)
{
    log_message(LEVEL::DATA, message);
}

//------------------------------------------------------------------------------
// Name:        log_info
// Description: Logs a string info message.
// Arguments:   - message: Message to log.
//------------------------------------------------------------------------------
void Logger::log_info(std::string message)
{
    log_message(LEVEL::INFO, message);
}

//------------------------------------------------------------------------------
// Name:        log_warning
// Description: Logs a string warning message.
// Arguments:   - message: Message to log.
//------------------------------------------------------------------------------
void Logger::log_warning(std::string message)
{
    log_message(LEVEL::WARNING, message);
}

//------------------------------------------------------------------------------
// Name:        log_error
// Description: Logs a string error message.
// Arguments:   - message: Message to log.
//------------------------------------------------------------------------------
void Logger::log_error(std::string message)
{
    log_message(LEVEL::ERROR, message);
}

//------------------------------------------------------------------------------
// Name:        style_seq
// Description: Returns the appropriate VT100 control sequence string for
//              the specified style. NOTE: We're using a flag of 255 to
//              indicate "no style" in which case an empty string is
//              returned.
// Arguments:   - style_code: Character representing the style.
// Returns:     Control sequence string that styles characters after it.
//------------------------------------------------------------------------------
std::string Logger::style_seq(unsigned char style_code)
{

    std::ostringstream output;

    if (style_code == 255)
        output << "";
    else
        output << "\033[" << static_cast<int>(style_code) << "m";

    return output.str();

}

//------------------------------------------------------------------------------
// Name:        log_message
// Description: Writes the log message to the console and/or the log file.
//              Throws std::ifstream::failure if writing to file fails.
// Arguments:   - level: Logging level.
//              - message: Message to be logged.
// See also:    log_error, log_data, et. al.
//------------------------------------------------------------------
//------------------------------------------------------------------------------
void Logger::log_message(LEVEL level, std::string message)
{

    // Format the log output string. The format is hardcoded as:
    //     [Timestamp][level][label] <message>
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(9);
    oss << "[" << avl::get_epoch_time() << "] ";
    oss << "[" << level_name[(int)level] << "] ";
    oss << "[" << log_name << "] ";
    oss <<  message;
    std::string output = oss.str();

    // Log to console if it is enabled for the specified log level
    if (log_to_console && console_log_config[level] == true)
    {

        // Pull the color codes for the specified logging level from the
        // (static) class table. Here we get format, foreground, and
        // background colors.
        unsigned char form  = colors[(int)level][0];
        unsigned char fg    = colors[(int)level][1];
        unsigned char bg    = colors[(int)level][2];

        // Print the style control sequences for foreground, background,
        // and formatting
        std::cout << style_seq(form) <<  style_seq(fg) <<  style_seq(bg);

        // Print the message itself
        std::cout << output;

        // Print the style reset control sequence and an endline
        std::cout << style_seq(CONSOLE_STYLE::style_reset) << std::endl;

    }

    // Log to file if it is enabled for the specified log level
    if (log_to_file && file_log_config[level] == true)
    {

        // Create the full log filepath
        std::string full_log_filepath = log_filepath + "/" + log_name + ".log";

        // Check if the file exists. If it does not, that means we are writing
        // to a new file and we want to log the registered data headers before
        // doing anything else. However, we cannot log the headers here
        // because we need to create the file first otherwise the recursion will
        // never end
        bool need_write_headers = !avl::file_exists(full_log_filepath);

        // Open the logging file for append each time we try to write. This
        // keeps the locking of the file to a minimum so multiple threads can
        // log to the same file.
        std::ofstream file;
        file.exceptions(std::ifstream::failbit | std::ifstream::badbit);

        try
        {
            file.open(full_log_filepath, std::ios::out | std::ios::app);
        }
        catch(const std::ifstream::failure&)
        {
            throw std::runtime_error("log_message: failed to open log file ("
                + full_log_filepath + ")");
        }

        // If the file did not exist and we need to log the registered data
        // headers, loop through the vector and log them
        if (need_write_headers)
            for (std::string header : data_headers)
                log_data(header);

        // Write the output to the file
        file << output << std::endl;
        file.close();

    }

}

//--------------------------------------------------------------------------
// Name:        log_vector
// Description: Logs every element from a vector from Eigen as data.
// Arguments:   - tag: Data tag to log.
//              - vec: Vector to be logged.
//              - precision: Number of decimal points for each element.
//--------------------------------------------------------------------------
void Logger::log_vector(std::string tag, Eigen::VectorXd vec, int precision)
{
    std::stringstream ss;
    ss << std::setprecision(precision) << std::scientific;
    ss << "[" << tag << "]";
    for (int i = 0; i < vec.size(); i++)
        ss << " " << vec(i);

    log_data(ss.str());
}
