//==============================================================================
// Autonomous Vehicle Library
//
// Description: Provides functions to open and read events from a generic Linux
//              joystick. Each joystick button and axis produces its own event
//              when pressed or moved. These events are stored in a queue which
//              can be polled periodically to read the events.
//==============================================================================

#include <avl_devices/driver/joystick.h>

// File control for interfacing with a joystick in Linux
#include <fcntl.h>
#include <errno.h>

// Included for avl::linear_scale
#include <avl_core/util/math.h>

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        Joystick constructor
// Description: Default constructor.
//------------------------------------------------------------------------------
Joystick::Joystick()
{

    // Default deadzone of 0.1
    set_deadzone(0.1);

}

//------------------------------------------------------------------------------
// Name:        Joystick destructor
// Description: Default virtual destructor.
//------------------------------------------------------------------------------
Joystick::~Joystick()
{

}

//------------------------------------------------------------------------------
// Name:        connect
// Description: Opens the joystick with the given number assuming the
//              joystick is located at /dev/input/js<num> where <num> is
//              the joystick number. Throws a std::runtime_error if the
//              opening fails.
// Arguments    - joy_num: joystick number to open
//------------------------------------------------------------------------------
void Joystick::connect(unsigned int joy_num)
{

    // Create the full joystick path from the joystick path and joystick
    // number. Linux joysticks are found under /dev/input/js<num> where
    // <num> is the joystick number
    std::string joystick_path = "/dev/input/js" + std::to_string(joy_num);

    // Close the joystick if it is already open
    close(joy_fd);

    // Open the joystick file descriptor in read only and non-blocking mode.
    // In non-blocking mode, reading from the file descriptor will not block
    // but may not read any bytes. We much check the return value of the
    // read function
    joy_fd = open(joystick_path.c_str(), O_RDONLY | O_NONBLOCK);

    // Close and re-open the joystick to get more accurate joystick initial
    // values
    close(joy_fd);
    joy_fd = open(joystick_path.c_str(), O_RDONLY | O_NONBLOCK);

    // Check whether the joystick file descriptor opened properly. If it
    // failed, the open function returns -1 and errno is set to an error
    // number corresponding to the failure
    if (joy_fd == -1)
    {
        throw std::runtime_error(std::string("open: failed to open joystick (")
            + strerror(errno) + ")");
    }

    // Get the number of axes and buttons
    axis_values = std::vector<float>(get_num_axes(), 0.0);
    button_values = std::vector<int>(get_num_buttons(), 0);

}

//------------------------------------------------------------------------------
// Name:        is_open
// Description: Returns the joystick's status.
// Returns:     True if the joystick is opened, false otherwise.
//------------------------------------------------------------------------------
bool Joystick::is_open()
{
    return joy_fd >= 0;
}

//------------------------------------------------------------------------------
// Name:        get_num_axes
// Description: Gets the number of joystick axes. This will be zero if
//              the joystick is not opened.
// Returns:     The number of joystick axes.
//------------------------------------------------------------------------------
uint8_t Joystick::get_num_axes()
{
    uint8_t num_axes;
    if (ioctl(joy_fd, JSIOCGAXES, &num_axes) == -1)
        return 0;
    return num_axes;
}

//------------------------------------------------------------------------------
// Name:        get_num_buttons
// Description: Gets the number of joystick buttons. This will be zero if
//              the joystick is not opened.
// Returns:     The number of joystick buttons.
//------------------------------------------------------------------------------
uint8_t Joystick::get_num_buttons()
{
    uint8_t num_buttons;
    if (ioctl(joy_fd, JSIOCGBUTTONS, &num_buttons) == -1)
        return 0;
    return num_buttons;
}

//------------------------------------------------------------------------------
// Name:        set_deadzone
// Description: Sets the joystick axis deadzone. The axis is scaled so that
//              the axis value is zero while in the dead zone, then linearly
//              increases from zero to -1 or 1 outside the dead zone. The
//              deadzone must be between 0.0 and 1.0.
// Arguments:   - deadzone: joystick axis deadzone value
//------------------------------------------------------------------------------
void Joystick::set_deadzone(float deadzone)
{

    // Deadzone must be in valid range
    if (deadzone < 0.0 || deadzone > 1.0)
        throw std::runtime_error(std::string("set_deadzone: invalid deadzone "
            "value, must be in range [0,1]"));

    axis_deadzone = deadzone;

}

//------------------------------------------------------------------------------
// Name:        set_state_callback
// Description: Sets the joystick state callback function. The joystick
//              state callback will be called at the specified state
//              callback rate with the most ecent joystick axis and button
//              states. This callback will also be called immediately when
//              a button event occurs despite the state callback rate in
//              order to prevent missed button events. The state callback
//              must have the following function signature:
//                void state_callback(std::vector<float>, std::vector<int>);
// Arguments:   - callback: joystick state callback function pointer
//------------------------------------------------------------------------------
void Joystick::set_state_callback(std::function<void(std::vector<float>,
    std::vector<int>)> callback)
{
    state_callback = callback;
}

//------------------------------------------------------------------------------
// Name:        spin_once
// Description: Processes asynchronous IO operations. This function handles
//              only a small set of operations at a time, and must therefore
//              be called in a loop.
//------------------------------------------------------------------------------
void Joystick::spin_once()
{

    // Don't do anything if they joystick is not open
    if (!is_open())
        return;

    // Poll the joystick for an event
    js_event event;
    if (read_event(&event))
    {

        if (event.type & JS_EVENT_AXIS)
            axis_values.at(event.number) = scale_axis(event.value);

        if (event.type & JS_EVENT_BUTTON)
            button_values.at(event.number) = event.value;

        // Call the joystick state callback to give the user the new value
        state_callback(axis_values, button_values);

    }

}

//------------------------------------------------------------------------------
// Name:        scale_axis
// Description: Scales a joystick axis value to a float and applies the axis
//              dead zone. The axis is scaled so that the axis value is zero
//              while in the dead zone, then linearly increases from zero to
//              -1 or 1 outside the dead zone.
// Arguments:   - axis_value: axis value to scale
// Returns:     Axis value as a float between -1.0 and 1.0
//------------------------------------------------------------------------------
float Joystick::scale_axis(int16_t axis_value)
{

    // Scale the axis value from its int8 value to a float from the edge of the
    // deadzone to 1.0
    float value = avl::linear_scale(axis_value, MIN_AXIS_VALUE, MAX_AXIS_VALUE,
        -1.0, 1.0);

    // Enforce the deadzone
    if(std::abs(value) < axis_deadzone)
        value = 0.0;
    else
        value *= ((std::abs(value) - axis_deadzone) / (1.0 - axis_deadzone));

    return value;

}

//------------------------------------------------------------------------------
// Name:        read_event
// Description: Polls the joystick for a joystick event. Throws a
//              std::runtime_error if the read fails.
// Arguments    - event: pointer to JoystickEvent to store the event data
// Returns:     True if a joystick event was successfully read, false if
//              there was no joystick event to read.
//------------------------------------------------------------------------------
bool Joystick::read_event(js_event* event)
{

    // Attempt to read a joystick event from the joystick file
    // descriptor. Returns the number of bytes read or -1 if there was
    // a read error
    int bytes_read = read(joy_fd, event, sizeof(js_event));
    if (bytes_read == -1 && errno != EAGAIN)
    {
        throw std::runtime_error(std::string("read: failed to read from "
            "joystick (") + strerror(errno) + ")");
    }

    // If we read the correct number of joystick event bytes then we have
    // correctly read a joystick event. This should be the correct number
    // every time unless we are out of sync with the read somehow
    return bytes_read == sizeof(js_event);

}
