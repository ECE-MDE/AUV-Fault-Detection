//==============================================================================
// Autonomous Vehicle Library
//
// Description: Provides functions to open and read events from a generic Linux
//              joystick. Each joystick button and axis produces its own event
//              when pressed or moved. These events are stored in a queue which
//              can be polled periodically to read the events.
//==============================================================================

#ifndef JOYSTICK_H
#define JOYSTICK_H

// Linux joystick support
#include <linux/joystick.h>

// C++ includes
#include <string>
#include <vector>
#include <functional>
#include <cstring>
#include <stdint.h>
#include <unistd.h>

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class Joystick
{

public:

    //--------------------------------------------------------------------------
    // Name:        Joystick constructor
    // Description: Default constructor.
    //--------------------------------------------------------------------------
    Joystick();

    //--------------------------------------------------------------------------
    // Name:        Joystick destructor
    // Description: Default virtual destructor.
    //--------------------------------------------------------------------------
    virtual ~Joystick();

    //--------------------------------------------------------------------------
    // Name:        connect
    // Description: Opens the joystick with the given number assuming the
    //              joystick is located at /dev/input/js<num> where <num> is
    //              the joystick number. Throws a std::runtime_error if the
    //              opening fails.
    // Arguments    - joy_num: joystick number to open
    //--------------------------------------------------------------------------
    void connect(unsigned int joy_num);

    //--------------------------------------------------------------------------
    // Name:        is_open
    // Description: Returns the joystick's status.
    // Returns:     True if the joystick is opened, false otherwise.
    //--------------------------------------------------------------------------
    bool is_open();

    //--------------------------------------------------------------------------
    // Name:        get_num_axes
    // Description: Gets the number of joystick axes. This will be zero if
    //              the joystick is not opened.
    // Returns:     The number of joystick axes.
    //--------------------------------------------------------------------------
    uint8_t get_num_axes();

    //--------------------------------------------------------------------------
    // Name:        get_num_buttons
    // Description: Gets the number of joystick buttons. This will be zero if
    //              the joystick is not opened.
    // Returns:     The number of joystick buttons.
    //--------------------------------------------------------------------------
    uint8_t get_num_buttons();

    //--------------------------------------------------------------------------
    // Name:        set_deadzone
    // Description: Sets the joystick axis deadzone. The axis is scaled so that
    //              the axis value is zero while in the dead zone, then linearly
    //              increases from zero to -1 or 1 outside the dead zone. The
    //              deadzone must be between 0.0 and 1.0.
    // Arguments:   - deadzone: joystick axis deadzone value
    //--------------------------------------------------------------------------
    void set_deadzone(float deadzone);

    //--------------------------------------------------------------------------
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
    //--------------------------------------------------------------------------
    void set_state_callback(std::function<void(std::vector<float>,
        std::vector<int>)> callback);

    //--------------------------------------------------------------------------
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
    //--------------------------------------------------------------------------
    template<class T>
    void set_state_callback(void (T::*callback)(std::vector<float>,
        std::vector<int>), T* obj)
    {
        state_callback = std::bind(callback, obj, std::placeholders::_1,
            std::placeholders::_2);
    }

    //--------------------------------------------------------------------------
    // Name:        spin_once
    // Description: Processes asynchronous IO operations. This function handles
    //              only a small set of operations at a time, and must therefore
    //              be called in a loop.
    //--------------------------------------------------------------------------
    void spin_once();

private:

    // Joystick file descriptor for reading from joystick
    int joy_fd;

    // Vectors of joystick axis values and button values in order of axis/button
    // number
    std::vector<float> axis_values;
    std::vector<int> button_values;

    // Joystick axis deadzone. A joystick axis value less than the deadzone
    // value is scaled to zero
    float axis_deadzone;

    // Joystick state callback function pointer
    std::function<void(std::vector<float>, std::vector<int>)> state_callback;

    // Joystick axis value range
    const int16_t MIN_AXIS_VALUE = -32767;
    const int16_t MAX_AXIS_VALUE =  32767;

private:

    //--------------------------------------------------------------------------
    // Name:        scale_axis
    // Description: Scales a joystick axis value to a float and applies the axis
    //              dead zone. The axis is scaled so that the axis value is zero
    //              while in the dead zone, then linearly increases from zero to
    //              -1 or 1 outside the dead zone.
    // Arguments:   - axis_value: axis value to scale
    // Returns:     Axis value as a float between -1.0 and 1.0
    //--------------------------------------------------------------------------
    float scale_axis(int16_t axis_value);

    //--------------------------------------------------------------------------
    // Name:        read_event
    // Description: Polls the joystick for a joystick event. Throws a
    //              std::runtime_error if the read fails.
    // Arguments    - event: pointer to JoystickEvent to store the event data
    // Returns:     True if a joystick event was successfully read, false if
    //              there was no joystick event to read.
    //--------------------------------------------------------------------------
    bool read_event(js_event* event);

};

#endif // JOYSTICK_H
