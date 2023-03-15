//==============================================================================
// Autonomous Vehicle Library
//
// Description: Provides basic functions to set PWM output values and other
//              settings on Pololu Maestro Servo Controllers through its native
//              USB interface. Requires libusb-1.0. Reference:
//    https://github.com/pololu/pololu-usb-sdk/blob/master/Maestro/protocol.h
//==============================================================================

#ifndef MAESTRO_PWM_H
#define MAESTRO_PWM_H

// USB driver
#include <libusb-1.0/libusb.h>

//==============================================================================
//                             DEFINES & CONSTANTS
//==============================================================================

// Vendor ID and product IDs for all Pololu Maestro PWM controller devices
static const uint16_t maestro_vendor_id = 0x1ffb;
static const uint16_t maestro_product_ids[] = {0x0089, 0x008a, 0x008b, 0x008c};
static const int num_maestro_ids = 4;

// USB request type for controlling the USB device, as per the USB spec. This
// value represents a vendor request to send data from the computer to
// the device
#define USB_VENDOR_REQUEST 0x40

// USB request timeout in milliseconds
#define USB_TIMEOUT 5000

// Pololu Maestro API request values
#define REQUEST_GET_PARAMETER 0x81
#define REQUEST_SET_PARAMETER 0x82
#define REQUEST_SET_SERVO_VARIABLE 0x84
#define REQUEST_SET_TARGET 0x85
#define REQUEST_CLEAR_ERRORS 0x86
#define REQUEST_REINITIALIZE 0x90

// Pololu Maestro API parameter values
#define PARAMETER_INITIALIZED 0
#define PARAMETER_SERIAL_MODE 3
#define PARAMETER_SERIAL_TIMEOUT 6
#define PARAMETER_OUTPUT_MASK_C 17

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class MaestroPwm
{

public:

    //--------------------------------------------------------------------------
    // Name:        MaestroPwm constructor
    //--------------------------------------------------------------------------
    MaestroPwm();

    //--------------------------------------------------------------------------
    // Name:        MaestroPwm destructor
    //--------------------------------------------------------------------------
    virtual ~MaestroPwm();

    //--------------------------------------------------------------------------
    // Name:        open
    // Description: Opens a USB connection to the first Maestro PWM controller
    //              found. Throws an exception if a Maestro PWM controller
    //              cannot be found or the USB connection fails.
    //--------------------------------------------------------------------------
    void open();

    //--------------------------------------------------------------------------
    // Name:        reset_settings
    // Description: Resets all Maestro settings.
    //--------------------------------------------------------------------------
    void reset_settings();

    //--------------------------------------------------------------------------
    // Name:        enable_dual_usb_mode
    // Description: Enables the Maestros Dual USB serial mode so that commands
    //              can be sent and received through the USB protocol. This is
    //              the only protocol we care about, so the others are not
    //              supported.
    //--------------------------------------------------------------------------
    void enable_dual_usb_mode();

    //--------------------------------------------------------------------------
    // Name:        enable_output
    // Description: Enables the PWM channel outputs.
    //--------------------------------------------------------------------------
    void enable_output();

    //--------------------------------------------------------------------------
    // Name:        disable_output
    // Description: Disables the PWM channel outputs.
    //--------------------------------------------------------------------------
    void disable_output();

    //--------------------------------------------------------------------------
    // Name:        clear_errors
    // Description: Clears PWM output errors.
    //--------------------------------------------------------------------------
    void clear_errors();

    //--------------------------------------------------------------------------
    // Name:        set_output_timeout
    // Description: Sets the PWM output timeout. If a setPulseWidth command is
    //              not received within this amount of time since the last one,
    //              PWM output will be disabled.
    // Arguments:   - timeout: PWM output timeout in milliseconds
    //--------------------------------------------------------------------------
    void set_output_timeout(int timeout);

    //--------------------------------------------------------------------------
    // Name:        set_pulse_width
    // Description: Sets the pulse width of a PWM channel in milliseconds. A
    //              value of zero disables the PWM channel. Throws an exception
    //              if pulse width setting fails.
    // Arguments:   - channel: PWM channel number to set
    //              - pulse_width: pulse width in milliseconds
    //--------------------------------------------------------------------------
    void set_pulse_width(uint8_t channel, float pulse_width);

    //--------------------------------------------------------------------------
    // Name:        set_speed
    // Description: Sets the maximum rate at which a PWM channel can transition
    //              to a new pulse width. A value of zero removes the speed
    //              limit. Throws an exception if speed setting fails.
    // Arguments:   - channel: PWM channel number to set
    //              - speed: PWM channel speed limit
    //--------------------------------------------------------------------------
    void set_speed(uint8_t channel, uint16_t speed);

    //--------------------------------------------------------------------------
    // Name:        set_acceleration
    // Description: Sets the maximum rate at which a PWM channel's transition
    //              speed ramps up to its maximum speed as it transitions to a
    //              new pulse width. A value of zero removes the acceleration
    //              limit. Throws an exception if acceleration setting fails.
    // Arguments:   - channel: PWM channel number to set
    //              - accel: PWM channel acceleration limit
    //--------------------------------------------------------------------------
    void set_acceleration(uint8_t channel, uint16_t accel);

private:

    // Handle to the Maestro device found by the class instance
    libusb_device_handle* device_handle;

    // Maestro device found flag
    bool found_maestro_device = false;

private:

    //--------------------------------------------------------------------------
    // Name:        is_maestro_device
    // Description: Checks whether a USB device is a Pololu Maestro device.
    //              - desc: USB device descriptor
    // Returns:     true if the USB device is a Pololu Maestro device
    //--------------------------------------------------------------------------
    bool is_maestro_device(libusb_device_descriptor& desc);

    //--------------------------------------------------------------------------
    // Name:        send_request
    // Description: Sends a USB control transfer to the Maestro device. The
    //              control transfer's setup packet is used for the command
    //              type and its arguments.
    //              - type: USB request type
    //              - request: request type code (see Pololu's USB SDK)
    //              - value: request value (see Pololu's USB SDK)
    //              - index: request index code (see Pololu's USB SDK)
    // Returns:     true if the transfer succeeded, false otherwise
    //--------------------------------------------------------------------------
    void send_request(uint8_t type, uint8_t request, uint16_t value, 
        uint16_t index);

    //--------------------------------------------------------------------------
    // Name:        set_parameter
    // Description: Sets a parameter on the Maestro device using the parameter
    //              codes used in the USB SDK.
    // Arguments:   - parameter: parameter code
    //              - value: parameter value
    //              - bytes: number of bytes in parameter value
    //--------------------------------------------------------------------------
    void set_parameter(uint16_t parameter, uint16_t value, uint16_t bytes);

};

#endif // MAESTRO_PWM_H
