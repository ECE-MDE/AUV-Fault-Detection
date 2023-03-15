//==============================================================================
// Autonomous Vehicle Library
//
// Description: Provides basic functions to set PWM output values and other
//              settings on Pololu Maestro Servo Controllers through its native
//              USB interface. Requires libusb-1.0. Reference:
//    https://github.com/pololu/pololu-usb-sdk/blob/master/Maestro/protocol.h
//==============================================================================

#include <avl_devices/driver/maestro_pwm.h>

// USB driver
#include <libusb-1.0/libusb.h>

// Standard exception throwing
#include <stdexcept>

// Millisecond sleep
#include <chrono>
#include <thread>

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        MaestroPwm constructor
//------------------------------------------------------------------------------
MaestroPwm::MaestroPwm()
{

}

//------------------------------------------------------------------------------
// Name:        MaestroPwm destructor
//------------------------------------------------------------------------------
MaestroPwm::~MaestroPwm()
{

}

//------------------------------------------------------------------------------
// Name:        open
// Description: Opens a USB connection to the first Maestro PWM controller
//              found. Throws an exception if a Maestro PWM controller
//              cannot be found or the USB connection fails.
//------------------------------------------------------------------------------
void MaestroPwm::open()
{

    // Initialize libusb with the default context
    libusb_init(NULL);

    // Get the list of all connected USB devices
    libusb_device** device_list;
    int num_devices = libusb_get_device_list(NULL, &device_list);

    // Search through the list of USB devices looking for any Pololu Maestro
    for (int i = 0; i < num_devices; i++)
    {

        libusb_device* device = device_list[i];

        // Get the device details
        struct libusb_device_descriptor desc;
        libusb_get_device_descriptor(device, &desc);

        // If the device is a Pololu Maestro, open it
        if (is_maestro_device(desc))
        {

            // Open the USB device and throw an exception if opening failed
            int status = libusb_open(device, &device_handle);
            if (status < 0)
            {

                char failure_message[128];
                libusb_error error = static_cast<libusb_error>(status);
                sprintf(failure_message, "open: failed to open Pololu Maestro "
                    "USB device (%s)", libusb_strerror(error));
                throw std::runtime_error(failure_message);

            }

            // If opening succeeded, stop looking for more devices
            found_maestro_device = true;
            break;

        }

    }

    libusb_free_device_list(device_list, 1);

    // If no Pololu Maestro device was found, close libusb and throw
    // an exception
    if (!found_maestro_device)
    {

        libusb_exit(NULL);
        throw std::runtime_error("open: failed to find a conencted Pololu "
            "Maestro USB device");

    }

}

//------------------------------------------------------------------------------
// Name:        reset_settings
// Description: Resets all Maestro settings.
//------------------------------------------------------------------------------
void MaestroPwm::reset_settings()
{
    set_parameter(PARAMETER_INITIALIZED, 0xFF, 1);
}

//------------------------------------------------------------------------------
// Name:        enable_dual_usb_mode
//
// Description: Enables the Maestros Dual USB serial mode so that commands
//              can be sent and received through the USB protocol. This is
//              the only protocol we care about, so the others are not
//              supported.
//------------------------------------------------------------------------------
void MaestroPwm::enable_dual_usb_mode()
{
    set_parameter(PARAMETER_SERIAL_MODE, 0x00, 1);
}

//------------------------------------------------------------------------------
// Name:        enable_output
// Description: Enables the PWM channel outputs.
//------------------------------------------------------------------------------
void MaestroPwm::enable_output()
{
    set_parameter(PARAMETER_OUTPUT_MASK_C, 0xFF, 1);
}

//------------------------------------------------------------------------------
// Name:        disable_output
// Description: Disables the PWM channel outputs.
//------------------------------------------------------------------------------
void MaestroPwm::disable_output()
{
    set_parameter(PARAMETER_OUTPUT_MASK_C, 0x00, 1);
}

//------------------------------------------------------------------------------
// Name:        clear_errors
// Description: Clears PWM output errors.
//------------------------------------------------------------------------------
void MaestroPwm::clear_errors()
{
    send_request(USB_VENDOR_REQUEST, REQUEST_CLEAR_ERRORS, 0, 0);
}

//------------------------------------------------------------------------------
// Name:        set_output_timeout
// Description: Sets the PWM output timeout. If a setPulseWidth command is
//              not received within this amount of time since the last one,
//              PWM output will be disabled.
// Arguments:   - timeout: PWM output timeout in milliseconds
//------------------------------------------------------------------------------
void MaestroPwm::set_output_timeout(int timeout)
{

    // Expects units of 10ms, so divide by 10.
    set_parameter(PARAMETER_SERIAL_TIMEOUT, (uint16_t)timeout/10.0, 2);

}

//------------------------------------------------------------------------------
// Name:        set_pulse_width
// Description: Sets the pulse width of a PWM channel in milliseconds. A
//              value of zero disables the PWM channel. Throws an exception
//              if pulse width setting fails.
// Arguments:   - channel: PWM channel number to set
//              - pulse_width: pulse width in milliseconds
//------------------------------------------------------------------------------
void MaestroPwm::set_pulse_width(uint8_t channel, float pulse_width)
{

    // Calculate the pulse width value in increments of 25 microseconds to
    // be sent to the Pololu Maestro device
    uint16_t pwm_value = (uint16_t)(pulse_width*4000.0);

    // Send the request to set the target pulse width
    send_request(USB_VENDOR_REQUEST, REQUEST_SET_TARGET, pwm_value, channel);

}

//------------------------------------------------------------------------------
// Name:        set_speed
// Description: Sets the maximum rate at which a PWM channel can transition
//              to a new pulse width. A value of zero removes the speed
//              limit. Throws an exception if speed setting fails.
// Arguments:   - channel: PWM channel number to set
//              - speed: PWM channel speed limit
//------------------------------------------------------------------------------
void MaestroPwm::set_speed(uint8_t channel, uint16_t speed)
{
    send_request(USB_VENDOR_REQUEST, REQUEST_SET_SERVO_VARIABLE, speed,
        channel);
}

//------------------------------------------------------------------------------
// Name:        set_acceleration
// Description: Sets the maximum rate at which a PWM channel's transition
//              speed ramps up to its maximum speed as it transitions to a
//              new pulse width. A value of zero removes the acceleration
//              limit. Throws an exception if acceleration setting fails.
// Arguments:   - channel: PWM channel number to set
//              - accel: PWM channel acceleration limit
//------------------------------------------------------------------------------
void MaestroPwm::set_acceleration(uint8_t channel, uint16_t accel)
{

    // Setting the high bit on channel number indicates acceleration setting
    send_request(USB_VENDOR_REQUEST, REQUEST_SET_SERVO_VARIABLE, accel,
        channel | 0x80);

}

//------------------------------------------------------------------------------
// Name:        is_maestro_device
// Description: Checks whether a USB device is a Pololu Maestro device.
//              - desc: USB device descriptor
// Returns:     true if the USB device is a Pololu Maestro device
//------------------------------------------------------------------------------
bool MaestroPwm::is_maestro_device(libusb_device_descriptor& desc)
{

    // If the vendor ID does not match the Maestro vendor ID, it is not a
    // Maestro device
    if (desc.idVendor != maestro_vendor_id)
    {
        return false;
    }

    // Loop through the possible Maestro product IDs and check if it
    // matches any of them
    for (int i = 0; i < num_maestro_ids; i++)
    {

        if (desc.idProduct == maestro_product_ids[i])
        {
            return true;
        }

    }

    return false;

}

//------------------------------------------------------------------------------
// Name:        send_request
// Description: Sends a USB control transfer to the Maestro device. The
//              control transfer's setup packet is used for the command
//              type and its arguments.
//              - type: USB request type
//              - request: request type code (see Pololu's USB SDK)
//              - value: request value (see Pololu's USB SDK)
//              - index: request index code (see Pololu's USB SDK)
// Returns:     the total number of bytes transfered in the request, or an
//              error code
//------------------------------------------------------------------------------
void MaestroPwm::send_request(uint8_t type, uint8_t request, uint16_t value,
    uint16_t index)
{

    // Don't attempt to send a request if the device is not connected
    int return_val = LIBUSB_ERROR_OTHER;
    if (found_maestro_device)
    {

        // Send the USB control transfer with no data buffer. The Pololu USB SDK
        // uses the setup packet value and index fields for controlling the
        // device
        return_val = libusb_control_transfer(device_handle, type, request,
            value, index, (unsigned char*)0, 0, USB_TIMEOUT);

        // If the control transfer result is not success, throw an exception
        if(return_val != LIBUSB_SUCCESS)
        {
            libusb_error error = static_cast<libusb_error>(return_val);
            throw std::runtime_error(std::string("send_request: failed to send "
                "USB request to Maestro (") + libusb_strerror(error) + ")");
        }

    }

}

//------------------------------------------------------------------------------
// Name:        set_parameter
// Description: Sets a parameter on the Maestro device using the parameter
//              codes used in the USB SDK.
// Arguments:   - parameter: parameter code
//              - value: parameter value
//              - bytes: number of bytes in parameter value
//------------------------------------------------------------------------------
void MaestroPwm::set_parameter(uint16_t parameter, uint16_t value,
    uint16_t bytes)
{

    // Form the index argument to set the parameter. The high byte represents
    // the number of bytes in the parameter.
    uint16_t index = (uint16_t)((bytes << 8) + parameter);

    // Send the request to set the parameter
    send_request(USB_VENDOR_REQUEST, REQUEST_SET_PARAMETER, value, index);

    // Send a reinitialize request
    send_request(USB_VENDOR_REQUEST, REQUEST_REINITIALIZE, 0, 0);

    // Sleep for 50 ms to allow the settings to be set. This is the same sleep
    // duration used in Pololu's USB SDK
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

}
