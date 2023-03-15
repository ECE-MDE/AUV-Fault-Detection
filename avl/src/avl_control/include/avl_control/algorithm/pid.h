//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements a PID control algorithm for calculating an output
//              value based on an input and a setpoint and proportional,
//              integral, and derivative gains.
//
//              The class provides an iterate function that must be called at a
//              constant iteration rate in Hz set with the set_iteration_rate
//              function.
//
//              Multiple iterate functions are provided depending on whether
//              a measured input rate of change is specified and how much
//              iteration result information is desired.
//==============================================================================

#ifndef PID_H
#define PID_H

// Numeric limits
#include <limits>

// NAN
#include <cmath>

//==============================================================================
//                         ENUM AND STRUCT DEFINITIONS
//==============================================================================

// Enum defining possible PID units. Units can be an angle in degrees or radians
// or a non-angle unit.
typedef enum
{
    PID_NON_ANGLE,
    PID_ANGLE_RAD,
    PID_ANGLE_DEG
} PidUnits;

// Structure containing additional information about an iteration
typedef struct
{

    // Input rate of change
    double input_rate = NAN;

    // Error between setpoint and input
    double error = NAN;

    // Individual term contributions to PID output. The sum of these is the
    // total output.
    double p_output = NAN;
    double i_output = NAN;
    double d_output = NAN;

    // Total integral error
    double i_error_sum = NAN;

    // Total output
    double output = NAN;

    // Total output without clamping
    double unclamped_output = NAN;

    // Flags indicating whether the output and integral are saturated
    bool output_saturated = NAN;
    bool i_saturated = NAN;

} IterationInfo;

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class Pid
{

public:

    //--------------------------------------------------------------------------
    // Name:        Pid constructor
    // Arguments:   - pid_type: Indicates the units of the input and setpoint.
    //                If the units are an angle, the error term will be
    //                calculated as a minumum angle to prevent wrapping
    //                problems.
    //--------------------------------------------------------------------------
    Pid(PidUnits units = PID_NON_ANGLE);

    //--------------------------------------------------------------------------
    // Name:        Pid destructor
    //--------------------------------------------------------------------------
    virtual ~Pid();

    //--------------------------------------------------------------------------
    // Name:        iterate
    // Description: Calculates the next PID output value based on the input and
    //              setpoint values. This function must be called at the
    //              iteration rate set with set_iteration_rate for proper
    //              operation. Input rate of change will be calculated from the
    //              previous input value.
    // Arguments:   - input: PID input value, the current value of the signal
    //                being controlled.
    //              - setpoint: PID setpoint value, the desired value of the
    //                signal being controlled.
    // Returns:     PID output value.
    //--------------------------------------------------------------------------
    double iterate(double input, double setpoint);

    //--------------------------------------------------------------------------
    // Name:        iterate
    // Description: Calculates the next PID output value based on the input and
    //              setpoint values. This function must be called at the
    //              iteration rate set with set_iteration_rate for proper
    //              operation.
    // Arguments:   - input: PID input value, the current value of the signal
    //                being controlled.
    //              - input_rate: Rate of change of the PID input value. Can
    //                be specfied instead of using the previous input value to
    //                calculate rate of change.
    //              - setpoint: PID setpoint value, the desired value of the
    //                signal being controlled.
    // Returns:     PID output value.
    //--------------------------------------------------------------------------
    double iterate(double input, double input_rate, double setpoint);

    //--------------------------------------------------------------------------
    // Name:        iterate
    // Description: Calculates the next PID output value based on the input and
    //              setpoint values. This function must be called at the
    //              iteration rate set with set_iteration_rate for proper
    //              operation. Input rate of change will be calculated from the
    //              previous input value.
    // Arguments:   - input: PID input value, the current value of the signal
    //                being controlled.
    //              - setpoint: PID setpoint value, the desired value of the
    //                signal being controlled.
    //              - info: Structure containing addition iteration information.
    // Returns:     PID output value.
    //--------------------------------------------------------------------------
    double iterate(double input, double setpoint, IterationInfo& info);

    //--------------------------------------------------------------------------
    // Name:        iterate
    // Description: Calculates the next PID output value based on the input and
    //              setpoint values. This function must be called at the
    //              iteration rate set with set_iteration_rate for proper
    //              operation.
    // Arguments:   - input: PID input value, the current value of the signal
    //                being controlled.
    //              - input_rate: Rate of change of the PID input value. Can
    //                be specfied instead of using the previous input value to
    //                calculate rate of change.
    //              - setpoint: PID setpoint value, the desired value of the
    //                signal being controlled.
    //              - info: Structure containing addition iteration information.
    // Returns:     PID output value.
    //--------------------------------------------------------------------------
    double iterate(double input, double input_rate, double setpoint,
        IterationInfo& info);

    //--------------------------------------------------------------------------
    // Name:        set_iteration_rate
    // Description: Sets the iteration rate in Hz. This is the rate at which the
    //              iterate function should be called.
    // Arguments:   - rate: Iteration rate in Hz.
    //--------------------------------------------------------------------------
    void set_iteration_rate(double rate);

    //--------------------------------------------------------------------------
    // Name:        set_gains
    // Description: Sets the proportional, integral, and derivative gains for
    //              the controller and adjusts the integral error sum to prevent
    //              spikes in the output.
    // Arguments:   - new_kp: Proportional gain value.
    //              - new_ki: Integral gain value.
    //              - new_kd: Derivative gain value.
    //--------------------------------------------------------------------------
    void set_gains(double new_kp, double new_ki, double new_kd);

    //--------------------------------------------------------------------------
    // Name:        set_output_limits
    // Description: Sets the output minimum and maximum values. The calculated
    //              PID output will be clamped to be within these limits.
    // Arguments:   - min: Minimum output value.
    //              - max: Maximum output value.
    //--------------------------------------------------------------------------
    void set_output_limits(double min, double max);

    //--------------------------------------------------------------------------
    // Name:        set_integral_limits
    // Description: Sets the minimum and maximum integral output (integral gain
    //              times accumulated integral error). The calculated integral
    //              output will be clamped to be within these limits.
    // Arguments:   - min: Minimum integral output value.
    //              - max: Maximum integral output value.
    //--------------------------------------------------------------------------
    void set_integral_limits(double min, double max);

    //--------------------------------------------------------------------------
    // Name:        clear_integral_error
    // Description: Sets the accumulated integral to zero.
    //--------------------------------------------------------------------------
    void clear_integral_error();

private:

    // PID setpoint and input units
    PidUnits pid_units;

    // PID control algorithm iteration rate in Hz set by the user.
    double iteration_rate = 1.0;

    // Proportional, intagral, and derivative gain values
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;

    // PID output limits. The calculated PID output will be forced to fall
    // within the range of these values
    double output_min = std::numeric_limits<double>::lowest();
    double output_max = std::numeric_limits<double>::max();

    // Previous input value used for derivative error calculations when input
    // rate of change is not specified by the user
    double previous_input = 0.0;

    // Integral error sum and integral output limits
    double i_error_sum = 0.0;
    double i_output_min = std::numeric_limits<double>::lowest();
    double i_output_max = std::numeric_limits<double>::max();

};

#endif // PID_H
