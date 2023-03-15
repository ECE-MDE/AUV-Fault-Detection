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

#include <avl_control/algorithm/pid.h>

// Util functions
#include "avl_core/util/math.h"
using namespace avl;

// Runtime error
#include <stdexcept>

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        Pid constructor
//------------------------------------------------------------------------------
Pid::Pid(PidUnits units) : pid_units(units)
{

}

//------------------------------------------------------------------------------
// Name:        Pid destructor
//------------------------------------------------------------------------------
Pid::~Pid()
{

}

//------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------
double Pid::iterate(double input, double setpoint)
{
    IterationInfo info;
    return iterate(input, setpoint, info);
}

//------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------
double Pid::iterate(double input, double input_rate, double setpoint)
{
    IterationInfo info;
    return iterate(input, input_rate, setpoint, info);
}

//------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------
double Pid::iterate(double input, double setpoint, IterationInfo& info)
{
    double input_rate = (input - previous_input) * iteration_rate;
    previous_input = input;
    return iterate(input, input_rate, setpoint, info);
}

//------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------
double Pid::iterate(double input, double input_rate, double setpoint,
    IterationInfo& info)
{

    // Calculate the error as the difference between the setpoint and input
    double error = setpoint - input;

    // If the setpoint is an angle, adjust the error to be the minimum angle
    // between the input and the setpoint
    if(pid_units == PID_ANGLE_RAD || pid_units == PID_ANGLE_DEG)
        error = avl::wrap(error, pid_units == PID_ANGLE_RAD);

    // Calculate the PID error terms. Input derivative is used instead of
    // error derivative to prevent spikes caused by setpoint change
    double p_error = error;
    double i_error = error / iteration_rate;
    double d_error = -input_rate;

    // Calculate the new integral error sum. This value will not be saved until
    // we check for integral output and total output saturation. If either the
    // integral output or the total output is saturated, we don't want to add to
    // the integral error sum to prevent integrator windup
    double new_i_error_sum = i_error_sum + i_error;

    // Calculate the output value by summing the proportional, integral and
    // derivative terms. The current iteration's integral error will not be
    // added to the running total until we check for saturation

    // Calculate the proportional, integralm and derivative output values
    double p_output = kp * p_error;
    double i_output = ki * new_i_error_sum;
    double d_output = kd * d_error;

    // Clamp the integral output to its limits and check for saturation
    i_output = clamp(i_output, i_output_min, i_output_max);
    bool i_saturated = i_output == i_output_min ||
                       i_output == i_output_max;

    // Calculate the output value by summing the proportional, integral and
    // derivative terms and check for saturation
    double output = p_output + i_output + d_output;
    output = clamp(output, output_min, output_max);
    bool output_saturated = output == output_min ||
                            output == output_max;

    // If either the integral output or the total output is saturated, we don't
    // want to add to the integral error sum to prevent integrator windup. The
    // integral error sum also does not need to be increased if ki is zero
    if (!i_saturated && !output_saturated && ki != 0.0)
        i_error_sum = new_i_error_sum;

    // Update the iteration info
    info.input_rate = input_rate;
    info.error = error;
    info.p_output = p_output;
    info.i_output = i_output;
    info.d_output = d_output;
    info.i_error_sum = i_error_sum;
    info.output = output;
    info.unclamped_output = p_output + i_output + d_output;
    info.output_saturated = output_saturated;
    info.i_saturated = i_saturated;

    return output;

}

//------------------------------------------------------------------------------
// Name:        set_iteration_rate
// Description: Sets the iteration rate in Hz. This is the rate at which the
//              iterate function should be called.
// Arguments:   - rate: Iteration rate in Hz.
//------------------------------------------------------------------------------
void Pid::set_iteration_rate(double rate)
{
    if (rate <= 0.0)
        throw std::runtime_error("PID iteration rate must be greater than 0");
    iteration_rate = rate;
}

//------------------------------------------------------------------------------
// Name:        set_gains
// Description: Sets the proportional, integral, and derivative gains for
//              the controller and adjusts the integral error sum to prevent
//              spikes in the output.
// Arguments:   - new_kp: Proportional gain value.
//              - new_kp: Integral gain value.
//              - new_kp: Derivative gain value.
//------------------------------------------------------------------------------
void Pid::set_gains(double new_kp, double new_ki, double new_kd)
{

    // If either the new or old Ki is zero, reset the accumulated error.
    if (new_ki == 0.0 || ki == 0.0)
    {
        i_error_sum = 0.0;
    }
    else
    {

        // Calculate the ratio between the old Ki and the new Ki so that the
        // accumulated integral error can be adjusted to prevent spikes due to
        // changing of gain
        double ki_ratio = new_ki / ki;

        // Adjust the accumulated integral error
        i_error_sum = i_error_sum / ki_ratio;

    }

    // Set the new gain values
    kp = new_kp;
    ki = new_ki;
    kd = new_kd;

}

//------------------------------------------------------------------------------
// Name:        set_output_limits
// Description: Sets the output minimum and maximum values. The calculated
//              PID output will be clamped to be within these limits.
// Arguments:   - min: Minimum output value.
//              - max: Maximum output value.
//------------------------------------------------------------------------------
void Pid::set_output_limits(double min, double max)
{
    output_min = min;
    output_max = max;
}

//------------------------------------------------------------------------------
// Name:        set_integral_limits
// Description: Sets the minimum and maximum integral output (integral gain
//              times accumulated integral error). The calculated integral
//              output will be clamped to be within these limits.
// Arguments:   - min: Minimum integral output value.
//              - max: Maximum integral output value.
//------------------------------------------------------------------------------
void Pid::set_integral_limits(double min, double max)
{
    i_output_min = min;
    i_output_max = max;
}

//------------------------------------------------------------------------------
// Name:        clear_integral_error
// Description: Sets the accumulated integral to zero.
//------------------------------------------------------------------------------
void Pid::clear_integral_error()
{
    i_error_sum = 0.0;
}
