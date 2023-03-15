//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node for control of vehicle surge (forward) velocity with
//              an input of surge velocity and surge velocity command and an
//              output of propeller RPM command.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  setpoint/rpm (std_msgs/Float64)
//
// Subscribers: setpoint/ground_speed (std_msgs/Float64)
//              device/velocity (geometry_msgs/Vector3)
//==============================================================================

// Control node base class
#include <avl_control/control_node.h>

// ROS and message includes
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <avl_msgs/DvlMsg.h>
#include <geometry_msgs/Vector3.h>
#include <avl_msgs/Float64SetpointMsg.h>
using namespace avl_msgs;

// Util functions
#include "avl_core/util/math.h"

// PID controller class
#include <avl_control/algorithm/pid.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class SpeedControlNode : public ControlNode
{

public:

    //--------------------------------------------------------------------------
    // Name:        SpeedControlNode constructor
    //--------------------------------------------------------------------------
    SpeedControlNode(int argc, char **argv) : ControlNode(argc, argv) { }

private:

    // Subscribers for RPM and RPM command messages
    ros::Publisher rpm_pub;

    // Speed control parameters
    double b = 200;
    double ksurge = 1.0;
    double Ki_surge = 1.4;
    double h = 0.1; // control loop rate
    double SURGE_MAX_INT = 75;
    double PROP_MAX_ERROR = 5; //ets slew rate
    double m_surge{162.86};
    double b_surge{-72.81};

    bool bottom_lock_hysteresis = false;

    //Initialize
    double prop_RPM_command = 0;
    double surge_integrator = 0;
    double next_prop_RPM_command = 0;

private:

    //--------------------------------------------------------------------------
    // Name:        iterate
    // Description: Called on each controller iteration. Controller iteration
    //              logic and output publishing should be implemented here.
    //--------------------------------------------------------------------------
    void iterate()
    {

      // Get the input and setpoint
      double surge = get_input<geometry_msgs::Vector3>("device/velocity").x;
      double surge_command = get_float64_setpoint("setpoint/ground_speed", 0.0);
      double surge_error = surge_command - surge;

      //If (bottom_lock_hysteresis == true),DVL has bottom_lock.The DVL
      //generates a bottom-lock flag.  If true, then we can use the surge
      //velocity from the DVL.If the DVL has bottom lock and then looses bottom
      //lock, we will continue to use the most recent surge measurement from the
      // DVL until the DVL reports no bottom-lock for X seconds.  For now, let’s
      // say that X = 2.

      // if(bottom_lock_hysteresis == true)
      // Here we assume bottom lock always true for now.
      if(1)
      {
        next_prop_RPM_command = (m_surge*surge  + b_surge) + Ki_surge * surge_integrator + b*surge_error;
        surge_integrator = surge_integrator + h*ksurge*( surge_error);

        if(abs(surge_integrator) >  SURGE_MAX_INT)
        {
          surge_integrator = sign(surge_integrator)*SURGE_MAX_INT;
        }
      }
      else //bottom_lock_hysteresis is false
      {
        next_prop_RPM_command = (m_surge*surge  + b_surge);
        surge_integrator = 0;
      }

      // rate-limit prop (max slew rate)
      if (abs(next_prop_RPM_command - prop_RPM_command) > PROP_MAX_ERROR)
      {
        next_prop_RPM_command = prop_RPM_command + sign(next_prop_RPM_command - prop_RPM_command)* PROP_MAX_ERROR;
      }

      prop_RPM_command = next_prop_RPM_command;

      // Create the RPM message and publish it
      Float64SetpointMsg rpm_msg;
      rpm_msg.enable = true;
      rpm_msg.data = prop_RPM_command;
      rpm_pub.publish(rpm_msg);

      log_data("[info] %.2f %.2f %.2f %.2f %.2f",
          surge, surge_command, surge_error, surge_integrator, prop_RPM_command);

    }

    //--------------------------------------------------------------------------
    // Name:        sign function
    // Description: Y = sign(x) returns an Y the same size as x,
    //              where each element of Y is:
    //              1 if the corresponding element of x is greater than 0.
    //              0 if the corresponding element of x equals 0.
    //              -1 if the corresponding element of x is less than 0.
    //--------------------------------------------------------------------------
    int sign(double x)
    {
      if (x > 0)
        return 1;
      if (x < 0)
        return -1;
      return 0;
    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Add data log headers
        add_data_header("[info] surge\\_input surge\\_setpoint surge\\_error surge\\_integrator rpm\\_output");
        add_data_header("[info] m/s m/s m/s m/s RPM");

        // Set the control node iteration rate
        set_iteration_rate(get_param<double>("~iteration_rate"));

        // Add the inputs and setpoints
        double min_input_rate = get_param<double>("~min_input_rate");
        double min_setpoint_rate = get_param<double>("~min_setpoint_rate");

        b = get_param<double>("~kp");
        ksurge = get_param<double>("~ksurge");
        Ki_surge = get_param<double>("~ki");
        SURGE_MAX_INT = get_param<double>("~max_int");
        m_surge = get_param<double>("~m_surge");
        b_surge = get_param<double>("~b_surge");

        add_input<geometry_msgs::Vector3>("device/velocity", min_input_rate);
        add_setpoint<Float64SetpointMsg>("setpoint/ground_speed",
            min_setpoint_rate);

        // Set up the publishers and subscribers
        rpm_pub = node_handle->advertise<Float64SetpointMsg>("setpoint/rpm", 1);

    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    SpeedControlNode node(argc, argv);
    node.start();
    return 0;
}
