//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node to control vehicle actuators in the form of fin angles
//              and motor speed percentage. Fin servos and the motor controller
//              are controlled by sending PWM messages to the Maestro PWM node.
//
//              Supports both reversible and non-reversible motor
//              controllers. Maps motor speed percentage from 0% to 100%
//              linearly between the neutral_pwm and max_speed_pwm pulse widths.
//              For example, if we have:
//
//              reversible: false
//              max_speed_pwm: 2.0
//              neutral_pwm: 1.0
//
//              Then the mapping is as follows:
//
//               neutral                max
//              0%, 1.0ms   <--->   100%, 2.0ms
//
//              If the reversible flag is enabled, the speed will also be mapped
//              symmetrically around the neutral speed. For example, if we have:
//
//              reversible: true
//              max_speed_pwm: 2.0
//              neutral_pwm: 1.5
//
//              Then the mapping is as follows:
//
//                 max reverse           neutral            max forward
//               -100%%, 1.0ms  <--->   0%, 1.5ms   <--->   100%, 2.0ms
//
// Services:    device/reset_fins (std_srvs/Trigger)
//
// Clients:     None
//
// Publishers:  device/pwm (avl_msgs/PwmMsg)
//
// Subscribers: device/actuator_control (avl_msgs/ActuatorControlMsg)
//              device/fins (avl_msgs/FinsMsg)
//              device/motor (std_msgs/Float64)
//==============================================================================

// Core utility
#include <avl_core/node.h>
#include <avl_core/monitored_subscriber.h>
#include <avl_core/util/math.h>
#include <avl_core/util/file.h>

// YAML C++
#include <yaml-cpp/yaml.h>

// ROS message includes
#include <std_srvs/Trigger.h>
#include <avl_msgs/FinsMsg.h>
#include <std_msgs/Float64.h>
#include <avl_msgs/PwmMsg.h>
#include <avl_msgs/ActuatorControlMsg.h>
using namespace avl_msgs;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class ActuatorsNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        ActuatorsNode constructor
    //--------------------------------------------------------------------------
    ActuatorsNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:


    // Timed subscribers for fins and motor messages
    MonitoredSubscriber<FinsMsg> fins_sub;
    MonitoredSubscriber<std_msgs::Float64> motor_sub;

    // Timer to output PWM messages at a specified rate and its corresponding
    // timer duration
    ros::Timer pwm_output_timer;
    ros::Duration pwm_output_timeout_duration;

    // Timer for motor controller calibration duration and its corresponding
    // calibration duration
    ros::Timer calibration_timer;
    ros::Duration calibration_timer_duration;

    // Motor calibrated flag. Turns true after the motor has been calibrated
    // for the calibration duration
    bool motor_calibrated = false;

    // Server for the reset_fins service
    ros::ServiceServer reset_fins_server;

    // Publisher for PWM messages
    ros::Publisher pwm_pub;

    // Fin angles that will be output by the output timer callback. Updated by
    // fins messages or the reset fins service
    double port_angle;
    double starboard_angle;
    double top_angle;
    double bottom_angle;
    double motor_percent;

    // Subscriber for actuator control messages and flag indicating whether
    // actuator control is enabled. If it is disabled, fins and motor messages
    // will be ignored
    ros::Subscriber actuator_control_sub;
    bool actuator_control_enabled = true;

    // Default slope and intercept values
    double default_slope = 1.0/180.0;
    double default_intercept = 1.5;

    // Mapping between fin name and calibration values. First value of the
    // pair is the slope and the second is the intercept
    std::map<std::string, std::pair<double, double>> fin_cal_map;

private:

    //--------------------------------------------------------------------------
    // Name:        calculate_fin_pwm
    // Description: Calculates the pulse width in ms corresponding to a given
    //              fin angle in radians using the fin calibration file mapping.
    // Arguments:   - fin: Fin name ("port", "starboard", "top", "bottom").
    //              - angle: Fin angle in radians.
    //--------------------------------------------------------------------------
    double calculate_fin_pwm(std::string fin, double angle)
    {

        // Calculate pulse width by using the slope and intercept from the
        // fin calibration parameters. pwm = slope*fin_angle + intercept
        angle = avl::rad_to_deg(angle);
        double slope = fin_cal_map[fin].first;
        double intercept = fin_cal_map[fin].second;
        double pulse_width = slope*angle + intercept;

        return pulse_width;

    }

    //--------------------------------------------------------------------------
    // Name:        calculate_motor_pwm
    // Description: Calculates the pulse width in ms corresponding to a given
    //              motor percentage using the config file mapping.
    // Arguments:   - percent: Motor speed percentage (-100 to 100).
    //--------------------------------------------------------------------------
    double calculate_motor_pwm(double percent)
    {

        // If the motor percentage is not valid for the motor configuration
        // and has to be clamped, print a warning message
        if (get_param<bool>("~motor/reversible") &&
            !avl::in_range(percent, -100.0, 100.0))
        {
            percent = avl::clamp(percent, -100.0, 100.0);
            log_warning("reversible motor percentage not in range -100 to 100, "
                "clamping");
        }
        else if (!get_param<bool>("~motor/reversible") &&
                 !avl::in_range(percent, 0.0, 100.0))
        {
            percent = avl::clamp(percent, 0.0, 100.0);
            log_warning("non-reversible motor percentage not in range 0 to 100,"
                " clamping");
        }

        // Scale the motor speed percentage to a pulse width using the config
        // file mapping
        double pulse_width = avl::linear_scale(percent, 0, 100.0,
                                   get_param<float>("~motor/neutral_pwm"),
                                   get_param<float>("~motor/max_speed_pwm"));

        return pulse_width;

    }

    //--------------------------------------------------------------------------
    // Name:        publish_pwm_values
    // Description: Publishes a PWM message with the PWM values for the four
    //              fins and the motor.
    //--------------------------------------------------------------------------
    void publish_pwm_values()
    {

        double pwm_port =      calculate_fin_pwm("port",      port_angle);
        double pwm_starboard = calculate_fin_pwm("starboard", starboard_angle);
        double pwm_top =       calculate_fin_pwm("top",       top_angle);
        double pwm_bottom =    calculate_fin_pwm("bottom",    bottom_angle);
        double pwm_motor =     calculate_motor_pwm(motor_percent);

        // Form the PWM mnessage using the channel number associated with the
        // specified fin (from the config file) and the calculated pulse width
        std::vector<double> pulse_width = {NAN, NAN, NAN, NAN, NAN, NAN};
        pulse_width.at(get_param<int>("~port/channel")) =      pwm_port;
        pulse_width.at(get_param<int>("~starboard/channel")) = pwm_starboard;
        pulse_width.at(get_param<int>("~top/channel")) =       pwm_top;
        pulse_width.at(get_param<int>("~bottom/channel")) =    pwm_bottom;
        pulse_width.at(get_param<int>("~motor/channel")) =     pwm_motor;

        PwmMsg pwm_msg;
        pwm_msg.pulse_width = pulse_width;
        pwm_pub.publish(pwm_msg);

        log_data("[actuators] %.2f %.2f %.2f %.2f %.2f",
            port_angle, starboard_angle,
            top_angle, bottom_angle,
            motor_percent);

    }

    //--------------------------------------------------------------------------
    // Name:        reset_fins
    // Description: Immediately sets all fins to their home positions, and sets
    //              the fin angle variables to the home position so that all
    //              future output iterations will keep the fins at the home
    //              position until new positions are received.
    //--------------------------------------------------------------------------
    void reset_fins()
    {
        port_angle =      avl::deg_to_rad(get_param<float>("~port/home_angle"));
        starboard_angle = avl::deg_to_rad(get_param<float>("~starboard/home_angle"));
        top_angle =       avl::deg_to_rad(get_param<float>("~top/home_angle"));
        bottom_angle =    avl::deg_to_rad(get_param<float>("~bottom/home_angle"));
        publish_pwm_values();
    }

    //--------------------------------------------------------------------------
    // Name:        fault_callback
    // Description: Called when a fault is detected by a monitored subscriber.
    // Arguments:   - fault: Fault event structure.
    //--------------------------------------------------------------------------
    void fault_callback(const avl::SubscriberFault& fault)
    {

        if (actuator_control_enabled)
        {

            // If the fault is on the fins topic, reset the fins
            if (fault.topic == "device/fins")
            {
                reset_fins();
                fins_sub.reset();
            }

            // If the fault is on the motor topic, turn off the motor
            else
            {
                motor_percent = 0.0;
                publish_pwm_values();
                motor_sub.reset();
            }

        }

    }

    //--------------------------------------------------------------------------
    // Name:        pwm_output_timer_callback
    // Description: Called when the PWM output timer expires. Publishes a PWM
    //              message corresponding to the fin angles and motor percent.
    // Arguments:   - event: ROS timer event structure.
    //--------------------------------------------------------------------------
    void pwm_output_timer_callback(const ros::TimerEvent& event)
    {
        publish_pwm_values();
    }

    //--------------------------------------------------------------------------
    // Name:        calibration_timer_callback
    // Description: Called when the calibration timer expires. Set the motor
    //              calibrated flag.
    // Arguments:   - event: ROS timer event structure.
    //--------------------------------------------------------------------------
    void calibration_timer_callback(const ros::TimerEvent& event)
    {
        motor_calibrated = true;
    }

    //--------------------------------------------------------------------------
    // Name:        fins_msg_callback
    // Description: Called when a fins message is received on the fins topic.
    //              Sets fin angle and publishes the corresponding PWM message
    //              if any fin angles have changed.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void fins_msg_callback(const FinsMsg& message)
    {

        if (actuator_control_enabled)
        {

            // We need to call the publish_pwm_values function in this callback
            // so that fins change position immediately without having to wait
            // for the next output timer iteration, but we don't want to call it
            // every time or we will be causing outputs every time we receive a
            // message AND every time the output timer iterates. Therefore, only
            // update the PWM when at least one angle has changed.

            if (port_angle      != message.port ||
                starboard_angle != message.starboard ||
                top_angle       != message.top ||
                bottom_angle    != message.bottom)
            {

                // Only update the fin position if the angle is not NaN. If it
                // is NaN, that means the fins message wants to keep that fin's
                // angle the same. Fin angle limits must also be enforced

                if (!std::isnan(message.port))
                {
                    port_angle = avl::clamp(message.port,
                        avl::deg_to_rad(get_param<float>("~port/min_angle")),
                        avl::deg_to_rad(get_param<float>("~port/max_angle")));
                }


                if (!std::isnan(message.starboard))
                {
                    starboard_angle = avl::clamp(message.starboard,
                        avl::deg_to_rad(get_param<float>("~starboard/min_angle")),
                        avl::deg_to_rad(get_param<float>("~starboard/max_angle")));
                }

                if (!std::isnan(message.top))
                {
                    top_angle = avl::clamp(message.top,
                        avl::deg_to_rad(get_param<float>("~top/min_angle")),
                        avl::deg_to_rad(get_param<float>("~top/max_angle")));
                }

                if (!std::isnan(message.bottom))
                {
                    bottom_angle = avl::clamp(message.bottom,
                        avl::deg_to_rad(get_param<float>("~bottom/min_angle")),
                        avl::deg_to_rad(get_param<float>("~bottom/max_angle")));
                }

                publish_pwm_values();

            }

        }

    }

    //--------------------------------------------------------------------------
    // Name:        motor_msg_callback
    // Description: Called when a motor message is received on the motor topic.
    //              Sets motor percent and publishes the corresponding PWM
    //              message if the motor percent has changed.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void motor_msg_callback(const std_msgs::Float64& message)
    {

        if (actuator_control_enabled)
        {

            // Only update the motor speed if the motor has been calibrated
            if (motor_calibrated)
            {

                // We need to call the publish_pwm_values function in this
                // callback so that the motor changes speed immediately without
                // having to wait for the next output timer iteration, but we
                // don't want to call it every time or we will be causing
                // outputs every time we receive a message AND every time the
                // output timer iterates. Therefore, only update the PWM when
                // the motor percent changes.

                if (motor_percent != message.data)
                {

                    // Only update the motor percent if it is not NaN. If it is
                    // NaN, that means the motor message wants to keep the motor
                    // speed the same
                    if (!std::isnan(message.data))
                        motor_percent = message.data;

                    publish_pwm_values();

                }

            }
            else
            {
                log_warning("ignored motor speed message while motor is "
                    "calibrating");
            }

        }

    }

    //--------------------------------------------------------------------------
    // Name:        reset_fins_srv_callback
    // Description: Called when the reset_fins service is requested.  Sets fins
    //              to their home positions given in the config file.
    // Arguments:   - req: Request received on the service.
    //              - res: Response to the service request.
    // Returns:     True if service succeeded, false if it failed and was not
    //              responded to.
    //--------------------------------------------------------------------------
    bool reset_fins_srv_callback(std_srvs::Trigger::Request& req,
                                 std_srvs::Trigger::Response& res)
    {

        // Fins can be reset even if control is disabled, so we do not need to
        // check that actuator control is enabled. This is for fault situations
        // where we want to put the actuators back to the home position when
        // a fault occurs

        port_angle =      avl::deg_to_rad(get_param<float>("~port/home_angle"));
        starboard_angle = avl::deg_to_rad(get_param<float>("~starboard/home_angle"));
        top_angle =       avl::deg_to_rad(get_param<float>("~top/home_angle"));
        bottom_angle =    avl::deg_to_rad(get_param<float>("~bottom/home_angle"));

        publish_pwm_values();

        res.success = true;
        res.message = "success";

        return true;

    }

    //--------------------------------------------------------------------------
    // Name:        actuator_control_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void actuator_control_msg_callback(const ActuatorControlMsg message)
    {
        log_debug("actuator control message received");
        if (message.enable)
        {
            actuator_control_enabled = true;
            fins_sub.reset();
            motor_sub.reset();
            log_debug("enabled actuators");
        }
        else
        {
            actuator_control_enabled = false;
            motor_percent = 0.0;
            log_debug("disabled actuators");
        }
    }

    //--------------------------------------------------------------------------
    // Name:        read_calibration_file
    // Description: Reads in the YAML formatted calibration config file at the
    //              given filepath and saves the pressure offset value from it.
    // Arguments:   - filepath: path to the calibration file
    //--------------------------------------------------------------------------
    void read_calibration_file(std::string filepath)
    {

        YAML::Node calibration_file;

        // Load calibration parameters from file if the file exists. Otherwise,
        // use the default values
        if (avl::file_exists(filepath))
        {

            calibration_file = YAML::LoadFile(filepath);
            log_info("loaded calibration file");
            for (std::string fin : {"top", "bottom", "port", "starboard"})
                fin_cal_map[fin] =
                    {calibration_file[fin]["slope"].as<double>(),
                     calibration_file[fin]["intercept"].as<double>()};

        }
        else
        {
            log_warning("no calibration file found at " + filepath +
                ", using default calibration");
            for (std::string fin : {"top", "bottom", "port", "starboard"})
                fin_cal_map[fin] = {default_slope, default_intercept};
        }

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Log the data header and units
        log_data("[actuators] port starboard top bottom motor");
        log_data("[actuators] deg deg deg deg percent");

        // Read the actuators calibration file if it exists
        read_calibration_file(get_param<std::string>("~calibration_filepath"));

        // Configure and create the PWM output timer
        double pwm_output_rate = get_param<double>("~pwm_output_rate");
        pwm_output_timeout_duration = ros::Duration(1.0/pwm_output_rate);
        pwm_output_timer = node_handle->createTimer(pwm_output_timeout_duration,
            &ActuatorsNode::pwm_output_timer_callback, this);

        // Set up the calibration timer as a one-shot timer.
        // Creating the timer also starts it
        double calibration_duration =
            get_param<double>("~motor/calibration_duration");
        calibration_timer_duration = ros::Duration(calibration_duration);
        calibration_timer = node_handle->createTimer(calibration_timer_duration,
            &ActuatorsNode::calibration_timer_callback, this, true);

        // Set up the monitored subscribers for fin angles and motor percentage
        fins_sub.set_message_rate(get_param<double>("~fins_input_rate"));
        fins_sub.enable_message_rate_check(true);
        fins_sub.enable_out_of_bounds_check(false);
        fins_sub.enable_repitition_check(false);
        fins_sub.subscribe("device/fins", 8,
            &ActuatorsNode::fins_msg_callback,
            &ActuatorsNode::fault_callback, this);

        motor_sub.set_message_rate(get_param<double>("~motor_input_rate"));
        motor_sub.enable_message_rate_check(true);
        motor_sub.enable_out_of_bounds_check(false);
        motor_sub.enable_repitition_check(false);
        motor_sub.subscribe("device/motor", 1,
            &ActuatorsNode::motor_msg_callback,
            &ActuatorsNode::fault_callback, this);

        // Set up the service server for resetting fins
        reset_fins_server = node_handle->advertiseService("device/reset_fins",
            &ActuatorsNode::reset_fins_srv_callback, this);

        // Set up the subscriber for actuator control messages
        actuator_control_sub = node_handle->subscribe("device/actuator_control",
            1, &ActuatorsNode::actuator_control_msg_callback, this);

        // Set up the PWM message publisher
        pwm_pub = node_handle->advertise<PwmMsg>("device/pwm", 1);

        // Ensure the motor is off and the fins are in their home positions
        motor_percent = 0.0;
        reset_fins();

    }

    //--------------------------------------------------------------------------
    // Name:        run
    // Description: Main node loop. Called after the node is initialized.
    //--------------------------------------------------------------------------
    void run()
    {
        ros::spin();
    }

    //--------------------------------------------------------------------------
    // Name:        shutdown
    // Description: Called after the run function when the node is started. Can
    //              be overriden by a derived node class.
    //--------------------------------------------------------------------------
    void shutdown()
    {
        motor_percent = 0.0;
        reset_fins();
    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    ActuatorsNode node(argc, argv);
    node.start();
    return 0;
}
