//==============================================================================
// Autonomous Vehicle Library
//
// Description: A ROS node to handle calibration of the NAVO tail junction board
//              by interfacing with the tail junction node. Presents the user
//              with a command line interface for calibration.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  None
//
// Subscribers: None
//==============================================================================

// Node base class
#include <avl_core/node.h>

// Util functions
#include <avl_core/util/vector.h>
#include <avl_core/util/byte.h>

// Serial port class
#include <avl_asio/serial_port.h>

// AAF command protocol
#include <avl_devices/protocol/aaf/aaf.h>

// C++ headers
#include <iostream>
#include <fstream>

// ROS message includes
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt16.h>
#include <avl_msgs/FinCalibrationMsg.h>
using namespace avl_msgs;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class TailJunctionCalibrationNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        TailJunctionCalibrationNode constructor
    //--------------------------------------------------------------------------
    TailJunctionCalibrationNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Enumeration of fin types
    typedef enum Fin
    {
        FIN_TOP = 0,
        FIN_STARBOARD = 1,
        FIN_BOTTOM = 2,
        FIN_PORT = 3
    } Fin;

private:

    // Publishers
    ros::Publisher fin_calibration_pub;
    ros::Publisher fin_measurement_pub;

    // Subscribers
    ros::Subscriber fin_measurement_request_sub;
    ros::Subscriber fin_calibration_status_sub;

    // Terminal color escape sequence for calibration node messages
    std::string font_color_string = "\033[1;35m";
    std::string reset_color_string = "\033[0m";

    // Flag indicating that a request for a fin measurement has been received
    // over serial from the tail junction board
    bool received_measurement_request = false;

    // Flag indicating that a fin calibration finish status has been received
    // over serial from the tail junction board
    bool received_fin_calibration_finished = false;

private:

    //--------------------------------------------------------------------------
    // Name:        fin_measurement_request_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void fin_measurement_request_msg_callback(const std_msgs::Empty& message)
    {
        log_debug("received measurement request");
        received_measurement_request = true;
    }

    //--------------------------------------------------------------------------
    // Name:        fin_calibration_status_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void fin_calibration_status_msg_callback(const std_msgs::UInt16& message)
    {
        log_debug("received calibration status");
        if (message.data == 4)
            received_fin_calibration_finished = true;
    }

    //--------------------------------------------------------------------------
    // Name:        print_help_message
    // Description: Prints the help menu for the actuator calibration node.
    //--------------------------------------------------------------------------
    void print_help_message()
    {

        std::cout << font_color_string
                  << "   Option     Description " << std::endl
                  << "----------------------------------------------" << std::endl
                  << "     t        calibrate top fin" << std::endl
                  << "     b        calibrate bottom fin" << std::endl
                  << "     p        calibrate port fin" << std::endl
                  << "     s        calibrate starboard fin" << std::endl
                  << "     a        set fins to an angle" << std::endl
                  << "     h        display this help list" << std::endl
                  << "     x        exit the program" << std::endl
                  << std::endl
                  << reset_color_string;

    }

    //--------------------------------------------------------------------------
    // Name:        print_intro_message
    // Description: Prints the intro message for the actuator calibration node.
    //--------------------------------------------------------------------------
    void print_intro_message()
    {

        std::cout << font_color_string
                  << "==============================================================================" << std::endl
                  << "------------------- Welcome to the fin calibration routine! ------------------" << std::endl
                  << "==============================================================================" << std::endl
                  << std::endl
                  << reset_color_string;

    }

    //--------------------------------------------------------------------------
    // Name:        send_start_calibration
    // Description: Sends a Fin_Start_Calibration command to the tail junction
    //              board to start a calibration procedure. The board will
    //              respond with requests for fin angle measurements.
    // Arguments:   - fin: fin to calibrate
    //              - min: fin min angle in degrees
    //              - max: fin max angle in degrees
    //              - home: fin home position in degrees
    //--------------------------------------------------------------------------
    void send_start_calibration(Fin fin, float min, float max, float home)
    {
        FinCalibrationMsg fin_calibration_msg;
        fin_calibration_msg.fin_id = static_cast<uint16_t>(fin);
        fin_calibration_msg.min_angle = min;
        fin_calibration_msg.max_angle = max;
        fin_calibration_msg.home_angle = home;
        fin_calibration_pub.publish(fin_calibration_msg);
    }

    //--------------------------------------------------------------------------
    // Name:        wait_for_measurement_request
    // Description: Waits for a fin measurement request to be received over
    //              serial from the tail junction board.
    //--------------------------------------------------------------------------
    void wait_for_measurement_request()
    {
        ros::Rate spin_rate(1000);
        while (!received_measurement_request &&
               !received_fin_calibration_finished &&
               ros::ok())
        {
            ros::spinOnce();
            spin_rate.sleep();
        }
        received_measurement_request = false;
    }

    //--------------------------------------------------------------------------
    // Name:        send_fin_angle_measurement
    // Description: Sends a Fin_Angle_Measurement command to the tail junction
    //              board in response to a request for a fin angle measurement.
    // Arguments:   - angle: measured fin angle in degrees
    //--------------------------------------------------------------------------
    void send_fin_angle_measurement(float angle)
    {
        std_msgs::Float64 fin_measurement_msg;
        fin_measurement_msg.data = angle;
        fin_measurement_pub.publish(fin_measurement_msg);
    }

    //--------------------------------------------------------------------------
    // Name:        send_fin_set_position
    // Description: Sets a Fin_Set_Position command to set a fin to the
    //              specified angle.
    // Arguments:   - fin: fin whose angle to set
    //              - angle: fin angle in degrees
    //--------------------------------------------------------------------------
    void send_fin_set_position(Fin fin, float angle)
    {
        // TODO
        log_error("send_fin_set_position not yet implemented");
    }

    //--------------------------------------------------------------------------
    // Name:        get_float_from_user
    // Description: Prompts the user to enter a float value and returns the
    //              user's input as a float. Re-prompts the user if an invalid
    //              value is entered.
    // Arguments:   - prompt: message to prompt the user with
    // Returns:     Float entered by the user.
    //--------------------------------------------------------------------------
    float get_float_from_user(const std::string& prompt)
    {

        // Request the user's input until a valid measurement is entered
        while (ros::ok())
        {

            // Prompt the user to enter a value
            std::cout << font_color_string
                      << prompt
                      << reset_color_string;

            // Get the user's input and store the results in a string
            std::string float_string;
            std::cin >> float_string;

            // Attempt to convert the input string to a float. If the
            // conversion fails, the input is not valid and we should
            // should request a new input
            float value = NAN;
            try
            {
                value = stod(float_string);
                log_debug("user entered %f", value);
                return value;
            }
            catch (...)
            {
                std::cout << font_color_string
                          << "Invalid value. Please enter a valid value." << std::endl
                          << reset_color_string;
            }

        }

        throw std::runtime_error("get_float_from_user: failed to get float");

    }

    //--------------------------------------------------------------------------
    // Name:        calibrate_fin
    // Description: Carries out the whole fin calibration process for a single
    //              fin. The calibration proceedure is as follows:
    //                  1. Request the fin min, max, and home angles from the
    //                     user.
    //                  2. Send a Fin_Start_Calibration command to the tail
    //                     junction board.
    //                  3. Wait for measurement requests from the tail junction
    //                     board and collect corresponding fin angle
    //                     measurements from the user.
    //                  4. Read the Fin_Calibration_Status command from the tail
    //                     junction board to indicate that calibration is
    //                     finished.
    // Arguments:   - fin: fin to be calibrated
    //--------------------------------------------------------------------------
    void calibrate_fin(Fin fin)
    {

        std::cout << font_color_string
                  << "Starting calibration..." << std::endl
                  << reset_color_string;
        float min_angle = get_float_from_user("Minimum fin angle: ");
        float max_angle = get_float_from_user("Maximum fin angle: ");
        float home_angle = get_float_from_user("Home fin angle: ");
        send_start_calibration(fin, min_angle, max_angle, home_angle);

        while (!received_fin_calibration_finished && ros::ok())
        {
            wait_for_measurement_request();
            if (!received_fin_calibration_finished)
            {
                float fin_angle = get_float_from_user("Measure and enter the fin angle: ");
                send_fin_angle_measurement(fin_angle);
            }
        }

        std::cout << font_color_string
                  << "Calibration finished." << std::endl
                  << reset_color_string;

        received_fin_calibration_finished = false;

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Set up the publishers
        fin_calibration_pub = node_handle->advertise<FinCalibrationMsg>("device/fin_calibration", 1);
        fin_measurement_pub = node_handle->advertise<std_msgs::Float64>("device/fin_measurement", 1);

        // Set up the subscribers
        fin_measurement_request_sub = node_handle->subscribe("device/fin_measurement_request", 1,
            &TailJunctionCalibrationNode::fin_measurement_request_msg_callback, this);
        fin_calibration_status_sub = node_handle->subscribe("device/fin_calibration_status", 1,
            &TailJunctionCalibrationNode::fin_calibration_status_msg_callback, this);

        print_intro_message();

    }

    //--------------------------------------------------------------------------
    // Name:        run
    // Description: Main node loop. Called after the node is initialized.
    //--------------------------------------------------------------------------
    void run()
    {

        ros::Rate spin_rate(1000);

        std::string option_input = "h";
        while (option_input != "x" && ros::ok())
        {

            ros::spinOnce();
            spin_rate.sleep();

            // Calibrate top fin
            if (option_input == "t")
            {
                log_debug("option: t");
                calibrate_fin(FIN_TOP);
            }

            // Calibrate bottom fin
            else if (option_input == "b")
            {
                log_debug("option: b");
                calibrate_fin(FIN_BOTTOM);
            }

            // Calibrate port fin
            else if (option_input == "p")
            {
                log_debug("option: p");
                calibrate_fin(FIN_PORT);
            }

            // Calibrate starboard fin
            else if (option_input == "s")
            {
                log_debug("option: s");
                calibrate_fin(FIN_STARBOARD);
            }

            // Set fins to an angle
            else if (option_input == "a")
            {

                log_debug("option: a");
                float fin_angle = get_float_from_user("Fin angle: ");
                send_fin_set_position(FIN_TOP, fin_angle);
                send_fin_set_position(FIN_BOTTOM, fin_angle);
                send_fin_set_position(FIN_PORT, fin_angle);
                send_fin_set_position(FIN_STARBOARD, fin_angle);

            }

            // Display help text
            else if (option_input == "h")
            {
                log_debug("option: h");
                print_help_message();
            }

            // Unrecognized option
            else
            {

                std::cout << font_color_string
                          << "Unrecognized option: " << option_input
                          << reset_color_string
                          << std::endl;
                print_help_message();

            }

            std::cout << font_color_string
                      << "Please select an option: ";
            std::cin >> option_input;

        }

    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    TailJunctionCalibrationNode node(argc, argv);
    node.start();
    return 0;
}
