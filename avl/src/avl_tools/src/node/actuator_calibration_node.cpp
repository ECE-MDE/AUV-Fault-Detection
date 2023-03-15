//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node used to calibrate the AUV fins. This node interacts
//              with the user using the standard C++ iostream. The node writes
//              the fin configuration file to a user defined location.
//              Uses the 3rd party library yaml-cpp. See:
//                    https://github.com/jbeder/yaml-cpp/wiki/Tutorial
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  device/pwm (avl_devices/PwmMsg)
//
// Subscribers: None
//==============================================================================

// Base node class
#include <avl_core/node.h>
#include <avl_core/util/file.h>

// ROS and message includes
#include "ros/ros.h"
#include <avl_msgs/PwmMsg.h>
using namespace avl_msgs;

// C++ headers
#include <iostream>
#include <fstream>

// Eigen
#include <Eigen/Dense>

// YAML C++
#include <yaml-cpp/yaml.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class ActuatorCalibrationNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        ActuatorCalibrationNode constructor
    //--------------------------------------------------------------------------
    ActuatorCalibrationNode(int argc, char **argv) : Node(argc, argv) { }

private:

    // Publishers for PWM messages
    ros::Publisher pwm_pub;

    // PWM signal sequence used for calibration
    std::vector<double> pulse_width_sequence;

    // Actuator configuration file location
    std::string fin_calibration_filepath;

    // Actuator configuration data in the form of a YAML node
    YAML::Node fin_calibration_file;

    // Default slope and intercept values
    double default_slope = 1.0/180.0;
    double default_intercept = 1.5;

    // Terminal color escape sequence for calibration node messages
    std::string font_color_string = "\033[1;35m";
    std::string reset_color_string = "\033[0m";

private:

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
                  << "     w        set fins to a pulse width" << std::endl
                  << "     a        set fins to an angle" << std::endl
                  << "     r        reset calibration file" << std:: endl
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
    // Name:        read_calibration_file
    // Description: Reads in the YAML formatted fin calibration config file
    //              at the given filepath.
    // Arguments:   - filepath: path to the fin calibration file
    // Returns:     YAML node containing the calibration file data.
    //--------------------------------------------------------------------------
    YAML::Node read_calibration_file(std::string filepath)
    {

        YAML::Node calibration_file;

        if (avl::file_exists(filepath))
            calibration_file = YAML::LoadFile(filepath);
        else
            throw std::runtime_error("the file " + filepath + " does not exist");

        return calibration_file;

    }

    //--------------------------------------------------------------------------
    // Name:        generate_default_calibration_file
    // Description: Generates a YAML node with default fin calibration
    //              parameters.
    // Returns:     YAML node containing default fin calibration data.
    //--------------------------------------------------------------------------
    YAML::Node generate_default_calibration_file()
    {

        YAML::Node calibration_file;

        std::vector<std::string> fin_strings = {"top","starboard","bottom", "port"};
        for (std::string fin: fin_strings)
        {
            calibration_file[fin]["slope"] = default_slope;
            calibration_file[fin]["intercept"] = default_intercept;
        }

        return calibration_file;

    }

    //--------------------------------------------------------------------------
    // Name:        write_calibration_file
    // Description: Writes a YAML formatted fin calibration config file at the
    //              given filepath.
    // Arguments:   - filepath: filepath to write configuration file to
    //--------------------------------------------------------------------------
    void write_calibration_file(YAML::Node calibration_file, std::string filepath)
    {

        // Calibration file header
        std::string calibration_file_header =
            "#===============================================================================\n"
            "# Autonomous Vehicle Library \n"
            "# \n"
            "# DO NOT MODIFY! Automatically generated from actuator calibration routine! \n"
            "# \n"
            "# PURPOSE: Calibration file generated from the calibration tool: \n"
            "#              avl_tools/src/actuator_calibration_node.cpp \n"
            "#          This file defines the slope and intercept for a linear model fit for \n"
            "#          mapping fin angles commands to PWM commands. \n"
            "# \n"
            "# AUTHOR:  avl_tools/src/actuator_calibration_node.cpp \n"
            "#===============================================================================\n\n";

        std::ofstream fout(filepath, std::ios::out);
        if (fout)
        {
            fout  << calibration_file_header
                  << calibration_file
                  << std::endl;
            fout.close();
        }
        else
        {
            throw std::runtime_error("unable to write to" + filepath);
        }

    }

    //--------------------------------------------------------------------------
    // Name:        get_fin_angle_measurements
    // Description: Acquires fin angle measurements from the user for the
    //              specified fin. The function publishes a series of PWM values
    //              to the specified fin and prompts the user to enter the
    //              corresponding fin angle that they measure.
    // Arguments:   - fin_name: name of fin to calibrate
    // Returns:     Vector of fin angle measurements corresponding to the node's
    //              pulse_width_sequence vector.
    //--------------------------------------------------------------------------
    std::vector<double> get_fin_angle_measurements(const std::string& fin_name)
    {

        // Vector to hold fin angle measurements entered by the user
        std::vector<double> fin_angle_measurements;

        // Print a message indicating which fin is being calibrated
        std::cout << font_color_string
                  << std::endl
                  << "Calibrating the " << fin_name << " fin..." << std::endl
                  << reset_color_string;

        // for each calibration pwm, ask the user to measure the fin angle
        for (double pulse_width: pulse_width_sequence)
        {

            std::cout << font_color_string
                      << "==============================================================================" << std::endl
                      << reset_color_string;

            std::cout << font_color_string
                      << "Publishing " << pulse_width << "ms pulse width to "
                      << fin_name << " fin" << std::endl
                      << reset_color_string;

            std::vector<double> pwm_values = {NAN, NAN, NAN, NAN, NAN, NAN};
            pwm_values.at(get_param<int>("~" + fin_name + "/channel")) = pulse_width;

            // Format and publish the PWM message
            PwmMsg pwm_msg;
            pwm_msg.pulse_width = pwm_values;
            pwm_pub.publish(pwm_msg);

            // Give the fin time to move to its new position
            ros::Duration(0.3).sleep();

            // Request the user's fin angle input until a valid measurement is
            // entered
            bool measurement_valid = false;
            while (!measurement_valid && ros::ok())
            {

                // Prompt the user to measure and enter the fin angle
                std::cout << font_color_string
                          << "Measure the fin's angle in degrees and enter it here: "
                          << reset_color_string;

                // Get the user's fin angle input and store the results in a vector
                std::string fin_angle_string;
                std::cin >> fin_angle_string;

                // Attempt to convert the input string to a double. If the
                // conversion fails, the input is not valid and we should
                // should request a new input
                double fin_angle = NAN;
                try
                {
                    fin_angle = stod(fin_angle_string);
                    fin_angle_measurements.push_back(fin_angle);
                    measurement_valid = true;
                }
                catch (...)
                {

                    measurement_valid = false;
                    std::cout << font_color_string
                              << "Invalid fin angle. Please enter a valid fin angle." << std::endl
                              << reset_color_string;

                }

            }

        }

        std::cout << font_color_string
                  << "==============================================================================" << std::endl
                  << reset_color_string;

        std::cout << font_color_string
                  << "Fin measurement collection complete." << std::endl << std::endl
                  << reset_color_string;

        return fin_angle_measurements;

    }

    //--------------------------------------------------------------------------
    // Name:        calculate_calibration_parameters
    // Description: Calculates a slope and intercept that relates fin angle in
    //              degrees to pulse width in ms using the relation
    //                  y = m*x + z
    //              where y is pulse width, x is fin angle, and m and z are the
    //              slope and intercept of interest for the calibration file.
    // Arguments:   - fin_angle_measurements: vector of fin angle measurements
    //                corresponding to the node's pulse width sequence
    //              - slope: pointer to variable to store the calculated slope
    //              - intercept: pointer to variable to store the calculated
    //                intercept
    //--------------------------------------------------------------------------
    void calculate_calibration_parameters(std::vector<double> fin_angle_measurements,
                                          double* slope, double* intercept)
    {

        std::vector<double> slope_intercept;

        // Find the least squares fit from the measurements assuming a linear
        //     y = m*x + z
        // where y is pulse width, x is fin angle, and m and z are the slope and
        // intercept of interest for the calibration file.
        //
        // The system is but into the form Ax = b for solving as follows:
        //     [x' ones][m; z] = [y']
        // Therefore A = [fin_angle_measurements' ones]
        // and       b = [pulse_width_sequence']
        //
        // The solution is [m, z]' = (A'*A)^(-1) * A' * y
        //
        // use the SVD decomposition function in the Eigen library
        // see https://eigen.tuxfamily.org/dox/group__LeastSquares.html

        // Construct the A matrix and b vector
        Eigen::MatrixXd A(fin_angle_measurements.size(),2);
        Eigen::VectorXd b(fin_angle_measurements.size(),1);
        for (size_t i = 0; i < fin_angle_measurements.size(); i++)
        {
            A(i,0) = fin_angle_measurements.at(i);
            A(i,1) = 1;
            b(i,0) = pulse_width_sequence.at(i);
        }

        // Solve for x using BDCSVD method
        Eigen::VectorXd solution = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

        std::cout << font_color_string
                  << "Least squares calibration parameters solution: " << std::endl
                  << "     slope: " << solution(0) << std::endl
                  << " intercept: " << solution(1) << std::endl
                  << std::endl;

        *slope = solution(0);
        *intercept = solution(1);

    }

    //--------------------------------------------------------------------------
    // Name:        calibrate_fin
    // Description: Carries out the whole fin calibration process for a single
    //              fin. The calibration proceedure is as follows:
    //                  1. Collect fin angle measurements from the user
    //                  2. Calculate fin calibration parameters from fin angle
    //                     measurements
    //                  3. Write the calibration file
    // Arguments:   - fin_name - name of fin to calibrate
    //--------------------------------------------------------------------------
    void calibrate_fin(const std::string& fin_name)
    {

        // Get the fin angle measurements
        std::vector<double> fin_angle_measurements = get_fin_angle_measurements(fin_name);

        // Use the fin angles and pwm sequence to calculate slope and intercept
        double slope, intercept;
        calculate_calibration_parameters(fin_angle_measurements, &slope, &intercept);

        // Write the slope and intercept to the YAML::Node
        fin_calibration_file[fin_name]["slope"] = slope;
        fin_calibration_file[fin_name]["intercept"] = intercept;

        // Write the calibration file
        write_calibration_file(fin_calibration_file, fin_calibration_filepath);

        // Set the fin to zero angle
        set_fin_angle(fin_name, 0.0);

    }

    //--------------------------------------------------------------------------
    // Name:        set_fin_angle
    // Description: Sets a fin to the specified angle based on the fin's
    //              calibration parameters.
    // Arguments:   - fin: fin name ("port", "starboard", "top", "bottom")
    //              - angle: fin angle in degrees
    //--------------------------------------------------------------------------
    void set_fin_angle(const std::string& fin, double angle)
    {

        // Calculate pulse width by using the slope and intercept from the
        // fin calibration parameters. pwm = slope*fin_angle + intercept
        double slope = fin_calibration_file[fin]["slope"].as<double>();
        double intercept = fin_calibration_file[fin]["intercept"].as<double>();
        double pulse_width = slope*angle + intercept;

        std::vector<double> pwm_values = {NAN, NAN, NAN, NAN, NAN, NAN};
        pwm_values.at(get_param<int>("~" + fin + "/channel")) = pulse_width;

        // Format and publish the PWM message
        PwmMsg pwm_msg;
        pwm_msg.pulse_width = pwm_values;
        pwm_pub.publish(pwm_msg);

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Set up the PWM message publisher
        pwm_pub = node_handle->advertise<PwmMsg>("device/pwm", 8);

        // Wait for the PWM node to start
        ros::Duration(1.0).sleep();

        print_intro_message();

        // Get calibration parameters from the config file
        fin_calibration_filepath = get_param<std::string>("~fin_calibration_filepath");
        pulse_width_sequence = get_param<std::vector<double>>("~pulse_width_sequence");

        // Check whether the calibration file already exists. If it does not,
        // generate a calibration file with the default values
        if (avl::file_exists(fin_calibration_filepath))
        {

            fin_calibration_file = read_calibration_file(fin_calibration_filepath);
            std::cout << font_color_string
                      << "Loaded existing calibration file" << std::endl << std::endl
                      << reset_color_string;

        }
        else
        {

            std::cout << font_color_string
                      << "No existing calibration file found, generating default file" << std::endl << std::endl
                      << reset_color_string;

            try
            {
                fin_calibration_file = generate_default_calibration_file();
                write_calibration_file(fin_calibration_file, fin_calibration_filepath);
            }
            catch (const std::exception& ex)
            {
                log_error("unable to write calibration file (%s)", ex.what());
            }

        }

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
                calibrate_fin("top");
            }

            // Calibrate bottom fin
            else if (option_input == "b")
            {
                log_debug("option: b");
                calibrate_fin("bottom");
            }

            // Calibrate port fin
            else if (option_input == "p")
            {
                log_debug("option: p");
                calibrate_fin("port");
            }

            // Calibrate starboard fin
            else if (option_input == "s")
            {
                log_debug("option: s");
                calibrate_fin("starboard");
            }

            // Set fins to a pulse width
            else if (option_input == "w")
            {

                log_debug("option: w");

                // Prompt the user to enter a fin angle
                std::cout << font_color_string
                          << "Enter a pulse width in ms: "
                          << reset_color_string;

                // Get the user's fin angle input and store the results in a vector
                std::string pulse_width_string;
                std::cin >> pulse_width_string;

                // Attempt to convert the input string to a double. If the
                // conversion fails, the input is not valid and we should
                // should request a new input
                double pulse_width = NAN;
                try
                {
                    pulse_width = stod(pulse_width_string);
                    std::vector<double> pwm_values = {NAN, NAN, NAN, NAN, NAN, NAN};

                    pwm_values.at(get_param<int>("~port/channel")) = pulse_width;
                    pwm_values.at(get_param<int>("~starboard/channel")) = pulse_width;
                    pwm_values.at(get_param<int>("~top/channel")) = pulse_width;
                    pwm_values.at(get_param<int>("~bottom/channel")) = pulse_width;

                    // Format and publish the PWM message
                    PwmMsg pwm_msg;
                    pwm_msg.pulse_width = pwm_values;
                    pwm_pub.publish(pwm_msg);

                }
                catch (...)
                {
                    std::cout << font_color_string
                              << "Invalid pulse width. Please try again." << std::endl
                              << reset_color_string;
                }

            }

            // Set fins to an angle
            else if (option_input == "a")
            {
                log_debug("option: a");

                // Prompt the user to enter a fin angle
                std::cout << font_color_string
                          << "Enter a fin angle in degrees: "
                          << reset_color_string;

                // Get the user's fin angle input and store the results in a vector
                std::string fin_angle_string;
                std::cin >> fin_angle_string;

                // Attempt to convert the input string to a double. If the
                // conversion fails, the input is not valid and we should
                // should request a new input
                double fin_angle = NAN;
                try
                {
                    fin_angle = stod(fin_angle_string);
                    set_fin_angle("top", fin_angle);
                    set_fin_angle("bottom", fin_angle);
                    set_fin_angle("port", fin_angle);
                    set_fin_angle("starboard", fin_angle);
                }
                catch (...)
                {
                    std::cout << font_color_string
                              << "Invalid fin angle. Please try again." << std::endl
                              << reset_color_string;
                }

            }

            // Reset calibration file
            else if (option_input == "r")
            {
                log_debug("option: r");
                fin_calibration_file = generate_default_calibration_file();
                write_calibration_file(fin_calibration_file, fin_calibration_filepath);
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

        std::cout << font_color_string
                  << "Terminating node"
                  << std::endl;

    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    ActuatorCalibrationNode node(argc, argv);
    node.start();
    return 0;
}
