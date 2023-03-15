//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node to read and publish IMU and attitude estimate data from
//              a Microstrain AHRS using the MIP protocol.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  device/ahrs (avl_msgs/AhrsMsg)
//
// Subscribers: None
//==============================================================================

// Node base class
#include <avl_core/node.h>

// Serial port class
#include <avl_asio/serial_port.h>

// Util functions
#include <avl_core/util/math.h>
#include <avl_core/util/misc.h>
#include <avl_core/util/file.h>

// Command handler class
#include <avl_comms/command_handler.h>
using namespace avl;

// ROS messages and services
#include <avl_msgs/AhrsMsg.h>
using namespace avl_msgs;

// Microstrain AHRS serial commands
#include <avl_devices/protocol/mip/mip.h>

// Eigen library for linear algebra
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>
#include <complex>
#include <unsupported/Eigen/MatrixFunctions>
using namespace Eigen;

//==============================================================================
//                              ENUM DEFINITION
//==============================================================================

// Enum listing calibration commands
enum CalCommand
{
    CAL_START,
    CAL_FINISH
};

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class MicrostrainAhrsNode : public Node
{

public:
    //--------------------------------------------------------------------------
    // Name:        MicrostrainNode constructor
    //--------------------------------------------------------------------------
    MicrostrainAhrsNode(int argc, char **argv) : Node(argc, argv) { }

private:

    // Serial device instance
    SerialPort serial;

    // Buffer to construct a MIP packet from bytes as they're received
    std::vector<uint8_t> mip_buffer;

    // Publisher for AHRS data message
    ros::Publisher ahrs_pub;

    // Command handler for handling incoming COMMAND packets
    CommandHandler command_handler;

    // 3xN matrix containing all magnetometer data for calibration. This matrix
    // will act as a ring buffer, looping back to the beginning of the matrix to
    // save data when the max number of data points is reached. The sample index
    // points to the column that the next data point will be saved to
    bool mag_cal_active = false;
    Matrix3Xf mag_data;
    int max_mag_samples;
    int sample_idx = 0;

private:

    //--------------------------------------------------------------------------
    // Name:        send_command
    // Description: Sends bytes to the AHRS and logs the transmission.
    // Arguments:   - bytes: Bytes to send to the AHRS.
    //--------------------------------------------------------------------------
    void send_command(std::vector<uint8_t> bytes)
    {
        serial.write(bytes);
        log_debug("[tx] %s", avl::byte_to_hex(bytes).c_str());
    }

    //--------------------------------------------------------------------------
    // Name:        get_mag_cal_params
    // Description: Calculates the soft iron matrix and hard iron vector from
    //              the 3xN matrix of magnetometer data points. Follows the
    //              algorithm outlined in:
    //          https://teslabs.com/articles/magnetometer-calibration/
    //              and the paper it references:
    //          https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=1290055
    // Arguments:   - data: 3 x N matrix of magneteometer data to use for cal.
    //              - A: Reference to save soft iron matrix.
    //              - b: Reference to save hard iron vector.
    //--------------------------------------------------------------------------
    void get_mag_cal_params(Matrix3Xf data, Matrix3f& A, Vector3f& b)
    {

        // Algorithm adapted from :
        //     Qingde Li; Griffiths, J.G., "Least squares ellipsoid specific
        //     fitting," in Geometric Modeling and Processing, 2004.
        //     Proceedings, vol., no., pp.335-340, 2004
        //  https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=1290055

        // Define the sample matrix D according to the beginning section 3 of
        // the paper where D = (X1, X2, ..., Xn) and each element Xi is defined
        // as X = (x^2, y^2, z^2, 2yz, 2xz, 2xy, 2x, 2y, 2z, 1)^T
        MatrixXf D = MatrixXf::Zero(10, data.cols());
        D.row(0) = data.array().square().row(0);
        D.row(1) = data.array().square().row(1);
        D.row(2) = data.array().square().row(2);
        D.row(3) = 2*(data.array().row(1)*data.array().row(2));
        D.row(4) = 2*(data.array().row(0)*data.array().row(2));
        D.row(5) = 2*(data.array().row(0)*data.array().row(1));
        D.row(6) = 2*(data.array().row(0));
        D.row(7) = 2*(data.array().row(1));
        D.row(8) = 2*(data.array().row(2));
        D.row(9) = MatrixXf::Constant(1, data.cols(), 1.0);

        // (Eq. 7) Define the 6x6 C matrix with k = 4
        MatrixXf C = MatrixXf::Zero(6,6);
        C.block(0,0,3,3) << -1,1,1,1,-1,1,1,1,-1;
        C.block(3,3,3,3) << -4, 0, 0, 0, -4, 0, 0, 0, -4;

        // (Eq. 11) Define the block matrices S
        MatrixXf S = D*D.transpose();
        MatrixXf S_11 = S.block(0,0,6,6);
        MatrixXf S_12 = S.block(0,6,6,4);
        MatrixXf S_21 = S.block(6,0,4,6);
        MatrixXf S_22 = S.block(6,6,4,4);

        // (Eq. 15) Calculate the eigen system matrix and get the eigenvalues
        // and eigenvectors
        MatrixXf E = C.inverse()*(S_11 - (S_12*(S_22.inverse()*S_21)));
        EigenSolver<MatrixXf> ces(E);
        VectorXcf E_w = ces.eigenvalues();
        MatrixXcf E_v = ces.eigenvectors();

        // Find the index of the largest eigenvalue
        size_t max_idx = 0;
        float max_eigenvalue = -std::numeric_limits<float>::max();
        for (int i = 0; i < E_w.size(); i++)
        {
            if (E_w(i).real() > max_eigenvalue)
            {
                max_eigenvalue = E_w(i).real();
                max_idx = i;
            }
        }

        // The solution v1 is the eigenvector associated with the largest
        // eigenvalue
        VectorXcf v_1 = E_v.col(max_idx);
        if(v_1(0).real() < 0.0)
            v_1 = -v_1;

        // (Eq. 13) Use v1 to calculate v2
        VectorXcf v_2 = (-S_22.inverse()*S_21)*v_1;

        // Calculate the quadratic form parameters M, n, and d
        Matrix3cf M;
        M << v_1(0), v_1(3), v_1(4),
             v_1(3), v_1(1), v_1(5),
             v_1(4), v_1(5), v_1(2);

        Vector3cf n;
        n << v_2(0),
             v_2(1),
             v_2(2);

        std::complex<float> d = v_2(3);

        // Radius to fit to
        float F = 1.0;

        // Using Equation 16 and Equation 17 from the teslabs webpage, calculate
        // the calibration parameters from the quadratic parameters
        Matrix3cf M_inv = M.inverse();
        b = (-M_inv*n).real();
        std::complex<float> temp = n.transpose()*M_inv*n;
        A = (F / (std::sqrt(temp-d)) * M.sqrt()).real();

    }

    //--------------------------------------------------------------------------
    // Name:        command_callback
    // Description: Called when a COMMAND packet is received by the
    //              communication architecture.
    // Arguments:   - channel: Channel that the packet was received through.
    //              - interface: Interface that the packet was received from.
    //              - command_name: Command name of the received command.
    //              - params: Command parameter list.
    //              - result: Should be set to indicate if the response to the
    //                packet is a success or failure.
    //              - data: Should be set to contain data as a response to the
    //                packet or indicate the reason for a failure.
    // Returns:     True if the packet was responded to, false if it was not.
    //--------------------------------------------------------------------------
    bool command_callback(CommsChannel channel, CommsInterface interface,
        std::string command_name, ParameterList params,
        bool& result, std::vector<uint8_t>& data)
    {

        // Handle MAG CAL commands
        if (command_name == "MAG CAL")
        {

            CalCommand command = params.get("COMMAND").to_enum<CalCommand>();

            // Command to start magnetometer calibration
            if (command == CAL_START)
            {

                if (!mag_cal_active)
                {

                    log_info("starting magnetometer calibration...");

                    // Reset the calibration parameters on the AHRS
                    Matrix3f A = Matrix3f::Identity();
                    Vector3f b = Vector3f::Zero();
                    send_command(AHRS_SET_SOFT_IRON(A));
                    send_command(AHRS_SET_HARD_IRON(b));

                    // Reset the mag data matrix
                    mag_data = Matrix3Xf();
                    sample_idx = 0;

                    // Set the active flag to start collecting data
                    mag_cal_active = true;

                    log_info("colleting magnetometer calibration data...");
                    result = true;

                }
                else
                {
                    result = false;
                    std::string msg = "calibration already started";
                    data = std::vector<uint8_t>(msg.begin(), msg.end());
                }

                return true;

            }

            // Command to finish magnetometer calibration
            else if (command == CAL_FINISH)
            {

                if (mag_cal_active)
                {

                    log_info("finishing magnetometer calibration...");

                    // Set the active flag to stop collecting data
                    mag_cal_active = false;

                    // Calculate the new calibration parameters from the data
                    log_info("calculating magnetometer calibration params...");
                    Matrix3f A = Matrix3f::Identity();
                    Vector3f b = Vector3f::Zero();
                    get_mag_cal_params(mag_data, A, b);

                    // Write the new calibration parameters to the AHRS and save
                    // them as the startup values
                    log_info("writing magnetometer calibration to AHRS...");
                    send_command(AHRS_SET_SOFT_IRON(A));
                    send_command(AHRS_SET_HARD_IRON(b));
                    send_command(AHRS_SAVE_SOFT_IRON());
                    send_command(AHRS_SAVE_HARD_IRON());

                    // Log the calibration parameters
                    log_info("samples used: %d", mag_data.cols());
                    log_info("A: %.2f %.2f %.2f", A(0,0), A(0,1), A(0,2));
                    log_info("   %.2f %.2f %.2f", A(1,0), A(1,1), A(1,2));
                    log_info("   %.2f %.2f %.2f", A(2,0), A(2,1), A(2,2));
                    log_info("b: %.2f %.2f %.2f", b(0), b(1), b(2));

                    // Write the calibration parameters to file
                    auto t = std::time(nullptr);
                    auto tm = *std::localtime(&t);
                    std::ostringstream oss;
                    oss << "/var/avl_calibration/mag_cal_"
                        << std::put_time(&tm, "%d-%m-%Y_%H-%M-%S");
                    std::string filepath = oss.str();

                    std::ofstream file(filepath);
                    file << A << std::endl << std::endl;
                    file << b << std::endl << std::endl;
                    file << mag_data.transpose();
                    file.close();
                    log_info("wrote calibration data to %s", filepath.c_str());

                    log_info("magnetometer calibration finished");
                    result = true;

                }
                else
                {
                    result = false;
                    std::string msg = "calibration not started";
                    data = std::vector<uint8_t>(msg.begin(), msg.end());
                }

                return true;

            }

                result = false;
                return true;

        }

        return false;

    }

    //--------------------------------------------------------------------------
    // Name:        configure_ahrs_settings
    // Description: Configures the AHRS following the steps given in the Data
    //              Communications Protocol Manual
    //                  2.4.1 Continuous Data Example Command Sequence
    //              The AHRS is configured to output Euler angles, angular
    //              velocities, linear accelerations, and magnetix flux.
    //--------------------------------------------------------------------------
    void configure_ahrs_settings()
    {

        log_info("configuring AHRS settings...");

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Step 1: Put the Device in Idle Mode
        log_info("entering idle mode...");
        send_command(AHRS_SET_IDLE());

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Step 2: Configure the IMU Data-stream format
        log_info("setting the IMU message format...");
        send_command(AHRS_SET_IMU_MESSAGE_FORMAT());

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Step 3: Configure the Estimation Filter Data-stream Format
        // We can skip this step since we aren't using the estimation filter

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Step 4: Save the IMU and Estimation Filter MIP Message Format
        // We can skip this since we configure the AHRS on every startup

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Extra step: Set the Sensor-to-vehicle Frame Transformation
        log_info("setting the sensor-to-vehicle frame transformation...");
        send_command(AHRS_SET_SENSOR_TO_VEHICLE_TRANSFORM(
            get_param<std::vector<float>>("~theta_sensor_to_vehicle")));

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		   // Step 5: Enable the IMU and Estimation Filter Data-streams
        log_info("enabling continuous AHRS data streaming...");
        send_command(AHRS_ENABLE_IMU_STREAM());

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Skip Step 6: Resume the Device: (Optional)
        // We can skip ths step since we are specifically enabling the IMU
        // data stream

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Skip Step 7: Initialize the Filter
        // We can skip ths step since we are using auto-initialize

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Read magnetometer calibration values
        log_info("reading magnetometer calibration...");
        send_command(AHRS_GET_SOFT_IRON());
        send_command(AHRS_GET_HARD_IRON());

        log_info("AHRS configuration completed");

    }

    //--------------------------------------------------------------------------
    // Name:        mip_header_read_callback
    // Description: Called when the MIP header is read.
    // Arguments:   - data: MIP packet header bytes
    //--------------------------------------------------------------------------
    void mip_header_read_callback(std::vector<uint8_t> data)
    {

        mip_buffer.clear();

        // Add the MIP packet header to the MIP packet buffer
        avl::append(mip_buffer, MIP_PACKET_HEADER);

        // Set the match condition to read the next two MIP packet descriptor
        // and payload length bytes.
        serial.set_match(Match(2,
            &MicrostrainAhrsNode::mip_description_read_callback, this));

    }

    //--------------------------------------------------------------------------
    // Name:        mip_description_read_callback
    // Description: Called when the MIP packet description bytes are read.
    // Arguments:   - data: MIP packet description bytes
    //--------------------------------------------------------------------------
    void mip_description_read_callback(std::vector<uint8_t> data)
    {

        // Add the MIP packet description bytes to the MIP packet buffer
        avl::append(mip_buffer, data);

        // Get the total payload length in bytes
        int payload_length = data.at(1);

        // Set the match condition to read the number of payload bytes given by
        // the payload length byte and the last two CRC bytes
        serial.set_match(Match(payload_length + 2,
            &MicrostrainAhrsNode::mip_payload_read_callback, this));

    }

    //--------------------------------------------------------------------------
    // Name:        mip_payload_read_callback
    // Description: Called when the MIP payload bytes are read.
    // Arguments:   - data: MIP packet payload bytes
    //--------------------------------------------------------------------------
    void mip_payload_read_callback(std::vector<uint8_t> data)
    {

        // Add the MIP packet payload bytes to the MIP packet buffer. The buffer
        // should now contain a complete MIP packet
        avl::append(mip_buffer, data);

        try
        {

            // Turn the bytes into a MIP packet class to be parsed
            MipPacket packet(mip_buffer);

            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // Handle IMU data command set
            if (packet.get_descriptor() == IMU_DATA)
            {

                // Parse the IMU data packet into an AHRS message and publish it
                AhrsMsg ahrs_msg = parse_ahrs_data(packet);
                ahrs_pub.publish(ahrs_msg);

                log_data("[attitude] %.6f %.6f %.6f",
                    ahrs_msg.theta.x, ahrs_msg.theta.y, ahrs_msg.theta.z);
                log_data("[accelerometer] %.6f %.6f %.6f",
                    ahrs_msg.a.x, ahrs_msg.a.y, ahrs_msg.a.z);
                log_data("[gyroscope] %.6f %.6f %.6f",
                    ahrs_msg.w.x, ahrs_msg.w.y, ahrs_msg.w.z);
                log_data("[magnetometer] %.6f %.6f %.6f",
                    ahrs_msg.mag.x, ahrs_msg.mag.y, ahrs_msg.mag.z);

                // If we are currently calibrating the magnetometer, samples
                // will be added to the buffer of mag data
                if (mag_cal_active)
                {

                    // Create a vector from the magnetometer data
                    Eigen::Vector3f sample;
                    sample << ahrs_msg.mag.x, ahrs_msg.mag.y, ahrs_msg.mag.z;

                    // The matrix of mag samples starts with no data, and must
                    // be expanded by 1 column for every data sample that comes
                    // in. We cannot pre-allocate the columns with zeros because
                    // that would affect the parameter calculations.
                    // Additionally, if we reach the max number of samples, we
                    // want to loop back to the beginning of the matrix and save
                    // incoming values there, like a ring buffer

                    // If the matrix does not have enough columns to hold the
                    // new data point, add a column
                    if (mag_data.cols() < sample_idx + 1)
                        mag_data.conservativeResize(Eigen::NoChange,
                            mag_data.cols() + 1);

                    // Add the sample to the matrix
                    mag_data.col(sample_idx) = sample;

                    // If we reach the maximum number of samples, loop back to
                    // the beginning of the matrix to save samples
                    sample_idx++;
                    if (sample_idx == max_mag_samples)
                        sample_idx = 0;

                }

            }

            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // Handle 3DM command set
            else if (packet.get_descriptor() == TDM_COMMAND)
            {
                uint8_t command_desc = packet.get_field(0xF1).get_data().at(0);

                // If the command is in response to a hard iron command,
                // print out the hard iron vector read from the AHRS
                if (command_desc == TDM_MAGNETOMETER_HARD_IRON)
                {
                    auto data = packet.get_field(0x9C).get_data();
                    std::stringstream ss;
                    ss.precision(2);
                    for (size_t i = 0; i < data.size(); i+=4)
                        ss << " " << from_bytes<float>(subvector(data, i, 4));
                    log_info("read hard iron vector b:%s", ss.str().c_str());
                }

                // If the command is in response to a soft iron command,
                // print out the soft iron matrix read from the AHRS
                else if (command_desc == TDM_MAGNETOMETER_SOFT_IRON)
                {
                    auto data = packet.get_field(0x9D).get_data();
                    std::stringstream ss;
                    ss.precision(2);
                    for (size_t i = 0; i < data.size(); i+=4)
                        ss << " " << from_bytes<float>(subvector(data, i, 4));
                    log_info("read soft iron matrix A:%s", ss.str().c_str());
                }

                // Ignore other commands
                else
                {
                    log_debug("ignoring unhandled 3DM command packet");
                }

            }

            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // Unhandled command set
            else
            {
                log_debug("ignoring unhandled MIP packet type (%s)",
                    avl::byte_to_hex(packet.get_descriptor()).c_str());
            }

        }
        catch (const std::exception& ex)
        {
            log_warning("error parsing MIP packet (%s), ignoring packet",
                ex.what());
        }

        // Set the match condition back to the MIP packet header to read the
        // next MIP packet
        serial.set_match(Match(MIP_PACKET_HEADER,
            &MicrostrainAhrsNode::mip_header_read_callback, this));

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Log data headers
	    log_data("[attitude] roll pitch yaw");
        log_data("[attitude] deg deg deg");
        log_data("[gyroscope] \\omega_x \\omega_y \\omega_z");
        log_data("[gyroscope] deg/s deg/s deg/s");
        log_data("[accelerometer] a_x a_y a_z");
        log_data("[accelerometer] m/s^2 m/s^2 m/s^2");
        log_data("[magnetometer] m_x m_y m_z");
        log_data("[magnetometer] gauss gauss gauss");

        // Get the maximum number of mag cal samples from the config file
        max_mag_samples = get_param<int>("~max_mag_samples");

        // Set up the publisher for AHRS data
        ahrs_pub = node_handle->advertise<AhrsMsg>("device/ahrs", 1);

        // Open the serial port
        serial.open(get_param<std::string>("~serial/port_name").c_str(),
                    get_param<int>("~serial/baudrate"));
        serial.set_read_timeout(get_param<int>("~serial/read_timeout"));

        // Set the match condition to the MIP packet header
        serial.set_match(Match(MIP_PACKET_HEADER,
            &MicrostrainAhrsNode::mip_header_read_callback, this));

        // Configure the AHRS settings
        configure_ahrs_settings();

        // Set the command handler's callback
        command_handler.set_callback(
            &MicrostrainAhrsNode::command_callback, this);

    }

    //--------------------------------------------------------------------------
    // Name:        run
    // Description: Main node loop. Called after the node is initialized.
    //--------------------------------------------------------------------------
    void run()
    {
        ros::Rate spin_rate(1000);
        while (ros::ok())
        {
            serial.spin_once();
            ros::spinOnce();
            spin_rate.sleep();
        }
    }

    //--------------------------------------------------------------------------
    // Name:        shutdown
    // Description: Called after the run function when the node is started.
    //--------------------------------------------------------------------------
    void shutdown()
    {
        serial.close();
    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    MicrostrainAhrsNode node(argc, argv);
    node.start();
    return 0;
}
