//==============================================================================
// Autonomous Vehicle Library
//
// Description: Processes published sensor data real-time through a navigation
//              filter and publishes the navigation messages generated from the
//              navigation filter estimate.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  nav/gyrocompass (avl_msgs/GyrocompassMsg)
//
// Subscribers: device/imu (avl_msgs/ImuMsg)
//              device/gps (avl_msgs/GpsMsg)
//==============================================================================

// Node base class
#include <avl_core/node.h>

// Util functions
#include <avl_core/util/time.h>

// Inertial navigation filter
#include <avl_navigation/algorithm/gyrocompass_mukf.h>

// ROS message includes
#include <avl_msgs/GyrocompassMsg.h>
#include <avl_msgs/ImuMsg.h>
#include <avl_msgs/GpsMsg.h>
using namespace avl_msgs;

// Alias for double vector
typedef std::vector<double> doubles_t;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class GyrocompassNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        GyrocompassNode constructor
    //--------------------------------------------------------------------------
    GyrocompassNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Gyrocompass filter
    GyrocompassMukf mukf;

    // Filter initial state, its covariance, and process noise covariance
    VectorXd x0 = VectorXd::Zero(15);
    MatrixXd P0;
    MatrixXd Q;

    // Publisher for gyrocompass messages
    bool initialized = false;
    ros::Publisher gyrocompass_pub;

    // Subscribers for navigation data
    ros::Subscriber imu_sub;
    ros::Subscriber gps_sub;

    // Most recent IMU measurements
    Vector3d w_ib_b;
    Vector3d f_ib_b;

    // Minimum number of sats for a GPS lock
    int min_sats;

    // GPS measurement noise covariance matrix and lever arm
    MatrixXd R_gps;
    Vector3d l_bS_b_gps;

private:

    //--------------------------------------------------------------------------
    // Name:        imu_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void imu_msg_callback(const ImuMsg& msg)
    {

        // Don't iterate if the filter is not initialized or data is invalid
        if (!initialized || !msg.valid)
            return;

        // Turn the IMU measurements into vectors
        w_ib_b(0) = msg.angular_velocity.x;
        w_ib_b(1) = msg.angular_velocity.y;
        w_ib_b(2) = msg.angular_velocity.z;

        f_ib_b(0) = msg.linear_acceleration.x;
        f_ib_b(1) = msg.linear_acceleration.y;
        f_ib_b(2) = msg.linear_acceleration.z;

        // Iterate the filter using the IMU data
        mukf.iterate(w_ib_b, f_ib_b, msg.dt);

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Publish and log the filter state if it is valid
        if(mukf.valid())
        {

            VectorXd x = mukf.get_state();
            VectorXd P = mukf.get_cov();

            // // Create and send a navigation message
            GyrocompassMsg msg;
            msg.roll =  x(6);
            msg.pitch = x(7);
            msg.yaw =   x(8);
            msg.roll_std =  sqrt(P(3));
            msg.pitch_std = sqrt(P(4));
            msg.yaw_std =   sqrt(P(5));
            gyrocompass_pub.publish(msg);

            log_data("[x] %s", avl::to_string(x, 9).c_str());
            log_data("[P] %s", avl::to_string(P, 9).c_str());

        }

    }

    //--------------------------------------------------------------------------
    // Name:        gps_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - msg: message received on the topic
    //--------------------------------------------------------------------------
    void gps_msg_callback(const GpsMsg& msg)
    {

        // Construct the GPS position vector
        Vector3d p_b_gps = {msg.lat, msg.lon, msg.alt};

        // Only process the GPS measurement if it has a lock
        if (p_b_gps.allFinite() && msg.num_sats >= min_sats)
        {

            if (initialized)
            {

                MeasInfo info = mukf.process_position(
                    p_b_gps,
                    R_gps,
                    l_bS_b_gps);

                log_data("[gps] %d %s", msg.num_sats,
                    info.to_string().c_str());

            }

            // Initialize the navigation if it is not already
            else
            {

                // Set initial attitude from IMU
                Vector3d theta_n_b = attitude_from_imu(w_ib_b, f_ib_b);
                x0.segment(0,3) = theta_n_b;

                // Set initial velocity and position from GPS
                Vector3d v_n_gps = {msg.ground_speed*cos(msg.track_angle),
                                    msg.ground_speed*sin(msg.track_angle),
                                    0.0};
                Vector3d p_b_gps = {msg.lat, msg.lon, msg.alt};
                x0.segment(3,3) = v_n_gps;
                x0.segment(6,3) = p_b_gps;

                // Initialize the filter
                mukf.init(x0, P0, Q);
                initialized = true;

                // Log the initial state and covariance
                VectorXd x = mukf.get_state();
                VectorXd P = mukf.get_cov();
                log_data("[x] %s", avl::to_string(x, 9).c_str());
                log_data("[P] %s", avl::to_string(P, 9).c_str());

                log_info("-------------------------------------------");
                log_info("Gyrocompass initialized!");
                log_info("    roll:  %.3f deg", avl::rad_to_deg(x0(0)));
                log_info("    pitch: %.3f deg", avl::rad_to_deg(x0(1)));
                log_info("    yaw:   %.3f deg", avl::rad_to_deg(x0(2)));
                log_info("    vN:    %.3f m/s", x0(3));
                log_info("    vE:    %.3f m/s", x0(4));
                log_info("    vD:    %.3f m/s", x0(5));
                log_info("    lat:   %.3f deg", avl::rad_to_deg(x0(6)));
                log_info("    lon:   %.3f deg", avl::rad_to_deg(x0(7)));
                log_info("    alt:   %.3f m",   x0(8));
                log_info("-------------------------------------------");

            }

        } // GPS has lock

    } // GPS message

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Get config file parameters
        min_sats = get_param<int>("~min_sats");
        P0 = avl::from_std_vector(get_param<doubles_t>("~P0")).asDiagonal();
        Q = avl::from_std_vector( get_param<doubles_t>("~Q")).asDiagonal();
        R_gps = avl::from_std_vector( get_param<doubles_t>("~R_gps")).asDiagonal();
        l_bS_b_gps = avl::from_std_vector( get_param<doubles_t>("~l_bS_b_gps"));

        // Set up the publishers and subscribers
        gyrocompass_pub = node_handle->advertise<GyrocompassMsg>("nav/gyrocompass", 1);
        imu_sub = node_handle->subscribe("device/imu", 1,
            &GyrocompassNode::imu_msg_callback, this);
        gps_sub = node_handle->subscribe("device/gps", 1,
            &GyrocompassNode::gps_msg_callback, this);

    }

    //--------------------------------------------------------------------------
    // Name:        run
    // Description: Main node loop. Called after the node is initialized.
    //--------------------------------------------------------------------------
    void run()
    {
        ros::spin();
    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    GyrocompassNode node(argc, argv);
    node.start();
    return 0;
}
