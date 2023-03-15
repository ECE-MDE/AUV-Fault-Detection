//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node to interface with the sprintnav_ins actuator.
//
// Servers:     /dive/sprintnav_align (std_srvs/Trigger)
//
// Clients:     None
//
// Publishers:  device/velocity (geometry_msg/Vector3)
//              device/imu (avl_msgs/ImuMsg)
//              device/ahrs (avl_msgs/AhrsMsg)
//              device/depth (std_msgs/Float64)
//              device/height (std_msgs/Float64)
//              /dive/course (avl_msgs/MagBearingMsg)
//              nav/inertial_nav (avl_msgs/Navigation)
//
// Subscribers: device/gps (avl_devices/GpsMsg)
// 		device/nmea (std_msgs/String)
// 		device/sound_velocity (std_msgs/Float64)
//              device/zda (std_msgs/String)
//              /dive/sprintnav_command (std_msgs/String)
//==============================================================================

// Node base class
#include <avl_devices/device_node.h>

// TCP port class
#include <avl_asio/tcp_socket.h>

// Utility functions
#include <avl_core/util/byte.h>
#include <avl_core/util/geo.h>
#include <avl_core/util/matrix.h>
#include <avl_core/util/misc.h>
#include <avl_core/util/vector.h>

// ROS message includes
#include <avl_msgs/AhrsMsg.h>
#include <avl_msgs/GpsMsg.h>
#include <avl_msgs/ImuMsg.h>
#include <avl_msgs/MagBearingMsg.h>
#include <avl_msgs/NavigationMsg.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>
using namespace avl_msgs;

//Sonardyne Multiplex protocol
#include <avl_devices/protocol/sprint_nav.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class SprintnavInsNode : public DeviceNode
{

public:
  //--------------------------------------------------------------------------
  // Name:        SprintnavInsNode constructor
  //--------------------------------------------------------------------------
  SprintnavInsNode(int argc, char **argv) : DeviceNode(argc, argv) {}

private:
  // TCP device instance
  TcpSocket tcp;

  // Publisher for sprintnav data
  ros::Publisher ahrs_pub;
  ros::Publisher depth_pub;
  ros::Publisher height_pub;
  ros::Publisher imu_pub;
  ros::Publisher inertial_nav_pub;
  ros::Publisher course_pub;
  ros::Publisher velocity_pub;

  // Setup subscribers
  ros::Subscriber gps_sub;
  ros::Subscriber nmea_sub;    // GGA and VTG messages
  ros::Subscriber SV_sub;      // Sound velocity readings
  ros::Subscriber zda_sub;     // ZDA message from CSAC
  ros::Subscriber command_sub; // SprintNav command

  // Setup services
  ros::ServiceServer sprintnav_align_srv;

  // Ros timer for tcp timeout
  ros::Timer tcp_timeout;

  // Status structs
  Dvl dvl{};
  Nav nav{};
  Pwrstat pwrstat{};
  Status status{};

  // Configuration struct for SprintNav
  Config config{};

  // Sensor observation structs
  Obst pdepth_obst{};
  Obst svs_obst{};
  Obst gps_obst{};
  Obst dvl_obst{};

  // Flag indicating a packet was terminated early by match function
  bool early_term = false;

  // Variable to store packet bytes as they are received
  std::vector<uint8_t> packet;

  // Store distance between gps hits and nav
  double gps_dist{};
  double gps_bearing{};

private:

  //--------------------------------------------------------------------------
  // Name:        sprint_align_srv_callback
  // Description: Performs a check on whether the SprintNav is aligned.
  // Arguments:   - request: request received on the service
  //              - response: true if sprintnav is aligned
  // Returns:     True if service succeeded, false if it failed and was not
  //              responded to.
  //--------------------------------------------------------------------------
  bool sprintnav_align_srv_callback(std_srvs::Trigger::Request &request,
                             std_srvs::Trigger::Response &response)
  {
    log_info("SprintNav Alignment check called");
    if(nav.status.pos)
    {
        log_info("SprintNav position valid");
        response.success = true;
        return true;
    }
    response.success = false;
    log_warning("SprintNav position invalid");
    return true;
  }

  //--------------------------------------------------------------------------
  // Name:        nmea_handler
  // Description: Called when nmea subscriber receives a message.
  // Arguments:   - msg: data from nmea ROS TOPIC. Presumably a nmea string
  //--------------------------------------------------------------------------
  void nmea_handler(std::string s, bool zda)
  {
    //Place nmea string into a vector
    std::vector<uint8_t> message;
    std::copy(s.begin(), s.end(), std::back_inserter(message));

    //Identify NMEA message type
    uint16_t mid{};                         //Message identifier
    std::string nmea_type = s.substr(3, 3); //pull the NMEA message type
    if (nmea_type == "GGA")
      mid = 64;
    else if (nmea_type == "VTG")
      mid = 66;
    else if ((nmea_type == "ZDA") && (zda))
      mid = 61;
    else
      return;

    //Apply multiplex wrapper
    message = SPRINTNAV_MULTIPLEX(message, mid);

    //send the multiplexed nmea message to the SprintNav
    tcp.write(message);
  }

  //--------------------------------------------------------------------------
  // Name:        gps_callback
  // Description: Called when gps subscriber receives a message.
  // Arguments:   - msg: data from gps ROS TOPIC.
  //--------------------------------------------------------------------------
  void gps_callback(const GpsMsg::ConstPtr &msg)
  {
    if(!std::isnan(msg->lat) && !std::isnan(msg->lon))
    {
        double gps_lat = msg->lat;
        double gps_lon = msg->lon;
        gps_dist = avl::distance(gps_lat, gps_lon, nav.latitude,
            nav.longitude);
        gps_bearing = avl::final_bearing(gps_lat, gps_lon, nav.latitude,
            nav.longitude);
        log_data("[GPS_TO_NAV] %.2f %.0f", gps_dist, gps_bearing );
    }
  }

  //--------------------------------------------------------------------------
  // Name:        nmea_callback
  // Description: Called when nmea subscriber receives a message.
  // Arguments:   - msg: data from nmea ROS TOPIC. Presumably a nmea string
  //--------------------------------------------------------------------------
  void nmea_callback(const std_msgs::String::ConstPtr &msg)
  {
    nmea_handler(msg->data, false);
  }

  //--------------------------------------------------------------------------
  // Name:        command_callback
  // Description: Called when SprintNav command requested.
  // Arguments:   - msg: data from SprintNav command ROS TOPIC.
  //--------------------------------------------------------------------------
  void command_callback(const std_msgs::String::ConstPtr &msg)
  {
    log_info("Command received: %s", msg->data.c_str());
    tcp.write(SPRINTNAV_COMMAND(msg->data));
  }

  //--------------------------------------------------------------------------
  // Name:        zda_callback
  // Description: Called when zda subscriber receives a message.
  // Arguments:   - msg: data from nmea ROS TOPIC. Presumably a nmea string
  //--------------------------------------------------------------------------
  void zda_callback(const std_msgs::String::ConstPtr &msg)
  {
    nmea_handler(msg->data, true);
  }

  //--------------------------------------------------------------------------
  // Name:        SV_callback
  // Description: Called when sound velocity subscriber receives a message.
  // Arguments:   - msg: data from sound velocity ROS TOPIC.
  //--------------------------------------------------------------------------
  void SV_callback(const std_msgs::Float64::ConstPtr &msg)
  {
    //Place sound velocity measurement into a valeport sensor telegram
    //log_debug("SV received: " + std::to_string(msg->data));
    std::string s = std::to_string(msg->data);
    size_t decimal{};
    //Only publish to 3 decimal places
    for (size_t i = 0; i < s.size(); i++)
    {
      if (s[i] == '.')
        decimal = i;
    }
    std::vector<uint8_t> message(s.begin(), s.begin() + decimal + 3);

    //Add header and footer
    message.insert(message.begin(), ' '); //message starts with 1 space
    message.push_back('\r');
    message.push_back('\n');

    //Apply multiplex wrapper; MID of 143 for SVS
    message = SPRINTNAV_MULTIPLEX(message, 143);

    //send the multiplexed nmea message to the SprintNav
    tcp.write(message);
  }

  //--------------------------------------------------------------------------
  // Name:        read_handler
  // Description: Handler for receiving TCP messages from SprintNav
  // Arguments:   - data: vector of bytes read
  //--------------------------------------------------------------------------
  void read_handler(std::vector<uint8_t> data)
  {
    avl::append(packet, data);
    //remove header and footer
    std::vector<uint8_t> message{SPRINTNAV_UNWRAP(packet, early_term)};

    //if SPRINTNAV_UNWRAP exited early, need to find next DLE/ETX pair
    if(early_term)
    {
      early_term = false;
      return;
    }
    if( !CHECKSUM_CHECK(message) )
    {
        log_warning("Checksum mismatch, skipping data");
        packet.clear();
        return;
    }

    // Get Source Id xx (2bits Source ID) (2bits Sensor ID)xx
    // uint16_t sourceId = (message[0] & 0x00) >> 4;

    // Get Sensor Id
    // uint16_t sensorId = (message[0] & 0x3C) >> 2;

    // Get Message ID
    uint16_t MID = ((message[0] & 0x03) << 8) | (message[1]);

    //Remove first byte so that iterators align with SprintNav command reference
    message.erase(message.begin());

    try
    {
      switch (MID)
      {
      case 90:
        raw_data_handler("ALARM");
        break;
      case 92:
        raw_data_handler("TXT");
        break;
      case 140:
        SPRINTNAV_PD4(message, &dvl);
        pd4_pub_handler();
        break;
      case 172:
        SPRINTNAV_OBST(message, &gps_obst);
        gps_obst_pub_handler();
        break;
      case 176:
        SPRINTNAV_OBST(message, &pdepth_obst);
        break;
      case 177:
        SPRINTNAV_OBST(message, &svs_obst);
        svs_obst_pub_handler();
        break;
      case 208:
        SPRINTNAV_TMS(message, &status);
        tms_pub_handler();
        break;
      case 217:
        SPRINTNAV_BIST(message, &status);
        bist_pub_handler();
        break;
      case 227:
        raw_data_handler("ASONDV");
        break;
      case 232:
        SPRINTNAV_LNAVUTC(message, &nav);
        lnav_pub_handler();
        raw_data_handler("LNAVUTC");
        break;
      case 235:
        SPRINTNAV_OBST(message, &dvl_obst);
        break;
      case 248:
        SPRINTNAV_PWRSTAT(message, &pwrstat);
        pwrstat_pub_handler();
        break;

      default:
        log_warning("Unrecognized message type: %u", MID);
        break;
      }
    }
    catch (const std::exception &ex)
    {
      log_warning("Message parse error");
    }
    packet.clear();
  }

  //--------------------------------------------------------------------------
  // Name:        LNAV_pub_handler
  // Description: Handler for publishing LNAV messages from SprintNav
  //--------------------------------------------------------------------------
  void lnav_pub_handler()
  {
    static size_t lnav_n{config.output_rate}; //logging at slower frequency
    if (lnav_n >= config.output_rate)
    {
      log_data("[LNAV] %.5f %.7f %.7f %.2f %.2f %.2f %.2f %.2f",
               nav.time, nav.latitude, nav.longitude, nav.depth, nav.altitude,
               nav.roll, nav.pitch, nav.heading);
      log_data("[LNAV_Velocity] %.3f %.3f %.3f", nav.vX, nav.vY,
               nav.vZ);
      log_data("[LNAV_Speed] %.3f %.3f %.3f", nav.speed, nav.course,
               nav.gamma);
      log_data("[LNAV_Rotation] %.3f %.3f %.3f", nav.wFwd, nav.wStbd,
               nav.wDwn);
      log_data("[LNAV_Acceleration] %.3f %.3f %.3f", nav.aFwd, nav.aStbd,
               nav.aDwn);
      log_data("[LNAV_Horiz_Error] %.3f %.3f %.3f", nav.posMajor,
               nav.posMinor, nav.dirPMajor);
      log_data("[LNAV_STD] %.3f %.3f %.3f %.3f %.3f", nav.stdDepth,
               nav.stdLevN, nav.stdLevE, nav.stdHeading, nav.velDown);
      log_data("[LNAV_Vel_Error] %.3f %.3f %.3f", nav.velMajor,
               nav.velMinor, nav.dirVMajor);
      log_data("[LNAV_Status] %u", nav.status.status);
      log_data("[LNAV_Status_String] %s", nav.status.message.c_str());

      MagBearingMsg course_msg{};
      course_msg.mag = nav.speed;
      course_msg.bearing = nav.course;
      course_pub.publish(course_msg);

      lnav_n = 0; //reset logging rate
    }
    // Publish this whenever it's true since it's only updated for one LNAV msg
    if (nav.status.alt)
        log_data("[LNAV_Bathy] %.1f", (nav.altitude + nav.depth) );
    if (nav.status.pos && nav.status.orientation)
    {
      AhrsMsg ahrs_msg;
      ahrs_msg.theta.x = nav.roll;
      ahrs_msg.theta.y = nav.pitch;
      ahrs_msg.theta.z = nav.heading;
      ahrs_msg.w.x = nav.wFwd;
      ahrs_msg.w.y = nav.wStbd;
      ahrs_msg.w.z = nav.wDwn;
      // Not publishing magnetic flux due to unavailable on SprintNav
      ahrs_pub.publish(ahrs_msg);

      std_msgs::Float64 depth_msg;
      depth_msg.data = nav.depth;
      depth_pub.publish(depth_msg);

      std_msgs::Float64 height_msg;
      height_msg.data = nav.altitude;
      height_pub.publish(height_msg);

      ImuMsg imu_msg;
      imu_msg.angular_velocity.x = nav.wFwd;
      imu_msg.angular_velocity.y = nav.wStbd;
      imu_msg.angular_velocity.z = nav.wDwn;
      imu_msg.linear_acceleration.x = nav.aFwd;
      imu_msg.linear_acceleration.y = nav.aStbd;
      imu_msg.linear_acceleration.z = nav.aDwn;
      // Not publishing temperature due to unavailable on SprintNav
      // Not publishing valid flag, unsure of implementation yet.
      imu_pub.publish(imu_msg);

      NavigationMsg inertial_nav_msg;
      inertial_nav_msg.roll = nav.roll;
      inertial_nav_msg.pitch = nav.pitch;
      inertial_nav_msg.yaw = nav.heading;

      inertial_nav_msg.vn = nav.vX;
      inertial_nav_msg.ve = nav.vY;
      inertial_nav_msg.vd = nav.vZ;

      inertial_nav_msg.lat = nav.latitude;
      inertial_nav_msg.lon = nav.longitude;
      inertial_nav_msg.alt = nav.altitude;
      inertial_nav_pub.publish(inertial_nav_msg);
    }
    else
	log_debug("Invalid position or orientation");
    lnav_n += config.log_rate;
  }

  //--------------------------------------------------------------------------
  // Name:        gps_obst_pub_handler
  // Description: Handler for publishing gps obst messages from SprintNav
  //--------------------------------------------------------------------------
  void gps_obst_pub_handler()
  {
    log_data("[GPS_OBST] %.6f %X %.3f %.6f", gps_obst.time, gps_obst.rejection,
        gps_obst.mahad, SPRINTNAV_UINT48_TIME(gps_obst.specific, 12));
  }

  //--------------------------------------------------------------------------
  // Name:        svs_obst_pub_handler
  // Description: Handler for publishing svs obst messages from SprintNav
  //--------------------------------------------------------------------------
  void svs_obst_pub_handler()
  {
    log_data("[SVS_OBST] %.6f %X %.3f %.6f", svs_obst.time, svs_obst.rejection,
        svs_obst.mahad, SPRINTNAV_UINT48_TIME(svs_obst.specific, 12));
  }

  //--------------------------------------------------------------------------
  // Name:        pd4_pub_handler
  // Description: Handler for publishing pd4 messages from SprintNav
  //--------------------------------------------------------------------------
  void pd4_pub_handler()
  {
    log_data("[DVL] %X %d %d %d %.2f", dvl.config, dvl.v.valid,
             dvl.current.valid, dvl.sound_speed, dvl.temp);
    log_data("[DVL_Range] %.2f %.2f %.2f %.2f", dvl.beam1.range,
             dvl.beam2.range, dvl.beam3.range, dvl.beam4.range);
    log_data("[DVL_Beam1] %d %d %d", dvl.beam1.correlation,
             dvl.beam1.echo, dvl.beam1.current_correlation);
    // If DVL velocity is valid, publish. Otherwise, publish inertial velocity
    if (dvl.v.valid)
    {
      log_data("[DVL_Vel] %.3f %.3f %.3f %.3f", dvl.v.x, dvl.v.y,
               dvl.v.z, dvl.v.err);
      geometry_msgs::Vector3 velocity_msg; //TODO validate coordinate frame
      velocity_msg.x = dvl.v.x;
      velocity_msg.y = dvl.v.y;
      velocity_msg.z = dvl.v.z;
      velocity_pub.publish(velocity_msg);
    }
    else
    {
      geometry_msgs::Vector3 velocity_msg;
      velocity_msg.x = nav.vX;
      velocity_msg.y = nav.vY;
      velocity_msg.z = nav.vZ;
      velocity_pub.publish(velocity_msg);
    }

    if (dvl.current.valid)
    {
      log_data("[DVL_WVel] %.3f %.3f %.3f %.3f", dvl.current.x,
               dvl.current.y, dvl.current.z, dvl.current.err);
    }
  }

  //--------------------------------------------------------------------------
  // Name:        tms_pub_handler
  // Description: Handler for publishing tms messages from SprintNav
  //--------------------------------------------------------------------------
  void tms_pub_handler()
  {
    log_data("[TMS_TIME] %.6f %.6f %.6f %.6f", status.tms.utc,
             status.tms.sys_time, status.tms.time_since_update, status.tms.sd);
    log_data("[TMS_SOURCE] %d", status.tms.source);
    log_data("[TMS_PPS] %d", status.tms.pps_rising);
    log_data("[TMS_COUNT] %d %d %d %d %d %d", status.tms.zda_count,
             status.tms.pps_count, status.tms.zda_rej_count,
             status.tms.pps_rej_count, status.tms.zda_pps_proc_count,
             status.tms.filter_reset_count);
  }

  //--------------------------------------------------------------------------
  // Name:        pwrstat_pub_handler
  // Description: Handler for publishing pwrstat messages from SprintNav
  //--------------------------------------------------------------------------
  void pwrstat_pub_handler()
  {
    log_data("[PWRSTAT] %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %s %s",
             pwrstat.ext_pwr, pwrstat.fifteen_v, pwrstat.fifteen_v_i, pwrstat.five_v,
             pwrstat.five_v_i, pwrstat.neg_fifteen_v, pwrstat.battery_v,
             pwrstat.battery_temp, pwrstat.status.c_str(),
             pwrstat.pic_charger_stat.c_str());
  }

  //--------------------------------------------------------------------------
  // Name:        bist_pub_handler
  // Description: Handler for publishing bist messages from SprintNav
  //--------------------------------------------------------------------------
  void bist_pub_handler()
  {
    // Reset tcp_timeout
    tcp_timeout.stop();
    tcp_timeout.start();
    log_data("[BIST] %.6f %s 0x%llx 0x%llx 0x%llx 0x%llx "
             "0x%llx",
             status.bist.sys_time, status.bist.fw.c_str(), status.bist.imu,
             status.bist.comms, status.bist.cca, status.bist.ains, status.bist.ahrs);
  }

  //--------------------------------------------------------------------------
  // Name:        raw_data_handler
  // Description: Handler for publishing raw SprintNav data
  //--------------------------------------------------------------------------
  void raw_data_handler(std::string mid)
  {
    std::string data;
    for (size_t i = 0; i < (packet.size() - 1); i++)
      data += std::to_string(packet[i]) + ' ';
    data += packet.back();
    // TO-DO determine if any raw_data_handler can be plotted in a graph
    // log_data('[' + mid + "] " + data);
  }

  //--------------------------------------------------------------------------
  // Name:        tcp_connect
  // Description: Connect to the tcp device. Used for startup or loss of comms
  // Arguments:   - event: ROS timer event structure
  //--------------------------------------------------------------------------
  void tcp_connect(const ros::TimerEvent &event)
  {
    try
    {
      std::string tcp_address = get_param<std::string>("~tcp/address");
      int tcp_port = get_param<int>("~tcp/port");
      log_info("Attempting to connect to " + tcp_address + ":" + std::to_string(tcp_port));
      // Open the TCP port
      tcp.connect(tcp_address, tcp_port);

      // Set the match condition to a newline for data packets
      tcp.set_match(Match({DLE, ETX}, &SprintnavInsNode::read_handler,
                          this));
      log_info("TCP Connection successful");
    }
    catch (const std::exception &ex)
    {
      log_warning("TCP Connection failed");
      // Disable publishers and subscribers if disconnected
      nmea_sub.shutdown();
      SV_sub.shutdown();
      zda_sub.shutdown();
      command_sub.shutdown();
      return;
      //TODO probably try to close the port (if it's already open)
    }

    // Set up subscribers
    gps_sub = node_handle->subscribe("device/gps", 1,
        &SprintnavInsNode::gps_callback, this);
    nmea_sub = node_handle->subscribe("device/nmea", 1,
    	&SprintnavInsNode::nmea_callback, this);
    SV_sub = node_handle->subscribe("device/sound_velocity", 1,
                                    &SprintnavInsNode::SV_callback, this);
    zda_sub = node_handle->subscribe("device/zda", 1,
                                     &SprintnavInsNode::zda_callback, this);
    command_sub = node_handle->subscribe("dive/sprintnav_command", 1,
                                         &SprintnavInsNode::command_callback, this);

    // Configure data output and logging rates
    config.output_rate = (get_param<int>("~output_rate"));
    config.log_rate = (get_param<int>("~log_rate"));
    std::string rate_string = "OP 4000 NET TCP MSG + LNAVUTC " +
                              std::to_string(config.output_rate) + " SRC 1";
    tcp.write(SPRINTNAV_COMMAND(rate_string));
    std::string dvl_string = "PORT 4 PWRPASS 1";
    tcp.write(SPRINTNAV_COMMAND(dvl_string));
  }

  //--------------------------------------------------------------------------
  // Name:        get_device_parameters
  // Description: Called when a DEVICE packet is requested from the node or
  //              when a DEVICE packet is needed to be published to the FSD
  //              or BSD interface. This function should be implemented by the
  //              device child class to create and return a parameter list.
  // Returns:     Parameter list containing parameters to be added to the
  //              device packet that will be transmitted.
  //--------------------------------------------------------------------------
  avl::ParameterList get_device_parameters()
  {
    log_debug("Get params");
    avl::ParameterList params;
/*
    params.add(Parameter("UTC TIME", status.tms.utc));
    params.add(Parameter("SYS TIME", status.bist.sys_time));
    params.add(Parameter("BIST FW", status.bist.fw.c_str()));
    params.add(Parameter("BIST IMU", BIST_TOSTRING(status.bist.imu, 0)));
    params.add(Parameter("BIST COMMS", BIST_TOSTRING(status.bist.comms, 1)));
    params.add(Parameter("BIST CCA", BIST_TOSTRING(status.bist.cca, 2)));
    params.add(Parameter("BIST AINS", BIST_TOSTRING(status.bist.ains, 3)));
    params.add(Parameter("BIST AHRS", BIST_TOSTRING(status.bist.ahrs, 4)));
*/    params.add(Parameter("LATITUDE", nav.latitude));
    params.add(Parameter("LONGITUDE", nav.longitude));
    params.add(Parameter("GPS_DIST", gps_dist));
    params.add(Parameter("GPS_REJECT", gps_obst.rejection));
/*    params.add(Parameter("DEPTH", nav.depth));
    params.add(Parameter("ALTITIUDE", nav.altitude));
    params.add(Parameter("ROLL", nav.roll));
    params.add(Parameter("PITCH", nav.pitch));
    params.add(Parameter("HEADING", nav.heading));
    params.add(Parameter("vX", nav.vX));
    params.add(Parameter("vY", nav.vY));
    params.add(Parameter("vZ", nav.vX));
*/    // Add LNAVUTC?
    log_debug("Params gotten");

    return params;
  }

  //--------------------------------------------------------------------------
  // Name:        init
  // Description: Initializes the node. Called when the node is started.
  //--------------------------------------------------------------------------
  void init()
  {
    log_info("initializing sprintnav INS...");

    // Log data headers for lnav TODO evaluate these headers...
    log_data("[LNAV] time lat long depth alt roll pitch heading");
    log_data("[LNAV] sec deg deg meters meters deg deg deg");

    log_data("[LNAV_Rotation] roll_rate pitch_rate yaw_rate");
    log_data("[LNAV_Rotation] deg/s deg/s deg/s");

    log_data("[LNAV_Acceleration] Forward Starboard Down");
    log_data("[LNAV_Acceleration] m/s^2 m/s^2 m/s^2");

    log_data("[LNAV_Horiz_Error] hori_pos_1sigma_err semi-minor_axis "
             "dir_of_semi-major_axis");
    log_data("[LNAV_Horiz_Error] m m deg");

    log_data("[LNAV_STD] lsigma_depth lsigma_north lsigma_east "
             "lsigma_heading lsigma_down_vel");
    log_data("[LNAV_STD] m deg deg deg m/s");

    log_data("[LNAV_Vel_Error] hori_vel_lsigma semi-minor_axis "
             "dir_of_semi-major_axis");
    log_data("[LNAV_Vel_Error] m/s m/s deg");

    log_data("[LNAV_Velocity] v_x v_y v_z");
    log_data("[LNAV_Velocity] m/s m/s m/s");

    log_data("[LNAV_NED_Velocity] v_Horizontal Course v_Down");
    log_data("[LNAV_NED_Velocity] m/s deg m/s");

    log_data("[LNAV_Status] status_bits");
    log_data("[LNAV_Status] flags");

    log_data("[LNAV_Status] status_string");
    log_data("[LNAV_Status] string");

    log_data("[LNAV_Bathy] total_bathy");
    log_data("[LNAV_Bathy] m");

    log_data("[GPS_OBST] system_time_of_first_data_receipt rejection_status mahalanobis_dist time");
    log_data("[GPS_OBST] unix_sec Flags m unix_sec");

    log_data("[SVS_OBST] system_time_of_first_data_receipt rejection_status mahalanobis_dist time");
    log_data("[SVS_OBST] unix_sec Flags m unix_sec");

    log_data("[DVL] config vel_valid current_valid sound_speed water_temp");
    log_data("[DVL] flags bool bool m/s degC");

    log_data("[DVL_Range] beam1_range beam2_range beam3_range"
             " beam4_range");
    log_data("[DVL_Range] m m m m");

    log_data("[DVL_Beam1] beam1_correlation, beam1_echo"
             " beam1_current_correlation");
    log_data("[DVL_Beam1] bool bool bool");

    log_data("[DVL_Vel] v_x v_y v_z v_err");
    log_data("[DVL_Vel] m/s m/s m/s m/s");

    log_data("[DVL_WVel] curr_x curr_y curr_z curr_err");
    log_data("[DVL_WVel] m/s m/s m/s m/s");

    log_data("[TMS_TIME] utc sys_time time_since_update sd");
    log_data("[TMS_TIME] s s s s");

    log_data("[TMS_SOURCE] source");
    log_data("[TMS_SOURCE] enum");

    log_data("[TMS_PPS] pps_rising");
    log_data("[TMS_PPS] bool");

    log_data("[TMS_COUNT] zda_count pps_count zda_rej_count pps_rej_count"
             " zda_pps_proc_count filter_reset_count");
    log_data("[TMS_COUNT] NA NA NA NA NA NA");

    // Log data headers for PWRSTAT
    log_data("[PWRSTAT] ext_power fifteen_v fifteen_v_i five_v five_v_i"
             " neg_fifteen_v pwrstat.batter_v battery_temp status pic_charger_stat");
    log_data("[PWRSTAT] V V A V A V V degC N/A N/A");

    log_data("[BIST] sys_time fw_version imu comms cca ains ahrs");
    log_data("[BIST] s string flags flags flags flags flags flags");

    log_data("[GPS_TO_NAV] distance bearing");
    log_data("[GPS_TO_NAV] m deg");

    // Set up the publishers
    ahrs_pub = node_handle->advertise<AhrsMsg>("device/ahrs", 1);
    depth_pub = node_handle->advertise<std_msgs::Float64>("device/depth", 1);
    height_pub = node_handle->advertise<std_msgs::Float64>("device/height", 1);
    imu_pub = node_handle->advertise<ImuMsg>("device/imu", 1);
    inertial_nav_pub = node_handle->advertise<NavigationMsg>("nav/inertial_nav", 1);
    velocity_pub = node_handle->advertise<geometry_msgs::Vector3>("device/velocity", 1);
    course_pub = node_handle->advertise<MagBearingMsg>("dive/course", 1);

    // Set up services
    sprintnav_align_srv = node_handle->advertiseService("dive/sprintnav_align",
        &SprintnavInsNode::sprintnav_align_srv_callback, this);

    // Timer to attempt reconnect if TCP connection is lost
    ros::Duration tcp_timeout_duration = ros::Duration(
        get_param<int>("~tcp/reconnect"));
    tcp_timeout = node_handle->createTimer(tcp_timeout_duration,
                                           &SprintnavInsNode::tcp_connect, this);

    // Set device name and default DEVICE packet output rates
    set_device_name("SPRINTNAVINS");
    set_device_packet_output_rate(1.0, INTERFACE_FSD);
//    set_device_packet_output_rate(1.0, INTERFACE_BSD);
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
      tcp.spin_once();
      ros::spinOnce();
      spin_rate.sleep();
    }
  }
};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
  SprintnavInsNode node(argc, argv);
  node.start();
  return 0;
  //TODO add shutdown to node exiting?
}
