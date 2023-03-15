//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node to interface with four 2G Engineering actuators.
//
// Servers:     device/reset_fins (std_srvs/Trigger)
//              /dive/actuator_enable (avl_msgs/ActuatorSrv)
//              /twog/configure (std_srvs/SetBool)
//
// Clients:     None
//
// Publishers:  device/fins_measured (avl_msgs/FinsMsg)
//
// Subscribers: device/actuator_control (avl_msgs/ActuatorControlMsg)
//              device/fins (avl_msgs/FinsMsg)
//              /twog/calibrate (std_msgs/String)
//==============================================================================

// Node base class
#include <avl_devices/device_node.h>

// TCP port class
#include <avl_asio/tcp_socket.h>

// Utility functions
#include <avl_core/util/byte.h>
#include <avl_core/util/math.h>
#include <avl_core/util/vector.h>
#include <avl_core/monitored_subscriber.h>

// Actuator position request service
#include <avl_msgs/ActuatorSrv.h>

// ROS message includes
#include <avl_msgs/ActuatorControlMsg.h>
#include <avl_msgs/FinsMsg.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
using namespace avl_msgs;

// Twog messaging protocol
#include <avl_devices/protocol/twog_actuator.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class TwoGActuatorNode : public DeviceNode
{

public:
  //--------------------------------------------------------------------------
  // Name:        TwoGActuatorNode constructor
  //--------------------------------------------------------------------------
  TwoGActuatorNode(int argc, char **argv) : DeviceNode(argc, argv) {}

private:
  // TCP device instance
  TcpSocket tcp;

  // Service servers
  ros::ServiceServer actuator_server;
  ros::ServiceServer configure_srv;

  // Publisher for measured fin positions
  ros::Publisher fins_measured_pub;

  // Subscribers
  MonitoredSubscriber<FinsMsg> fins_sub;
  ros::Subscriber actuator_control_sub;
  ros::Subscriber calib_sub;

  // Timer for RX and TX comms with the actuators and tcp_timeout
  ros::Timer twog_feedback_timer;
  ros::Duration twog_feedback_duration;
  ros::Timer tcp_timeout;

  //Servers for the reset_fins and disable/enable actuators services
  ros::ServiceServer reset_fins_server;

  // Actuator current status structs, with addresses initialized
  Act fin1{};
  Act fin2{};
  Act fin3{};
  Act fin4{};

  // Vector of pointers to actuator structures for iterative access
  Act *fins[twog_act_count] = {&fin1, &fin2, &fin3, &fin4};

  // Flags indicating whether motor control is enabled.
  // Safety is meant to prevent commands from reaching motors. State is meant
  // to prevent motors from accepting/reacting to commands
  bool actuator_safety_enabled = false;
  bool actuator_state_enabled = false;

  // Flag indicating a new setpoint was received
  bool new_setpoint = false;

  // Variable to track which fin was last measured
  size_t last_fin = 0;

  // Fin angles received on the fins topic. These will be updated by
  // 45deg rotation for use on Xform fins
  double port_angle;
  double starboard_angle;
  double top_angle;
  double bottom_angle;

  //device class variables
  double fin_position_top_device;
  double fin_position_starboard_device;
  double fin_position_bottom_device;
  double fin_position_port_device;

  uint8_t fault_address_device;
  uint8_t fault_position_device;

  bool response_wait{false};

private:
  //--------------------------------------------------------------------------
  // Name:        calibrate_callback
  // Description: Called when a calibration is requested
  // Arguments:   - msg: data from ROS TOPIC
  //--------------------------------------------------------------------------
  void calibrate_callback(const std_msgs::String::ConstPtr &msg)
  {
    // Recalibrate current fin position to 0 deg for all fins.
    std::vector<uint8_t> calibrate = {0x43};
    calibrate.push_back(0x00);
    calibrate.push_back(0x00);
    calibrate.push_back(0x00);
    calibrate.push_back(0x00);
    tcp_interruption(TWOG_WRAP(0, calibrate));
    // Save to EEPROM.
    tcp_interruption(TWOG_WRAP(0, SAVE_EE));
  }

  //--------------------------------------------------------------------------
  // Name:        configure_srv_callback
  // Description: Called when a configuration is requested
  // Arguments:   - request.bool: true to configure, false to just query config
  //              - response.bool: true if configuration chaged per request
  //              - response.message: unused
  //--------------------------------------------------------------------------
  bool configure_srv_callback(std_srvs::SetBool::Request &request,
    std_srvs::SetBool::Response &response)
  {
    for(size_t i{0}; i < twog_act_count; i++)
    {
        // Request Configuration Parameters
        tcp_interruption(TWOG_WRAP(i+1, REQ_AUTO)); // Auto-Info Config
        tcp_interruption(TWOG_WRAP(i+1, REQ_FS)); // Failsafe Config
        tcp_interruption(TWOG_WRAP(i+1, REQ_SAMP)); // Position Sampling Config
        tcp_interruption(TWOG_WRAP(i+1, REQ_CFG1)); // System Config 1
        tcp_interruption(TWOG_WRAP(i+1, REQ_CFG2)); // System Config 2
        tcp_interruption(TWOG_WRAP(i+1, REQ_CFG3)); // System Config 3
        tcp_interruption(TWOG_WRAP(i+1, REQ_GS)); // Gain Scheduling Config
        tcp_interruption(TWOG_WRAP(i+1, REQ_STD)); // Stall Detection Config
        tcp_interruption(TWOG_WRAP(i+1, REQ_MP)); // Motion Profile Config
        if(request.data)
        {
            // Obtain and Set Configuration Vectors
            std::vector<std::vector<uint8_t>> configure = TWOG_CONFIGURE();
            for(size_t j{0}; j < configure.size(); j++)
                tcp_interruption(TWOG_WRAP(i+1, configure[j]));
            // Save to EEPROM
            tcp_interruption(TWOG_WRAP(i+1, SAVE_EE));
        }
    }
    response.success = true;
    return true;
  }

  //--------------------------------------------------------------------------
  // Name:        actuator_srv_callback
  // Description: Called when the actuator service is requested.
  // Arguments:   - request: request received on the service
  //              - res: response to the service request
  // Returns:     True if service succeeded, false if it failed and was not
  //              responded to.
  //--------------------------------------------------------------------------
  bool actuator_srv_callback(ActuatorSrv::Request &request,
    ActuatorSrv::Response &response)
  {
    log_info("Actuator %s service called for fin %u.%.1f deg, rate check %s",
        request.state_enable ? "enable" :  "disable", request.addr, request.pos,
        request.rate_check ? "enabled" : "disabled");

    actuator_state_enabled = request.state_enable;
    for (size_t i = 0; i < twog_act_count; i++)
      tcp_interruption(TWOG_ENABLE(fins[i]->address, actuator_state_enabled));

    fins_sub.enable_message_rate_check(request.rate_check);

    if ((request.addr > 0) && (request.addr <= twog_act_count))
      fins[request.addr - 1]->pos.command = request.pos;
    else if (request.addr == 0)
    {
      for (size_t i = 0; i < twog_act_count; i++)
      {
        fins[i]->pos.command = request.pos;
      }
    }
    else
    {
      log_warning("Invalid fin requested");
      response.success = false;
      return false;
    }

    response.success = true;
    return true;
  }

  //--------------------------------------------------------------------------
  // Name:        fins_publish
  // Description: Changes fin measurements from Xform into FinsMsg and
  //              publishes.
  // Arguments:   none
  //--------------------------------------------------------------------------
  void fins_publish()
  {
    FinsMsg fins_measured;
    fins_measured.top = fin1.pos.measured;
    fins_measured.starboard = fin2.pos.measured;
    fins_measured.bottom = fin3.pos.measured;
    fins_measured.port = fin4.pos.measured;
    fins_measured_pub.publish(fins_measured);

    fin_position_top_device = fin1.pos.measured;
    fin_position_starboard_device = fin2.pos.measured;
    fin_position_bottom_device = fin3.pos.measured;
    fin_position_port_device = fin4.pos.measured;
  }

  //--------------------------------------------------------------------------
  // Name:        fins_transform
  // Description: Changes fin measurements from FinsMsg into Xform angles
  // Arguments:   - message: Message received on the topic.
  //--------------------------------------------------------------------------
  void fins_transform(const FinsMsg &message)
  {
    //Check if message contains NaNs, which indicate no update is requested
    if (!std::isnan(message.top))
      top_angle = message.top;
    if (!std::isnan(message.starboard))
      starboard_angle = message.starboard;
    if (!std::isnan(message.bottom))
      bottom_angle = message.bottom;
    if (!std::isnan(message.port))
      port_angle = message.port;

    // Convert cruciform fins to Xform
    //TODO use sum of forces and moments to optimize this conversion
    /*      float rudder = (bottom_angle) - (top_angle);
      float elevator = (port_angle) - (starboard_angle);
      float roll = (top_angle + starboard_angle + bottom_angle
        + port_angle);
      Eigen::Matrix<double, 3, 3> C;
      double a = 0.7071;
      C <<    -a,  2*a,    a,
              -a, -2*a,    a,
             1.0,  0.0,  1.0;
      Eigen::Matrix<double, 3, 1> B;
      B << rudder, elevator, roll;
      Eigen::Matrix<double, 3, 1> XFIN;
      XFIN = C.inverse()*B;
      double msg_fin[4];
      msg_fin[0] = XFIN(0);
      msg_fin[1] = XFIN(1);
      msg_fin[2] = XFIN(2);
      msg_fin[3] = -XFIN(1);
      log_debug("Rudder: %.4f   Elevator: %.4f   Roll: %.4f", rudder, elevator, roll);
*/
    float msg_fin[twog_act_count];
    msg_fin[0] = (1 * top_angle) + (1 * starboard_angle);
    msg_fin[1] = (1 * bottom_angle) + (1 * starboard_angle);
    msg_fin[2] = (1 * bottom_angle) + (1 * port_angle);
    msg_fin[3] = (1 * top_angle) + (1 * port_angle);
    // Send angles over 180deg as negative; check if fin command has changed
    for (size_t j = 0; j < twog_act_count; j++)
    {
      if (msg_fin[j] > 180.0)
        msg_fin[j] -= 360.0;
      if (abs(msg_fin[j] - fins[j]->pos.command) > 0.2)
        fins[j]->pos.command = avl::clamp(msg_fin[j], -fins[j]->pos.max,
                                          fins[j]->pos.max);
      //log_debug("FIN%d: %.4f", j+1, fins[j]->pos.command);
    }
  }

  //--------------------------------------------------------------------------
  // Name:        fins_msg_callback
  // Description: Called when a fins message is received on the fins topic.
  //              Sets fin angle and publishes the corresponding PWM message
  //              if any fin angles have changed.
  // Arguments:   - message: Message received on the topic.
  //--------------------------------------------------------------------------
  void fins_msg_callback(const FinsMsg &message)
  {
    new_setpoint = true;
    if (actuator_safety_enabled)
      fins_transform(message);
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
    for (size_t i = 0; i < twog_act_count; i++)
      fins[i]->pos.command = fins[i]->pos.home;
  }

  //--------------------------------------------------------------------------
  // Name:        fault_callback
  // Description: Called when a fault is detected by a monitored subscriber.
  // Arguments:   - fault: Fault event structure.
  //--------------------------------------------------------------------------
  void fault_callback(const avl::SubscriberFault &fault)
  {
    if (actuator_safety_enabled)
    {
      if (new_setpoint)
      {
        log_warning("fin angle input timed out, resetting fins");
        new_setpoint = false;
      }
      reset_fins();
      fins_sub.reset();
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
  bool reset_fins_srv_callback(std_srvs::Trigger::Request &req,
                               std_srvs::Trigger::Response &res)
  {
    log_info("resetting fins");
    reset_fins();

    res.success = true; //TODO update these to check for ACK
    res.message = "success";

    return true;
  }

  //--------------------------------------------------------------------------
  // Name:        actuator_control_msg_callback
  // Description: Called when a message is received on the topic.
  // Arguments:   - message: message received on the topic
  //--------------------------------------------------------------------------
  void actuator_control_msg_callback(const ActuatorControlMsg
                                         message)
  {
    log_info("Actuator %s request received", message.enable ? "enable" : "disable");
    actuator_safety_enabled = message.enable;
    fins_sub.reset();
  }

  //--------------------------------------------------------------------------
  // Name:        feedback_timer_callback
  // Description: Called when feedback timer reach. Send the request packet.
  // Arguments:   - event: ROS timer event structure
  //--------------------------------------------------------------------------
  void twog_feedback_timer_callback(const ros::TimerEvent &event)
  {
    // Check if message has been received since last query, to prevent
    // serial collisions
    static size_t wait_count{0};
    if (response_wait)
    {
      // After consecutive skips, assume response was missed
      if (wait_count >= 5)
      {
        response_wait = false;
        wait_count = 0;
      }
      else
        wait_count++;
    return;
    }
    static size_t timer_count{0}; //count to iterate through all channels
    //iterate through each query first, then each channel
    size_t channel = (timer_count) / twog_query_count;
    size_t query_no = timer_count % twog_query_count;

    //construct message for actuator
    if (tcp.is_connected())
      tcp.write(TWOG_QUERY(fins[channel], query_no));
    else
      log_warning("Attempting to write feedback, but port is disconnected");

    timer_count++;
    if (timer_count >= (twog_query_count * twog_act_count))
      timer_count = 0;

    response_wait = true;
    }

  //--------------------------------------------------------------------------
  // Name:        match_check
  // Description: Handles receipt of end delimiter ("]") and determines
  //              whether packet is complete, incomplete, or corrupted.
  // Arguments:   - data: vector of bytes read
  //--------------------------------------------------------------------------
  void match_check(std::vector<uint8_t> data)
  {
    // define a buffer variable to be kept in memory between match calls
    static std::vector<uint8_t> packet{};
    avl::append(packet, data);

    // the number of bytes in a complete packet is payload length + 5
    size_t reported_size = packet[2] + 5;
    if (packet[0] == start_delim)
    {
      if (packet.size() == reported_size)
      {
        packet.erase(packet.begin() + 2); // remove length byte
        read_handler(packet);
        packet.clear(); //clear packet buffer once done processing packet
      }
      else if (packet.size() > reported_size)
      {
        log_warning("Reported payload size mismatch: %dr %dm", reported_size,
                    packet.size());
        for (size_t i = 0; i < packet.size(); i++)
          log_warning("Byte #%d: 0x%x", i, packet[i]);
        packet.clear(); //clear packet buffer because we don't expect this
      }                 //and need to reset
    }
    else //all packets begin with start delimiter, anything else is corrupt
    {
      log_warning("Unrecognized message format");
      for (size_t i = 0; i < packet.size(); i++)
        log_warning("Byte #%d: 0x%x", i, packet[i]);
      packet.clear();
    }
  }

  //--------------------------------------------------------------------------
  // Name:        read_handler
  // Description: Handler for parsing complete twog packets
  // Arguments:   - packet: complete twog message packet
  //--------------------------------------------------------------------------
  void read_handler(std::vector<uint8_t> packet)
  {
    response_wait = false;
    // remove start delimiter, so iterators match command reference position
    packet.erase(packet.begin());

    // identify which actuator was the source of the message
    size_t address = packet[0];
    packet.erase(packet.begin());

    // parse message
    size_t type = packet[0];
    try
    {
      switch (type)
      {
      case 0x41:
        TWOG_ACK(packet, fins[address - 1]);
        break;
      case 0x46:
        if (TWOG_FAULT(packet, fins[address - 1]))
          fault_handler(fins[address - 1]);
        break;
      case 0x48:
        TWOG_VEL(packet, fins[address - 1]);
        vel_pub_handler(fins[address - 1]);
        break;
      case 0x50:
        TWOG_SYS_STAT(packet, fins[address - 1]);
        sys_stat_pub_handler(fins[address - 1]);
        // Reset tcp_timeout
        tcp_timeout.stop();
        tcp_timeout.start();
        break;
      case 0x3F:
        TWOG_FW(packet, fins[address - 1]);
        log_data("[FIN" + std::to_string(address) + "_FW] " + (fins[address - 1]->fw));
        break;
      case 0xA2:
        TWOG_SYS_STAT2(packet, fins[address - 1]);
        sys_stat2_pub_handler(fins[address - 1]);
        break;
      case 0xBC:
        TWOG_NAME(packet, fins[address - 1]);
        log_data("[FIN" + std::to_string(address) + "_SN] " + (fins[address - 1]->sn));
        break;
      case 0xB2:
      case 0x92:
      case 0x45:
      case 0x4D:
      case 0x44:
      case 0xA6:
      case 0xAB:
      case 0xA8:
      case 0x9A:
      case 0xA4:
      {
        log_info("Configuration packet 0x%x received from SN%u", type, address);
        std::string config_string = "{ ";
        for(size_t i{0}; i < packet.size(); i++)
            config_string += std::to_string(packet.at(i)) + ", ";
        config_string.pop_back();
        config_string.pop_back();
        config_string += " }";
        log_info(config_string);
        break;
      }

      default:
        log_warning("Unrecognized message type: %x", type);
        response_wait = true;
        break;
      }
    }
    catch (const std::exception &ex)
    {
      log_warning("Message parse error");
      response_wait = true;
    }
  }

  //--------------------------------------------------------------------------
  // Name:        sys_stat_pub_handler
  // Description: Handles publishing of system status information.
  //--------------------------------------------------------------------------
  void sys_stat_pub_handler(Act *act)
  {
    log_data("[POS_FIN%d] %.3f %.3f", act->address, act->pos.measured,
             act->pos.command);
    log_data("[STAT_FIN%d] %d %d %d %.3f %.3f", act->address,
             act->enabled, act->temp1, act->temp2, act->voltage, act->current);
    last_fin = act->address;
    fins_publish();
  }

  //--------------------------------------------------------------------------
  // Name:        sys_stat2_pub_handler
  // Description: Handles publishing of system status 2 information.
  //--------------------------------------------------------------------------
  void sys_stat2_pub_handler(Act *act)
  {
    //Motor match is not worth publishing; board current already logged
  }

  //--------------------------------------------------------------------------
  // Name:        vel_pub_handler
  // Description: Handles publishing of velocity information.
  //--------------------------------------------------------------------------
  void vel_pub_handler(Act *act)
  {
    log_data("[VEL_FIN%d] %.3f", act->address, act->vel);
  }

  //--------------------------------------------------------------------------
  // Name:        fault_handler
  // Description: Handles publishing of hardware fault information.
  //--------------------------------------------------------------------------
  void fault_handler(Act *act)
  {
    if (act->fault.motor)
      log_warning("[FAULT_FIN%d] Motor Fault Reported 0x%x", act->address,
                  act->fault.motor);
    else if (act->fault.pos)
    {
      log_warning("[FAULT_FIN%d] Position Sensor Fault Reported 0x%x", act->address,
                  act->fault.pos);
      fault_address_device = act->address;
      fault_position_device = act->fault.pos;
    }
    else if (act->fault.temp)
      log_warning("[FAULT_FIN%d] Temperature Fault Reported 0x%x", act->address,
                  act->fault.temp);
    else if (act->fault.comms)
      log_warning("[FAULT_FIN%d] Comms Fault Reported 0x%x", act->address,
                  act->fault.comms);
  }

  //--------------------------------------------------------------------------
  // Name:        tcp_interruption
  // Description: Handles writing a tcp message outside the feedback timer
  //              scope. Will pause feedback for ~2x feedback_duration.
  // Description: Handles writing a tcp message outside the feedback timer
  //              scope. Will pause feedback for ~2x feedback_duration
  // Arguments:   - message: formatted message ready for transmission
  //--------------------------------------------------------------------------
  void tcp_interruption(std::vector<uint8_t> message)
  {
    if (tcp.is_connected())
    {
      //stop the feedback timer to prevent serial line collisions
      twog_feedback_timer.stop();
      twog_feedback_duration.sleep();
      //process any data received on the serial line
      for (int i = 0; i < 100; i++)
      {
        tcp.spin_once();
        ros::spinOnce();
      }

      tcp.write(message);

      twog_feedback_duration.sleep();
      //process any data received on the serial line
      for (int i = 0; i < 100; i++)
      {
        tcp.spin_once();
        ros::spinOnce();
      }
      twog_feedback_timer.start();
    }
    else
      log_warning("Attempting to write message, but port is disconnected");
  }

  //--------------------------------------------------------------------------
  // Name:        device_info
  // Description: Queries each actuator for serial no and firmware info
  //--------------------------------------------------------------------------
  void device_info()
  {
    //Read FW version string and Actuator name
    for (size_t i = 0; i < twog_act_count; i++)
    {
      // Query FW
      tcp_interruption(TWOG_WRAP((i + 1), {0x3F, 0, 0, 0, 0}));
      // Query name
      tcp_interruption(TWOG_WRAP((i + 1), {0xBD, 0x00}));
    }
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
      response_wait = false;
      std::string tcp_address = get_param<std::string>("~tcp/address");
      int tcp_port = get_param<int>("~tcp/port");
      log_info("Attempting to connect to " + tcp_address + ":" + std::to_string(tcp_port));
      // Open the TCP port
      tcp.connect(tcp_address, tcp_port);

      // Set the match condition to a newline for data packets
      tcp.set_match(Match("]", &TwoGActuatorNode::match_check, this));
      log_info("TCP Connection successful");
      twog_feedback_timer.start();
    }
    catch (const std::exception &ex)
    {
      response_wait = true;
      log_warning("TCP Connection failed");
      twog_feedback_timer.stop();
      // Shutdown subscriber to re-subscribe on connection
      actuator_control_sub.shutdown();
      calib_sub.shutdown();
      fins_sub.shutdown();
      return;
      //TODO probably try to close the port (if it's already open)
    }
    // Collect actuator FW and Serial info
    device_info();

    // Set up the subscribers for actuator control messages
    actuator_control_sub = node_handle->subscribe("device/actuator_control",
                                                  1, &TwoGActuatorNode::actuator_control_msg_callback, this);
    calib_sub = node_handle->subscribe("twog/calibrate", 1,
                                       &TwoGActuatorNode::calibrate_callback, this);
    fins_sub.subscribe("device/fins", 8,
                       &TwoGActuatorNode::fins_msg_callback,
                       &TwoGActuatorNode::fault_callback, this);
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

    avl::ParameterList params;
    params.add(Parameter("TOP", fin_position_top_device));
    params.add(Parameter("STARD", fin_position_starboard_device));
    params.add(Parameter("BOTTM", fin_position_bottom_device));
    params.add(Parameter("PORT", fin_position_port_device));
    params.add(Parameter("FAULT_ADDR", fault_address_device));
    params.add(Parameter("FAULT_POS", fault_position_device));
    return params;
  }

  //--------------------------------------------------------------------------
  // Name:        init
  // Description: Initializes the node. Called when the node is started.
  //--------------------------------------------------------------------------
  void init()
  {
    // Initialize fin addresses
    fin1.address = 1;
    fin2.address = 2;
    fin3.address = 3;
    fin4.address = 4;

    for (int i = 1; i < 5; i++)
    {
      log_data("[FIN%d_FW] fw", i);
      log_data("[FIN%d_FW] string", i);

      log_data("[FIN%d_SN] sn", i);
      log_data("[FIN%d_SN] string", i);

      log_data("[VEL_FIN%d] velocity", i);
      log_data("[VEL_FIN%d] deg/s", i);

      log_data("[POS_FIN%d] position_measured position_command", i);
      log_data("[POS_FIN%d] deg deg", i);

      log_data("[STAT_FIN%d] enabled temp1 temp2 voltage current", i);
      log_data("[STAT_FIN%d] bool degC degC V A", i);
    }

    // Configure feedback request timer
    twog_feedback_duration = ros::Duration((get_param<float>
      ("~twog_feedback_duration")) / (twog_query_count * twog_act_count));
    twog_feedback_timer = node_handle->createTimer(twog_feedback_duration,                                     &TwoGActuatorNode::twog_feedback_timer_callback, this);
    twog_feedback_timer.stop();

    // Set up the service servers
    actuator_server = node_handle->advertiseService("dive/actuator_enable",                                   &TwoGActuatorNode::actuator_srv_callback, this);
    configure_srv = node_handle->advertiseService("twog/configure",                                           &TwoGActuatorNode::configure_srv_callback, this);
    reset_fins_server = node_handle->advertiseService("device/reset_fins",                                    &TwoGActuatorNode::reset_fins_srv_callback, this);

    // Set up the fin position publisher
    fins_measured_pub = node_handle->advertise<FinsMsg>("device/fins_measured", 1);

    // Set up subscribers
    fins_sub.set_message_rate(get_param<double>("~fins_input_rate"));
    fins_sub.enable_message_rate_check(true);
    fins_sub.enable_out_of_bounds_check(false);
    fins_sub.enable_repitition_check(false);

    // Configure fin home, max, and min travel positions
    fin1.pos.home = get_param<float>("~top_starboard/home_angle");
    fin1.pos.max = get_param<float>("~top_starboard/max_angle");
    fin1.pos.abs_max = get_param<float>("~top_starboard/abs_max_angle");
    fin2.pos.home = get_param<float>("~bottom_starboard/home_angle");
    fin2.pos.max = get_param<float>("~bottom_starboard/max_angle");
    fin2.pos.abs_max = get_param<float>("~bottom_starboard/abs_max_angle");
    fin3.pos.home = get_param<float>("~bottom_port/home_angle");
    fin3.pos.max = get_param<float>("~bottom_port/max_angle");
    fin3.pos.abs_max = get_param<float>("~bottom_port/abs_max_angle");
    fin4.pos.home = get_param<float>("~top_port/home_angle");
    fin4.pos.max = get_param<float>("~top_port/max_angle");
    fin4.pos.abs_max = get_param<float>("~top_port/abs_max_angle");

    // Initial attempt to connect to TCP port
    const ros::TimerEvent event_empty;
    tcp_connect(event_empty);

    // Timer to attempt reconnect if TCP connection is lost
    ros::Duration tcp_timeout_duration = ros::Duration(
        get_param<int>("~tcp/reconnect"));
    tcp_timeout = node_handle->createTimer(tcp_timeout_duration,
                                           &TwoGActuatorNode::tcp_connect, this);

    // Set device name and default DEVICE packet output rates
    set_device_name("TWOG");
    set_device_packet_output_rate(1.0, INTERFACE_FSD);
    set_device_packet_output_rate(1.0, INTERFACE_BSD);
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

  //--------------------------------------------------------------------------
  // Name:        shutdown
  // Description: Called after the run function when the node is started. Can
  //              be overriden by a derived node class.
  //--------------------------------------------------------------------------
  void shutdown()
  {
    log_debug("resetting fins");
    reset_fins();
    //TODO probably need to spin here to send commands to actuators
  }
};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
  TwoGActuatorNode node(argc, argv);
  node.start();
  return 0;
}
