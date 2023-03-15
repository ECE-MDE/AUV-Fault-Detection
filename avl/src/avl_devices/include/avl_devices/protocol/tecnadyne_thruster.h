//==============================================================================
// Autonomous Vehicle Library
//
// Description: Provides functions to communicate with the Tecnadyne 2050DD
// 		Thruster.
//==============================================================================

#ifndef TEC_THRUST_H
#define TEC_THRUST_H

//==============================================================================
//                            GLOBAL CONSTANT DEFINITIONS
//==============================================================================

constexpr size_t TEC_DEF_LOOP = 100; // Default loop interval in ms
constexpr size_t retry_attempts{10};
constexpr char ACK = 0x06; //Acknowledge
constexpr char DLE = 0x10; //Data link escape
constexpr char ETX = 0x03; //End of text
constexpr char NAK = 0x15; //Negative acknowledge
constexpr char SOH = 0x01; //Start of header (master only)
constexpr char STX = 0x02; //Start of text (slave only)
constexpr char BROADCAST = 0x6F; //Address to broadcast to any thruster
const std::vector<uint8_t> thruster_queries= {0x21, 0x41, 0x32, 0x37};
const std::vector<uint8_t> thruster_header = { DLE , SOH }; //string to 
    //start a message to the thruster.
const std::vector<uint8_t> thruster_footer = { DLE , ETX }; //string to end a
    //message to the thruster.
const std::vector<uint8_t> speed_loop_enable = {0x041, 0x01, 0x00, 0x00, 0x00};
    //command string to enable motor speed loop. Sent after setting the loop.
const std::vector<uint8_t> speed_loop_disable = {0x041, 0x00, 0x00, 0x00, 0x00};
    //command string to enable motor speed loop. Sent after setting the loop.
  
//==============================================================================
//                            THRUSTER STATUS
//==============================================================================

typedef struct Thruster
{
    struct RPM
    {
        float measured;
        float cmd;
        float error;
    } rpm;
  
    struct Temperature
    {
        float controller;
        float heatsink;
        float winding;
    } temp;
  
    struct Settings
    {
        std::string fw_ver;
        std::string fw_date;
    
        size_t pole_pair_count;    
        
        float max_rpm;
    
        uint8_t address;
        uint8_t tx_delay; //communication delay before responding
        uint8_t watchdog; //timeout in s
        uint8_t slew;     //motor slew rate UKNOWN units
        uint8_t baud;
        uint8_t freq;     //PWM frequency in kHz(?)
        uint8_t fwd_lim;  //PWM FWD limit
        uint8_t rev_lim;  //PWM reverse limit
      
        bool ACK;  
        bool analog;
    } settings;
  
    bool speed_loop_en;
    bool fault;
    bool reset;
  
    double t_s; //seconds of runtime in s
      
    uint16_t speed_loop_interval; //ms
  
    float pwm_pct;
    float v; //Voltage in V
    float i; //Current in A 
    float p; //Power in W 
} Thruster;

  


//==============================================================================
//                            FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        TEC_CHECKSUM
// Description: Calculation of a packet checksum.
// Arguments:   - packet: "all bytes header thru footer"
// Returns:     Corresponding checksum in hexidecimal.
//------------------------------------------------------------------------------
uint8_t TEC_CHECKSUM(std::vector<uint8_t> packet, size_t dle_stuffs = 0)
{
    uint8_t checksum{};
    int sum{};

    //Add all bytes
    for (size_t count{ 0 }; count < packet.size(); ++count)
    {
        sum += packet[count];
    }
    //Extra DLE's for byte stuffing don't count towards Checksum
    sum -= (DLE)*dle_stuffs;
    
    //truncate for only last byte
    sum &= 0xFF;
    checksum = static_cast<uint8_t>(sum);
    
    return checksum;
}

//------------------------------------------------------------------------------
// Name:        TEC_WRAP 
// Description: Provides a completed and formatted message to send the motor 
// Arguments:   - packet: desired message to send
//              - STN: address of device to send to
// Returns:     Completed message for writing to motor.
//------------------------------------------------------------------------------
std::vector<uint8_t> TEC_WRAP(std::vector<uint8_t> packet, uint8_t STN)
{
    //DLE stuffing count
    size_t dle_stuffs = 0;
    //search packet for any 'DLE' (0x10) characters for byte stuffing
    for (size_t i = 0; i < packet.size(); i++)
    {
        if ( packet[i] == DLE)
        {
            //Stuff DLE byte
            packet.insert( (packet.begin() + i) , DLE);
            dle_stuffs++;
            i++; //iterate to move to next character
        }
    }
	  
    //Insert the station address
    packet.insert(packet.begin(), STN);
    //Insert the message header
    packet.insert(packet.begin(), thruster_header.begin(), 
        thruster_header.end());
    //Insert the message footer
    avl::append(packet, thruster_footer);
    //Insert the checksum
    packet.push_back( TEC_CHECKSUM(packet, dle_stuffs) );
  
    return packet;

}

//------------------------------------------------------------------------------
// Name:        TEC_CMD
// Description: Provides a completed and formatted message to send the motor 
// Arguments:   - cmd: desired message to send
//              - STN: address of device to send to
// Returns:     Completed message for writing to motor.
//------------------------------------------------------------------------------
std::vector<uint8_t> TEC_CMD(uint8_t cmd, uint8_t STN)
{
    std::vector<uint8_t> payload{};
    payload.push_back(cmd);
    avl::append(payload, {0, 0, 0, 0});
    return TEC_WRAP(payload, STN);
}

//------------------------------------------------------------------------------
// Name:        TEC_UNWRAP 
// Description: Returns a payload from a tecnadyne formatted message  
// Arguments:   - message: formatted message from motor
//              - &early_term: boolean to signal if more data is needed
// Returns:     payload message.
//------------------------------------------------------------------------------
std::vector<uint8_t> TEC_UNWRAP(std::vector<uint8_t> message, bool &early_term)
{
    bool DLEReceived = false;
    bool DLE_STX_found = false;
    bool DLE_ETX_found = false;
    std::vector<uint8_t> payload{};
  
    for (size_t i = 0; i < message.size(); i++)
    {
        uint8_t Databyte = message.at(i);
    
        switch (Databyte)
        {
            case DLE:
                if (DLEReceived == false)
                    //first DLE, wait for the next DLE to determine its use  
                    DLEReceived = true;
                else if (DLE_STX_found)
                // record DLE pairs in the packet, if DLE_STX has been found
                {
                    //DLE DLE found -- copy one DLE to payload buffer
                    DLEReceived = false;
                    payload.push_back(Databyte);
                }
                break;
        
            case STX:
                if (DLEReceived == true)
                {
                    // DLE STX Start of the payload found
                    DLEReceived = false;
                    DLE_STX_found = true;
                }
                else
	        {
                    if (DLE_STX_found)
	                payload.push_back(Databyte);
	        }
                break;
          
            case ETX:
                if (DLEReceived == true)
                {
                    //DLE ETX found
                    DLEReceived = false;
                    if (DLE_STX_found == true)
                        DLE_ETX_found = true;
                    DLE_STX_found = false;
                }
                else
                {
                    if (DLE_STX_found)
	                payload.push_back(Databyte);
                }
                break;
          
            default: //Data
                if (DLEReceived == true)
                    DLEReceived = false;
                if (DLE_STX_found)
                    payload.push_back(Databyte);
                break;
        
        }
    }
    
    if (!DLE_ETX_found)
        early_term = true;
  
    return payload;
}

//------------------------------------------------------------------------------
// Name:        RPM_SETPOINT
// Description: Provides a payload message to set the closed loop control of the
// 		motor
// Arguments:   - rpm: desired motor speed in revolutions per minute
// 		- closed_loop: defaults to true, set to false for open loop mode
// 		- loop_interval: ms, speed control sampling rate. Defaults to 100
// Returns:     Completed payload packet to set the desired RPM.
//------------------------------------------------------------------------------
std::vector<uint8_t> RPM_SETPOINT(Thruster *thruster, bool closed_loop = true, 
    uint8_t loop_interval = TEC_DEF_LOOP) 
{
    std::vector<uint8_t> payload;
    int8_t rev = 1; //Set to -1 if thruster cmd is reverse
  
    //CMD - command ID
    payload.push_back(0x41);// CMD 41 is "write digital data"
    
    //DB1 - Mode byte
    if (closed_loop)
        payload.push_back(0x21);
    else
        payload.push_back(0x20);
  
    //DB2 - Loop time interval
    if (closed_loop)
        payload.push_back( loop_interval/5 ); //count is in 5ms increments
    else
        payload.push_back(0x20);// no explanation for this magic number in 
  			      // Thruster Communication Interface v1.8
  
    //DB3 - Direction
    if (thruster->rpm.cmd > 0)
        payload.push_back(0x00); //"Normal" direction TODO verify this is 
                                 //forward for vehicle
    else
    {
        payload.push_back(0x01); //"Reverse"
        rev = -1; //convert rpm to magnitude
    }
  
    //DB4 - Set point
    if (closed_loop) //TODO check that RPM is below max allowable (835)
    {
        float tach_set = (rev * thruster-> rpm.cmd * loop_interval * 
           thruster->settings.pole_pair_count * 6) / (60 * 1000);
        //Will not evaluate remainder of integer division -safer to always round
        //down
        payload.push_back(static_cast<uint8_t>(tach_set));
    }
    else //Unsure how to handle PWM yet TODO
        payload.push_back(0);
  
    return payload;
}

//------------------------------------------------------------------------------
// Name:        TEC_THERMISTOR
// Description: Returns temperature of thermistor in degC based on Steinhart & 
//              Hart Model. Reference "Thermistor" by AntaresAdroit 
// Arguments:   - count: ADC counts of resistance measurement
// Returns:     Temperature in degC.
//------------------------------------------------------------------------------
float TEC_THERMISTOR(uint16_t count)
{
    double r_25 = 10000; //resistance of thermistor in ohms at 25C
    //Constants from Tecnadyne Comms Interace v1.8
    double a1 = 0.003354016;
    double b1 = 0.000256985;
    double c1 = 2.62013e-6;
    double d1 = 6.38309e-8;
  
    
    if (count == 0x3FF)
        return -273.15; //absolute zero
  
    double resistance = (count/1023.0 * r_25) / (1 - (count / 1023.0)); //ohms 
    double log_ratio = log(resistance/r_25);
    double inv_temp_K = a1 + b1*log_ratio + c1*log_ratio*log_ratio +
        d1*log_ratio*log_ratio*log_ratio; // Steinhart & Hart
    double temp = (1/inv_temp_K) -273.15; //temp in degC
  
    return static_cast<float>(temp);
}

//------------------------------------------------------------------------------
// Name:        TEC_ANALOG_STATUS
// Description: Reads an analog status message (0x32) from the thruster and 
//              updates the current status struct
// Arguments:   - payload: 0x32 message payload
//              - thruster: thruster current status struct
// Returns:     - nothing
//------------------------------------------------------------------------------
void TEC_ANALOG_STATUS(std::vector<uint8_t> payload, Thruster *thruster)
{
    int16_t thruster_v_n = ( (payload[1] << 8) | payload[2] );//ADC count
    thruster->v = thruster_v_n * 0.02475; //Voltage in V
  
    uint16_t thruster_i_n = ( (payload[3] << 8) | payload[4] );//ADC count
    thruster->i = thruster_i_n * 0.125; //Current in A 
  
    thruster->p = thruster->i * thruster->p;
    //TODO Decide what to do with analog control voltage
  
    uint16_t controller_t_n = ( (payload[7] << 8) | payload[8] );//ADC count
    thruster->temp.controller = ( (controller_t_n * 4.88) - 480) / 15.6; //degC
  
    uint16_t heatsink_r_n = ( (payload[9] << 8) | payload[10] );//ADC Count
    thruster->temp.heatsink = TEC_THERMISTOR(heatsink_r_n);//Temp in C
  
    uint16_t winding_r_n = ( (payload[11] << 8) | payload[12] );//ADC Count
    thruster->temp.winding = TEC_THERMISTOR(winding_r_n);//Temp in C
  
}

//------------------------------------------------------------------------------
// Name:        TEC_FLASH_DATA
// Description: Reads a flash data message (0x35) from the thruster and 
//              updates the current status struct
// Arguments:   - payload: 0x35 message payload
//              - thruster: thruster current status struct
// Returns:     - nothing
//------------------------------------------------------------------------------
void TEC_FLASH_DATA(std::vector<uint8_t> payload, Thruster *thruster)
{
    thruster->settings.address = payload[1];
    thruster->settings.tx_delay = payload[2]; //comms delay before responding
    thruster->settings.watchdog = payload[3]; //timeout in s
    thruster->settings.slew = payload[4];     //motor slew rate UNKNOWN units
    thruster->settings.baud = payload[5];
    
    thruster->settings.freq = 21;       //default, PWM frequency in kHz
    if (payload[8] == 0x02)
        thruster->settings.freq = 30;     //alternate
    
    thruster->settings.fwd_lim = payload[9];  //PWM FWD limit
    thruster->settings.rev_lim = payload[10];  //PWM reverse limit
  
    thruster->settings.ACK = static_cast<bool>(payload[6]);  
    thruster->settings.analog = static_cast<bool>(payload[7]);
}  

//------------------------------------------------------------------------------
// Name:        TEC_EXT_STATUS
// Description: Reads an extended status message (0x37) from the thruster and 
//              updates the current status struct
// Arguments:   - payload: 0x37 message payload
//              - thruster: thruster current status struct
// Returns:     - nothing
//------------------------------------------------------------------------------
void TEC_EXT_STATUS(std::vector<uint8_t> payload, Thruster *thruster)
{
    thruster->speed_loop_en = static_cast<bool>(payload[1]);
    bool reverse = static_cast<bool>(payload[5]);
    thruster->fault = static_cast<bool>(payload[9]);
    thruster->reset = static_cast<bool>(payload[10]);
  
    uint8_t mins{payload[2]};
    uint8_t secs{payload[3]};
    uint16_t ms = 5 * payload[4];
    thruster->t_s = mins*60 + secs + ms/1000.0;
    
    thruster->pwm_pct = payload[6]/100.0;
    
    uint16_t tach = (payload[7] << 8) | payload[8];
    thruster->speed_loop_interval = 5 * payload[11]; //ms
    uint8_t tach_error{payload[12]};
  
    thruster->rpm.measured = (tach * 60.0 ) / 
        ( thruster->settings.pole_pair_count * 6 );
    if (thruster->speed_loop_en)
        thruster->rpm.measured *= ( 1000 / thruster->speed_loop_interval );
    if (reverse)
        thruster->rpm.measured *= -1;
  
    thruster->rpm.error = ( tach_error * 60.0 ) / 
        ( thruster->settings.pole_pair_count * 6 );
    if (thruster->speed_loop_en)
        thruster->rpm.error *= ( 1000 / thruster->speed_loop_interval );
  
    int16_t thruster_v_n = ( (payload[13] << 8) | payload[14] );//ADC count
    thruster->v = thruster_v_n * 0.02475; //Voltage in V
    uint16_t thruster_i_n = ( (payload[15] << 8) | payload[16] );//ADC count
    thruster->i = thruster_i_n * 0.125; //Current in A
}

//------------------------------------------------------------------------------
// Name:        TEC_FW
// Description: Reads a firmware status message (0x70) from the thruster and 
//              returns parsed information
// Arguments:   - payload: 0x70 message payload
//              - thruster: thruster current status struct
// Returns:     thruster current status struct
//------------------------------------------------------------------------------
void TEC_FW(std::vector<uint8_t> payload, Thruster *thruster)
{
    thruster->settings.fw_ver = std::to_string(payload[1]) + '.' 
                              + std::to_string(payload[2]) + '.' 
                              + std::to_string(payload[3]);
    thruster->settings.fw_date = std::to_string(payload[5]) + '/' 
                               + std::to_string(payload[6]) + '/' 
                               + std::to_string(payload[4]) + ' ' 
                               + std::to_string(payload[7]) + ':' 
                               + std::to_string(payload[8]);

}
#endif // TEC_THRUST_H
