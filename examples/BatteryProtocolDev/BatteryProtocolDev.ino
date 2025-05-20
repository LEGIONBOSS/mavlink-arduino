/*
    File:    BatteryProtocolDev.ino
    Created: 2025.05.16 (Fri)
    Author:  Daniel Szilagyi

    Implement battery protocol (DEV) according to https://mavlink.io/en/services/battery.html
    Assume that there is a 2s Li-Ion battery (7.2V) externally converted down to 5V connected to ADC pin A0
*/

#include <MAVLink_development.h>

#define BATTERY_STATUS_V2 // Use BATTERY_STATUS_V2 (WIP) instead of BATTERY_STATUS
#define BATTERY_INFO // Send BATTERY_INFO messages (WIP)
#define SYSTEM_STATUS // Send SYSTEM_STATUS messages (optional)

const uint8_t sys_id = 1; // Assume connected Autopilot has System ID 1
const uint8_t comp_id = MAV_COMP_ID_BATTERY;

#ifdef BATTERY_INFO
bool connected = false;
bool battery_info_flag = false;
bool command_ack_flag = false;
#endif

const uint8_t battery_cell_num = 2;
float battery_voltage = 0;
float battery_cell_voltage[battery_cell_num];

void battery_read()
{
    battery_voltage = 7.4;
    battery_cell_voltage[0] = 3.6;
    battery_cell_voltage[1] = 3.6;
    // TODO: Update battery_voltage and battery_cell_voltage accurately
}

#ifdef BATTERY_INFO
void mavlink_read()
{
    mavlink_status_t status;
    mavlink_message_t msg;
    uint8_t chan = MAVLINK_COMM_0;

    while (Serial2.available() > 0)
    {
        uint8_t byte = Serial2.read();
        if (mavlink_parse_char(chan, byte, &msg, &status))
        {
            switch (msg.msgid)
            {
                case MAVLINK_MSG_ID_HEARTBEAT:
                {
                    if (!connected && msg.compid == MAV_COMP_ID_AUTOPILOT1)
                    {
                        battery_info_flag = true;
                        connected = true;

                        Serial.println("Battery info requested (connection)");
                    }
                }
                break;
                case MAVLINK_MSG_ID_COMMAND_LONG:
                {
                    mavlink_command_long_t command_long;
                    mavlink_msg_command_long_decode(&msg, &command_long);
                    if ((command_long.target_system == sys_id                            // Sent to this system
                            || command_long.target_system == 0)                          // Sent to all systems
                        && (command_long.target_component == comp_id                     // Sent for this component
                            || command_long.target_component == MAV_COMP_ID_ALL)         // Sent for all components
                        && command_long.command == MAV_CMD_REQUEST_MESSAGE               // Is a message request
                        && (uint32_t)command_long.param1 == MAVLINK_MSG_ID_BATTERY_INFO) // Is a battery info request
                    {
                        battery_info_flag = true;
                        command_ack_flag = true;

                        Serial.println("Battery info requested (request)");
                    }
                }
                break;
                default: break;
            }
        }
    }
}
#endif // ifdef BATTERY_INFO

void mavlink_send_heartbeat()
{
    mavlink_message_t msg;

    mavlink_msg_heartbeat_pack(sys_id,                            // system_id
                               comp_id,                           // component_id
                               &msg,                              // msg pointer
                               MAV_TYPE_BATTERY,                  // type
                               MAV_AUTOPILOT_INVALID,             // autopilot
                               MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, // base_mode (n/a)
                               0,                                 // custom_mode (n/a)
                               MAV_STATE_ACTIVE);                 // system_status

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial2.write(buf, len);

    Serial.println("Heartbeat out");
}

void mavlink_send_battery_status()
{
    mavlink_message_t msg;

    uint16_t voltages[battery_cell_num];
    for (uint8_t i = 0; i < battery_cell_num; i++)
    {
        voltages[i] = (uint16_t)(battery_cell_voltage[i] * 1000);
    }
    uint16_t invalid[] = {0};
#ifdef BATTERY_STATUS_V2
    mavlink_msg_battery_status_v2_pack(sys_id,          // system_id
                                       comp_id,         // component_id
                                       &msg,            // msg pointer
                                       0,               // id
                                       INT16_MAX,       // temperature (n/a)
                                       battery_voltage, // voltage
                                       NULL,            // current (n/a)
                                       NULL,            // capacity_consumed (n/a)
                                       NULL,            // capacity_remaining (n/a)
                                       UINT8_MAX,       // percent_remaining (n/a)
                                       0);              // status_flags (n/a)
#else
    mavlink_msg_battery_status_pack(sys_id,                       // system_id
                                    comp_id,                      // component_id
                                    &msg,                         // msg pointer
                                    0,                            // id
                                    MAV_BATTERY_FUNCTION_UNKNOWN, // battery_function
                                    MAV_BATTERY_TYPE_LION,        // type
                                    INT16_MAX,                    // temperature (n/a)
                                    voltages,                     // voltages
                                    -1,                           // current_battery (n/a)
                                    -1,                           // current_consumed (n/a)
                                    -1,                           // energy_consumed (n/a)
                                    -1,                           // battery_remaining (n/a)
                                    -1,                           // time_remaining  (n/a)
                                    MAV_BATTERY_CHARGE_STATE_OK,  // charge_state
                                    invalid,                      // voltages_ext (n/a)
                                    MAV_BATTERY_MODE_UNKNOWN,     // mode (n/a)
                                    0);                           // fault_bitmask (n/a)
#endif

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial2.write(buf, len);

    Serial.print("Battery status");
#ifdef BATTERY_STATUS_V2
    Serial.print(" v2");
#endif
    Serial.println(" out");
}

#ifdef BATTERY_INFO
void mavlink_send_battery_info()
{
    mavlink_message_t msg;
    
    char* invalid = "0";
    mavlink_msg_battery_info_pack(sys_id,                       // system_id
                                  comp_id,                      // component_id
                                  &msg,                         // msg pointer
                                  0,                            // id
                                  MAV_BATTERY_FUNCTION_UNKNOWN, // battery_function
                                  MAV_BATTERY_TYPE_LION,        // type
                                  UINT8_MAX,                    // state_of_health (n/a)
                                  battery_cell_num,             // cells_in_series
                                  UINT16_MAX,                   // cycle_count (n/a)
                                  0,                            // weight (n/a)
                                  0,                            // discharge_minimum_voltage (n/a)
                                  0,                            // charging_minimum_voltage (n/a)
                                  0,                            // resting_minimum_voltage (n/a)
                                  0,                            // charging_maximum_voltage (n/a)
                                  0,                            // charging_maximum_current (n/a)
                                  7.4,                          // nominal_voltage
                                  0,                            // discharge_maximum_current (n/a)
                                  0,                            // discharge_maximum_burst_current (n/a)
                                  0,                            // design_capacity (n/a)
                                  0,                            // full_charge_capacity (n/a)
                                  invalid,                      // manufacture_date (n/a)
                                  invalid,                      // serial_number (n/a)
                                  invalid);                     // name (n/a)
    
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial2.write(buf, len);
    
    Serial.println("Battery info out");
}

void mavlink_send_command_ack()
{
    mavlink_message_t msg;

    mavlink_msg_command_ack_pack(sys_id,                  // system_id
                                 comp_id,                 // component_id
                                 &msg,                    // msg pointer
                                 MAV_CMD_REQUEST_MESSAGE, // command
                                 MAV_RESULT_ACCEPTED,     // result
                                 UINT8_MAX,               // progress (n/a)
                                 0,                       // result_param2 (n/a)
                                 sys_id,                  // target_system
                                 MAV_COMP_ID_AUTOPILOT1); // target_component

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial2.write(buf, len);

    Serial.println("Command ack out");
}
#endif // ifdef BATTERY_INFO

#ifdef SYSTEM_STATUS
void mavlink_send_system_status()
{
    mavlink_message_t msg;

    mavlink_msg_sys_status_pack(sys_id,                             // system_id
                                comp_id,                            // component_id
                                &msg,                               // msg pointer
                                MAV_SYS_STATUS_SENSOR_BATTERY,      // onboard_control_sensors_present
                                MAV_SYS_STATUS_SENSOR_BATTERY,      // onboard_control_sensors_enabled
                                MAV_SYS_STATUS_SENSOR_BATTERY,      // onboard_control_sensors_health
                                0,                                  // load (n/a)
                                (uint16_t)(battery_voltage * 1000), // voltage_battery
                                -1,                                 // current_battery (n/a)
                                -1,                                 // battery_remaining (n/a)
                                0,                                  // drop_rate_comm (n/a)
                                0,                                  // errors_comm (n/a)
                                0,                                  // errors_count1 (n/a)
                                0,                                  // errors_count2 (n/a)
                                0,                                  // errors_count3 (n/a)
                                0,                                  // errors_count4 (n/a)
                                0,                                  // onboard_control_sensors_present_extended (n/a)
                                0,                                  // onboard_control_sensors_enabled_extended (n/a)
                                0);                                 // onboard_control_sensors_health_extended (n/a)

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial2.write(buf, len);

    Serial.println("System status out");
}
#endif // ifdef SYSTEM_STATUS

void setup(void)
{
    Serial.begin(115200);
    Serial2.begin(57600); // Assume MAVLink is connected to UART2

    Serial.println("Setup complete, starting loop");
}

void loop(void)
{
    // Read battery
    battery_read();

#ifdef BATTERY_INFO
    // Read MAVLink
    mavlink_read();
#endif

    // Send heartbeat (1Hz)
    static uint32_t hb_last;
    if (millis() - hb_last >= 1000)
    {
        mavlink_send_heartbeat();
        hb_last = millis();
    }

    // Send battery status (0.5Hz)
    static uint32_t battery_last;
    if (millis() - battery_last >= 2000)
    {
        mavlink_send_battery_status();
        battery_last = millis();
    }

#ifdef BATTERY_INFO
    // Send battery info (on request)
    if (battery_info_flag)
    {
        mavlink_send_battery_info();
        battery_info_flag = false;
    }

    // Send command ack (on request)
    if (command_ack_flag)
    {
        mavlink_send_command_ack();
        command_ack_flag = false;
    }
#endif // ifdef BATTERY_INFO

#ifdef SYSTEM_STATUS
    // Send system status (1Hz)
    static uint32_t sys_status_last;
    if (millis() - sys_status_last >= 1000)
    {
        mavlink_send_system_status();
        sys_status_last = millis();
    }
#endif
}
