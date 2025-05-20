/*
    File:    BatteryProtocol.ino
    Created: 2025.05.20 (Tue)
    Author:  Daniel Szilagyi

    Implement battery protocol according to https://mavlink.io/en/services/battery.html
    Assume that there is a 2s Li-Ion battery (7.2V) externally converted down to 5V connected to ADC pin A0
*/

#include <MAVLink.h>

#define SYSTEM_STATUS // Send SYSTEM_STATUS messages (optional)

const uint8_t sys_id = 1; // Assume connected Autopilot has System ID 1
const uint8_t comp_id = MAV_COMP_ID_BATTERY;

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

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial2.write(buf, len);

    Serial.println("Battery status out");
}

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
