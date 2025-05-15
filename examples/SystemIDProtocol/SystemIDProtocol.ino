/*
    File:	 SystemIDProtocol.ino
    Created: 2025.05.15 (Thu)
    Author:  Daniel Szilagyi

    Implement System ID assignment according to https://mavlink.io/en/services/mavlink_id_assignment.html
*/

#include <MAVLink.h>

const uint8_t comp_id = MAV_COMP_ID_USER1;
uint8_t sys_id = 1;
bool sys_id_set = false;

const uint32_t sys_id_get_time = 3000; // ms
uint32_t first_heartbeat_time = UINT32_MAX; // ms
uint8_t first_sys_id = 0;

void mavlink_read()
{
    mavlink_status_t status;
    mavlink_message_t msg;
    uint8_t chan = MAVLINK_COMM_0;

    while (Serial2.available())
    {
        uint8_t byte = Serial2.read();
        if (mavlink_parse_char(chan, byte, &msg, &status))
        {
            switch (msg.msgid)
            {
                case MAVLINK_MSG_ID_HEARTBEAT:
                {
                    if (msg.compid == MAV_COMP_ID_AUTOPILOT1)
                    {
                        Serial.print("Autopilot heartbeat in, system ID is ");
                        Serial.println(msg.sysid);

                        if (!sys_id_set)
                        {
                            if (first_heartbeat_time == UINT32_MAX) // First autopilot HB
                            {
                                first_heartbeat_time = millis();
                                first_sys_id = msg.sysid;

                                Serial.println("System ID acquisition timer start");
                            }
                            else if (msg.sysid != first_sys_id) // Subsequent autopilot HB, different sysid
                            {
                                // Leave sys_id as default
                                sys_id_set = true;

                                Serial.println("Conflicting system IDs detected");
                                Serial.print("System ID set to ");
                                Serial.println(sys_id);

                                break; // Sys ID was set prematurely
                            }

                            if (millis() - first_heartbeat_time > sys_id_get_time) // Sysid acquisition time (3s) is up
                            {
                                sys_id = first_sys_id;
                                sys_id_set = true;

                                Serial.println("System ID acquisition timer end");
                                Serial.print("System ID set to ");
                                Serial.println(sys_id);
                            }
                        }
                    }
                    else
                    {
                        Serial.println("Heartbeat in");
                    }
                }
                break;
                default: break;
            }
        }
    }
}

void mavlink_send_heartbeat()
{
    mavlink_message_t msg;

    mavlink_msg_heartbeat_pack(sys_id,                            // system_id
                               comp_id,                           // component_id
                               &msg,                              // msg pointer
                               MAV_TYPE_GENERIC,                  // type
                               MAV_AUTOPILOT_INVALID,             // autopilot
                               MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, // base_mode
                               0,                                 // custom_mode
                               MAV_STATE_ACTIVE);                 // system_status

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial2.write(buf, len);

    Serial.println("Heartbeat out");
}

void setup(void)
{
    Serial.begin(115200);
    Serial2.begin(57600); // Assume MAVLink is connected to UART2

    Serial.println("Setup complete, starting loop");
}

void loop(void)
{
    // Read MAVLink
    mavlink_read();

    // Send heartbeat (1Hz)
    static uint32_t hb_last;
    if (sys_id_set && millis() - hb_last >= 1000)
    {
        mavlink_send_heartbeat();
        hb_last = millis();
    }
}
