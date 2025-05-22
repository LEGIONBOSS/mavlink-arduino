/*
    File:    ParameterProtocol.ino
    Created: 2025.05.21 (Wed)
    Author:  Daniel Szilagyi

    Implement parameter protocol according to https://mavlink.io/en/services/parameter.html
    The Arduino syncs 3 new INT32 parameters with the Autopilot (AR_PARAM1, AR_PARAM2 and AR_PARAM3)
*/

#include <MAVLink.h>

uint8_t sys_id = 1; // Assume connected Autopilot has System ID 1
const uint8_t comp_id = MAV_COMP_ID_USER1;

const uint16_t param_num = 3;
char* param_names[param_num] = {"AR_PARAM1", "AR_PARAM2", "AR_PARAM3"};
int32_t param_values[param_num] = {0, 0, 0};
bool param_send_flags[param_num] = {false, false, false};

int32_t param_index_from_name(char* name)
{
    uint16_t i = 0;
    while (i < param_num && strcmp(name, param_names[i]) != 0)
    {
        i++;
    }
    if (i < param_num)
    {
        return (int32_t)i;
    }
    else
    {
        return -1;
    }
}

bool param_exists(int32_t index, char* name)
{
    if (index < 0)
    {
        return param_index_from_name(name) != -1;
    }
    else
    {
        return (uint16_t)index < param_num
            && strcmp(name, param_names[(uint16_t)index]) == 0;
    }
}

void param_set(int32_t index, char* name, int32_t value)
{
    if (param_exists(index, name))
    {
        if (index == -1)
        {
            index = param_index_from_name(name);
        }
        param_values[(uint16_t)index] = value;
        param_send_flags[(uint16_t)index] = true;
    }
}

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
                case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
                {
                    mavlink_param_request_list_t param_request_list;
                    mavlink_msg_param_request_list_decode(&msg, &param_request_list);
                    if ((param_request_list.target_system == sys_id                     // Sent to this system
                            || param_request_list.target_system == 0)                   // Sent to all systems
                        && (param_request_list.target_component == comp_id              // Sent to this component
                            || param_request_list.target_component == MAV_COMP_ID_ALL)) // Sent to all components
                    {
                        for (uint16_t i = 0; i < param_num; i++)
                        {
                            param_send_flags[i] = true;
                        }

                        Serial.println("Param value list requested");
                    }
                }
                break;
                case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
                {
                    mavlink_param_request_read_t param_request_read;
                    mavlink_msg_param_request_read_decode(&msg, &param_request_read);
                    if ((param_request_read.target_system == sys_id                                   // Sent to this system
                            || param_request_read.target_system == 0)                                 // Sent to all systems
                        && (param_request_read.target_component == comp_id                            // Sent to this component
                            || param_request_read.target_component == MAV_COMP_ID_ALL)                // Sent to all components
                        && param_exists((int32_t)param_request_read.param_index, param_request_read.param_id)) // Param exists
                    {
                        int32_t index = (int32_t)param_request_read.param_index;
                        if (index == -1)
                        {
                            index = param_index_from_name(param_request_read.param_id);
                        }
                        param_send_flags[(uint16_t)index] = true;

                        Serial.println("Param ");
                        Serial.print(param_request_read.param_id);
                        Serial.print("[");
                        Serial.print(param_request_read.param_index);
                        Serial.println("] value requested");
                    }
                }
                break;
                case MAVLINK_MSG_ID_PARAM_SET:
                {
                    mavlink_param_set_t param_set;
                    mavlink_msg_param_set_decode(&msg, &param_set);
                    if ((param_set.target_system == sys_id                    // Sent to this system
                            || param_set.target_system == 0)                  // Sent to all systems
                        && (param_set.target_component == comp_id             // Sent to this component
                            || param_set.target_component == MAV_COMP_ID_ALL) // Sent to all components
                        && param_exists(-1, param_set.param_id)               // Param exists
                        && param_set.param_type == MAV_PARAM_TYPE_INT32)      // Param has correct type
                    {
                        param_set(-1, param_set.param_id, (int32_t)param_set.param_value);

                        Serial.print("New parameter value (");
                        Serial.print(param_set.param_id);
                        Serial.print("=");
                        Serial.println(")");
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

    Serial.println("HEARTBEAT out");
}

void mavlink_send_param_value(uint16_t i)
{
    if (i < param_num)
    {
        mavlink_message_t msg;

        mavlink_msg_param_value_pack(sys_id,                 // system_id
                                     comp_id,                // component_id
                                     &msg,                   // msg pointer
                                     param_names[i],         // param_id
                                     (float)param_values[i], // param_value
                                     MAV_PARAM_TYPE_INT32,   // param_type
                                     param_num,              // param_count
                                     i);                     // param_index

        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        Serial2.write(buf, len);

        Serial.print("PARAM_VALUE (");
        Serial.print(param_names[i]);
        Serial.print("=");
        Serial.print(param_values[i]);
        Serial.println(") out");
    }
    else
    {
        Serial.print("PARAM_VALUE out failed, index ");
        Serial.print(i);
        Serial.print(" is out of range ");
        Serial.println(param_num);
    }
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
    if (millis() - hb_last >= 1000)
    {
        mavlink_send_heartbeat();
        hb_last = millis();
    }

    // Send param values (on request)
    for (uint16_t i = 0; i < param_num; i++)
    {
        if (param_send_flags[i])
        {
            mavlink_send_param_value(i);
            param_send_flags[i] = false;
        }
    }
}
