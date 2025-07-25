# Arduino MAVLink battery protocol (DEV version)

**Goal:** Implement battery status reporting protocol in it's entirety (including WIP messages)

## Communications

```mermaid
sequenceDiagram
    participant AP as Autopilot
    participant AR as Arduino

    loop continuous
        note over AR: Measure battery
    end
    AP ->>+ AR: 1st HEARTBEAT
    AR ->>- AP: BATTERY_INFO
    AP ->>+ AR: COMMAND_LONG-<br>REQUEST_MESSAGE<br>(BATTERY_INFO)
    AR ->> AP: BATTERY_INFO
    AR ->>- AP: COMMAND_ACK
    loop 1Hz
        AR ->> AP: HEARTBEAT<br>(MAV_TYPE_BATTERY)
    end
    loop 0.5Hz
        AR ->> AP: BATTERY_STATUS or<br>BATTERY_STATUS_V2
    end
    loop 1Hz
        AR ->> AP: SYSTEM_STATUS
    end
```

## Sources
- Battery protocol: https://mavlink.io/en/services/battery.html
- Command protocol: https://mavlink.io/en/services/command.html
- BATTERY_STATUS message: https://mavlink.io/en/messages/common.html#BATTERY_STATUS
- BATTERY_STATUS_V2 message: https://mavlink.io/en/messages/development.html#BATTERY_STATUS_V2
- SYSTEM_STATUS message: https://mavlink.io/en/messages/common.html#SYS_STATUS
- COMMAND_LONG message: https://mavlink.io/en/messages/common.html#COMMAND_LONG
- COMMAND_ACK message: https://mavlink.io/en/messages/common.html#COMMAND_ACK
- REQUEST_MESSAGE command: https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_MESSAGE
- BATTERY_INFO message: https://mavlink.io/en/messages/common.html#BATTERY_INFO
- HEARTBEAT message: https://mavlink.io/en/messages/common.html#HEARTBEAT
