# Arduino MAVLink battery protocol

**Goal:** Implement battery status reporting protocol

## Communications

```mermaid
sequenceDiagram
    participant AP as Autopilot
    participant AR as Arduino

    loop continuous
        note over AR: Measure battery
    end
    loop 1Hz
        AR ->> AP: HEARTBEAT<br>(MAV_TYPE_BATTERY)
    end
    loop 0.5Hz
        AR ->> AP: BATTERY_STATUS
    end
    loop 1Hz
        AR ->> AP: SYSTEM_STATUS
    end
```

## Sources
- Battery protocol: https://mavlink.io/en/services/battery.html
- BATTERY_STATUS message: https://mavlink.io/en/messages/common.html#BATTERY_STATUS
- SYSTEM_STATUS message: https://mavlink.io/en/messages/common.html#SYS_STATUS
- HEARTBEAT message: https://mavlink.io/en/messages/common.html#HEARTBEAT
