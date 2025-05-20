# Arduino MAVLink System ID assignment protocol (simple version)

**Goal:** Implement simple System ID assignment - "Steal" the System ID from the Autopilot HEARTBEAT message

## Communications

```mermaid
sequenceDiagram
    participant AP as Autopilot
    participant AR as Arduino

    note over AP: Set SysID (10)
    note over AR: Set SysID (1)
    AP ->> AR: 1st HEARTBEAT
    note over AR: Set SysD (10)
    loop Later HEARTBEATs
        AP ->> AR: Nth HEARTBEAT
        note over AR: Keep SysID (10)
    end
```

## Sources
- ID assignment protocol: https://mavlink.io/en/services/mavlink_id_assignment.html
- HEARTBEAT message: https://mavlink.io/en/messages/common.html#HEARTBEAT
