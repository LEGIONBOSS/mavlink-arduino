# Arduino MAVLink System ID assignment protocol

**Goal:** Implement System ID assignment - "Steal" the System ID from the Autopilot HEARTBEAT message

## Case 1: Single Autopilot in the system

```mermaid
sequenceDiagram
    participant AP as Autopilot
    participant AR as Arduino

    note over AP: Set SysID (10)
    note over AR: Set SysID (1)
    loop System ID acquisition time (3s)
        AP ->> AR: 1st HEARTBEAT
        note over AR: Save SysD (10)
        loop Later HEARTBEATs
            AP ->> AR: Nth HEARTBEAT
        end
    end
    note over AR: Set SysID (10)
    loop Later HEARTBEATs
        AP ->> AR: Nth HEARTBEAT
        note over AR: Keep SysID (10)
    end
```

## Case 2: Multiple Autopilots in the system (System ID conflict)

```mermaid
sequenceDiagram
    participant AP1 as Autopilot 1
    participant AR as Arduino
    participant AP2 as Autopilot 2

    note over AP1: Set SysID (10)
    note over AP2: Set SysID (20)
    note over AR: Set SysID (1)
    loop System ID acquisition time (3s)
        AP1 ->> AR: 1st HEARTBEAT
        note over AR: Save SysD (10)
        loop Later HEARTBEATs
            AP1 ->> AR: Nth HEARTBEAT
        end
        AP2 ->> AR: 1st HEARTBEAT
        note over AR: Detect conflicting SysID (20),<br>break out of System ID acquisition
    end
    note over AR: Keep initial SysID (1)
    loop Later HEARTBEATs
        AP1 ->> AR: Nth HEARTBEAT
        AP2 ->> AR: Nth HEARTBEAT
        note over AR: Keep SysID (1)
    end
```

## Sources
- ID assignment protocol: https://mavlink.io/en/services/mavlink_id_assignment.html
- HEARTBEAT message: https://mavlink.io/en/messages/common.html#HEARTBEAT
