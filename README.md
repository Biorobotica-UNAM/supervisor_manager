# supervisor_manager

## Overview

`supervisor_manager` is the high-level coordination package responsible for orchestrating supervisory modules within the robotic architecture.

It integrates:

- `supervisor_system`
- `supervisor_plan`
- (Future supervisory components)

Its objective is to ensure global coherence between system operability and plan-level supervision.

---

## Architectural Role

Within the layered architecture:

Planner → Supervisor → Executor

`supervisor_manager` acts as the supervisory coordinator, ensuring that:

- The system is operational.
- Plans are coherent.
- Corrective strategies are applied when necessary.

---

## Functional Responsibilities

### 1. System-State Verification

The manager queries `supervisor_system` to determine:

- Whether required modules are active.
- Whether recovery actions are required.

If the system is unstable, corrective measures are initiated.

---

### 2. Plan Coordination

The manager supervises `supervisor_plan` and can:

- Trigger plan corrections.
- Inject corrective instructions.
- Request re-evaluation of current plan states.

This allows cross-layer corrective behavior.

---

### 3. Cross-Supervisor Arbitration

The manager serves as a mediator between:

- System-level supervision
- Plan-level supervision

If system instability affects plan feasibility, the manager can:

- Request plan adjustment.
- Inject fallback instructions.
- Delay execution until stability is restored.

---

## Design Objectives

- Hierarchical supervisory control.
- Decoupled module coordination.
- Runtime adaptability.
- Structural modularity.
- Scalability for additional supervisors.

---

## Dependencies

- ROS 2 (rclcpp)
- supervisor_system
- supervisor_plan

---

## Future Work

- Global mission-level monitoring.
- Supervisor priority arbitration.

---

## Author: Joshua Martinez

Bio-Robotics Laboratory  
Universidad Nacional Autónoma de México
