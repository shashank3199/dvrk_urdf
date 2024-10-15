# Mtm.Ros2_Control.Xacro

## Macro: mtm_ros2_control | Parameters: `<prefix>`

### Command Joints

| Joint Number    | Joint Name     | Command Interface | State Interface       |
| --------------- | -------------- | ----------------- | --------------------- |
| Command Joint 0 | Outer Yaw      | `position`        | `position` `velocity` |
| Command Joint 1 | Shoulder Pitch | `position`        | `position` `velocity` |
| Command Joint 2 | Elbow Pitch    | `position`        | `position` `velocity` |
| Command Joint 3 | Wrist Platform | `position`        | `position` `velocity` |
| Command Joint 4 | Wrist Pitch    | `position`        | `position` `velocity` |
| Command Joint 5 | Wrist Yaw      | `position`        | `position` `velocity` |
| Command Joint 6 | Wrist Roll     | `position`        | `position` `velocity` |

### Mimic Joints

| Joint Number  | Joint Name              | Command Joint   | State Interface       |
| ------------- | ----------------------- | --------------- | --------------------- |
| Mimic Joint 0 | Shoulder Pitch Parallel | Command Joint 1 | `position` `velocity` |

---
