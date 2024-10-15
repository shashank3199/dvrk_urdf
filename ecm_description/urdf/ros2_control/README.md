# Ecm.Base.Ros2_Control.Xacro

## Macro: ecm_base_ros2_control | Parameters: `<prefix>`

### Command Joints

| Joint Number    | Joint Name | Command Interface | State Interface       |
| --------------- | ---------- | ----------------- | --------------------- |
| Command Joint 0 | Yaw        | `position`        | `position` `velocity` |
| Command Joint 1 | Pitch      | `position`        | `position` `velocity` |
| Command Joint 2 | Insertion  | `position`        | `position` `velocity` |
| Command Joint 3 | Roll       | `position`        | `position` `velocity` |

### Mimic Joints

| Joint Number  | Joint Name   | Command Joint   | State Interface       |
| ------------- | ------------ | --------------- | --------------------- |
| Mimic Joint 0 | Pitch Bottom | Command Joint 1 | `position` `velocity` |
| Mimic Joint 1 | Pitch End    | Command Joint 1 | `position` `velocity` |
| Mimic Joint 2 | Pitch Top    | Command Joint 1 | `position` `velocity` |
| Mimic Joint 3 | Pitch Back   | Command Joint 1 | `position` `velocity` |

---

# Ecm.Ros2_Control.Xacro

## Macro: ecm_ros2_control | Parameters: `<prefix>`

### Command Joints

| Joint Number    | Joint Name  | Command Interface | State Interface       |
| --------------- | ----------- | ----------------- | --------------------- |
| Command Joint 0 | Outer Yaw   | `position`        | `position` `velocity` |
| Command Joint 1 | Outer Pitch | `position`        | `position` `velocity` |
| Command Joint 2 | Insertion   | `position`        | `position` `velocity` |
| Command Joint 3 | Outer Roll  | `position`        | `position` `velocity` |

### Mimic Joints

| Joint Number  | Joint Name    | Command Joint   | State Interface       |
| ------------- | ------------- | --------------- | --------------------- |
| Mimic Joint 0 | Outer Pitch 0 | Command Joint 1 | `position` `velocity` |
| Mimic Joint 1 | Outer Pitch 1 | Command Joint 1 | `position` `velocity` |
| Mimic Joint 2 | Outer Pitch 2 | Command Joint 1 | `position` `velocity` |

---
