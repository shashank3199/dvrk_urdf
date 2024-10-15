# Psm.Base.Ros2_Control.Xacro

## Macro: psm_base_ros2_control | Parameters: `<prefix>`

### Command Joints

| Joint Number    | Joint Name | Command Interface | State Interface       |
| --------------- | ---------- | ----------------- | --------------------- |
| Command Joint 0 | Yaw        | `position`        | `position` `velocity` |
| Command Joint 1 | Pitch      | `position`        | `position` `velocity` |

### Mimic Joints

| Joint Number  | Joint Name | Command Joint   | State Interface       |
| ------------- | ---------- | --------------- | --------------------- |
| Mimic Joint 0 | Pitch 2    | Command Joint 1 | `position` `velocity` |
| Mimic Joint 1 | Pitch 3    | Command Joint 1 | `position` `velocity` |

---

# Psm.Tool.P420006.Ros2_Control.Xacro

## Macro: P420006_ros2_control | Parameters: `<prefix>`

### Command Joints

| Joint Number    | Joint Name  | Command Interface | State Interface       |
| --------------- | ----------- | ----------------- | --------------------- |
| Command Joint 0 | Insertion   | `position`        | `position` `velocity` |
| Command Joint 1 | Roll        | `position`        | `position` `velocity` |
| Command Joint 2 | Wrist Pitch | `position`        | `position` `velocity` |
| Command Joint 3 | Wrist Yaw   | `position`        | `position` `velocity` |
| Command Joint 4 | Jaw         | `position`        | `position` `velocity` |

### Mimic Joints

| Joint Number  | Joint Name | Command Joint   | State Interface       |
| ------------- | ---------- | --------------- | --------------------- |
| Mimic Joint 0 | Jaw 1      | Command Joint 4 | `position` `velocity` |
| Mimic Joint 1 | Jaw 2      | Command Joint 4 | `position` `velocity` |

---

# Psm.Tool.Ros2_Control.Xacro

## Macro: psm_tool_ros2_control | Parameters: `<tool_name prefix>`

---

# Psm.Tool.Sf826001.Ros2_Control.Xacro

## Macro: SF826001_ros2_control | Parameters: `<prefix>`

### Command Joints

| Joint Number    | Joint Name | Command Interface | State Interface       |
| --------------- | ---------- | ----------------- | --------------------- |
| Command Joint 0 | Insertion  | `position`        | `position` `velocity` |
| Command Joint 1 | Roll       | `position`        | `position` `velocity` |

---
