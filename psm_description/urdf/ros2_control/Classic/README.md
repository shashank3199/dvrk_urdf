# Psm.Base.Ros2_Control.Xacro

## Macro: psm_base_ros2_control | Parameters: `<prefix>`

### Command Joints

| Joint Number    | Joint Name      | Command Interface | State Interface       |
| --------------- | --------------- | ----------------- | --------------------- |
| Command Joint 0 | Outer Yaw       | `position`        | `position` `velocity` |
| Command Joint 1 | Pitch           | `position`        | `position` `velocity` |
| Command Joint 2 | Outer Insertion | `position`        | `position` `velocity` |
| Command Joint 3 | Outer Roll      | `position`        | `position` `velocity` |

### Mimic Joints

| Joint Number  | Joint Name    | Command Joint   | State Interface       |
| ------------- | ------------- | --------------- | --------------------- |
| Mimic Joint 0 | Outer Pitch 1 | Command Joint 1 | `position` `velocity` |
| Mimic Joint 1 | Outer Pitch 2 | Command Joint 1 | `position` `velocity` |
| Mimic Joint 2 | Outer Pitch 3 | Command Joint 1 | `position` `velocity` |
| Mimic Joint 3 | Outer Pitch 4 | Command Joint 1 | `position` `velocity` |
| Mimic Joint 4 | Outer Pitch 5 | Command Joint 1 | `position` `velocity` |

---

# Psm.Tool.Blade.Ros2_Control.Xacro

## Macro: psm_tool_blade_ros2_control | Parameters: `<prefix>`

### Command Joints

| Joint Number    | Joint Name        | Command Interface | State Interface       |
| --------------- | ----------------- | ----------------- | --------------------- |
| Command Joint 0 | Outer Wrist Pitch | `position`        | `position` `velocity` |
| Command Joint 1 | Outer Wrist Yaw   | `position`        | `position` `velocity` |
| Command Joint 2 | Jaw               | `position`        | `position` `velocity` |

---

# Psm.Tool.Caudier.Blade.Ros2_Control.Xacro

## Macro: psm_tool_caudier_blade_ros2_control | Parameters: `<prefix>`

### Command Joints

| Joint Number    | Joint Name        | Command Interface | State Interface       |
| --------------- | ----------------- | ----------------- | --------------------- |
| Command Joint 0 | Outer Wrist Pitch | `position`        | `position` `velocity` |
| Command Joint 1 | Outer Wrist Yaw   | `position`        | `position` `velocity` |

---

# Psm.Tool.Caudier.Ros2_Control.Xacro

## Macro: psm_tool_caudier_ros2_control | Parameters: `<prefix>`

### Command Joints

| Joint Number    | Joint Name        | Command Interface | State Interface       |
| --------------- | ----------------- | ----------------- | --------------------- |
| Command Joint 0 | Outer Wrist Pitch | `position`        | `position` `velocity` |
| Command Joint 1 | Outer Wrist Yaw   | `position`        | `position` `velocity` |
| Command Joint 2 | Jaw               | `position`        | `position` `velocity` |

### Mimic Joints

| Joint Number  | Joint Name  | Command Joint   | State Interface       |
| ------------- | ----------- | --------------- | --------------------- |
| Mimic Joint 0 | Jaw Mimic 1 | Command Joint 2 | `position` `velocity` |
| Mimic Joint 1 | Jaw Mimic 2 | Command Joint 2 | `position` `velocity` |

---

# Psm.Tool.Ros2_Control.Xacro

## Macro: psm_tool_ros2_control | Parameters: `<tool_name prefix>`

---

# Psm.Tool.Sca.Blade.Ros2_Control.Xacro

## Macro: psm_tool_sca_blade_ros2_control | Parameters: `<prefix>`

### Command Joints

| Joint Number    | Joint Name        | Command Interface | State Interface       |
| --------------- | ----------------- | ----------------- | --------------------- |
| Command Joint 0 | Outer Wrist Pitch | `position`        | `position` `velocity` |
| Command Joint 1 | Outer Wrist Yaw   | `position`        | `position` `velocity` |

---

# Psm.Tool.Sca.Ros2_Control.Xacro

## Macro: psm_tool_sca_ros2_control | Parameters: `<prefix>`

### Command Joints

| Joint Number    | Joint Name        | Command Interface | State Interface       |
| --------------- | ----------------- | ----------------- | --------------------- |
| Command Joint 0 | Outer Wrist Pitch | `position`        | `position` `velocity` |
| Command Joint 1 | Outer Wrist Yaw   | `position`        | `position` `velocity` |
| Command Joint 2 | Jaw               | `position`        | `position` `velocity` |

### Mimic Joints

| Joint Number  | Joint Name  | Command Joint   | State Interface       |
| ------------- | ----------- | --------------- | --------------------- |
| Mimic Joint 0 | Jaw Mimic 1 | Command Joint 2 | `position` `velocity` |
| Mimic Joint 1 | Jaw Mimic 2 | Command Joint 2 | `position` `velocity` |

---

# Psm.Tool.Snake.Ros2_Control.Xacro

## Macro: psm_tool_snake_ros2_control | Parameters: `<prefix>`

### Command Joints

| Joint Number    | Joint Name         | Command Interface | State Interface       |
| --------------- | ------------------ | ----------------- | --------------------- |
| Command Joint 0 | Outer Wrist Pitch1 | `position`        | `position` `velocity` |
| Command Joint 1 | Outer Wrist Yaw1   | `position`        | `position` `velocity` |
| Command Joint 2 | Outer Wrist Yaw2   | `position`        | `position` `velocity` |
| Command Joint 3 | Outer Wrist Pitch2 | `position`        | `position` `velocity` |
| Command Joint 4 | Jaw                | `position`        | `position` `velocity` |

### Mimic Joints

| Joint Number  | Joint Name  | Command Joint   | State Interface       |
| ------------- | ----------- | --------------- | --------------------- |
| Mimic Joint 0 | Jaw Mimic 1 | Command Joint 4 | `position` `velocity` |
| Mimic Joint 1 | Jaw Mimic 2 | Command Joint 4 | `position` `velocity` |

---
