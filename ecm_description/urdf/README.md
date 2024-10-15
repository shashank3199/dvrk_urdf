# ecm.base.urdf.xacro

## Robot: ecm_base

### Included URDF Macros

| Macro    | Source File Name                          | Params                              |
| -------- | ----------------------------------------- | ----------------------------------- |
| ecm_base | [ecm.base.xacro](./xacros/ecm.base.xacro) | `prefix:"ecm", parent_link:"world"` |

### ROS2 Controller Macros

| Macro                 | Source File Name                                                          | Params         |
| --------------------- | ------------------------------------------------------------------------- | -------------- |
| ecm_base_ros2_control | [ecm.base.ros2_control.xacro](./ros2_control/ecm.base.ros2_control.xacro) | `prefix:"ecm"` |

### ROS2 Control Hardware: fake_components/GenericSystem

### Links

| Link Number | Link Name   | Mesh File | Origin (`xyz`) | Orientation (`rpy`) |
| ----------- | ----------- | --------- | -------------- | ------------------- |
| Link 0      | Parent Link | N/A       | N/A            | N/A                 |

---

# ecm.urdf.xacro

## Robot: ecm

### Included URDF Macros

| Macro | Source File Name                | Params                                                                  |
| ----- | ------------------------------- | ----------------------------------------------------------------------- |
| ecm   | [ecm.xacro](./xacros/ecm.xacro) | `prefix:"ecm", parent_link:"world", xyz:"0 0.0 0.0", rpy:"0.0 0.0 0.0"` |

### ROS2 Controller Macros

| Macro            | Source File Name                                                | Params         |
| ---------------- | --------------------------------------------------------------- | -------------- |
| ecm_ros2_control | [ecm.ros2_control.xacro](./ros2_control/ecm.ros2_control.xacro) | `prefix:"ecm"` |

### ROS2 Control Hardware: fake_components/GenericSystem

### Links

| Link Number | Link Name   | Mesh File | Origin (`xyz`) | Orientation (`rpy`) |
| ----------- | ----------- | --------- | -------------- | ------------------- |
| Link 0      | Parent Link | N/A       | N/A            | N/A                 |

---
