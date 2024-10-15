# mtms.urdf.xacro

## Robot: mtms

### Included URDF Macros

| Macro     | Source File Name                | Params                                                                       |
| --------- | ------------------------------- | ---------------------------------------------------------------------------- |
| mtm_left  | [mtm.xacro](./xacros/mtm.xacro) | `prefix:"MTML", parent_link:"world", xyz:"-0.25 0.0 1.0", rpy:"0.0 0.0 0.0"` |
| mtm_right | [mtm.xacro](./xacros/mtm.xacro) | `prefix:"MTMR", parent_link:"world", xyz:"0.25 0.0 1.0", rpy:"0.0 0.0 0.0"`  |

### ROS2 Controller Macros

| Macro            | Source File Name                                                | Params          |
| ---------------- | --------------------------------------------------------------- | --------------- |
| mtm_ros2_control | [mtm.ros2_control.xacro](./ros2_control/mtm.ros2_control.xacro) | `prefix:"MTML"` |
| mtm_ros2_control | [mtm.ros2_control.xacro](./ros2_control/mtm.ros2_control.xacro) | `prefix:"MTML"` |

### ROS2 Control Hardware: fake_components/GenericSystem

### Links

| Link Number | Link Name   | Mesh File | Origin (`xyz`) | Orientation (`rpy`) |
| ----------- | ----------- | --------- | -------------- | ------------------- |
| Link 0      | Parent Link | N/A       | N/A            | N/A                 |

---
