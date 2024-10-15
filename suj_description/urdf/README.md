# suj.classic.urdf.xacro

## Robot: suj_classic

### Included URDF Macros

| Macro    | Source File Name                                                     | Params                                                                                               |
| -------- | -------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------- |
| property | N/A                                                                  | `name:"PI", value:"3.1415926535897931"`                                                              |
| psm2_j   | [psm2j.xacro](../../suj_description/urdf/xacros/Classic/psm2j.xacro) | `parent_link:"base_link", xyz:"0.1912 -0.101599999999999 0.761099999999999", rpy:"-${PI} 0.0 ${PI}"` |
| ecm_j    | [ecmj.xacro](../../suj_description/urdf/xacros/Classic/ecmj.xacro)   | `parent_link:"base_link", xyz:"0 0.0896 0.7611", rpy:"${PI} 0 -${PI/2}"`                             |
| psm1_j   | [psm1j.xacro](../../suj_description/urdf/xacros/Classic/psm1j.xacro) | `parent_link:"base_link", xyz:"-0.191199999999999 -0.1016 0.761099999999999", rpy:"${PI} 0 0"`       |
| psm3_j   | [psm3j.xacro](../../suj_description/urdf/xacros/Classic/psm3j.xacro) | `parent_link:"base_link", xyz:"0 0.0895999999999999 0.3813", rpy:"${PI} 0 -${PI/2}"`                 |

### ROS2 Controller Macros

| Macro             | Source File Name                                                                           | Params       |
| ----------------- | ------------------------------------------------------------------------------------------ | ------------ |
| ecmj_ros2_control | [ecmj.ros2_control.xacro](../../suj_description/urdf/ros2_control/ecmj.ros2_control.xacro) | N/A          |
| psmj_ros2_control | [psmj.ros2_control.xacro](../../suj_description/urdf/ros2_control/psmj.ros2_control.xacro) | `prefix:"1"` |
| psmj_ros2_control | [psmj.ros2_control.xacro](../../suj_description/urdf/ros2_control/psmj.ros2_control.xacro) | `prefix:"2"` |
| psmj_ros2_control | [psmj.ros2_control.xacro](../../suj_description/urdf/ros2_control/psmj.ros2_control.xacro) | `prefix:"3"` |

### ROS2 Control Hardware: fake_components/GenericSystem

### Links

| Link Number | Link Name | Mesh File                                                       | Origin (`xyz`) | Orientation (`rpy`) |
| ----------- | --------- | --------------------------------------------------------------- | -------------- | ------------------- |
| Link 0      | World     | N/A                                                             | N/A            | N/A                 |
| Link 1      | Base Link | [Base Link](../../suj_description/meshes/Classic/base_link.stl) | `0 0 0`        | `0 0 0`             |

### Joints

| Joint Number | Joint Name | Parent | Child     | Type  | Origin (`xyz`) | Orientation (`rpy`) |
| ------------ | ---------- | ------ | --------- | ----- | -------------- | ------------------- |
| Joint 0      | fixed      | World  | Base Link | Fixed | `0 0 0.167458` | `0 0 0`             |

---

# suj.si.urdf.xacro

## Robot: suj_si

### Included URDF Macros

| Macro    | Source File Name                                                  | Params                                                                           |
| -------- | ----------------------------------------------------------------- | -------------------------------------------------------------------------------- |
| property | N/A                                                               | `name:"PI", value:"3.1415926535897931"`                                          |
| psm12_j  | [psm12j.xacro](../../suj_description/urdf/xacros/Si/psm12j.xacro) | `prefix:"1", parent_link:"SUJ_column", xyz:"0 0.228 0.528", rpy:"0 0 ${PI/2}"`   |
| psm12_j  | [psm12j.xacro](../../suj_description/urdf/xacros/Si/psm12j.xacro) | `prefix:"2", parent_link:"SUJ_column", xyz:"0 -0.228 0.528", rpy:"0 0 -${PI/2}"` |
| psm3_j   | [psm3j.xacro](../../suj_description/urdf/xacros/Si/psm3j.xacro)   | `parent_link:"SUJ_column", xyz:"-0.223 0 0.528", rpy:"0 0 -${PI}"`               |
| ecm_j    | [ecmj.xacro](../../suj_description/urdf/xacros/Si/ecmj.xacro)     | `parent_link:"SUJ_column", xyz:"0.223 0 0.528", rpy:"0 0 0"`                     |

### ROS2 Controller Macros

| Macro             | Source File Name                                                                           | Params       |
| ----------------- | ------------------------------------------------------------------------------------------ | ------------ |
| ecmj_ros2_control | [ecmj.ros2_control.xacro](../../suj_description/urdf/ros2_control/ecmj.ros2_control.xacro) | N/A          |
| psmj_ros2_control | [psmj.ros2_control.xacro](../../suj_description/urdf/ros2_control/psmj.ros2_control.xacro) | `prefix:"1"` |
| psmj_ros2_control | [psmj.ros2_control.xacro](../../suj_description/urdf/ros2_control/psmj.ros2_control.xacro) | `prefix:"2"` |
| psmj_ros2_control | [psmj.ros2_control.xacro](../../suj_description/urdf/ros2_control/psmj.ros2_control.xacro) | `prefix:"3"` |

### ROS2 Control Hardware: fake_components/GenericSystem

### Links

| Link Number | Link Name  | Mesh File                                          | Origin (`xyz`) | Orientation (`rpy`) |
| ----------- | ---------- | -------------------------------------------------- | -------------- | ------------------- |
| Link 0      | World      | N/A                                                | N/A            | N/A                 |
| Link 1      | SUJ Column | [Tower](../../suj_description/meshes/Si/tower.stl) | `0 0 0`        | `0 0 0`             |

### Joints

| Joint Number | Joint Name  | Parent | Child      | Type  | Origin (`xyz`) | Orientation (`rpy`) |
| ------------ | ----------- | ------ | ---------- | ----- | -------------- | ------------------- |
| Joint 0      | World Fixed | World  | SUJ Column | Fixed | `0 0 0`        | `0 0 0`             |

---
