# both_psms.classic.urdf.xacro

## Robot: both_psms

### Included URDF Macros

| Macro    | Source File Name                                                           | Params                                                                         |
| -------- | -------------------------------------------------------------------------- | ------------------------------------------------------------------------------ |
| psm_base | [psm.base.xacro](../../psm_description/urdf/xacros/Classic/psm.base.xacro) | `prefix:"one", parent_link:"camera", xyz:"-0.25 0.0 0.5", rpy:"0.0 0.0 ${PI}"` |
| psm_base | [psm.base.xacro](../../psm_description/urdf/xacros/Classic/psm.base.xacro) | `prefix:"two", parent_link:"camera", xyz:"0.25 0.0 0.5", rpy:"0.0 0.0 ${PI}"`  |

### ROS2 Controller Macros

| Macro                 | Source File Name                                                                                           | Params         |
| --------------------- | ---------------------------------------------------------------------------------------------------------- | -------------- |
| psm_base_ros2_control | [psm.base.ros2_control.xacro](../../psm_description/urdf/ros2_control/Classic/psm.base.ros2_control.xacro) | `prefix:"one"` |
| psm_base_ros2_control | [psm.base.ros2_control.xacro](../../psm_description/urdf/ros2_control/Classic/psm.base.ros2_control.xacro) | `prefix:"two"` |

### ROS2 Control Hardware: fake_components/GenericSystem

### Links

| Link Number | Link Name   | Mesh File | Origin (`xyz`) | Orientation (`rpy`) |
| ----------- | ----------- | --------- | -------------- | ------------------- |
| Link 0      | Parent Link | N/A       | N/A            | N/A                 |
| Link 1      | Camera Link | N/A       | N/A            | N/A                 |

### Joints

| Joint Number | Joint Name  | Parent      | Child       | Type  | Origin (`xyz`) | Orientation (`rpy`) |
| ------------ | ----------- | ----------- | ----------- | ----- | -------------- | ------------------- |
| Joint 0      | World Joint | Parent Link | Camera Link | Fixed | `0.0 0.0 0.0`  | `0.0 0.0 0.0`       |

---

# psm.classic.urdf.xacro

## Robot: psm

### Included URDF Macros

| Macro    | Source File Name                                                           | Params                                                                                        |
| -------- | -------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------- |
| arg      | N/A                                                                        | `name:"tool", default:"sca"`                                                                  |
| property | N/A                                                                        | `name:"tool_name", default:"'$(arg tool)'"`                                                   |
| property | N/A                                                                        | `name:"prefix", default:"PSM"`                                                                |
| property | N/A                                                                        | `name:"parent_link", default:"world"`                                                         |
| if       | N/A                                                                        | `value:"${tool_name == 'caudier'}"`                                                           |
| unless   | N/A                                                                        | `value:"${tool_name == 'caudier'}"`                                                           |
| psm_base | [psm.base.xacro](../../psm_description/urdf/xacros/Classic/psm.base.xacro) | `prefix:"${prefix}", parent_link:"${parent_link}", xyz:"${xyz}", rpy:"${rpy}"`                |
| psm_tool | [psm.tool.xacro](../../psm_description/urdf/xacros/Classic/psm.tool.xacro) | `tool_name:"${tool_name}", prefix:"${prefix}", parent_link:"${prefix}_tool_wrist_shaft_link"` |

### ROS2 Controller Macros

| Macro                 | Source File Name                                                                                           | Params                                         |
| --------------------- | ---------------------------------------------------------------------------------------------------------- | ---------------------------------------------- |
| psm_base_ros2_control | [psm.base.ros2_control.xacro](../../psm_description/urdf/ros2_control/Classic/psm.base.ros2_control.xacro) | `prefix:"${prefix}"`                           |
| psm_tool_ros2_control | [psm.tool.ros2_control.xacro](../../psm_description/urdf/ros2_control/Classic/psm.tool.ros2_control.xacro) | `tool_name:"${tool_name}", prefix:"${prefix}"` |

### ROS2 Control Hardware: fake_components/GenericSystem

### Links

| Link Number | Link Name      | Mesh File | Origin (`xyz`) | Orientation (`rpy`) |
| ----------- | -------------- | --------- | -------------- | ------------------- |
| Link 0      | ${parent_link} | N/A       | N/A            | N/A                 |

---

# psm.si.urdf.xacro

## Robot: psm

### Included URDF Macros

| Macro    | Source File Name                                                      | Params                                                                                                          |
| -------- | --------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------- |
| arg      | N/A                                                                   | `name:"tool", default:"P420006"`                                                                                |
| property | N/A                                                                   | `name:"tool_name", default:"'$(arg tool)'"`                                                                     |
| property | N/A                                                                   | `name:"prefix", default:"PSM"`                                                                                  |
| property | N/A                                                                   | `name:"parent_link", default:"world"`                                                                           |
| property | N/A                                                                   | `name:"xyz", default:"0.0 0.0 0.0"`                                                                             |
| property | N/A                                                                   | `name:"rpy", default:"0.0 0.0 0.0"`                                                                             |
| psm_base | [psm.base.xacro](../../psm_description/urdf/xacros/Si/psm.base.xacro) | `prefix:"${prefix}", parent_link:"${parent_link}", xyz:"${xyz}", rpy:"${rpy}"`                                  |
| psm_tool | [psm.tool.xacro](../../psm_description/urdf/xacros/Si/psm.tool.xacro) | `tool_name:"${tool_name}", prefix:"${prefix}", parent_link:"${prefix}_tool_parent", xyz:"${xyz}", rpy:"${rpy}"` |

### ROS2 Controller Macros

| Macro                 | Source File Name                                                                                      | Params                                               |
| --------------------- | ----------------------------------------------------------------------------------------------------- | ---------------------------------------------------- |
| psm_base_ros2_control | [psm.base.ros2_control.xacro](../../psm_description/urdf/ros2_control/Si/psm.base.ros2_control.xacro) | {'prefix': '${prefix}'}                              |
| psm_tool_ros2_control | [psm.tool.ros2_control.xacro](../../psm_description/urdf/ros2_control/Si/psm.tool.ros2_control.xacro) | {'tool_name': '${tool_name}', 'prefix': '${prefix}'} |

### ROS2 Control Hardware: fake_components/GenericSystem

### Links

| Link Number | Link Name      | Mesh File | Origin (`xyz`) | Orientation (`rpy`) |
| ----------- | -------------- | --------- | -------------- | ------------------- |
| Link 0      | ${parent_link} | N/A       | N/A            | N/A                 |

---
