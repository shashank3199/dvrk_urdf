# patient_cart.classic.urdf.xacro

## Robot: patient_cart_classic

### Included URDF Macros

| Macro    | Source File Name                                                           | Params                                                                                               |
| -------- | -------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------- |
| psm2_j   | [psm2j.xacro](../../suj_description/urdf/xacros/Classic/psm2j.xacro)       | `parent_link:"base_link", xyz:"0.1912 -0.101599999999999 0.761099999999999", rpy:"-${PI} 0.0 ${PI}"` |
| ecm_j    | [ecmj.xacro](../../suj_description/urdf/xacros/Classic/ecmj.xacro)         | `parent_link:"base_link", xyz:"0 0.0896 0.7611", rpy:"${PI} 0 -${PI/2}"`                             |
| psm1_j   | [psm1j.xacro](../../suj_description/urdf/xacros/Classic/psm1j.xacro)       | `parent_link:"base_link", xyz:"-0.191199999999999 -0.1016 0.761099999999999", rpy:"${PI} 0 0"`       |
| psm3_j   | [psm3j.xacro](../../suj_description/urdf/xacros/Classic/psm3j.xacro)       | `parent_link:"base_link", xyz:"0 0.0895999999999999 0.3813", rpy:"${PI} 0 -${PI/2}"`                 |
| psm_base | [psm.base.xacro](../../psm_description/urdf/xacros/Classic/psm.base.xacro) | `prefix:"PSM2", parent_link:"PSM2_mounting_point", xyz:"0.0 0.0 0.0", rpy:"0.0 0.0 0.0"`             |
| psm_tool | [psm.tool.xacro](../../psm_description/urdf/xacros/Classic/psm.tool.xacro) | `tool_name:"caudier", prefix:"PSM2", parent_link:"PSM2_tool_wrist_shaft_link"`                       |
| ecm_base | [ecm.base.xacro](../../ecm_description/urdf/xacros/ecm.base.xacro)         | `prefix:"ECM", parent_link:"ECM_mounting_point", xyz:"-0.625 0.0 -0.125", rpy:"0.0 ${PI} ${PI}"`     |
| psm_base | [psm.base.xacro](../../psm_description/urdf/xacros/Classic/psm.base.xacro) | `prefix:"PSM1", parent_link:"PSM1_mounting_point", xyz:"0.0 0.0 0.0", rpy:"0.0 0.0 0.0"`             |
| psm_tool | [psm.tool.xacro](../../psm_description/urdf/xacros/Classic/psm.tool.xacro) | `tool_name:"sca", prefix:"PSM1", parent_link:"PSM1_tool_wrist_shaft_link"`                           |
| psm_base | [psm.base.xacro](../../psm_description/urdf/xacros/Classic/psm.base.xacro) | `prefix:"PSM3", parent_link:"PSM3_mounting_point", xyz:"0.0 0.0 0.0", rpy:"0.0 0.0 0.0"`             |
| psm_tool | [psm.tool.xacro](../../psm_description/urdf/xacros/Classic/psm.tool.xacro) | `tool_name:"caudier", prefix:"PSM3", parent_link:"PSM3_tool_wrist_shaft_link"`                       |

### ROS2 Controller Macros

| Macro                 | Source File Name                                                                                           | Params                               |
| --------------------- | ---------------------------------------------------------------------------------------------------------- | ------------------------------------ |
| ecm_base_ros2_control | [ecm.base.ros2_control.xacro](../../ecm_description/urdf/ros2_control/ecm.base.ros2_control.xacro)         | `prefix:"ECM"`                       |
| ecmj_ros2_control     | [ecmj.ros2_control.xacro](../../suj_description/urdf/ros2_control/Classic/ecmj.ros2_control.xacro)         | N/A                                  |
| psm_base_ros2_control | [psm.base.ros2_control.xacro](../../psm_description/urdf/ros2_control/Classic/psm.base.ros2_control.xacro) | `prefix:"PSM1"`                      |
| psm_tool_ros2_control | [psm.tool.ros2_control.xacro](../../psm_description/urdf/ros2_control/Classic/psm.tool.ros2_control.xacro) | `tool_name:"sca", prefix:"PSM1"`     |
| psmj_ros2_control     | [psmj.ros2_control.xacro](../../suj_description/urdf/ros2_control/Classic/psmj.ros2_control.xacro)         | `prefix:"1"`                         |
| psm_base_ros2_control | [psm.base.ros2_control.xacro](../../psm_description/urdf/ros2_control/Classic/psm.base.ros2_control.xacro) | `prefix:"PSM2"`                      |
| psm_tool_ros2_control | [psm.tool.ros2_control.xacro](../../psm_description/urdf/ros2_control/Classic/psm.tool.ros2_control.xacro) | `tool_name:"caudier", prefix:"PSM2"` |
| psmj_ros2_control     | [psmj.ros2_control.xacro](../../suj_description/urdf/ros2_control/Classic/psmj.ros2_control.xacro)         | `prefix:"2"`                         |
| psm_base_ros2_control | [psm.base.ros2_control.xacro](../../psm_description/urdf/ros2_control/Classic/psm.base.ros2_control.xacro) | `prefix:"PSM3"`                      |
| psm_tool_ros2_control | [psm.tool.ros2_control.xacro](../../psm_description/urdf/ros2_control/Classic/psm.tool.ros2_control.xacro) | `tool_name:"caudier", prefix:"PSM3"` |
| psmj_ros2_control     | [psmj.ros2_control.xacro](../../suj_description/urdf/ros2_control/Classic/psmj.ros2_control.xacro)         | `prefix:"3"`                         |

### ROS2 Control Hardware: fake_components/GenericSystem

### Links

| Link Number | Link Name   | Mesh File                                                       | Origin (`xyz`) | Orientation (`rpy`) |
| ----------- | ----------- | --------------------------------------------------------------- | -------------- | ------------------- |
| Link 0      | Parent Link | N/A                                                             | N/A            | N/A                 |
| Link 1      | Base Link   | [Base Link](../../suj_description/meshes/Classic/base_link.stl) | `0 0 0`        | `0 0 0`             |

### Joints

| Joint Number | Joint Name | Parent      | Child     | Type  | Origin (`xyz`) | Orientation (`rpy`) |
| ------------ | ---------- | ----------- | --------- | ----- | -------------- | ------------------- |
| Joint 0      | fixed      | Parent Link | Base Link | Fixed | `0 0 0.167458` | `0 0 0`             |

---
