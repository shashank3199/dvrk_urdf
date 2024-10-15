# daVinci.urdf.xacro

## Robot: daVinci

### Included URDF Macros

| Macro       | Source File Name                                | Params                                                                                    |
| ----------- | ----------------------------------------------- | ----------------------------------------------------------------------------------------- |
| daVinci_arm | [daVinci.arm.xacro](./xacros/daVinci.arm.xacro) | `arm_name:"left_arm", parent:"mounting_platform", X:"0.38114", Y:"0.48531", Z:"1.60767"`  |
| daVinci_arm | [daVinci.arm.xacro](./xacros/daVinci.arm.xacro) | `arm_name:"right_arm", parent:"mounting_platform", X:"0.38114", Y:"1.03237", Z:"1.60767"` |

### ROS2 Controller Macros

| Macro                     | Source File Name                                                               | Params                 |
| ------------------------- | ------------------------------------------------------------------------------ | ---------------------- |
| davinci_core_ros2_control | [davinci_core_ros2_control.xacro](ros2_control/daVinci.arm.ros2_control.xacro) | `arm_name:"left_arm"`  |
| davinci_core_ros2_control | [davinci_core_ros2_control.xacro](ros2_control/daVinci.arm.ros2_control.xacro) | `arm_name:"right_arm"` |

### ROS2 Control Hardware: fake_components/GenericSystem

### Links

| Link Number | Link Name         | Mesh File                                | Origin (`xyz`) | Orientation (`rpy`) |
| ----------- | ----------------- | ---------------------------------------- | -------------- | ------------------- |
| Link 0      | Parent Link       | N/A                                      | N/A            | N/A                 |
| Link 1      | Mounting Platform | [Slave Frame](../meshes/slave_frame.stl) | `0 0 0`        | `0 0 0`             |

### Joints

| Joint Number | Joint Name        | Parent      | Child             | Type  | Origin (`xyz`) | Orientation (`rpy`) |
| ------------ | ----------------- | ----------- | ----------------- | ----- | -------------- | ------------------- |
| Joint 0      | Mounting Platform | Parent Link | Mounting Platform | Fixed | `0 -0.762 0`   | `0 0 0`             |

---
