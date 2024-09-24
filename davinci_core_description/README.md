# daVinci Core Description

The daVinci surgical system model includes two arms (left and right). Below is the detailed description for each arm, including the links, joints, mesh files, and their respective origins and orientations.

```bash
ros2 launch davinci_core_description urdf.launch.py file:=daVinci.urdf.xacro
```
---

## daVinci Arm Xacro

### Links

| Link Number | Link Name                           | Mesh File                                                 | Origin (`xyz`)          | Orientation (`rpy`) |
|-------------|-------------------------------------|-----------------------------------------------------------|-------------------------|---------------------|
| Link -1     | Mounting Platform                   | [slave_frame.stl](./meshes/slave_frame.stl)               | `0 0 0`                 | `0 0 0`             |
| Link 0      | Mounting Base Link                  | [mounting_base.stl](./meshes/mounting_base.stl)           | `0 0 0`                 | `0 0 0`             |
| Link 1      | Outer Yaw Link                      | [outer_yaw.stl](./meshes/outer_yaw.stl)                   | `0.001 -0.1 -0.1375`    | `0 0 0`             |
| Link 2      | Outer Pitch Base Link               | [outer_pitch_base.stl](./meshes/outer_pitch_base.stl)     | `-0.040 -0.130 -0.0901` | `0 0 0`             |
| Link 3      | Outer Pitch Front Link              | [outer_pitch_front.stl](./meshes/outer_pitch_front.stl)   | `0 -0.066 -0.010`       | `0 0 0`             |
| Link 4      | Outer Pitch Bottom Link             | [outer_pitch_bottom.stl](./meshes/outer_pitch_bottom.stl) | `0 -0.093 -0.010`       | `0 0 0`             |
| Link 5      | Outer Pitch Top Link                | [outer_pitch_top.stl](./meshes/outer_pitch_top.stl)       | `0 -0.093 -0.010`       | `0 0 0`             |
| Link 6      | Outer Insertion Link                | [outer_insertion.stl](./meshes/outer_insertion.stl)       | `-0.031 -0.086 -0.070`  | `0 0 0`             |
| Link 7      | Tool Adaptor Link                   | [tool_adaptor.stl](./meshes/tool_adaptor.stl)             | `0 -0.041 0`            | `0 0 0`             |
| Link 8      | Tool Link                           | [tool_asm.stl](./meshes/tool_asm.stl)                     | `0 -0.034 0`            | `0 0 0`             |

### Joints

| Joint Number | Joint Name               | Associated Links (Parent to Child)     | Type        | Origin (`xyz`)           | Orientation (`rpy`) |
|--------------|--------------------------|----------------------------------------|-------------|--------------------------|---------------------|
| Joint -1     | Wall Joint               | `world` to Mounting Platform           | Fixed       | `0 -0.762 0`             | `0 0 0`             |
| Joint 0      | Mounting Base Joint      | Mounting Platform to Link 0            | Fixed       | `X Y Z`                  | `0 0 0`             |
| Joint 1      | Outer Yaw Joint          | Link 0 to Link 1                       | Continuous  | `0.094 0.0525 0.147`     | `0 0 0`             |
| Joint 2      | Outer Pitch Base Joint   | Link 1 to Link 2                       | Continuous  | `0.150 0.085 0`          | `0 0 0`             |
| Joint 3      | Outer Pitch Front Joint  | Link 1 to Link 3                       | Continuous  | `0.245 0.085 0`          | `0 0 0`             |
| Joint 4      | Outer Pitch Bottom Joint | Link 2 to Link 4                       | Continuous  | `-0.03 -0.040 0.147`     | `0 0 0`             |
| Joint 5      | Outer Pitch Top Joint    | Link 2 to Link 5                       | Continuous  | `-0.03 -0.040 0.185`     | `0 0 0`             |
| Joint 6      | Outer Insertion Joint    | Link 4 to Link 6                       | Continuous  | `0.510 0.003 0`          | `0 0 0`             |
| Joint 7      | Tool Insertion Joint     | Link 6 to Link 7                       | Prismatic   | `0.005 -0.048 -0.200`    | `0 0 0`             |
| Joint 8      | Tool Joint               | Link 7 to Link 8                       | Fixed       | `0.010 0 0`              | `0 0 0`             |

**Note:** In Joint 0, `X`, `Y`, and `Z` represent the mounting position of each arm:

- **Left Arm:**
  - `X = 0.38114`
  - `Y = 0.48531`
  - `Z = 1.60767`
- **Right Arm:**
  - `X = 0.38114`
  - `Y = 1.03237`
  - `Z = 1.60767`

---
