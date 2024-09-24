# MTM: Master Tool Manipulator

The MTM (Master Tool Manipulator) model includes both left and right manipulators, defined using macros in the Xacro files. Below is the detailed description for each arm, including the links, joints, mesh files, and their respective origins and orientations.

```bash
ros2 launch mtm_description urdf.launch.py file:=mtms.urdf.xacro
```

---

## MTM Xacro

### Links

| Link Number | Link Name                           | Mesh File                                                    | Origin (`xyz`)       | Orientation (`rpy`)    |
|-------------|-------------------------------------|--------------------------------------------------------------|----------------------|------------------------|
| Link 0      | Top Panel Link                      | [TopPanel.stl](./meshes/TopPanel.stl)                        | `0 0 0.19037`        | `0 0 0`                |
| Link 1      | Outer Yaw Link                      | [OutPitch_Shoulder.stl](./meshes/OutPitch_Shoulder.stl)      | `0.025 0 0.19037`    | `0 0 0`                |
| Link 2      | Back Parallel Link                  | [ArmParallel.stl](./meshes/ArmParallel.stl)                  | `0 0 0`              | `0 0 0`                |
| Link 3      | Top Parallel Link                   | [ArmParallel1.stl](./meshes/ArmParallel1.stl)                | `0 0 0.065`          | `0 0 -1.5708`          |
| Link 4      | Bottom Parallel Link                | [BottomArm.stl](./meshes/BottomArm.stl)                      | `0 0 0`              | `0 0 0`                |
| Link 5      | Wrist Platform Link                 | [WristPlatform.stl](./meshes/WristPlatform.stl)              | `0 0 0`              | `0 0 0`                |
| Link 6      | Wrist Pitch Link                    | [WristPitch.stl](./meshes/WristPitch.stl)                    | `0 0 0`              | `0 0 0`                |
| Link 7      | Wrist Yaw Link                      | [WristYaw.stl](./meshes/WristYaw.stl)                        | `0 0 0`              | `0 0 0`                |
| Link 8      | Wrist Roll Link                     | [WristRoll.stl](./meshes/WristRoll.stl)                      | `0 0 0.039`          | `3.1416 0 0`           |
| Link 9      | Wrist Platform Link (with Origins)  | [WristPlatform.stl](./meshes/WristPlatform.stl)              | `0 0 0`              | `0 0 3.1416`           |
| Link 10     | Wrist Pitch Link (with Origins)     | [WristPitch.stl](./meshes/WristPitch.stl)                    | `0 0 0`              | `0 3.1416 0`           |
| Link 11     | Wrist Yaw Link (with Origins)       | [WristYaw.stl](./meshes/WristYaw.stl)                        | `0 0 0`              | `0 0 0`                |
| Link 12     | Wrist Roll Link (with Origins)      | [WristRoll.stl](./meshes/WristRoll.stl)                      | `0 0 0.039`          | `3.1416 0 0`           |

### Joints

| Joint Number  | Joint Name                | Associated Links (Parent to Child)        | Type      | Origin (`xyz`)           | Orientation (`rpy`)    |
|---------------|---------------------------|-------------------------------------------|-----------|--------------------------|------------------------|
| Joint 0       | Fixed to World            | Macro Parent Link to Link 0               | Fixed     | As per macro parameters  | As per macro parameters|
| Joint 1       | Outer Yaw                 | Link 0 to Link 1                          | Revolute  | `0 0 0`                  | `0 0 1.5708`           |
| Joint 2       | Shoulder Pitch            | Link 1 to Link 2                          | Revolute  | `0 0 0`                  | `-1.5708 -1.5708 0`    |
| Joint 3       | Shoulder Pitch Parallel   | Link 1 to Link 3                          | Revolute  | `0 0 0`                  | `-1.5708 -1.5708 0`    |
| Joint 4       | Elbow Pitch               | Link 2 to Link 4                          | Revolute  | `-0.2794 0 0`            | `0 0 1.5708`           |
| Joint 5       | Wrist Platform Joint      | Link 4 to Link 5                          | Revolute  | `-0.3645 -0.1506 0.0`    | `1.5708 0 0`           |
| Joint 6       | Wrist Pitch Joint         | Link 5 to Link 6                          | Revolute  | `0 0 0`                  | `-1.5708 0 0`          |
| Joint 7       | Wrist Yaw Joint           | Link 6 to Link 7                          | Revolute  | `0 0 0`                  | `1.5708 -1.5708 0`     |
| Joint 8       | Wrist Roll Joint          | Link 7 to Link 8                          | Revolute  | `0 0 0`                  | `0 -1.5708 1.5708`     |
| Joint 9       | Wrist Platform Joint      | Link 4 to Link 9                          | Revolute  | `-0.3645 -0.1506 0.0`    | `1.5708 0 0`           |
| Joint 10      | Wrist Pitch Joint         | Link 9 to Link 10                         | Revolute  | `0 0 0`                  | `-1.5708 0 0`          |
| Joint 11      | Wrist Yaw Joint           | Link 10 to Link 11                        | Revolute  | `0 0 0`                  | `1.5708 -1.5708 0`     |
| Joint 12      | Wrist Roll Joint          | Link 11 to Link 12                        | Revolute  | `0 0 0`                  | `0 -1.5708 1.5708`     |

---

**Note:** In Joint 0, the `xyz` and `rpy` values represent the mounting position and orientation of each arm, defined by macro parameters:

- **Left MTM (`MTML_`):**
  - **Parent Link:** `world`
  - **Position (`xyz`):** `-0.25 0.0 1.0`
  - **Orientation (`rpy`):** `0.0 0.0 0.0`

- **Right MTM (`MTMR_`):**
  - **Parent Link:** `world`
  - **Position (`xyz`):** `0.25 0.0 1.0`
  - **Orientation (`rpy`):** `0.0 0.0 0.0`

---
