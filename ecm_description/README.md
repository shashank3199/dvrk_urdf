# ECM: Endoscope Camera Manipulator

The ECM (Endoscope Camera Manipulator) model includes the full manipulator with its joints and links. Below is the detailed description, including the links, joints, mesh files, and their respective origins and orientations.

ECM

```bash
ros2 launch ecm_description view_robot.launch.py file:=ecm.urdf.xacro
```

```bash
ros2 launch ecm_description ecm_base_bringup.launch.py
```

ECM Base

```bash
ros2 launch ecm_description view_robot.launch.py file:=ecm.base.urdf.xacro
````

```bash
ros2 launch ecm_description ecm_bringup.launch.py
```

---

## ECM Xacro

### Links

| Link Number | Link Name       | Mesh File                                           | Origin (`xyz`)  | Orientation (`rpy`) |
| ----------- | --------------- | --------------------------------------------------- | --------------- | ------------------- |
| Link 0      | Setup Base Link | N/A                                                 | N/A             | N/A                 |
| Link 1      | Setup Link      | [Endo_Arm.stl](./meshes/ecm/Endo_Arm.stl)           | `0 0 0`         | `0 0 0`             |
| Link 2      | Base Link       | N/A                                                 | N/A             | N/A                 |
| Link 3      | Yaw Link        | [Endo_Link_5.stl](./meshes/ecm/Endo_Link_5.stl)     | `0.0 0 0.603`   | `0 0 -1.5708`       |
| Link 4      | Pitch Link      | N/A                                                 | N/A             | N/A                 |
| Link 5      | Pitch Link 1    | [Endo_Link_5_2.stl](./meshes/ecm/Endo_Link_5_2.stl) | `0.0 0 0.0`     | `3.1416 0 1.3708`   |
| Link 6      | Pitch Link 2    | [Endo_Link_6_2.stl](./meshes/ecm/Endo_Link_6_2.stl) | `0.0 0 -0.0225` | `0 0 0`             |
| Link 7      | Pitch Link 3    | [Endo_Link_7.stl](./meshes/ecm/Endo_Link_7.stl)     | `0.0 0 0.025`   | `0 0 -1.5708`       |
| Link 8      | Insertion Link  | [Endo_Link_8.stl](./meshes/ecm/Endo_Link_8.stl)     | `0.0 0 -0.015`  | `0 0 -1.5708`       |
| Link 9      | Roll Link       | [EndoScope.stl](./meshes/ecm/EndoScope.stl)         | `0.0 0 -0.3979` | `0 0 1.5708`        |

### Joints

| Joint Number | Joint Name          | Associated Links (Parent to Child) | Type       | Origin (`xyz`)             | Orientation (`rpy`)     |
| ------------ | ------------------- | ---------------------------------- | ---------- | -------------------------- | ----------------------- |
| Joint 0      | Setup Fixed Joint   | Macro Parent Link to Link 0        | Fixed      | As per macro parameters    | As per macro parameters |
| Joint 1      | Setup Tip Joint     | Link 0 to Link 1                   | Fixed      | `0.91324 -0.02111 0.03166` | `-0.870464 0 -1.5708`   |
| Joint 2      | Fixed Joint         | Link 1 to Link 2                   | Fixed      | `0 0 0`                    | `0 0 0`                 |
| Joint 3      | Outer Yaw           | Link 2 to Link 3                   | Revolute   | `0 0 0`                    | `0 -1.5708 1.5708`      |
| Joint 4      | Outer Pitch         | Link 3 to Link 4                   | Revolute   | `0 0 0`                    | `-1.5708 -1.5708 0`     |
| Joint 5      | Outer Pitch Joint 0 | Link 3 to Link 5                   | Continuous | `0 0 0.378`                | `-1.5708 -1.5708 0`     |
| Joint 6      | Outer Pitch Joint 1 | Link 5 to Link 6                   | Continuous | `0.0558629 0.274575 0`     | `0 0 0`                 |
| Joint 7      | Outer Pitch Joint 2 | Link 6 to Link 7                   | Continuous | `-0.340 0 0`               | `0 0 0`                 |
| Joint 8      | Insertion           | Link 4 to Link 8                   | Prismatic  | `0 0.3822 0`               | `1.5708 0 0`            |
| Joint 9      | Outer Roll          | Link 8 to Link 9                   | Revolute   | `0 0 0.3829`               | `0 0 0`                 |

---

**Note:** In Joint 0, the `xyz` and `rpy` values represent the mounting position and orientation of the ECM, defined by macro parameters:

-   **ECM (`ecm`):**
    -   **Parent Link:** `world`
    -   **Position (`xyz`):** `0 0.0 0.0`
    -   **Orientation (`rpy`):** `0.0 0.0 0.0`

---

## ECM Base Xacro

The ECM Base model includes the base structure of the ECM manipulator. Below is the detailed description, including the links, joints, mesh files, and their respective origins and orientations.

### Links

| Link Number | Link Name           | Mesh File                                                                    | Origin (`xyz`) | Orientation (`rpy`) |
| ----------- | ------------------- | ---------------------------------------------------------------------------- | -------------- | ------------------- |
| Link 0      | Base Link           | [ecm_base_link.stl](./meshes/ecm_base/ecm_base_link.stl)                     | `0 0 0`        | `0 0 0`             |
| Link 1      | Yaw Link            | [ecm_yaw_link.stl](./meshes/ecm_base/ecm_yaw_link.stl)                       | `0 0 0`        | `0 0 0`             |
| Link 2      | Pitch Front Link    | [ecm_pitch_front_link.stl](./meshes/ecm_base/ecm_pitch_front_link.stl)       | `0 0 0`        | `0 0 0`             |
| Link 3      | Pitch Bottom Link   | [ecm_pitch_bottom_link.stl](./meshes/ecm_base/ecm_pitch_bottom_link.stl)     | `0 0 0`        | `0 0 0`             |
| Link 4      | Pitch End Link      | [ecm_pitch_end_link.stl](./meshes/ecm_base/ecm_pitch_end_link.stl)           | `0 0 0`        | `0 0 0`             |
| Link 5      | Main Insertion Link | [ecm_main_insertion_link.stl](./meshes/ecm_base/ecm_main_insertion_link.stl) | `0 0 0`        | `0 0 0`             |
| Link 6      | Tool Link           | [ecm_tool_link.stl](./meshes/ecm_base/ecm_tool_link.stl)                     | `0 0 0`        | `0 0 0`             |
| Link 7      | Pitch Top Link      | [ecm_pitch_top_link.stl](./meshes/ecm_base/ecm_pitch_top_link.stl)           | `0 0 0`        | `0 0 0`             |
| Link 8      | Pitch Back Link     | [ecm_pitch_back_link.stl](./meshes/ecm_base/ecm_pitch_back_link.stl)         | `0 0 0`        | `0 0 0`             |
| Link 9      | Remote Center Link  | [ecm_remote_center_link.stl](./meshes/ecm_base/ecm_remote_center_link.stl)   | `0 0 0`        | `0 0 0`             |
| Link 10     | End Link            | N/A                                                                          | N/A            | N/A                 |

### Joints

| Joint Number | Joint Name          | Associated Links (Parent to Child)    | Type      | Origin (`xyz`)          | Orientation (`rpy`)     |
| ------------ | ------------------- | ------------------------------------- | --------- | ----------------------- | ----------------------- |
| Joint 0      | Fixed Joint         | Parent Link to Base Link              | Fixed     | As per macro parameters | As per macro parameters |
| Joint 1      | Yaw Joint           | Base Link to Yaw Link                 | Revolute  | `0.0732 0 0.10146`      | `1.5708 0 1.5708`       |
| Joint 2      | Pitch Joint         | Yaw Link to Pitch Front Link          | Revolute  | `0 0 0.199`             | `3.1416 -1.5708 0`      |
| Joint 3      | Pitch Bottom Joint  | Pitch Front Link to Pitch Bottom Link | Revolute  | `-0.1030 -0.2868 0`     | `0 0 0`                 |
| Joint 4      | Pitch End Joint     | Pitch Bottom Link to Pitch End Link   | Revolute  | `0.3404 -0.0001356 0`   | `0 0 0`                 |
| Joint 5      | Insertion Joint     | Pitch End Link to Main Insertion Link | Prismatic | `0.042 -0.086143 0`     | `-1.5708 0 0`           |
| Joint 6      | Roll Joint          | Main Insertion Link to Tool Link      | Revolute  | `0.061 0 0`             | `0 0 0`                 |
| Joint 7      | Pitch Top Joint     | Pitch Front Link to Pitch Top Link    | Revolute  | `-0.1085 -0.3243 0`     | `0 0 0`                 |
| Joint 8      | Pitch Back Joint    | Yaw Link to Pitch Back Link           | Revolute  | `0 -0.0098 0.16243`     | `3.1416 -1.5708 0`      |
| Joint 9      | Remote Center Joint | Base Link to Remote Center Link       | Fixed     | `0.6126 0 0.1016`       | `0 0 -1.5708`           |
| Joint 10     | End Joint           | Tool Link to End Link                 | Fixed     | `0 0 0.37364`           | `0 0 0`                 |

---
