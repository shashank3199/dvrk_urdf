# Davinci.Arm.Xacro

## Macro: daVinci_arm | Parameters: `<arm_name parent X Y Z>`

### Links

| Link Number | Link Name               | Mesh File                                                 | Origin (`xyz`)               | Orientation (`rpy`)          |
| ----------- | ----------------------- | --------------------------------------------------------- | ---------------------------- | ---------------------------- |
| Link 0      | Mounting Base Link      | [Mounting Bas](../../meshes/mounting_base.stl)            | `       0        0        0` | `       0        0        0` |
| Link 1      | Outer Yaw Link          | [Outer Yaw](../../meshes/outer_yaw.stl)                   | `   0.001     -0.1  -0.1375` | `       0        0        0` |
| Link 2      | Outer Pitch Base Link   | [Outer Pitch Bas](../../meshes/outer_pitch_base.stl)      | `  -0.040   -0.130  -0.0901` | `       0        0        0` |
| Link 3      | Outer Pitch Front Link  | [Outer Pitch Fron](../../meshes/outer_pitch_front.stl)    | `       0   -0.066   -0.010` | `       0        0        0` |
| Link 4      | Outer Pitch Bottom Link | [Outer Pitch Bottom](../../meshes/outer_pitch_bottom.stl) | `       0   -0.093   -0.010` | `       0        0        0` |
| Link 5      | Outer Pitch Top Link    | [Outer Pitch Top](../../meshes/outer_pitch_top.stl)       | `       0   -0.093   -0.010` | `       0        0        0` |
| Link 6      | Outer Insertion Link    | [Outer Insertion](../../meshes/outer_insertion.stl)       | `  -0.031   -0.086   -0.070` | `       0        0        0` |
| Link 7      | Tool Adaptor Link       | [Tool Adaptor](../../meshes/tool_adaptor.stl)             | `       0   -0.041        0` | `       0        0        0` |
| Link 8      | Tool Link               | [Tool Asm](../../meshes/tool_asm.stl)                     | `       0   -0.034        0` | `       0        0        0` |

### Joints

| Joint Number | Joint Name               | Parent                  | Child                   | Type       | Origin (`xyz`)               | Orientation (`rpy`)          |
| ------------ | ------------------------ | ----------------------- | ----------------------- | ---------- | ---------------------------- | ---------------------------- |
| Joint 0      | Mounting Base Joint      | Macro Parent            | Mounting Base Link      | Fixed      | `    ${X}     ${Y}     ${Z}` | `       0        0        0` |
| Joint 1      | Outer Yaw Joint          | Mounting Base Link      | Outer Yaw Link          | Continuous | `   0.094   0.0525    0.147` | `       0        0        0` |
| Joint 2      | Outer Pitch Base Joint   | Outer Yaw Link          | Outer Pitch Base Link   | Continuous | `   0.150    0.085        0` | `       0        0        0` |
| Joint 3      | Outer Pitch Front Joint  | Outer Yaw Link          | Outer Pitch Front Link  | Continuous | `   0.245    0.085        0` | `       0        0        0` |
| Joint 4      | Outer Pitch Bottom Joint | Outer Pitch Base Link   | Outer Pitch Bottom Link | Continuous | `   -0.03   -0.040    0.147` | `       0        0        0` |
| Joint 5      | Outer Pitch Top Joint    | Outer Pitch Base Link   | Outer Pitch Top Link    | Continuous | `   -0.03   -0.040    0.185` | `       0        0        0` |
| Joint 6      | Outer Insertion Joint    | Outer Pitch Bottom Link | Outer Insertion Link    | Continuous | `   0.510    0.003        0` | `       0        0        0` |
| Joint 7      | Tool Insertion Joint     | Outer Insertion Link    | Tool Adaptor Link       | Prismatic  | `   0.005   -0.048   -0.200` | `       0        0        0` |
| Joint 8      | Tool Joint               | Tool Adaptor Link       | Tool Link               | Fixed      | `   0.010        0        0` | `       0        0        0` |

---

**Note:** In Joint 0, `X`, `Y`, and `Z` represent the mounting position of each arm:

-   **Left Arm:**
    -   `X = 0.38114`
    -   `Y = 0.48531`
    -   `Z = 1.60767`
-   **Right Arm:**
    -   `X = 0.38114`
    -   `Y = 1.03237`
    -   `Z = 1.60767`
