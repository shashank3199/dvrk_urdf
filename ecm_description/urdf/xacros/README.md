# Ecm.Base.Xacro

## Macro: ecm_base | Parameters: `<prefix parent_link xyz:='-0.612599 0.0 -0.101595' rpy:='0.0 0.0 0.0'>`

### Links

| Link Number | Link Name           | Mesh File                                                                             | Origin (`xyz`)               | Orientation (`rpy`)          |
| ----------- | ------------------- | ------------------------------------------------------------------------------------- | ---------------------------- | ---------------------------- |
| Link 0      | Base Link           | [Ecm Base/ecm Base Link](../../meshes/ecm_base/ecm_base_link.stl)                     | `       0        0        0` | `       0        0        0` |
| Link 1      | Yaw Link            | [Ecm Base/ecm Yaw Link](../../meshes/ecm_base/ecm_yaw_link.stl)                       | `       0        0        0` | `       0        0        0` |
| Link 2      | Pitch Front Link    | [Ecm Base/ecm Pitch Front Link](../../meshes/ecm_base/ecm_pitch_front_link.stl)       | `       0        0        0` | `       0        0        0` |
| Link 3      | Pitch Bottom Link   | [Ecm Base/ecm Pitch Bottom Link](../../meshes/ecm_base/ecm_pitch_bottom_link.stl)     | `       0        0        0` | `       0        0        0` |
| Link 4      | Pitch End Link      | [Ecm Base/ecm Pitch End Link](../../meshes/ecm_base/ecm_pitch_end_link.stl)           | `       0        0        0` | `       0        0        0` |
| Link 5      | Main Insertion Link | [Ecm Base/ecm Main Insertion Link](../../meshes/ecm_base/ecm_main_insertion_link.stl) | `       0        0        0` | `       0        0        0` |
| Link 6      | Tool Link           | [Ecm Base/ecm Tool Link](../../meshes/ecm_base/ecm_tool_link.stl)                     | `       0        0        0` | `       0        0        0` |
| Link 7      | Pitch Top Link      | [Ecm Base/ecm Pitch Top Link](../../meshes/ecm_base/ecm_pitch_top_link.stl)           | `       0        0        0` | `       0        0        0` |
| Link 8      | Pitch Back Link     | [Ecm Base/ecm Pitch Back Link](../../meshes/ecm_base/ecm_pitch_back_link.stl)         | `       0        0        0` | `       0        0        0` |
| Link 9      | Remote Center Link  | [Ecm Base/ecm Remote Center Link](../../meshes/ecm_base/ecm_remote_center_link.stl)   | `       0        0        0` | `       0        0        0` |
| Link 10     | End Link            | [Ecm Base/ecm Remote Center Link](../../meshes/ecm_base/ecm_remote_center_link.stl)   | `       0        0        0` | `       0        0        0` |

### Joints

| Joint Number | Joint Name           | Parent              | Child               | Type      | Origin (`xyz`)               | Orientation (`rpy`)          |
| ------------ | -------------------- | ------------------- | ------------------- | --------- | ---------------------------- | ---------------------------- |
| Joint 0      | Fixed to Setup Joint | Macro Parent        | Base Link           | Fixed     | `  ${xyz}`                   | `  ${rpy}`                   |
| Joint 1      | Yaw Joint            | Base Link           | Yaw Link            | Revolute  | `0.073199        0 0.101460` | ` ${PI/2}        0  ${PI/2}` |
| Joint 2      | Pitch Front Joint    | Yaw Link            | Pitch Front Link    | Revolute  | `       0        0 0.199000` | `   ${PI} -${PI/2}        0` |
| Joint 3      | Pitch Bottom Joint   | Pitch Front Link    | Pitch Bottom Link   | Revolute  | `-0.10300 -0.28680        0` | `       0        0        0` |
| Joint 4      | Pitch End Joint      | Pitch Bottom Link   | Pitch End Link      | Revolute  | `0.340400 -0.00013        0` | `       0        0        0` |
| Joint 5      | Insertion Joint      | Pitch End Link      | Main Insertion Link | Prismatic | `   0.042 -0.08614        0` | `-${PI/2}        0        0` |
| Joint 6      | Roll Joint           | Main Insertion Link | Tool Link           | Revolute  | `0.060999        0        0` | `       0        0        0` |
| Joint 7      | Pitch Top Joint      | Pitch Front Link    | Pitch Top Link      | Revolute  | `-0.10847 -0.32425        0` | `       0        0        0` |
| Joint 8      | Pitch Back Joint     | Yaw Link            | Pitch Back Link     | Revolute  | `       0 -0.00979 0.162430` | `   ${PI} -${PI/2}        0` |
| Joint 9      | Remote Center Joint  | Base Link           | Remote Center Link  | Fixed     | `0.612599        0 0.101595` | `       0        0 -${PI/2}` |
| Joint 10     | End Joint            | Tool Link           | End Link            | Fixed     | `       0        0  0.37364` | `       0        0        0` |

---

# Ecm.Xacro

## Macro: ecm | Parameters: `<prefix parent_link xyz rpy>`

### Links

| Link Number | Link Name       | Mesh File                                               | Origin (`xyz`)               | Orientation (`rpy`)          |
| ----------- | --------------- | ------------------------------------------------------- | ---------------------------- | ---------------------------- |
| Link 0      | Setup Base Link | [N/A](N/A)                                              | N/A                          | N/A                          |
| Link 1      | Setup Link      | [Ecm/endo Arm](../../meshes/ecm/Endo_Arm.stl)           | `       0        0        0` | `       0        0        0` |
| Link 2      | Base Link       | [Ecm/endo Arm](../../meshes/ecm/Endo_Arm.stl)           | `       0        0        0` | `       0        0        0` |
| Link 3      | Yaw Link        | [Ecm/endo Link 5](../../meshes/ecm/Endo_Link_5.stl)     | `     0.0        0    0.603` | `       0        0 -${PI/2}` |
| Link 4      | Pitch Link      | [Ecm/endo Link 5](../../meshes/ecm/Endo_Link_5.stl)     | `     0.0        0    0.603` | `       0        0 -${PI/2}` |
| Link 5      | Pitch Link 1    | [Ecm/endo Link 5 2](../../meshes/ecm/Endo_Link_5_2.stl) | `     0.0        0      0.0` | `   ${PI}        0 ${PI*78.` |
| Link 6      | Pitch Link 2    | [Ecm/endo Link 6 2](../../meshes/ecm/Endo_Link_6_2.stl) | `     0.0        0  -0.0225` | `       0        0        0` |
| Link 7      | Pitch Link 3    | [Ecm/endo Link 7](../../meshes/ecm/Endo_Link_7.stl)     | `     0.0        0    0.025` | `       0        0 -${PI/2}` |
| Link 8      | Insertion Link  | [Ecm/endo Link 8](../../meshes/ecm/Endo_Link_8.stl)     | `     0.0        0   -0.015` | `       0        0 -${PI/2}` |
| Link 9      | Roll Link       | [Ecm/endoscop](../../meshes/ecm/EndoScope.stl)          | `     0.0        0  -0.3979` | `       0        0  ${PI/2}` |

### Joints

| Joint Number | Joint Name           | Parent          | Child           | Type      | Origin (`xyz`)               | Orientation (`rpy`)          |
| ------------ | -------------------- | --------------- | --------------- | --------- | ---------------------------- | ---------------------------- |
| Joint 0      | Fixed to World       | Macro Parent    | Setup Base Link | Fixed     | `  ${xyz}`                   | `  ${rpy}`                   |
| Joint 1      | Fixed to Setup Joint | Setup Base Link | Setup Link      | Fixed     | ` 0.91324 -0.02111  0.03166` | `-0.87046        0 -${PI/2}` |
| Joint 2      | Fixed to Base Joint  | Setup Link      | Base Link       | Fixed     | `       0    0.603        0` | `       0        0        0` |
| Joint 3      | Outer Yaw Joint      | Base Link       | Yaw Link        | Revolute  | `       0        0        0` | `       0 -${PI/2}  ${PI/2}` |
| Joint 4      | Outer Pitch Joint    | Yaw Link        | Pitch Link      | Revolute  | `       0        0        0` | `-${PI/2} -${PI/2}        0` |
| Joint 5      | Outer Pitch Joint 0  | Yaw Link        | Pitch Link 1    | Revolute  | `       0        0    0.378` | `-${PI/2} -${PI/2}        0` |
| Joint 6      | Outer Pitch Joint 1  | Pitch Link 1    | Pitch Link 2    | Revolute  | `0.055862 0.274575        0` | `       0        0        0` |
| Joint 7      | Outer Pitch Joint 2  | Pitch Link 2    | Pitch Link 3    | Revolute  | `  -0.340        0        0` | `       0        0        0` |
| Joint 8      | Insertion Joint      | Pitch Link      | Insertion Link  | Prismatic | `       0   0.3822        0` | ` ${PI/2}        0        0` |
| Joint 9      | Outer Roll Joint     | Insertion Link  | Roll Link       | Revolute  | `       0        0   0.3829` | `       0        0        0` |

---
