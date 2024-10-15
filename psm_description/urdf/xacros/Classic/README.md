# Psm.Base.Xacro

## Macro: psm_base | Parameters: `<prefix parent_link xyz rpy>`

### Links

| Link Number | Link Name               | Mesh File                                                                          | Origin (`xyz`)               | Orientation (`rpy`)          |
| ----------- | ----------------------- | ---------------------------------------------------------------------------------- | ---------------------------- | ---------------------------- |
| Link 0      | PSM Base Link           | [Classic/psm Bas](../../../meshes/Classic/psm_base.stl)                            | `   0.039 -0.40788 -0.07879` | ` ${PI/2}        0    ${PI}` |
| Link 1      | Outer Yaw Link          | [Classic/outer Yaw](../../../meshes/Classic/outer_yaw.stl)                         | `  0.0125        0   0.5265` | `   ${PI}        0  ${PI/2}` |
| Link 2      | Outer Pitch Link        | [Classic/outer Yaw](../../../meshes/Classic/outer_yaw.stl)                         | `  0.0125        0   0.5265` | `   ${PI}        0  ${PI/2}` |
| Link 2-1    | Outer Pitch Back Link   | [Classic/outer Pitch Back](../../../meshes/Classic/outer_pitch_back.stl)           | `       0        0        0` | `       0        0 -0.27129` |
| Link 2-2    | Outer Pitch Front Link  | [Classic/outer Pitch Fron](../../../meshes/Classic/outer_pitch_front.stl)          | `       0        0        0` | `       0        0 -0.27129` |
| Link 2-3    | Outer Pitch Bottom Link | [Classic/outer Pitch Bottom](../../../meshes/Classic/outer_pitch_bottom.stl)       | `   0.009        0        0` | `       0 -${PI/2}        0` |
| Link 2-4    | Outer Pitch Top Link    | [Classic/outer Pitch Top](../../../meshes/Classic/outer_pitch_top.stl)             | `   0.009        0        0` | `       0 -${PI/2}        0` |
| Link 2-5    | Outer Insertion Link    | [Classic/outer Insertion](../../../meshes/Classic/outer_insertion.stl)             | ` 0.02528    0.429        0` | `       0 -${PI/2}  ${PI/2}` |
| Link 3      | Tool Main Link          | [Classic/tool Main](../../../meshes/Classic/tool_main.stl)                         | `       0        0    0.041` | `       0        0  ${PI/2}` |
| Link 4      | Tool Wrist Link         | [Classic/tool Wrist Link](../../../meshes/Classic/tool_wrist_link.stl)             | `       0        0  -0.0091` | `       0        0  ${PI/2}` |
| Link 4-1    | Tool Wrist Shaft Link   | [Classic/tool Wrist Shaft Link](../../../meshes/Classic/tool_wrist_shaft_link.stl) | `       0  0.00401        0` | ` ${PI/2}        0        0` |

### Joints

| Joint Number | Joint Name             | Parent                  | Child                   | Type       | Origin (`xyz`)               | Orientation (`rpy`)          |
| ------------ | ---------------------- | ----------------------- | ----------------------- | ---------- | ---------------------------- | ---------------------------- |
| Joint 0      | Fixed Joint            | Macro Parent            | PSM Base Link           | Fixed      | `  ${xyz}`                   | `  ${rpy}`                   |
| Joint 1      | Outer Yaw Joint        | PSM Base Link           | Outer Yaw Link          | Revolute   | `     0.0      0.0      0.0` | `       0 -${PI/2}  ${PI/2}` |
| Joint 2      | Outer Pitch Joint      | Outer Yaw Link          | Outer Pitch Link        | Revolute   | `       0        0        0` | `-${PI/2} -${PI/2}        0` |
| Joint 2-1    | Outer Pitch 1 Joint    | Outer Yaw Link          | Outer Pitch Back Link   | Continuous | `       0   0.0295   0.5185` | `-${PI/2} -${PI/2}        0` |
| Joint 2-2    | Outer Pitch 2 Joint    | Outer Yaw Link          | Outer Pitch Front Link  | Continuous | `       0   0.0295   0.4285` | `-${PI/2} -${PI/2}        0` |
| Joint 2-3    | Outer Pitch 3 Joint    | Outer Pitch Back Link   | Outer Pitch Bottom Link | Continuous | ` 0.04178  0.15007  -0.0137` | `       0        0        0` |
| Joint 2-4    | Outer Pitch 4 Joint    | Outer Pitch Back Link   | Outer Pitch Top Link    | Continuous | ` 0.04209  0.18695 -0.02412` | `       0        0        0` |
| Joint 2-5    | Outer Pitch 5 Joint    | Outer Pitch Bottom Link | Outer Insertion Link    | Continuous | `  -0.520        0  -0.0155` | `       0        0        0` |
| Joint 3      | Outer Insertion Joint  | Outer Pitch Link        | Tool Main Link          | Prismatic  | `       0   0.4318        0` | ` ${PI/2}        0        0` |
| Joint 4      | Outer Roll Joint       | Tool Main Link          | Tool Wrist Link         | Revolute   | `       0        0   0.4162` | `       0        0        0` |
| Joint 4-1    | Outer Roll Shaft Joint | Tool Wrist Link         | Tool Wrist Shaft Link   | Fixed      | `       0        0      0.0` | `       0        0        0` |

---

# Psm.Tool.Blade.Xacro

## Macro: psm_tool_blade | Parameters: `<prefix parent_link>`

### Links

| Link Number | Link Name                     | Mesh File                                                                                  | Origin (`xyz`)               | Orientation (`rpy`)          |
| ----------- | ----------------------------- | ------------------------------------------------------------------------------------------ | ---------------------------- | ---------------------------- |
| Link 5      | Tool Wrist Sca Link           | [Classic/tool Wrist Sca Link](../../../meshes/Classic/tool_wrist_sca_link.stl)             | `  0.0051   0.0032        0` | ` ${PI/2}    ${PI}        0` |
| Link 6      | Tool Wrist Sca Shaft Link     | [Classic/tool Wrist Sca Shaft Link](../../../meshes/Classic/tool_wrist_sca_shaft_link.stl) | `       0  -0.0002  -0.0025` | `       0        0        0` |
| Link 7-0    | Outer Open Angle Virtual Link | [Classic/tool Wrist Sca Shaft Link](../../../meshes/Classic/tool_wrist_sca_shaft_link.stl) | `       0        0        0` | `       0        0        0` |
| Link 8      | Tool Tip Link                 | [Classic/tool Wrist Sca Shaft Link](../../../meshes/Classic/tool_wrist_sca_shaft_link.stl) | `       0        0        0` | `       0        0        0` |

### Joints

| Joint Number | Joint Name               | Parent                    | Child                         | Type     | Origin (`xyz`)               | Orientation (`rpy`)          |
| ------------ | ------------------------ | ------------------------- | ----------------------------- | -------- | ---------------------------- | ---------------------------- |
| Joint 5      | Outer Wrist Pitch Joint  | Macro Parent              | Tool Wrist Sca Link           | Revolute | `       0        0        0` | `-${PI/2} -${PI/2}        0` |
| Joint 6      | Outer Wrist Yaw Joint    | Tool Wrist Sca Link       | Tool Wrist Sca Shaft Link     | Revolute | `  0.0091        0        0` | `-${PI/2} -${PI/2}        0` |
| Joint 7-0    | Outer Open Angle 1 Joint | Tool Wrist Sca Shaft Link | Outer Open Angle Virtual Link | Revolute | `       0        0        0` | `       0        0        0` |
| Joint 7-1    | Tool Tip Joint           | Tool Wrist Sca Shaft Link | Tool Tip Link                 | Fixed    | `       0     0.45        0` | `       0  ${PI/2}  ${PI/2}` |

---

# Psm.Tool.Caudier.Blade.Xacro

## Macro: psm_tool_caudier_blade | Parameters: `<prefix parent_link>`

### Links

| Link Number | Link Name                     | Mesh File                                                                                             | Origin (`xyz`)               | Orientation (`rpy`)          |
| ----------- | ----------------------------- | ----------------------------------------------------------------------------------------------------- | ---------------------------- | ---------------------------- |
| Link 5      | Tool Wrist Caudier Link       | [Classic/tool Wrist Caudier Link 1](../../../meshes/Classic/tool_wrist_caudier_link_1.stl)            | ` -0.0024     0.00    0.000` | `    0.00 -${PI/2}      0.0` |
| Link 6      | Tool Wrist Caudier Shaft Link | [Classic/tool Wrist Caudier Link 1 Shaf](../../../meshes/Classic/tool_wrist_caudier_link_1_shaft.stl) | `     0.0  -0.0002    0.000` | `    0.00  ${PI/2}      0.0` |
| Link 7-0    | Outer Open Angle Virtual Link | [Classic/knif](../../../meshes/Classic/knife.stl)                                                     | `       0        0        0` | `       0        0        0` |
| Link 6-3    | Tool Tip Link                 | [Classic/knif](../../../meshes/Classic/knife.stl)                                                     | `       0        0        0` | `       0        0        0` |

### Joints

| Joint Number | Joint Name              | Parent                        | Child                         | Type     | Origin (`xyz`)               | Orientation (`rpy`)          |
| ------------ | ----------------------- | ----------------------------- | ----------------------------- | -------- | ---------------------------- | ---------------------------- |
| Joint 5      | Outer Wrist Pitch Joint | Macro Parent                  | Tool Wrist Caudier Link       | Revolute | `       0        0        0` | `-${PI/2} -${PI/2}        0` |
| Joint 6      | Tool Wrist Yaw Joint    | Tool Wrist Caudier Link       | Tool Wrist Caudier Shaft Link | Revolute | `  0.0091        0        0` | `    0.00 -${PI/2} -${PI/2}` |
| Joint 7-0    | Outer Open Angle 1      | Tool Wrist Caudier Shaft Link | Outer Open Angle Virtual Link | Fixed    | `       0        0        0` | `-${PI/2}        0        0` |
| Joint 7-1    | Tool Tip Joint          | Tool Wrist Caudier Shaft Link | Tool Tip Link                 | Fixed    | `       0    0.045   0.0025` | `-${PI/2}        0        0` |

---

# Psm.Tool.Caudier.Xacro

## Macro: psm_tool_caudier | Parameters: `<prefix parent_link>`

### Links

| Link Number | Link Name                     | Mesh File                                                                                             | Origin (`xyz`)               | Orientation (`rpy`)          |
| ----------- | ----------------------------- | ----------------------------------------------------------------------------------------------------- | ---------------------------- | ---------------------------- |
| Link 5      | Tool Wrist Caudier Link       | [Classic/tool Wrist Caudier Link 1](../../../meshes/Classic/tool_wrist_caudier_link_1.stl)            | ` -0.0024     0.00    0.000` | `    0.00 -${PI/2}      0.0` |
| Link 6      | Tool Wrist Caudier Shaft Link | [Classic/tool Wrist Caudier Link 1 Shaf](../../../meshes/Classic/tool_wrist_caudier_link_1_shaft.stl) | `     0.0  -0.0002    0.000` | `    0.00  ${PI/2}      0.0` |
| Link 7-0    | Outer Open Angle Virtual Link | [Classic/tool Wrist Caudier Link 1 Shaf](../../../meshes/Classic/tool_wrist_caudier_link_1_shaft.stl) | `       0        0        0` | `       0        0        0` |
| Link 7-1    | Outer Open Angle 1 Link       | [Classic/tool Wrist Caudier Link 2](../../../meshes/Classic/tool_wrist_caudier_link_2.stl)            | `       0        0  0.00195` | `     0.0        0      0.0` |
| Link 7-2    | Outer Open Angle 2 Link       | [Classic/tool Wrist Caudier Link 2](../../../meshes/Classic/tool_wrist_caudier_link_2.stl)            | `       0        0  0.00195` | `       0        0   -${PI}` |
| Link 6-3    | Tool Wrist Caudier EE Link    | [Classic/tool Wrist Caudier Link 2](../../../meshes/Classic/tool_wrist_caudier_link_2.stl)            | `       0        0  0.00195` | `       0        0   -${PI}` |

### Joints

| Joint Number | Joint Name              | Parent                        | Child                         | Type     | Origin (`xyz`)               | Orientation (`rpy`)          |
| ------------ | ----------------------- | ----------------------------- | ----------------------------- | -------- | ---------------------------- | ---------------------------- |
| Joint 5      | Outer Wrist Pitch Joint | Macro Parent                  | Tool Wrist Caudier Link       | Revolute | `       0        0        0` | `-${PI/2} -${PI/2}        0` |
| Joint 6      | Tool Wrist Yaw Joint    | Tool Wrist Caudier Link       | Tool Wrist Caudier Shaft Link | Revolute | `  0.0091        0        0` | `    0.00 -${PI/2} -${PI/2}` |
| Joint 7-0    | Outer Open Angle 1      | Tool Wrist Caudier Shaft Link | Outer Open Angle Virtual Link | Revolute | `       0        0        0` | `       0        0        0` |
| Joint 7-1    | Outer Open Angle 1      | Tool Wrist Caudier Shaft Link | Outer Open Angle 1 Link       | Revolute | `       0        0        0` | `       0        0        0` |
| Joint 7-2    | Outer Open Angle 2      | Tool Wrist Caudier Shaft Link | Outer Open Angle 2 Link       | Revolute | `       0        0        0` | `   ${PI}        0        0` |
| Joint 7-3    | Tool Tip Joint          | Tool Wrist Caudier Shaft Link | Tool Wrist Caudier EE Link    | Fixed    | `       0        0        0` | `     0.0  ${PI/2}  ${PI/2}` |

---

# Psm.Tool.Sca.Blade.Xacro

## Macro: psm_tool_sca_blade | Parameters: `<prefix parent_link>`

### Links

| Link Number | Link Name                     | Mesh File                                                                                  | Origin (`xyz`)               | Orientation (`rpy`)          |
| ----------- | ----------------------------- | ------------------------------------------------------------------------------------------ | ---------------------------- | ---------------------------- |
| Link 5      | Tool Wrist Sca Link           | [Classic/tool Wrist Sca Link](../../../meshes/Classic/tool_wrist_sca_link.stl)             | `  0.0051   0.0032        0` | ` ${PI/2}    ${PI}        0` |
| Link 6      | Tool Wrist Sca Shaft Link     | [Classic/tool Wrist Sca Shaft Link](../../../meshes/Classic/tool_wrist_sca_shaft_link.stl) | `       0  -0.0002  -0.0025` | `       0        0        0` |
| Link 7-0    | Outer Open Angle Virtual Link | [Classic/tool Wrist Sca Shaft Link](../../../meshes/Classic/tool_wrist_sca_shaft_link.stl) | `       0   0.0225        0` | `     0.0      0.0      0.0` |
| Link 8      | Tool Tip Link                 | [Classic/tool Wrist Sca Shaft Link](../../../meshes/Classic/tool_wrist_sca_shaft_link.stl) | `       0   0.0225        0` | `     0.0      0.0      0.0` |

### Joints

| Joint Number | Joint Name               | Parent                        | Child                         | Type     | Origin (`xyz`)               | Orientation (`rpy`)          |
| ------------ | ------------------------ | ----------------------------- | ----------------------------- | -------- | ---------------------------- | ---------------------------- |
| Joint 5      | Outer Wrist Pitch Joint  | Macro Parent                  | Tool Wrist Sca Link           | Revolute | `       0        0        0` | `-${PI/2} -${PI/2}        0` |
| Joint 6      | Outer Wrist Yaw Joint    | Tool Wrist Sca Link           | Tool Wrist Sca Shaft Link     | Revolute | `  0.0091        0        0` | `-${PI/2} -${PI/2}        0` |
| Joint 7-0    | Outer Open Angle 1 Joint | Tool Wrist Sca Shaft Link     | Outer Open Angle Virtual Link | Fixed    | `       0        0        0` | `       0        0        0` |
| Joint 7-1    | Tool Tip Joint           | Outer Open Angle Virtual Link | Tool Tip Link                 | Fixed    | `       0    0.045        0` | `-${PI/2}      0.0      0.0` |

---

# Psm.Tool.Sca.Xacro

## Macro: psm_tool_sca | Parameters: `<prefix parent_link>`

### Links

| Link Number | Link Name                     | Mesh File                                                                                  | Origin (`xyz`)               | Orientation (`rpy`)          |
| ----------- | ----------------------------- | ------------------------------------------------------------------------------------------ | ---------------------------- | ---------------------------- |
| Link 5      | Tool Wrist Sca Link           | [Classic/tool Wrist Sca Link](../../../meshes/Classic/tool_wrist_sca_link.stl)             | `  0.0051   0.0032        0` | ` ${PI/2}    ${PI}        0` |
| Link 6      | Tool Wrist Sca Shaft Link     | [Classic/tool Wrist Sca Shaft Link](../../../meshes/Classic/tool_wrist_sca_shaft_link.stl) | `       0  -0.0002  -0.0025` | `       0        0        0` |
| Link 7-0    | Outer Open Angle Virtual Link | [Classic/tool Wrist Sca Shaft Link](../../../meshes/Classic/tool_wrist_sca_shaft_link.stl) | `       0        0        0` | `       0        0        0` |
| Link 7-1    | Outer Open Angle 1 Link       | [Classic/tool Wrist Sca Link 2](../../../meshes/Classic/tool_wrist_sca_link_2.stl)         | `       0        0        0` | `       0        0        0` |
| Link 7-2    | Outer Open Angle 2 Link       | [Classic/tool Wrist Sca Link 2](../../../meshes/Classic/tool_wrist_sca_link_2.stl)         | `       0        0        0` | `       0   3.1516        0` |
| Link 8      | Tool Tip Link                 | [Classic/tool Wrist Sca Link 2](../../../meshes/Classic/tool_wrist_sca_link_2.stl)         | `       0        0        0` | `       0   3.1516        0` |

### Joints

| Joint Number | Joint Name               | Parent                    | Child                         | Type     | Origin (`xyz`)               | Orientation (`rpy`)          |
| ------------ | ------------------------ | ------------------------- | ----------------------------- | -------- | ---------------------------- | ---------------------------- |
| Joint 5      | Outer Wrist Pitch Joint  | Macro Parent              | Tool Wrist Sca Link           | Revolute | `       0        0        0` | `-${PI/2} -${PI/2}        0` |
| Joint 6      | Outer Wrist Yaw Joint    | Tool Wrist Sca Link       | Tool Wrist Sca Shaft Link     | Revolute | `  0.0091        0        0` | `-${PI/2} -${PI/2}        0` |
| Joint 7-0    | Outer Open Angle Joint   | Tool Wrist Sca Shaft Link | Outer Open Angle Virtual Link | Revolute | `       0        0        0` | `       0        0        0` |
| Joint 7-1    | Outer Open Angle 1 Joint | Tool Wrist Sca Shaft Link | Outer Open Angle 1 Link       | Revolute | `       0        0        0` | `       0        0        0` |
| Joint 7-2    | Outer Open Angle 2 Joint | Tool Wrist Sca Shaft Link | Outer Open Angle 2 Link       | Revolute | `       0        0        0` | `       0        0        0` |
| Joint 7-3    | Tool Tip Joint           | Tool Wrist Sca Shaft Link | Tool Tip Link                 | Fixed    | `       0   0.0102        0` | `       0  ${PI/2}  ${PI/2}` |

---

# Psm.Tool.Snake.Xacro

## Macro: psm_tool_snake | Parameters: `<prefix parent_link>`

### Links

| Link Number | Link Name                     | Mesh File                                                        | Origin (`xyz`)               | Orientation (`rpy`)          |
| ----------- | ----------------------------- | ---------------------------------------------------------------- | ---------------------------- | ---------------------------- |
| Link 6      | Tool Snake Disc 1 Link        | [Snake Tool/link 1](../../../meshes/snake_tool/link_1.stl)       | `  0.0027        0        0` | ` ${PI/2}        0        0` |
| Link 7      | Tool Snake Disc 2 Link        | [Snake Tool/link 2](../../../meshes/snake_tool/link_2.stl)       | `  0.0027        0        0` | ` ${PI/2}        0        0` |
| Link 8      | Tool Snake Disc 3 Link        | [Snake Tool/link 3](../../../meshes/snake_tool/link_3.stl)       | `  0.0027        0        0` | ` ${PI/2}        0        0` |
| Link 9      | Tool Snake End Link           | [Snake Tool/link 4](../../../meshes/snake_tool/link_4.stl)       | `       0  -0.0027        0` | `       0  ${PI/2}        0` |
| Link 11-0   | Outer Open Angle Virtual Link | [Snake Tool/link 4](../../../meshes/snake_tool/link_4.stl)       | `       0        0      0.0` | `       0        0        0` |
| Link 11-1   | Outer Open Angle 1 Link       | [Snake Tool/gripper 2](../../../meshes/snake_tool/gripper_2.stl) | `       0        0        0` | `-${PI/2}  -0.7854        0` |
| Link 11-2   | Outer Open Angle 2 Link       | [Snake Tool/gripper 3](../../../meshes/snake_tool/gripper_3.stl) | `       0        0        0` | `-${PI/2}   2.3561        0` |

### Joints

| Joint Number | Joint Name                | Parent                 | Child                         | Type     | Origin (`xyz`)               | Orientation (`rpy`)          |
| ------------ | ------------------------- | ---------------------- | ----------------------------- | -------- | ---------------------------- | ---------------------------- |
| Joint 6      | Outer Wrist Pitch 1 Joint | Macro Parent           | Tool Snake Disc 1 Link        | Revolute | `     0.0        0        0` | `-${PI/2} -${PI/2}        0` |
| Joint 7      | Outer Wrist Yaw 1 Joint   | Tool Snake Disc 1 Link | Tool Snake Disc 2 Link        | Revolute | `  0.0034        0        0` | `-${PI/2}        0        0` |
| Joint 8      | Outer Wrist Yaw 2 Joint   | Tool Snake Disc 2 Link | Tool Snake Disc 3 Link        | Revolute | `  0.0034        0        0` | `       0        0        0` |
| Joint 9      | Outer Wrist Pitch 2 Joint | Tool Snake Disc 3 Link | Tool Snake End Link           | Revolute | `  0.0034        0        0` | ` ${PI/2} -${PI/2}        0` |
| Joint 11-0   | Outer Open Angle Joint    | Tool Snake End Link    | Outer Open Angle Virtual Link | Revolute | `       0        0        0` | `       0        0        0` |
| Joint 11-1   | Outer Open Angle 1 Joint  | Tool Snake End Link    | Outer Open Angle 1 Link       | Revolute | `       0    -0.01        0` | `       0   2.3562        0` |
| Joint 11-2   | Outer Open Angle 2 Joint  | Tool Snake End Link    | Outer Open Angle 2 Link       | Revolute | `       0    -0.01        0` | `       0   2.3562        0` |

---

# Psm.Tool.Xacro

## Macro: psm_tool | Parameters: `<tool_name prefix parent_link>`

---
