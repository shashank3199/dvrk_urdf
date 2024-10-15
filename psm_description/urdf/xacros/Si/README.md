# Psm.Base.Xacro

## Macro: psm_base | Parameters: `<prefix parent_link xyz:='0.0 0.0 0.0' rpy:='0.0 0.0 0.0'>`

### Links

| Link Number | Link Name        | Mesh File                                  | Origin (`xyz`)               | Orientation (`rpy`)          |
| ----------- | ---------------- | ------------------------------------------ | ---------------------------- | ---------------------------- |
| Link 0      | Link 0           | [Si/link 0](../../../meshes/Si/link_0.stl) | `       0        0        0` | `       0        0        0` |
| Link 1      | Link 1           | [Si/link 1](../../../meshes/Si/link_1.stl) | `       0        0        0` | `       0        0        0` |
| Link 2      | Link 2           | [Si/link 2](../../../meshes/Si/link_2.stl) | `       0        0        0` | `       0        0        0` |
| Link 3      | Link 3           | [Si/link 3](../../../meshes/Si/link_3.stl) | `       0        0        0` | `       0        0        0` |
| Link 4      | Link 4           | [Si/link 4](../../../meshes/Si/link_4.stl) | `       0        0        0` | `       0        0        0` |
| Link 5      | Tool Parent Link | [Si/link 4](../../../meshes/Si/link_4.stl) | `       0        0        0` | `       0        0        0` |
| Link 6      | RCM Link         | [Si/link 4](../../../meshes/Si/link_4.stl) | `       0        0        0` | `       0        0        0` |

### Joints

| Joint Number | Joint Name                      | Parent       | Child            | Type     | Origin (`xyz`)               | Orientation (`rpy`)          |
| ------------ | ------------------------------- | ------------ | ---------------- | -------- | ---------------------------- | ---------------------------- |
| Joint 0      | Fixed Joint                     | Macro Parent | Link 0           | Fixed    | `  ${xyz}`                   | `  ${rpy}`                   |
| Joint 1      | Yaw Joint                       | Link 0       | Link 1           | Revolute | `   0.158        0        0` | `     0.0      0.0      0.0` |
| Joint 2      | Pitch Joint                     | Link 1       | Link 2           | Revolute | `  0.2987        0 -0.07122` | `       0  -1.1345   -${PI}` |
| Joint 3      | Pitch Joint                     | Link 2       | Link 3           | Revolute | `  0.2032        0        0` | `       0 -0.90761   -${PI}` |
| Joint 4      | Pitch Joint                     | Link 3       | Link 4           | Revolute | ` 0.35966        0  0.16013` | `       0 -0.06396        0` |
| Joint 5      | Outer Insertion Reference Joint | Link 4       | Tool Parent Link | Fixed    | ` 0.05061        0  0.41911` | `       0        0        0` |
| Joint 6      | RCM Joint                       | Link 4       | RCM Link         | Fixed    | ` 0.10643        0  -0.1731` | `   ${PI}        0 -${PI/2}` |

---

# Psm.Tool.P420006.Xacro

## Macro: P420006 | Parameters: `<prefix parent_link xyz rpy>`

### Links

| Link Number | Link Name                 | Mesh File                                                                                  | Origin (`xyz`)               | Orientation (`rpy`)          |
| ----------- | ------------------------- | ------------------------------------------------------------------------------------------ | ---------------------------- | ---------------------------- |
| Link 0      | Tool Main Link            | [P420006/tool Main Link](../../../meshes/P420006/tool_main_link.stl)                       | `       0        0        0` | `       0        0        0` |
| Link 1      | Tool Wrist Link           | [P420006/tool Wrist Link](../../../meshes/P420006/tool_wrist_link.stl)                     | `       0        0        0` | `       0        0        0` |
| Link 2      | Tool Wrist Shaft Link     | [P420006/tool Wrist Shaft Link](../../../meshes/P420006/tool_wrist_shaft_link.stl)         | `       0        0        0` | `       0        0        0` |
| Link 3      | Tool Wrist SCA Link       | [P420006/tool Wrist Scal Link](../../../meshes/P420006/tool_wrist_scal_link.stl)           | `       0        0        0` | `       0        0        0` |
| Link 4      | Tool Wrist SCA Shaft Link | [P420006/tool Wrist Sca Shaft Link](../../../meshes/P420006/tool_wrist_sca_shaft_link.stl) | `       0        0        0` | `       0        0        0` |
| Link 5      | Tool Wrist SCA EE Link    | [P420006/tool Wrist Sca Shaft Link](../../../meshes/P420006/tool_wrist_sca_shaft_link.stl) | `       0        0        0` | `       0        0        0` |
| Link 6      | Tool Wrist SCA EE Link 1  | [P420006/tool Wrist Sca Ee Link 1](../../../meshes/P420006/tool_wrist_sca_ee_link_1.stl)   | `       0        0        0` | `       0        0        0` |
| Link 7      | Tool Wrist SCA EE Link 2  | [P420006/tool Wrist Sca Ee Link 2](../../../meshes/P420006/tool_wrist_sca_ee_link_2.stl)   | `       0        0        0` | `       0        0        0` |

### Joints

| Joint Number | Joint Name        | Parent                    | Child                     | Type      | Origin (`xyz`)               | Orientation (`rpy`)          |
| ------------ | ----------------- | ------------------------- | ------------------------- | --------- | ---------------------------- | ---------------------------- |
| Joint 0      | Insertion Joint   | Macro Parent              | Tool Main Link            | Prismatic | `  ${xyz}`                   | `  ${rpy}`                   |
| Joint 1      | Roll Joint        | Tool Main Link            | Tool Wrist Link           | Revolute  | ` 0.05591        0 -0.53559` | `  -${PI}        0        0` |
| Joint 2      | Roll Shaft Joint  | Tool Wrist Link           | Tool Wrist Shaft Link     | Fixed     | `       0    1E-05    0.009` | `       0        0        0` |
| Joint 3      | Wrist Pitch Joint | Tool Wrist Shaft Link     | Tool Wrist SCA Link       | Revolute  | `       0        0        0` | `       0        0        0` |
| Joint 4      | Wrist Yaw Joint   | Tool Wrist SCA Link       | Tool Wrist SCA Shaft Link | Revolute  | `       0        0 0.008697` | `       0        0        0` |
| Joint 5      | Jaw Joint         | Tool Wrist SCA Shaft Link | Tool Wrist SCA EE Link    | Revolute  | `       0        0        0` | `       0        0        0` |
| Joint 6      | Jaw 1 Joint       | Tool Wrist SCA Shaft Link | Tool Wrist SCA EE Link 1  | Revolute  | `       0        0        0` | `       0        0        0` |
| Joint 7      | Jaw 2 Joint       | Tool Wrist SCA Shaft Link | Tool Wrist SCA EE Link 2  | Revolute  | `       0        0        0` | `       0        0        0` |

---

# Psm.Tool.Sf826001.Xacro

## Macro: SF826001 | Parameters: `<prefix parent_link xyz rpy>`

### Links

| Link Number | Link Name      | Mesh File                                                              | Origin (`xyz`)               | Orientation (`rpy`)          |
| ----------- | -------------- | ---------------------------------------------------------------------- | ---------------------------- | ---------------------------- |
| Link 0      | Tool Main Link | [Sf826001/tool Main Link](../../../meshes/SF826001/tool_main_link.stl) | `       0        0        0` | `       0        0        0` |
| Link 1      | Tool Roll Link | [Sf826001/tool Roll Link](../../../meshes/SF826001/tool_roll_link.stl) | `       0        0        0` | `       0        0        0` |

### Joints

| Joint Number | Joint Name      | Parent         | Child          | Type      | Origin (`xyz`)               | Orientation (`rpy`)          |
| ------------ | --------------- | -------------- | -------------- | --------- | ---------------------------- | ---------------------------- |
| Joint 0      | Insertion Joint | Macro Parent   | Tool Main Link | Prismatic | `  ${xyz}`                   | `  ${rpy}`                   |
| Joint 1      | Roll Joint      | Tool Main Link | Tool Roll Link | Revolute  | ` 0.05591        0 -0.53039` | `       0        0        0` |

---

# Psm.Tool.Xacro

## Macro: psm_tool | Parameters: `<tool_name prefix parent_link xyz rpy>`

---
