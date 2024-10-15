# Ecmj.Xacro

## Macro: ecm_j | Parameters: `<parent_link xyz rpy>`

### Links

| Link Number | Link Name          | Mesh File                                          | Origin (`xyz`)               | Orientation (`rpy`)          |
| ----------- | ------------------ | -------------------------------------------------- | ---------------------------- | ---------------------------- |
| Link 13     | ECM Joint Link 0   | [Si/ecm/link 0](../../../meshes/Si/ECM/link_0.stl) | `       0        0        0` | `       0        0        0` |
| Link 14     | ECM Joint Link 1   | [Si/ecm/link 1](../../../meshes/Si/ECM/link_1.stl) | `       0        0        0` | `       0        0        0` |
| Link 15     | ECM Joint Link 2   | [Si/ecm/link 2](../../../meshes/Si/ECM/link_2.stl) | `       0        0        0` | `       0        0        0` |
| Link 16     | ECM Joint Link 3   | [Si/ecm/link 3](../../../meshes/Si/ECM/link_3.stl) | `       0        0        0` | `       0        0        0` |
| Link 17     | ECM Mounting Point | [Si/ecm/link 3](../../../meshes/Si/ECM/link_3.stl) | `       0        0        0` | `       0        0        0` |

### Joints

| Joint Number | Joint Name  | Parent           | Child              | Type      | Origin (`xyz`)               | Orientation (`rpy`)          |
| ------------ | ----------- | ---------------- | ------------------ | --------- | ---------------------------- | ---------------------------- |
| Joint 12     | ECM Joint 0 | Macro Parent     | ECM Joint Link 0   | Prismatic | `  ${xyz}`                   | `  ${rpy}`                   |
| Joint 13     | ECM Joint 1 | ECM Joint Link 0 | ECM Joint Link 1   | Revolute  | `  -0.025      0.0   0.5232` | `       0        0        0` |
| Joint 14     | ECM Joint 2 | ECM Joint Link 1 | ECM Joint Link 2   | Revolute  | `  0.3302        0   0.1313` | `       0        0        0` |
| Joint 15     | ECM Joint 3 | ECM Joint Link 2 | ECM Joint Link 3   | Revolute  | `  0.3302        0    0.064` | `       0        0        0` |
| Joint 16     | ECM Joint 4 | ECM Joint Link 3 | ECM Mounting Point | Fixed     | ` 0.12857        0  -0.0109` | `       0   1.2217        0` |

---

# Psm3J.Xacro

## Macro: psm3_j | Parameters: `<parent_link xyz rpy>`

### Links

| Link Number | Link Name           | Mesh File                                              | Origin (`xyz`)               | Orientation (`rpy`)          |
| ----------- | ------------------- | ------------------------------------------------------ | ---------------------------- | ---------------------------- |
| Link 7      | PSM3 Link 0         | [Si/psm/3/link 0](../../../meshes/Si/PSM/3/link_0.stl) | `       0        0        0` | `       0        0        0` |
| Link 8      | PSM3 Link 1         | [Si/psm/3/link 1](../../../meshes/Si/PSM/3/link_1.stl) | `       0        0        0` | `       0        0        0` |
| Link 9      | PSM3 Link 2         | [Si/psm/3/link 2](../../../meshes/Si/PSM/3/link_2.stl) | `       0        0        0` | `       0        0        0` |
| Link 10     | PSM3 Link 3         | [Si/psm/3/link 3](../../../meshes/Si/PSM/3/link_3.stl) | `       0        0        0` | `       0        0        0` |
| Link 11     | PSM3 Link 4         | [Si/psm/3/link 4](../../../meshes/Si/PSM/3/link_4.stl) | `       0        0        0` | `       0        0        0` |
| Link 12     | PSM3 Mounting Point | [Si/psm/3/link 4](../../../meshes/Si/PSM/3/link_4.stl) | `       0        0        0` | `       0        0        0` |

### Joints

| Joint Number | Joint Name       | Parent       | Child               | Type      | Origin (`xyz`)               | Orientation (`rpy`)          |
| ------------ | ---------------- | ------------ | ------------------- | --------- | ---------------------------- | ---------------------------- |
| Joint 6      | PSM3 Joint 0     | Macro Parent | PSM3 Link 0         | Prismatic | `  ${xyz}`                   | `  ${rpy}`                   |
| Joint 7      | PSM3 Joint 1     | PSM3 Link 0  | PSM3 Link 1         | Revolute  | `-0.02499        0    0.513` | `       0        0 -0.23716` |
| Joint 8      | PSM3 Joint 2     | PSM3 Link 1  | PSM3 Link 2         | Revolute  | `  0.3429        0  -0.1942` | `   ${PI}        0  0.41417` |
| Joint 9      | PSM3 Joint 3     | PSM3 Link 2  | PSM3 Link 3         | Revolute  | `  0.4575        0  -0.0722` | `   ${PI}        0  0.17701` |
| Joint 10     | PSM3 Joint 4     | PSM3 Link 3  | PSM3 Link 4         | Revolute  | `   0.458        0 -0.01875` | `       0        0        0` |
| Joint 11     | PSM3 Fixed Joint | PSM3 Link 4  | PSM3 Mounting Point | Fixed     | `0.045254        0 -0.11671` | `       0   0.2618        0` |

---

# Psm12J.Xacro

## Macro: psm12_j | Parameters: `<prefix parent_link xyz rpy>`

### Links

| Link Number | Link Name                    | Mesh File                                                | Origin (`xyz`)               | Orientation (`rpy`)          |
| ----------- | ---------------------------- | -------------------------------------------------------- | ---------------------------- | ---------------------------- |
| Link 2      | PSM 1/2 Joint Link 0         | [Si/psm/12/link 0](../../../meshes/Si/PSM/12/link_0.stl) | `       0        0        0` | `       0        0        0` |
| Link 3      | PSM 1/2 Joint Link 1         | [Si/psm/12/link 1](../../../meshes/Si/PSM/12/link_1.stl) | `       0        0        0` | `       0        0        0` |
| Link 4      | PSM 1/2 Joint Link 2         | [Si/psm/12/link 2](../../../meshes/Si/PSM/12/link_2.stl) | `       0        0        0` | `       0        0        0` |
| Link 5      | PSM 1/2 Joint Link 3         | [Si/psm/12/link 3](../../../meshes/Si/PSM/12/link_3.stl) | `       0        0        0` | `       0        0        0` |
| Link 6      | PSM 1/2 Joint Mounting Point | [Si/psm/12/link 3](../../../meshes/Si/PSM/12/link_3.stl) | `       0        0        0` | `       0        0        0` |

### Joints

| Joint Number | Joint Name                    | Parent               | Child                        | Type      | Origin (`xyz`)               | Orientation (`rpy`)          |
| ------------ | ----------------------------- | -------------------- | ---------------------------- | --------- | ---------------------------- | ---------------------------- |
| Joint 1      | PSM 1/2 Joint Prismatic Joint | Macro Parent         | PSM 1/2 Joint Link 0         | Prismatic | `  ${xyz}`                   | `  ${rpy}`                   |
| Joint 2      | PSM 1/2 Joint Revolute Joint  | PSM 1/2 Joint Link 0 | PSM 1/2 Joint Link 1         | Revolute  | `-0.02499        0     0.59` | `       0        0        0` |
| Joint 3      | PSM 1/2 Joint Revolute Joint  | PSM 1/2 Joint Link 1 | PSM 1/2 Joint Link 2         | Revolute  | `  0.4575        0    0.144` | `       0        0        0` |
| Joint 4      | PSM 1/2 Joint Revolute Joint  | PSM 1/2 Joint Link 2 | PSM 1/2 Joint Link 3         | Revolute  | `   0.458        0 -0.01875` | `       0        0        0` |
| Joint 5      | PSM 1/2 Fixed Joint           | PSM 1/2 Joint Link 3 | PSM 1/2 Joint Mounting Point | Fixed     | `0.045255        0 -0.04525` | `       0   0.7854        0` |

---
