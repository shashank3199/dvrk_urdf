# Ecmj.Xacro

## Macro: ecm_j | Parameters: `<parent_link xyz rpy>`

### Links

| Link Number | Link Name          | Mesh File                                                    | Origin (`xyz`)               | Orientation (`rpy`)          |
| ----------- | ------------------ | ------------------------------------------------------------ | ---------------------------- | ---------------------------- |
| Link 8      | ECM Link 0         | [Classic/suj Ecm L0](../../../meshes/Classic/suj_ecm_L0.stl) | `       0        0        0` | `       0        0        0` |
| Link 9      | ECM Link 1         | [Classic/suj Ecm L1](../../../meshes/Classic/suj_ecm_L1.stl) | `       0        0        0` | `       0        0        0` |
| Link 10     | ECM Link 2         | [Classic/suj Ecm L2](../../../meshes/Classic/suj_ecm_L2.stl) | `       0        0        0` | `       0        0        0` |
| Link 11     | ECM Link 3         | [Classic/suj Ecm L3](../../../meshes/Classic/suj_ecm_L3.stl) | `       0        0        0` | `       0        0        0` |
| Link 12     | ECM Mounting Point | [Classic/suj Ecm L3](../../../meshes/Classic/suj_ecm_L3.stl) | `       0        0        0` | `       0        0        0` |

### Joints

| Joint Number | Joint Name    | Parent       | Child              | Type       | Origin (`xyz`)               | Orientation (`rpy`)          |
| ------------ | ------------- | ------------ | ------------------ | ---------- | ---------------------------- | ---------------------------- |
| Joint 7      | ECM Joint 0   | Macro Parent | ECM Link 0         | Prismatic  | `  ${xyz}`                   | `  ${rpy}`                   |
| Joint 8      | ECM Joint 1   | ECM Link 0   | ECM Link 1         | Continuous | `       0        0        0` | `   ${PI}        0  ${PI/2}` |
| Joint 9      | ECM Joint 2   | ECM Link 1   | ECM Link 2         | Continuous | `       0  -0.4318  0.17125` | `   ${PI}        0 -${PI/2}` |
| Joint 10     | ECM Joint 3   | ECM Link 2   | ECM Link 3         | Continuous | `  0.4318      0.0    0.052` | ` ${PI/2}      0.0      0.0` |
| Joint 11     | ECM RCM Joint | ECM Link 3   | ECM Mounting Point | Fixed      | `-0.45788  0.55053      0.0` | ` ${PI/2}    ${PI} -${PI/4}` |

---

# Psm1J.Xacro

## Macro: psm1_j | Parameters: `<parent_link xyz rpy>`

### Links

| Link Number | Link Name           | Mesh File                                                      | Origin (`xyz`)               | Orientation (`rpy`)          |
| ----------- | ------------------- | -------------------------------------------------------------- | ---------------------------- | ---------------------------- |
| Link 13     | PSM1 Link 0         | [Classic/suj Psm1 L0](../../../meshes/Classic/suj_psm1_L0.stl) | `       0        0        0` | `       0        0        0` |
| Link 14     | PSM1 Link 1         | [Classic/suj Psm1 L1](../../../meshes/Classic/suj_psm1_L1.stl) | `       0        0        0` | `       0        0        0` |
| Link 15     | PSM1 Link 2         | [Classic/suj Psm1 L2](../../../meshes/Classic/suj_psm1_L2.stl) | `       0        0        0` | `       0        0        0` |
| Link 16     | PSM1 Link 3         | [Classic/suj Psm1 L3](../../../meshes/Classic/suj_psm1_L3.stl) | `       0        0        0` | `       0        0        0` |
| Link 17     | PSM1 Link 4         | [Classic/suj Psm1 L4](../../../meshes/Classic/suj_psm1_L4.stl) | `       0        0        0` | `       0        0        0` |
| Link 18     | PSM1 Mounting Point | [Classic/suj Psm1 L4](../../../meshes/Classic/suj_psm1_L4.stl) | `       0        0        0` | `       0        0        0` |

### Joints

| Joint Number | Joint Name     | Parent       | Child               | Type       | Origin (`xyz`)               | Orientation (`rpy`)          |
| ------------ | -------------- | ------------ | ------------------- | ---------- | ---------------------------- | ---------------------------- |
| Joint 12     | PSM1 Joint 0   | Macro Parent | PSM1 Link 0         | Prismatic  | `  ${xyz}`                   | `  ${rpy}`                   |
| Joint 13     | PSM1 Joint 1   | PSM1 Link 0  | PSM1 Link 1         | Continuous | `       0        0        0` | `       0        0  ${PI/2}` |
| Joint 14     | PSM1 Joint 2   | PSM1 Link 1  | PSM1 Link 2         | Continuous | `       0   0.4318 -0.17125` | `   ${PI}        0  ${PI/2}` |
| Joint 15     | PSM1 Joint 3   | PSM1 Link 2  | PSM1 Link 3         | Continuous | `  0.4318        0        0` | ` ${PI/2}        0 -${PI/2}` |
| Joint 16     | PSM1 Joint 4   | PSM1 Link 3  | PSM1 Link 4         | Continuous | `       0 -0.07304 -0.06099` | `-${PI/2}        0    ${PI}` |
| Joint 17     | PSM1 RCM Joint | PSM1 Link 4  | PSM1 Mounting Point | Fixed      | `     0.0   0.8343 -0.04984` | `     0.0    ${PI}      0.0` |

---

# Psm2J.Xacro

## Macro: psm2_j | Parameters: `<parent_link xyz rpy>`

### Links

| Link Number | Link Name           | Mesh File                                                      | Origin (`xyz`)               | Orientation (`rpy`)          |
| ----------- | ------------------- | -------------------------------------------------------------- | ---------------------------- | ---------------------------- |
| Link 2      | PSM2 Link 0         | [Classic/suj Psm2 L0](../../../meshes/Classic/suj_psm2_L0.stl) | `       0        0        0` | `       0        0        0` |
| Link 3      | PSM2 Link 1         | [Classic/suj Psm2 L1](../../../meshes/Classic/suj_psm2_L1.stl) | `       0        0        0` | `       0        0        0` |
| Link 4      | PSM2 Link 2         | [Classic/suj Psm2 L2](../../../meshes/Classic/suj_psm2_L2.stl) | `       0        0        0` | `       0        0        0` |
| Link 5      | PSM2 Link 3         | [Classic/suj Psm2 L3](../../../meshes/Classic/suj_psm2_L3.stl) | `       0        0        0` | `       0        0        0` |
| Link 6      | PSM2 Link 4         | [Classic/suj Psm2 L4](../../../meshes/Classic/suj_psm2_L4.stl) | `       0        0        0` | `       0        0        0` |
| Link 7      | PSM2 Mounting Point | [Classic/suj Psm2 L4](../../../meshes/Classic/suj_psm2_L4.stl) | `       0        0        0` | `       0        0        0` |

### Joints

| Joint Number | Joint Name     | Parent       | Child               | Type       | Origin (`xyz`)               | Orientation (`rpy`)          |
| ------------ | -------------- | ------------ | ------------------- | ---------- | ---------------------------- | ---------------------------- |
| Joint 1      | PSM2 Joint 0   | Macro Parent | PSM2 Link 0         | Prismatic  | `  ${xyz}`                   | `  ${rpy}`                   |
| Joint 2      | PSM2 Joint 1   | PSM2 Link 0  | PSM2 Link 1         | Continuous | `       0        0        0` | `       0        0  ${PI/2}` |
| Joint 3      | PSM2 Joint 2   | PSM2 Link 1  | PSM2 Link 2         | Continuous | `       0   0.4318 -0.17125` | `   ${PI}      0.0  ${PI/2}` |
| Joint 4      | PSM2 Joint 3   | PSM2 Link 2  | PSM2 Link 3         | Continuous | `  0.4318        0        0` | `-${PI/2}        0 -${PI/2}` |
| Joint 5      | PSM2 Joint 4   | PSM2 Link 3  | PSM2 Link 4         | Continuous | `       0 0.073049 0.060999` | ` ${PI/2}      0.0    ${PI}` |
| Joint 6      | PSM2 RCM Joint | PSM2 Link 4  | PSM2 Mounting Point | Fixed      | `     0.0   0.8343 -0.04984` | `     0.0    ${PI}      0.0` |

---

# Psm3J.Xacro

## Macro: psm3_j | Parameters: `<parent_link xyz rpy>`

### Links

| Link Number | Link Name           | Mesh File                                                      | Origin (`xyz`)               | Orientation (`rpy`)          |
| ----------- | ------------------- | -------------------------------------------------------------- | ---------------------------- | ---------------------------- |
| Link 19     | PSM3 Link 0         | [Classic/suj Psm3 L0](../../../meshes/Classic/suj_psm3_L0.stl) | `       0        0        0` | `       0        0        0` |
| Link 20     | PSM3 Link 1         | [Classic/suj Psm3 L1](../../../meshes/Classic/suj_psm3_L1.stl) | `       0        0        0` | `       0        0        0` |
| Link 21     | PSM3 Link 2         | [Classic/suj Psm3 L2](../../../meshes/Classic/suj_psm3_L2.stl) | `       0        0        0` | `       0        0        0` |
| Link 22     | PSM3 Link 3         | [Classic/suj Psm3 L3](../../../meshes/Classic/suj_psm3_L3.stl) | `       0        0        0` | `       0        0        0` |
| Link 23     | PSM3 Link 4         | [Classic/suj Psm3 L4](../../../meshes/Classic/suj_psm3_L4.stl) | `       0        0        0` | `       0        0        0` |
| Link 24     | PSM3 Mounting Point | [Classic/suj Psm3 L4](../../../meshes/Classic/suj_psm3_L4.stl) | `       0        0        0` | `       0        0        0` |

### Joints

| Joint Number | Joint Name     | Parent       | Child               | Type       | Origin (`xyz`)               | Orientation (`rpy`)          |
| ------------ | -------------- | ------------ | ------------------- | ---------- | ---------------------------- | ---------------------------- |
| Joint 18     | PSM3 Joint 0   | Macro Parent | PSM3 Link 0         | Prismatic  | `  ${xyz}`                   | `  ${rpy}`                   |
| Joint 19     | PSM3 Joint 1   | PSM3 Link 0  | PSM3 Link 1         | Continuous | `       0        0        0` | `   ${PI}        0  ${PI/2}` |
| Joint 20     | PSM3 Joint 2   | PSM3 Link 1  | PSM3 Link 2         | Continuous | `       0  -0.5842 -0.00024` | `   ${PI}        0 -${PI/2}` |
| Joint 21     | PSM3 Joint 3   | PSM3 Link 2  | PSM3 Link 3         | Continuous | `0.431800        0        0` | ` ${PI/2}        0 -${PI/2}` |
| Joint 22     | PSM3 Joint 4   | PSM3 Link 3  | PSM3 Link 4         | Continuous | `       0 -0.19994   -0.061` | `-${PI/2}        0        0` |
| Joint 23     | PSM3 RCM Joint | PSM3 Link 4  | PSM3 Mounting Point | Fixed      | `     0.0   0.8343 -0.04984` | `     0.0    ${PI}      0.0` |

---
