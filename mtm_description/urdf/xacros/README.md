# Mtm.Xacro

## Macro: None | Parameters: `<None>`

---

## Macro: master_console | Parameters: `<prefix parent_link xyz rpy>`

### Links

| Link Number | Link Name        | Mesh File  | Origin (`xyz`) | Orientation (`rpy`) |
| ----------- | ---------------- | ---------- | -------------- | ------------------- |
| Link -1     | Master Base Link | [N/A](N/A) | N/A            | N/A                 |

### Joints

| Joint Number | Joint Name     | Parent       | Child            | Type  | Origin (`xyz`) | Orientation (`rpy`) |
| ------------ | -------------- | ------------ | ---------------- | ----- | -------------- | ------------------- |
| Joint -1     | Fixed to World | Macro Parent | Master Base Link | Fixed | `  ${xyz}`     | `  ${rpy}`          |

---

## Macro: mtm | Parameters: `<prefix parent_link xyz rpy>`

### Links

| Link Number | Link Name            | Mesh File                                               | Origin (`xyz`)               | Orientation (`rpy`)          |
| ----------- | -------------------- | ------------------------------------------------------- | ---------------------------- | ---------------------------- |
| Link 0      | Top Panel Link       | [Toppanel](../../meshes/TopPanel.dae)                   | `       0        0  0.19037` | `       0        0        0` |
| Link 1      | Outer Yaw Link       | [Outpitch Shoulder](../../meshes/OutPitch_Shoulder.dae) | `   0.025        0  0.19037` | `       0        0        0` |
| Link 2      | Back Parallel Link   | [Armparallel](../../meshes/ArmParallel.dae)             | `   0.025        0  0.19037` | `       0        0        0` |
| Link 3      | Top Parallel Link    | [Armparallel1](../../meshes/ArmParallel1.dae)           | `       0        0    0.065` | `       0        0 ${-PI/2}` |
| Link 4      | Bottom Parallel Link | [Bottomarm](../../meshes/BottomArm.dae)                 | `       0        0    0.065` | `       0        0 ${-PI/2}` |

### Joints

| Joint Number | Joint Name              | Parent             | Child                | Type     | Origin (`xyz`)               | Orientation (`rpy`)          |
| ------------ | ----------------------- | ------------------ | -------------------- | -------- | ---------------------------- | ---------------------------- |
| Joint 0      | Fixed to World          | Macro Parent       | Top Panel Link       | Fixed    | `  ${xyz}`                   | `  ${rpy}`                   |
| Joint 1      | Outer Yaw               | Top Panel Link     | Outer Yaw Link       | Revolute | `       0        0        0` | `       0        0  ${PI/2}` |
| Joint 2      | Shoulder Pitch          | Outer Yaw Link     | Back Parallel Link   | Revolute | `       0        0        0` | `${-PI/2} ${-PI/2}        0` |
| Joint 3      | Shoulder Pitch Parallel | Outer Yaw Link     | Top Parallel Link    | Revolute | `       0        0        0` | `${-PI/2} ${-PI/2}        0` |
| Joint 4      | Elbow Pitch             | Back Parallel Link | Bottom Parallel Link | Revolute | ` -0.2794        0        0` | `       0        0  ${PI/2}` |

---

## Macro: mtm_platform_right | Parameters: `<prefix>`

### Links

| Link Number | Link Name           | Mesh File                                       | Origin (`xyz`)               | Orientation (`rpy`)          |
| ----------- | ------------------- | ----------------------------------------------- | ---------------------------- | ---------------------------- |
| Link 5      | Wrist Platform Link | [Wristplatform](../../meshes/WristPlatform.dae) | N/A                          | N/A                          |
| Link 6      | Wrist Pitch Link    | [Wristpitch](../../meshes/WristPitch.dae)       | N/A                          | N/A                          |
| Link 7      | Wrist Yaw Link      | [Wristyaw](../../meshes/WristYaw.dae)           | N/A                          | N/A                          |
| Link 8      | Wrist Roll Link     | [Wristroll](../../meshes/WristRoll.dae)         | `       0        0    0.039` | `   ${PI}        0        0` |

### Joints

| Joint Number | Joint Name           | Parent               | Child               | Type     | Origin (`xyz`)               | Orientation (`rpy`)          |
| ------------ | -------------------- | -------------------- | ------------------- | -------- | ---------------------------- | ---------------------------- |
| Joint 5      | Wrist Platform Joint | Bottom Parallel Link | Wrist Platform Link | Revolute | ` -0.3645  -0.1506      0.0` | ` ${PI/2}        0        0` |
| Joint 6      | Wrist Pitch Joint    | Wrist Platform Link  | Wrist Pitch Link    | Revolute | `       0        0        0` | `${-PI/2}        0        0` |
| Joint 7      | Wrist Yaw Joint      | Wrist Pitch Link     | Wrist Yaw Link      | Revolute | `       0        0        0` | ` ${PI/2} ${-PI/2}        0` |
| Joint 8      | Wrist Roll Joint     | Wrist Yaw Link       | Wrist Roll Link     | Revolute | `       0        0        0` | `       0 ${-PI/2}  ${PI/2}` |

---

## Macro: mtm_platform_left | Parameters: `<prefix>`

### Links

| Link Number | Link Name                          | Mesh File                                       | Origin (`xyz`)               | Orientation (`rpy`)          |
| ----------- | ---------------------------------- | ----------------------------------------------- | ---------------------------- | ---------------------------- |
| Link 9      | Wrist Platform Link (with Origins) | [Wristplatform](../../meshes/WristPlatform.dae) | `     0.0      0.0      0.0` | `       0        0    ${PI}` |
| Link 10     | Wrist Pitch Link (with Origins)    | [Wristpitch](../../meshes/WristPitch.dae)       | `     0.0      0.0      0.0` | `       0    ${PI}        0` |
| Link 11     | Wrist Yaw Link (with Origins)      | [Wristyaw](../../meshes/WristYaw.dae)           | `     0.0      0.0      0.0` | `       0    ${PI}        0` |
| Link 12     | Wrist Roll Link (with Origins)     | [Wristroll](../../meshes/WristRoll.dae)         | `       0        0    0.039` | `   ${PI}        0        0` |

### Joints

| Joint Number | Joint Name           | Parent                             | Child                              | Type     | Origin (`xyz`)               | Orientation (`rpy`)          |
| ------------ | -------------------- | ---------------------------------- | ---------------------------------- | -------- | ---------------------------- | ---------------------------- |
| Joint 9      | Wrist Platform Joint | Bottom Parallel Link               | Wrist Platform Link (with Origins) | Revolute | ` -0.3645  -0.1506      0.0` | ` ${PI/2}        0        0` |
| Joint 10     | Wrist Pitch Joint    | Wrist Platform Link (with Origins) | Wrist Pitch Link (with Origins)    | Revolute | `       0        0        0` | `${-PI/2}        0        0` |
| Joint 11     | Wrist Yaw Joint      | Wrist Pitch Link (with Origins)    | Wrist Yaw Link (with Origins)      | Revolute | `       0        0        0` | ` ${PI/2} ${-PI/2}        0` |
| Joint 12     | Wrist Roll Joint     | Wrist Yaw Link (with Origins)      | Wrist Roll Link (with Origins)     | Revolute | `       0        0        0` | `       0 ${-PI/2}  ${PI/2}` |

---

## Macro: mtm_right | Parameters: `<prefix parent_link xyz rpy>`

---

## Macro: mtm_left | Parameters: `<prefix parent_link xyz rpy>`

---
