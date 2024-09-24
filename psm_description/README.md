# PSM (Patient Side Manipulator) Description

This README provides a comprehensive overview of the **Patient Side Manipulator (PSM)** models, including their URDF (Unified Robot Description Format) structures, meshes, and configurations.

---

## Table of Contents

- [Meshes](#meshes)
- [PSM Classic](#psm-classic)
  - [Launching the PSM Classic Model](#launching-the-psm-classic-model)
  - [Changing the Tool Type](#changing-the-tool-type)
  - [PSM Base Xacro](#psm-base-xacro)
    - [Links](#links)
    - [Joints](#joints)
  - [PSM Tool Xacros](#psm-tool-xacros)
    - [Blade Tool](#blade-tool)
    - [Caudier Blade Tool](#caudier-blade-tool)
    - [Caudier Tool](#caudier-tool)
    - [SCA Blade Tool](#sca-blade-tool)
    - [SCA Tool](#sca-tool)
    - [Snake Tool](#snake-tool)
- [PSM Si](#psm-si)
  - [Launching the PSM Si Model](#launching-the-psm-si-model)
  - [Changing the Tool Type](#changing-the-tool-type-1)
  - [PSM Si Base Xacro](#psm-si-base-xacro)
    - [Links](#links-1)
    - [Joints](#joints-1)
  - [PSM Si Tool Xacros](#psm-si-tool-xacros)
    - [P420006 Tool](#p420006-tool)
    - [SF826001 Tool](#sf826001-tool)
- [URDF Structures](#urdf-structures)
- [Notes](#notes)

---

## Meshes

The PSM models utilize various mesh files located in the `meshes` directory. These meshes represent the physical components of the manipulators and tools.

```
meshes
├── Classic
│   ├── knife.stl
│   ├── outer_insertion.dae
│   ├── outer_insertion.stl
│   ├── outer_pitch_back.dae
│   ├── outer_pitch_back.stl
│   ├── outer_pitch_bottom.dae
│   ├── outer_pitch_bottom.stl
│   ├── outer_pitch_front.dae
│   ├── outer_pitch_front.stl
│   ├── outer_pitch_top.dae
│   ├── outer_pitch_top.stl
│   ├── outer_yaw.dae
│   ├── outer_yaw.stl
│   ├── psm_base.dae
│   ├── psm_base.stl
│   ├── tool_adapter.dae
│   ├── tool_adapter.stl
│   ├── tool_main.dae
│   ├── tool_main.stl
│   ├── tool_wrist_caudier_link_1_shaft.stl
│   ├── tool_wrist_caudier_link_1.stl
│   ├── tool_wrist_caudier_link_2.stl
│   ├── tool_wrist_link.dae
│   ├── tool_wrist_link.stl
│   ├── tool_wrist_sca_link_2.dae
│   ├── tool_wrist_sca_link_2.stl
│   ├── tool_wrist_sca_link.dae
│   ├── tool_wrist_sca_link.stl
│   ├── tool_wrist_sca_shaft_link.dae
│   ├── tool_wrist_sca_shaft_link.stl
│   ├── tool_wrist_shaft_link.dae
│   └── tool_wrist_shaft_link.stl
├── P420006
│   ├── tool_main_link.stl
│   ├── tool_wrist_link.stl
│   ├── tool_wrist_sca_ee_link_1.stl
│   ├── tool_wrist_sca_ee_link_2.stl
│   ├── tool_wrist_sca_link.stl
│   ├── tool_wrist_scal_link.stl
│   ├── tool_wrist_sca_shaft_link.stl
│   └── tool_wrist_shaft_link.stl
├── SF826001
│   ├── tool_main_link.stl
│   └── tool_roll_link.stl
├── Si
│   ├── link_0.stl
│   ├── link_1.stl
│   ├── link_2.stl
│   ├── link_3.stl
│   └── link_4.stl
└── snake_tool
    ├── gripper_2.stl
    ├── gripper_3.stl
    ├── link_0.stl
    ├── link_1.stl
    ├── link_2.stl
    ├── link_3.stl
    └── link_4.stl

5 directories, 54 files
```

---

## PSM Classic

### Launching the PSM Classic Model

To launch the PSM Classic model with both PSMs, use the following command:

```bash
ros2 launch psm_description urdf.launch.py file:=both_psms.classic.urdf.xacro
```

For a single PSM, use:

```bash
ros2 launch psm_description urdf.launch.py file:=psm.classic.urdf.xacro
```

### Changing the Tool Type

You can change the type of tool mounted on the PSM by modifying the `tool_name` parameter in the `psm.classic.urdf.xacro` file:

```xml
<!-- Tool Arguments -->
<!-- Select tool_name from blade, caudier_blade, caudier, sca_blade, sca, snake -->
<xacro:property name="tool_name" default="sca" />
```

---

### PSM Base Xacro

#### Links

| Link Number | Link Name                       | Mesh File                                                                 |
|-------------|---------------------------------|---------------------------------------------------------------------------|
| Link 0      | PSM Base Link                   | [psm_base.stl](./meshes/Classic/psm_base.stl)                             |
| Link 1      | Outer Yaw Link                  | [outer_yaw.stl](./meshes/Classic/outer_yaw.stl)                           |
| Link 2      | Outer Pitch Link                | [outer_pitch.stl](./meshes/Classic/outer_pitch_front.stl)                 |
| Link 2-1    | Outer Pitch Back Link           | [outer_pitch_back.stl](./meshes/Classic/outer_pitch_back.stl)             |
| Link 2-2    | Outer Pitch Front Link          | [outer_pitch_front.stl](./meshes/Classic/outer_pitch_front.stl)           |
| Link 2-3    | Outer Pitch Bottom Link         | [outer_pitch_bottom.stl](./meshes/Classic/outer_pitch_bottom.stl)         |
| Link 2-4    | Outer Pitch Top Link            | [outer_pitch_top.stl](./meshes/Classic/outer_pitch_top.stl)               |
| Link 2-5    | Outer Insertion Link            | [outer_insertion.stl](./meshes/Classic/outer_insertion.stl)               |
| Link 3      | Tool Main Link                  | [tool_main.stl](./meshes/Classic/tool_main.stl)                           |
| Link 4      | Tool Wrist Link                 | [tool_wrist_link.stl](./meshes/Classic/tool_wrist_link.stl)               |
| Link 4-1    | Tool Wrist Shaft Link           | [tool_wrist_shaft_link.stl](./meshes/Classic/tool_wrist_shaft_link.stl)   |

#### Joints

| Joint Number | Joint Name              | Parent Link              | Child Link                   | Type       |
|--------------|-------------------------|--------------------------|------------------------------|------------|
| Joint 0      | Fixed Joint             | Parent Link              | Link 0                       | Fixed      |
| Joint 1      | Outer Yaw Joint         | Link 0                   | Link 1                       | Revolute   |
| Joint 2      | Outer Pitch Joint       | Link 1                   | Link 2                       | Revolute   |
| Joint 2-1    | Outer Pitch 1 Joint     | Link 1                   | Link 2-1                     | Continuous |
| Joint 2-2    | Outer Pitch 2 Joint     | Link 1                   | Link 2-2                     | Continuous |
| Joint 2-3    | Outer Pitch 3 Joint     | Link 2-1                 | Link 2-3                     | Continuous |
| Joint 2-4    | Outer Pitch 4 Joint     | Link 2-1                 | Link 2-4                     | Continuous |
| Joint 2-5    | Outer Pitch 5 Joint     | Link 2-3                 | Link 2-5                     | Continuous |
| Joint 3      | Outer Insertion Joint   | Link 2                   | Link 3                       | Prismatic  |
| Joint 4      | Outer Roll Joint        | Link 3                   | Link 4                       | Revolute   |
| Joint 4-1    | Outer Roll Shaft Joint  | Link 4                   | Link 4-1                     | Fixed      |

---

### PSM Tool Xacros

#### Blade Tool

##### Links

| Link Number | Link Name                       | Mesh File                                                                         |
|-------------|---------------------------------|-----------------------------------------------------------------------------------|
| Link 5      | Tool Wrist SCA Link             | [tool_wrist_sca_link.stl](./meshes/Classic/tool_wrist_sca_link.stl)               |
| Link 6      | Tool Wrist SCA Shaft Link       | [tool_wrist_sca_shaft_link.stl](./meshes/Classic/tool_wrist_sca_shaft_link.stl)   |
| Link 7-0    | Outer Open Angle Virtual Link   | N/A                                                                               |
| Link 8      | Tool Tip Link                   | N/A                                                                               |

##### Joints

| Joint Number | Joint Name              | Parent Link              | Child Link                   | Type       |
|--------------|-------------------------|--------------------------|------------------------------|------------|
| Joint 5      | Outer Wrist Pitch Joint | Link 4-1                 | Link 5                       | Revolute   |
| Joint 6      | Outer Wrist Yaw Joint   | Link 5                   | Link 6                       | Revolute   |
| Joint 7-0    | Outer Open Angle Joint  | Link 6                   | Link 7-0                     | Revolute   |
| Joint 7-1    | Tool Tip Joint          | Link 7-0                 | Link 8                       | Fixed      |

---

#### Caudier Blade Tool

##### Links

| Link Number | Link Name                       | Mesh File                                                                         |
|-------------|---------------------------------|-----------------------------------------------------------------------------------|
| Link 5      | Tool Wrist Caudier Link         | [tool_wrist_caudier_link_1.stl](./meshes/Classic/tool_wrist_caudier_link_1.stl)   |
| Link 6      | Tool Wrist Caudier Shaft Link   | N/A                                                                               |
| Link 7-0    | Outer Open Angle Virtual Link   | N/A                                                                               |
| Link 7-1    | Tool Tip Link                   | N/A                                                                               |

##### Joints

| Joint Number | Joint Name              | Parent Link              | Child Link                   | Type       |
|--------------|-------------------------|--------------------------|------------------------------|------------|
| Joint 5      | Outer Wrist Pitch Joint | Link 4-1                 | Link 5                       | Revolute   |
| Joint 6      | Tool Wrist Yaw Joint    | Link 5                   | Link 6                       | Revolute   |
| Joint 7-0    | Outer Open Angle Joint  | Link 6                   | Link 7-0                     | Fixed      |
| Joint 7-1    | Tool Tip Joint          | Link 6                   | Link 6-3                     | Fixed      |

---

#### Caudier Tool

##### Links

| Link Number | Link Name                       |
|-------------|---------------------------------|
| Link 5      | Tool Wrist Caudier Link         |
| Link 6      | Tool Wrist Caudier Shaft Link   |
| Link 7-0    | Outer Open Angle Virtual Link   |
| Link 7-1    | Outer Open Angle 1 Link         |
| Link 7-2    | Outer Open Angle 2 Link         |
| Link 6-3    | Tool Wrist Caudier EE Link      |

##### Joints

| Joint Number | Joint Name              | Parent Link              | Child Link                   | Type       |
|--------------|-------------------------|--------------------------|------------------------------|------------|
| Joint 5      | Outer Wrist Pitch Joint | Link 4-1                 | Link 5                       | Revolute   |
| Joint 6      | Tool Wrist Yaw Joint    | Link 5                   | Link 6                       | Revolute   |
| Joint 7-0    | Outer Open Angle Joint  | Link 6                   | Link 7-0                     | Revolute   |
| Joint 7-1    | Outer Open Angle 1      | Link 6                   | Link 7-1                     | Revolute   |
| Joint 7-2    | Outer Open Angle 2      | Link 6                   | Link 7-2                     | Revolute   |
| Joint 7-3    | Tool Tip Joint          | Link 6                   | Link 6-3                     | Fixed      |

---

#### SCA Blade Tool

##### Links

| Link Number | Link Name                       |
|-------------|---------------------------------|
| Link 5      | Tool Wrist SCA Link             |
| Link 6      | Tool Wrist SCA Shaft Link       |
| Link 7-0    | Outer Open Angle Virtual Link   |
| Link 8      | Tool Tip Link                   |

##### Joints

| Joint Number | Joint Name              | Parent Link              | Child Link                   | Type       |
|--------------|-------------------------|--------------------------|------------------------------|------------|
| Joint 5      | Outer Wrist Pitch Joint | Link 4-1                 | Link 5                       | Revolute   |
| Joint 6      | Outer Wrist Yaw Joint   | Link 5                   | Link 6                       | Revolute   |
| Joint 7-0    | Outer Open Angle Joint  | Link 6                   | Link 7-0                     | Fixed      |
| Joint 7-1    | Tool Tip Joint          | Link 7-0                 | Link 8                       | Fixed      |

---

#### SCA Tool

##### Links

| Link Number | Link Name                       |
|-------------|---------------------------------|
| Link 5      | Tool Wrist SCA Link             |
| Link 6      | Tool Wrist SCA Shaft Link       |
| Link 7-0    | Outer Open Angle Virtual Link   |
| Link 7-1    | Outer Open Angle 1 Link         |
| Link 7-2    | Outer Open Angle 2 Link         |
| Link 8      | Tool Tip Link                   |

##### Joints

| Joint Number | Joint Name              | Parent Link              | Child Link                   | Type       |
|--------------|-------------------------|--------------------------|------------------------------|------------|
| Joint 5      | Outer Wrist Pitch Joint | Link 4-1                 | Link 5                       | Revolute   |
| Joint 6      | Outer Wrist Yaw Joint   | Link 5                   | Link 6                       | Revolute   |
| Joint 7-0    | Outer Open Angle Joint  | Link 6                   | Link 7-0                     | Revolute   |
| Joint 7-1    | Outer Open Angle 1 Joint| Link 6                   | Link 7-1                     | Revolute   |
| Joint 7-2    | Outer Open Angle 2 Joint| Link 6                   | Link 7-2                     | Revolute   |
| Joint 7-3    | Tool Tip Joint          | Link 6                   | Link 8                       | Fixed      |

---

#### Snake Tool

##### Links

| Link Number | Link Name                       |
|-------------|---------------------------------|
| Link 6      | Tool Snake Disc 1 Link          |
| Link 7      | Tool Snake Disc 2 Link          |
| Link 8      | Tool Snake Disc 3 Link          |
| Link 9      | Tool Snake End Link             |
| Link 11-0   | Outer Open Angle Virtual Link   |
| Link 11-1   | Outer Open Angle 1 Link         |
| Link 11-2   | Outer Open Angle 2 Link         |

##### Joints

| Joint Number | Joint Name              | Parent Link              | Child Link                   | Type       |
|--------------|-------------------------|--------------------------|------------------------------|------------|
| Joint 6      | Outer Wrist Pitch 1 Joint| Link 4-1                | Link 6                       | Revolute   |
| Joint 7      | Outer Wrist Yaw 1 Joint | Link 6                   | Link 7                       | Revolute   |
| Joint 8      | Outer Wrist Yaw 2 Joint | Link 7                   | Link 8                       | Revolute   |
| Joint 9      | Outer Wrist Pitch 2 Joint| Link 8                  | Link 9                       | Revolute   |
| Joint 11-0   | Outer Open Angle Joint  | Link 9                   | Link 11-0                    | Revolute   |
| Joint 11-1   | Outer Open Angle 1 Joint| Link 9                   | Link 11-1                    | Revolute   |
| Joint 11-2   | Outer Open Angle 2 Joint| Link 9                   | Link 11-2                    | Revolute   |

---

## PSM Si

### Launching the PSM Si Model

To launch the PSM Si model, use the following command:

```bash
ros2 launch psm_description urdf.launch.py file:=psm.si.urdf.xacro
```

### Changing the Tool Type

Change the tool type in the `psm.si.urdf.xacro` file:

```xml
<!-- Tool Arguments -->
<!-- Select tool_name from P420006, SF826001 -->
<xacro:property name="tool_name" default="P420006" />
```

---

### PSM Si Base Xacro

#### Links

| Link Number | Link Name           | Mesh File                            |
|-------------|---------------------|--------------------------------------|
| Link 0      | Link 0              | [link_0.stl](./meshes/Si/link_0.stl) |
| Link 1      | Link 1              | [link_1.stl](./meshes/Si/link_1.stl) |
| Link 2      | Link 2              | [link_2.stl](./meshes/Si/link_2.stl) |
| Link 3      | Link 3              | [link_3.stl](./meshes/Si/link_3.stl) |
| Link 4      | Link 4              | [link_4.stl](./meshes/Si/link_4.stl) |
| Link 5      | Tool Parent Link    | N/A                                  |
| Link 6      | RCM Link            | N/A                                  |

#### Joints

| Joint Number | Joint Name                      | Parent Link          | Child Link         | Type    |
|--------------|---------------------------------|----------------------|--------------------|---------|
| Joint 0      | Fixed Joint                     | Macro Parent Link    | Link 0             | Fixed   |
| Joint 1      | Yaw Joint                       | Link 0               | Link 1             | Revolute|
| Joint 2      | Pitch Joint                     | Link 1               | Link 2             | Revolute|
| Joint 3      | Pitch Joint                     | Link 2               | Link 3             | Revolute|
| Joint 4      | Pitch Joint                     | Link 3               | Link 4             | Revolute|
| Joint 5      | Outer Insertion Reference Joint | Link 4               | Link 5             | Fixed   |
| Joint 6      | RCM Joint                       | Link 4               | Link 6             | Fixed   |

---

### PSM Si Tool Xacros

#### P420006 Tool

##### Links

| Link Number | Link Name                   | Mesh File                                                     |
|-------------|-----------------------------|---------------------------------------------------------------|
| Link 0      | Tool Main Link              | [tool_main_link.stl](./meshes/P420006/tool_main_link.stl)     |
| Link 1      | Tool Wrist Link             | [tool_wrist_link.stl](./meshes/P420006/tool_wrist_link.stl)   |
| Link 2      | Tool Wrist Shaft Link       | N/A                                                           |
| Link 3      | Tool Wrist SCA Link         | N/A                                                           |
| Link 4      | Tool Wrist SCA Shaft Link   | N/A                                                           |
| Link 5      | Tool Wrist SCA EE Link      | N/A                                                           |
| Link 6      | Tool Wrist SCA EE Link 1    | N/A                                                           |
| Link 7      | Tool Wrist SCA EE Link 2    | N/A                                                           |

##### Joints

| Joint Number | Joint Name              | Parent Link              | Child Link                   | Type       |
|--------------|-------------------------|--------------------------|------------------------------|------------|
| Joint 0      | Insertion Joint         | Parent Link              | Link 0                       | Prismatic  |
| Joint 1      | Roll Joint              | Link 0                   | Link 1                       | Revolute   |
| Joint 2      | Roll Shaft Joint        | Link 1                   | Link 2                       | Fixed      |
| Joint 3      | Wrist Pitch Joint       | Link 2                   | Link 3                       | Revolute   |
| Joint 4      | Wrist Yaw Joint         | Link 3                   | Link 4                       | Revolute   |
| Joint 5      | Jaw Joint               | Link 4                   | Link 5                       | Revolute   |
| Joint 6      | Jaw 1 Joint             | Link 4                   | Link 6                       | Revolute   |
| Joint 7      | Jaw 2 Joint             | Link 4                   | Link 7                       | Revolute   |

---

#### SF826001 Tool

##### Links

| Link Number | Link Name                   | Mesh File                                                     |
|-------------|-----------------------------|---------------------------------------------------------------|
| Link 0      | Tool Main Link              | [tool_main_link.stl](./meshes/SF826001/tool_main_link.stl)    |
| Link 1      | Tool Roll Link              | [tool_roll_link.stl](./meshes/SF826001/tool_roll_link.stl)    |

##### Joints

| Joint Number | Joint Name              | Parent Link              | Child Link                   | Type       |
|--------------|-------------------------|--------------------------|------------------------------|------------|
| Joint 0      | Insertion Joint         | Parent Link              | Link 0                       | Prismatic  |
| Joint 1      | Roll Joint              | Link 0                   | Link 1                       | Revolute   |

---

## URDF Structures

The URDF files define the robot models using XML and XACRO macros. They include definitions for links, joints, and other robot properties.

- **`both_psms.classic.urdf.xacro`**: Defines a model with two PSM Classic manipulators.
- **`psm.classic.urdf.xacro`**: Defines a single PSM Classic manipulator with configurable tools.
- **`psm.si.urdf.xacro`**: Defines a PSM Si manipulator with configurable tools.

---

## Notes

- **Tool Configurations**: Different tools can be mounted on the PSMs by changing the `tool_name` parameter in the URDF files.
- **Meshes**: All meshes are stored in the `meshes` directory and are referenced in the URDF files.

---
