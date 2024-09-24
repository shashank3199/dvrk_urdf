# SUJ (Setup Joint) Description

This README provides a comprehensive overview of the **Setup Joint (SUJ)** models, including their URDF (Unified Robot Description Format) structures, meshes, and configurations. The SUJ serves as the mounting and positioning mechanism for the manipulators (PSMs and ECM).

---

## Table of Contents

- [Meshes](#meshes)
- [SUJ Classic](#suj-classic)
  - [Launching the SUJ Classic Model](#launching-the-suj-classic-model)
  - [SUJ Classic ECMJ](#suj-classic-ecmj)
    - [Links](#links)
    - [Joints](#joints)
  - [SUJ Classic PSM1J](#suj-classic-psm1j)
    - [Links](#links-1)
    - [Joints](#joints-1)
  - [SUJ Classic PSM2J](#suj-classic-psm2j)
    - [Links](#links-2)
    - [Joints](#joints-2)
  - [SUJ Classic PSM3J](#suj-classic-psm3j)
    - [Links](#links-3)
    - [Joints](#joints-3)
- [SUJ Si](#suj-si)
  - [Launching the SUJ Si Model](#launching-the-suj-si-model)
  - [SUJ Si PSM12](#suj-si-psm12)
    - [Links](#links-4)
    - [Joints](#joints-4)
  - [SUJ Si PSM3](#suj-si-psm3)
    - [Links](#links-5)
    - [Joints](#joints-5)
  - [SUJ Si ECM](#suj-si-ecm)
    - [Links](#links-6)
    - [Joints](#joints-6)
- [URDF Structures](#urdf-structures)
- [Notes](#notes)

---

## Meshes

The SUJ models utilize various mesh files located in the `meshes` directory. These meshes represent the physical components of the setup joints and are essential for visualizing the robot in simulations.

```
meshes
├── Classic
│   ├── base_link.stl
│   ├── suj_ecm_L0.stl
│   ├── suj_ecm_L1.stl
│   ├── suj_ecm_L2.stl
│   ├── suj_ecm_L3.stl
│   ├── suj_psm1_L0.stl
│   ├── suj_psm1_L1.stl
│   ├── suj_psm1_L2.stl
│   ├── suj_psm1_L3.stl
│   ├── suj_psm1_L4.stl
│   ├── suj_psm2_L0.stl
│   ├── suj_psm2_L1.stl
│   ├── suj_psm2_L2.stl
│   ├── suj_psm2_L3.stl
│   ├── suj_psm2_L4.stl
│   ├── suj_psm3_L0.stl
│   ├── suj_psm3_L1.stl
│   ├── suj_psm3_L2.stl
│   ├── suj_psm3_L3.stl
│   └── suj_psm3_L4.stl
└── Si
    ├── ECM
    │   ├── link_0.stl
    │   ├── link_1.stl
    │   ├── link_2.stl
    │   └── link_3.stl
    ├── PSM
    │   ├── 12
    │   │   ├── link_0.stl
    │   │   ├── link_1.stl
    │   │   ├── link_2.stl
    │   │   └── link_3.stl
    │   └── 3
    │       ├── link_0.stl
    │       ├── link_1.stl
    │       ├── link_2.stl
    │       ├── link_3.stl
    │       └── link_4.stl
    └── tower.stl

6 directories, 34 files
```

---

## SUJ Classic

### Launching the SUJ Classic Model

To launch the SUJ Classic model, use the following command:

```bash
ros2 launch suj_description urdf.launch.py file:=suj.classic.urdf.xacro
```

---

### SUJ Classic ECMJ

#### Links

| Link Number | Link Name           |
|-------------|---------------------|
| Link 8      | ECM Link 0          |
| Link 9      | ECM Link 1          |
| Link 10     | ECM Link 2          |
| Link 11     | ECM Link 3          |
| Link 12     | ECM Mounting Point  |

#### Joints

| Joint Number | Joint Name     | Parent Link | Child Link   | Type       |
|--------------|----------------|-------------|--------------|------------|
| Joint 7      | ECM Joint 0    | Link 1      | Link 8       | Prismatic  |
| Joint 8      | ECM Joint 1    | Link 8      | Link 9       | Continuous |
| Joint 9      | ECM Joint 2    | Link 9      | Link 10      | Continuous |
| Joint 10     | ECM Joint 3    | Link 10     | Link 11      | Continuous |
| Joint 11     | ECM RCM Joint  | Link 11     | Link 12      | Fixed      |

---

### SUJ Classic PSM1J

#### Links

| Link Number | Link Name           |
|-------------|---------------------|
| Link 13     | PSM1 Link 0         |
| Link 14     | PSM1 Link 1         |
| Link 15     | PSM1 Link 2         |
| Link 16     | PSM1 Link 3         |
| Link 17     | PSM1 Link 4         |
| Link 18     | PSM1 Mounting Point |

#### Joints

| Joint Number | Joint Name     | Parent Link | Child Link   | Type       |
|--------------|----------------|-------------|--------------|------------|
| Joint 12     | PSM1 Joint 0   | Link 1      | Link 13      | Prismatic  |
| Joint 13     | PSM1 Joint 1   | Link 13     | Link 14      | Continuous |
| Joint 14     | PSM1 Joint 2   | Link 14     | Link 15      | Continuous |
| Joint 15     | PSM1 Joint 3   | Link 15     | Link 16      | Continuous |
| Joint 16     | PSM1 Joint 4   | Link 16     | Link 17      | Continuous |
| Joint 17     | PSM1 RCM Joint | Link 17     | Link 18      | Fixed      |

---

### SUJ Classic PSM2J

#### Links

| Link Number | Link Name           |
|-------------|---------------------|
| Link 2      | PSM2 Link 0         |
| Link 3      | PSM2 Link 1         |
| Link 4      | PSM2 Link 2         |
| Link 5      | PSM2 Link 3         |
| Link 6      | PSM2 Link 4         |
| Link 7      | PSM2 Mounting Point |

#### Joints

| Joint Number | Joint Name     | Parent Link | Child Link   | Type       |
|--------------|----------------|-------------|--------------|------------|
| Joint 1      | PSM2 Joint 0   | Link 1      | Link 2       | Prismatic  |
| Joint 2      | PSM2 Joint 1   | Link 2      | Link 3       | Continuous |
| Joint 3      | PSM2 Joint 2   | Link 3      | Link 4       | Continuous |
| Joint 4      | PSM2 Joint 3   | Link 4      | Link 5       | Continuous |
| Joint 5      | PSM2 Joint 4   | Link 5      | Link 6       | Continuous |
| Joint 6      | PSM2 RCM Joint | Link 6      | Link 7       | Fixed      |

---

### SUJ Classic PSM3J

#### Links

| Link Number | Link Name           |
|-------------|---------------------|
| Link 19     | PSM3 Link 0         |
| Link 20     | PSM3 Link 1         |
| Link 21     | PSM3 Link 2         |
| Link 22     | PSM3 Link 3         |
| Link 23     | PSM3 Link 4         |
| Link 24     | PSM3 Mounting Point |

#### Joints

| Joint Number | Joint Name     | Parent Link | Child Link   | Type       |
|--------------|----------------|-------------|--------------|------------|
| Joint 18     | PSM3 Joint 0   | Link 1      | Link 19      | Prismatic  |
| Joint 19     | PSM3 Joint 1   | Link 19     | Link 20      | Continuous |
| Joint 20     | PSM3 Joint 2   | Link 20     | Link 21      | Continuous |
| Joint 21     | PSM3 Joint 3   | Link 21     | Link 22      | Continuous |
| Joint 22     | PSM3 Joint 4   | Link 22     | Link 23      | Continuous |
| Joint 23     | PSM3 RCM Joint | Link 23     | Link 24      | Fixed      |

---

## SUJ Si

### Launching the SUJ Si Model

To launch the SUJ Si model, use the following command:

```bash
ros2 launch suj_description urdf.launch.py file:=suj.si.urdf.xacro
```

---

### SUJ Si PSM12

#### Links

| Link Number | Link Name                    |
|-------------|------------------------------|
| Link 2      | PSM 1/2 Joint Link 0         |
| Link 3      | PSM 1/2 Joint Link 1         |
| Link 4      | PSM 1/2 Joint Link 2         |
| Link 5      | PSM 1/2 Joint Link 3         |
| Link 6      | PSM 1/2 Joint Mounting Point |

#### Joints

| Joint Number | Joint Name                   | Parent Link          | Child Link             | Type       |
|--------------|------------------------------|----------------------|------------------------|------------|
| Joint 1      | PSM 1/2 Joint Prismatic      | Link 1               | Link 2                 | Prismatic  |
| Joint 2      | PSM 1/2 Joint Revolute       | Link 2               | Link 3                 | Revolute   |
| Joint 3      | PSM 1/2 Joint Revolute       | Link 3               | Link 4                 | Revolute   |
| Joint 4      | PSM 1/2 Joint Revolute       | Link 4               | Link 5                 | Revolute   |
| Joint 5      | PSM 1/2 Fixed Joint          | Link 5               | Link 6                 | Fixed      |

---

### SUJ Si PSM3

#### Links

| Link Number | Link Name           |
|-------------|---------------------|
| Link 7      | PSM3 Link 0         |
| Link 8      | PSM3 Link 1         |
| Link 9      | PSM3 Link 2         |
| Link 10     | PSM3 Link 3         |
| Link 11     | PSM3 Link 4         |
| Link 12     | PSM3 Mounting Point |

#### Joints

| Joint Number | Joint Name        | Parent Link | Child Link   | Type     |
|--------------|-------------------|-------------|--------------|----------|
| Joint 6      | PSM3 Joint 0      | Link 1      | Link 7       | Prismatic|
| Joint 7      | PSM3 Joint 1      | Link 7      | Link 8       | Revolute |
| Joint 8      | PSM3 Joint 2      | Link 8      | Link 9       | Revolute |
| Joint 9      | PSM3 Joint 3      | Link 9      | Link 10      | Revolute |
| Joint 10     | PSM3 Joint 4      | Link 10     | Link 11      | Revolute |
| Joint 11     | PSM3 Fixed Joint  | Link 11     | Link 12      | Fixed    |

---

### SUJ Si ECM

#### Links

| Link Number | Link Name           |
|-------------|---------------------|
| Link 13     | ECM Link 0          |
| Link 14     | ECM Link 1          |
| Link 15     | ECM Link 2          |
| Link 16     | ECM Link 3          |
| Link 17     | ECM Mounting Point  |

#### Joints

| Joint Number | Joint Name       | Parent Link | Child Link   | Type     |
|--------------|------------------|-------------|--------------|----------|
| Joint 12     | ECM Joint 0      | Link 1      | Link 13      | Prismatic|
| Joint 13     | ECM Joint 1      | Link 13     | Link 14      | Revolute |
| Joint 14     | ECM Joint 2      | Link 14     | Link 15      | Revolute |
| Joint 15     | ECM Joint 3      | Link 15     | Link 16      | Revolute |
| Joint 16     | ECM Fixed Joint  | Link 16     | Link 17      | Fixed    |

---

## Notes

- **Mounting Points**: The SUJ models include mounting points for the PSMs and ECM, facilitating the connection between the setup joints and the manipulators.

---
