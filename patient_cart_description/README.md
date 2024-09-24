# Patient Cart Description

Patient Cart Classic

```bash
ros2 launch patient_cart_description urdf.launch.py file:=patient_cart.classic.urdf.xacro
```

Patient Cart Si

```bash
ros2 launch patient_cart_description urdf.launch.py file:=patient_cart.si.urdf.xacro
```

---

## Patient Cart Classic URDF

### Links

| Link Number | Link Name             | Mesh File                                                                     | Origin (`xyz`)              | Orientation (`rpy`)   |
|-------------|-----------------------|-------------------------------------------------------------------------------|-----------------------------|-----------------------|
| Link 0      | World                 | N/A                                                                           | N/A                         | N/A                   |
| Link 1      | Base Link             | [base_link.stl](./meshes/Classic/base_link.stl)                               | `0 0 0`                     | `0 0 0`               |
| Link 2      | PSM2 Mounting Point   | Defined in [psm2j.xacro](./suj_description/urdf/xacros/Classic/psm2j.xacro)   | `0.1912 -0.1016 0.7611`     | `-π 0 π`              |
| Link 3      | ECM Mounting Point    | Defined in [ecmj.xacro](./suj_description/urdf/xacros/Classic/ecmj.xacro)     | `0 0.0896 0.7611`           | `π 0 -π/2`            |
| Link 4      | PSM1 Mounting Point   | Defined in [psm1j.xacro](./suj_description/urdf/xacros/Classic/psm1j.xacro)   | `-0.1912 -0.1016 0.7611`    | `π 0 0`               |
| Link 5      | PSM3 Mounting Point   | Defined in [psm3j.xacro](./suj_description/urdf/xacros/Classic/psm3j.xacro)   | `0 0.0896 0.3813`           | `π 0 -π/2`            |
| ...         | ...                   | ...                                                                           | ...                         | ...                   |

### Joints

| Joint Number | Joint Name            | Associated Links (Parent to Child)   | Type         | Origin (`xyz`)              | Orientation (`rpy`)   |
|--------------|-----------------------|--------------------------------------|--------------|-----------------------------|-----------------------|
| Joint 0      | Fixed                 | `world` to `base_link`               | Fixed        | `0 0 0.167458`              | `0 0 0`               |
| Joint 1      | PSM2 Joint            | `base_link` to `PSM2_mounting_point` | As per macro | `0.1912 -0.1016 0.7611`     | `-π 0 π`              |
| Joint 2      | ECM Joint             | `base_link` to `ECM_mounting_point`  | As per macro | `0 0.0896 0.7611`           | `π 0 -π/2`            |
| Joint 3      | PSM1 Joint            | `base_link` to `PSM1_mounting_point` | As per macro | `-0.1912 -0.1016 0.7611`    | `π 0 0`               |
| Joint 4      | PSM3 Joint            | `base_link` to `PSM3_mounting_point` | As per macro | `0 0.0896 0.3813`           | `π 0 -π/2`            |
| ...          | ...                   | ...                                  | ...          | ...                         | ...                   |

**Note:** The specific definitions of the links and joints for the PSMs and ECM are included via macros. The macros `psm2j.xacro`, `ecmj.xacro`, `psm1j.xacro`, and `psm3j.xacro` define the mounting points and configurations for the PSMs and ECM.

Each PSM and the ECM also include their own links and joints, defined in their respective description packages (`psm_description` and `ecm_description`), which are included via xacro macros.

---

## Patient Cart Si URDF

### Links

| Link Number | Link Name             | Mesh File                                                                                   | Origin (`xyz`)   | Orientation (`rpy`)   |
|-------------|-----------------------|---------------------------------------------------------------------------------------------|------------------|-----------------------|
| Link 0      | World                 | N/A                                                                                         | N/A              | N/A                   |
| Link 1      | Base Link             | [base_link.stl](./meshes/Classic/base_link.stl)                                             | `0 0 0`          | `0 0 0`               |
| Link 2      | PSM2 Mounting Point   | Defined in [psm12j.xacro](./suj_description/urdf/xacros/Si/psm12j.xacro) with `prefix="2"`  | `0 -0.228 0.528` | `0 0 -π/2`            |
| Link 3      | ECM Mounting Point    | Defined in [ecmj.xacro](./suj_description/urdf/xacros/Si/ecmj.xacro)                        | `0.173 0 0.478`  | `0 0 0`               |
| Link 4      | PSM1 Mounting Point   | Defined in [psm12j.xacro](./suj_description/urdf/xacros/Si/psm12j.xacro) with `prefix="1"`  | `0 0.228 0.528`  | `0 0 π/2`             |
| Link 5      | PSM3 Mounting Point   | Defined in [psm3j.xacro](./suj_description/urdf/xacros/Si/psm3j.xacro)                      | `-0.223 0 0.528` | `0 0 -π`              |
| ...         | ...                   | ...                                                                                         | ...              | ...                   |

### Joints

| Joint Number | Joint Name            | Associated Links (Parent to Child)   | Type         | Origin (`xyz`)           | Orientation (`rpy`)   |
|--------------|-----------------------|--------------------------------------|--------------|--------------------------|-----------------------|
| Joint 0      | Fixed                 | `world` to `base_link`               | Fixed        | `0 0 0.167458`           | `0 0 0`               |
| Joint 1      | PSM2 Joint            | `base_link` to `PSM2_mounting_point` | As per macro | `0 -0.228 0.528`         | `0 0 -π/2`            |
| Joint 2      | ECM Joint             | `base_link` to `ECM_mounting_point`  | As per macro | `0.173 0 0.478`          | `0 0 0`               |
| Joint 3      | PSM1 Joint            | `base_link` to `PSM1_mounting_point` | As per macro | `0 0.228 0.528`          | `0 0 π/2`             |
| Joint 4      | PSM3 Joint            | `base_link` to `PSM3_mounting_point` | As per macro | `-0.223 0 0.528`         | `0 0 -π`              |
| ...          | ...                   | ...                                  | ...          | ...                      | ...                   |

**Note:** As with the Classic Patient Cart, the specific definitions of the links and joints for the PSMs and ECM are included via macros. The macros `psm12j.xacro` and `ecmj.xacro` define the mounting points and configurations for the PSMs and ECM.

Each PSM and the ECM also include their own links and joints, defined in their respective description packages (`psm_description` and `ecm_description`), which are included via xacro macros.

---

**Notes:**

- The Patient Cart model includes multiple subsystems:
  - **PSMs (Patient Side Manipulators):** Robotic arms used for surgical procedures.
  - **ECM (Endoscope Camera Manipulator):** Controls the endoscopic camera.

- The specific configurations and definitions of the PSMs and ECM are included via xacro macros from their respective description packages.

- To fully understand the structure and details of the PSMs and ECM, refer to their individual README files and description packages.

---
