# ECM Description

The Endoscopic Camera Manipulator (ECM) package provides the URDF descriptions, launch files, and configurations necessary to simulate and visualize the ECM in ROS 2. This package includes both the ECM base and the endoscopic arm components.

## Table of Contents

-   [Package Structure](#package-structure)
-   [URDF and Xacro Files](#urdf-and-xacro-files)
    -   [URDF Directory (`urdf/`)](#urdf-directory-urdf)
    -   [Main URDF Files](#main-urdf-files)
    -   [Xacro Macros (`urdf/xacros/`)](#xacro-macros-urdfxacros)
    -   [ROS 2 Control Configurations (`urdf/ros2_control/`)](#ros-2-control-configurations-urdfros2_control)
-   [Meshes](#meshes)
-   [Controllers](#controllers)
-   [RViz Configurations](#rviz-configurations)
-   [Building the Package](#building-the-package)
-   [Usage Examples](#usage-examples)
    -   [Simulate the ECM with Controllers](#simulate-the-ecm-with-controllers)
    -   [Visualize and Manipulate the ECM with GUI](#visualize-and-manipulate-the-ecm-with-gui)
-   [Additional Resources](#additional-resources)

## Package Structure

```
ecm_description
├── CMakeLists.txt
├── config
│   ├── ecm.base.controllers.yaml
│   └── ecm.controllers.yaml
├── launch
│   ├── ecm_base_bringup.launch.py
│   ├── ecm_bringup.launch.py
│   ├── README.md
│   └── view_robot.launch.py
├── meshes
│   ├── ecm
│   │   └── [ECM arm mesh files]
│   └── ecm_base
│       └── [ECM base mesh files]
├── package.xml
├── README.md
├── rviz
│   └── ecm_description.rviz
├── src
│   └── ecm_joint_controller.cpp
└── urdf
    ├── ecm.base.urdf.xacro
    ├── ecm.urdf.xacro
    ├── README.md
    ├── ros2_control
    │   ├── ecm.base.ros2_control.xacro
    │   ├── ecm.ros2_control.xacro
    │   └── README.md
    └── xacros
        ├── common.xacro
        ├── ecm.base.xacro
        ├── ecm.xacro
        └── README.md
```

## URDF and Xacro Files

### URDF Directory (`urdf/`)

-   Contains the Unified Robot Description Format (URDF) files and Xacro macros defining the ECM model.
-   **README:** See [`urdf/README.md`](./urdf/README.md) for detailed information.

### Main URDF Files

-   [`ecm.base.urdf.xacro`](./urdf/ecm.base.urdf.xacro): Defines the ECM base structure.
-   [`ecm.urdf.xacro`](./urdf/ecm.urdf.xacro): Defines the complete ECM structure, including the base and arm.

### Xacro Macros (`urdf/xacros/`)

-   Contains reusable macros for building the ECM's URDF.
-   Files:
    -   [`common.xacro`](./urdf/xacros/common.xacro): Common macros used across the ECM description.
    -   [`ecm.base.xacro`](./urdf/xacros/ecm.base.xacro): Macros for the ECM base.
    -   [`ecm.xacro`](./urdf/xacros/ecm.xacro): Macros for the ECM arm.
-   **README:** See [`urdf/xacros/README.md`](./urdf/xacros/README.md) for more details on Xacro macros.

### ROS 2 Control Configurations (`urdf/ros2_control/`)

-   Contains Xacro files defining the ROS 2 control interfaces for the ECM.
-   Files:
    -   [`ecm.base.ros2_control.xacro`](./urdf/ros2_control/ecm.base.ros2_control.xacro): Defines control interfaces for the ECM base.
    -   [`ecm.ros2_control.xacro`](./urdf/ros2_control/ecm.ros2_control.xacro): Defines control interfaces for the complete ECM.
-   **README:** See [`urdf/ros2_control/README.md`](./urdf/ros2_control/README.md) for more information on ROS 2 control configurations.

## Meshes

-   **ECM Base Meshes (`meshes/ecm_base/`)**
    -   Contains STL files for the ECM base components (e.g., `ecm_base_link.stl`, `ecm_yaw_link.stl`, etc.).
-   **ECM Arm Meshes (`meshes/ecm/`)**
    -   Contains STL files for the ECM arm components (e.g., `Endo_Arm.stl`, `EndoScope.stl`, etc.).

## Controllers

-   **Controller Configurations**

    -   [`config/ecm.base.controllers.yaml`](./config/ecm.base.controllers.yaml): Defines controllers for the ECM base.
    -   [`config/ecm.controllers.yaml`](./config/ecm.controllers.yaml): Defines controllers for the complete ECM.

-   **Custom Joint Controller Source (`ecm_joint_controller.cpp`)**
    -   Located at [`src/ecm_joint_controller.cpp`](./src/ecm_joint_controller.cpp).
    -   Implements specific control logic for the ECM.

## RViz Configurations

-   **RViz Configuration (`ecm_description.rviz`)**
    -   Located at [`rviz/ecm_description.rviz`](./rviz/ecm_description.rviz).
    -   Pre-configured settings for visualizing the ECM in RViz.

## Building the Package

Ensure you have a ROS 2 workspace set up. Clone the `ecm_description` package into the `src` directory of your workspace, and then build the workspace:

```bash
colcon build
```

Source your workspace after building:

```bash
source install/setup.bash
```

## Usage Examples

### Simulate the ECM with Controllers

To simulate the ECM with ROS 2 control and visualize it in RViz:

```bash
ros2 launch ecm_description ecm_bringup.launch.py
```

For the ECM base only:

```bash
ros2 launch ecm_description ecm_base_bringup.launch.py
```

-   **Launch Files:** [`launch/ecm_bringup.launch.py`](./launch/ecm_bringup.launch.py) and [`launch/ecm_base_bringup.launch.py`](./launch/ecm_base_bringup.launch.py)
-   **Launch Files README:** See [`launch/README.md`](./launch/README.md) for detailed explanations of the launch files.

### Visualize and Manipulate the ECM with GUI

To launch the ECM and manipulate its joints using the Joint State Publisher GUI:

```bash
ros2 launch ecm_description view_robot.launch.py
```

-   **Launch File:** [`launch/view_robot.launch.py`](./launch/view_robot.launch.py)
-   **Launch Files README:** See [`launch/README.md`](./launch/README.md) for detailed explanations of the launch files.

## Additional Resources

-   **URDF README:** [`urdf/README.md`](./urdf/README.md) - Detailed documentation on the URDF files.
-   **Xacro Macros README:** [`urdf/xacros/README.md`](./urdf/xacros/README.md) - Information on Xacro macros used in the robot description.
-   **ROS 2 Control README:** [`urdf/ros2_control/README.md`](./urdf/ros2_control/README.md) - Details about the ROS 2 control configurations.
-   **Launch Files README:** [`launch/README.md`](./launch/README.md) - Explanations of the provided launch files.
