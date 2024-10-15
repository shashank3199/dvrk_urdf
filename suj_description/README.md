# SUJ Description

The Setup Joint (SUJ) Description package provides the URDF descriptions, launch files, and configurations necessary to simulate and visualize the Setup Joints of the da Vinci Surgical System in ROS 2. This package includes models for both Classic and Si versions of the SUJ.

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
    -   [Simulate the SUJ with Controllers](#simulate-the-suj-with-controllers)
    -   [Visualize and Manipulate the SUJ with GUI](#visualize-and-manipulate-the-suj-with-gui)
-   [Additional Resources](#additional-resources)

## Package Structure

```
suj_description
├── CMakeLists.txt
├── config
│   └── suj.controllers.yaml
├── launch
│   ├── README.md
│   ├── suj_bringup.launch.py
│   └── view_robot.launch.py
├── meshes
│   ├── Classic
│   └── Si
├── package.xml
├── README.md
├── rviz
│   └── suj_description.rviz
├── src
│   └── suj_joint_controller.cpp
└── urdf
    ├── README.md
    ├── ros2_control
    ├── suj.classic.urdf.xacro
    ├── suj.si.urdf.xacro
    └── xacros
        ├── Classic
        └── Si
```

## URDF and Xacro Files

### URDF Directory (`urdf/`)

-   Contains the Unified Robot Description Format (URDF) files and Xacro macros defining the SUJ models.
-   **README:** See [`urdf/README.md`](./urdf/README.md) for detailed information.

### Main URDF Files

-   [`suj.classic.urdf.xacro`](./urdf/suj.classic.urdf.xacro): Main URDF file for the Classic SUJ model.
-   [`suj.si.urdf.xacro`](./urdf/suj.si.urdf.xacro): Main URDF file for the Si SUJ model.

### Xacro Macros (`urdf/xacros/`)

-   Contains reusable macros for building the SUJ URDF.
-   Classic SUJ macros:
    -   [`ecmj.xacro`](./urdf/xacros/Classic/ecmj.xacro): Macro for the ECM SUJ.
    -   [`psm1j.xacro`](./urdf/xacros/Classic/psm1j.xacro): Macro for PSM1 SUJ.
    -   [`psm2j.xacro`](./urdf/xacros/Classic/psm2j.xacro): Macro for PSM2 SUJ.
    -   [`psm3j.xacro`](./urdf/xacros/Classic/psm3j.xacro): Macro for PSM3 SUJ.
-   Si SUJ macros:
    -   [`ecmj.xacro`](./urdf/xacros/Si/ecmj.xacro): Macro for the ECM SUJ (Si version).
    -   [`psm12j.xacro`](./urdf/xacros/Si/psm12j.xacro): Macro for PSM1 and PSM2 SUJ (Si version).
    -   [`psm3j.xacro`](./urdf/xacros/Si/psm3j.xacro): Macro for PSM3 SUJ (Si version).
-   **README:** See [`urdf/xacros/Classic/README.md`](./urdf/xacros/Classic/README.md) and [`urdf/xacros/Si/README.md`](./urdf/xacros/Si/README.md) for more details on Xacro macros.

### ROS 2 Control Configurations (`urdf/ros2_control/`)

-   Contains Xacro files defining the ROS 2 control interfaces.
-   Files:
    -   [`ecmj.ros2_control.xacro`](./urdf/ros2_control/ecmj.ros2_control.xacro): Defines the control interfaces for the ECM SUJ.
    -   [`psmj.ros2_control.xacro`](./urdf/ros2_control/psmj.ros2_control.xacro): Defines the control interfaces for the PSM SUJs.
-   **README:** See [`urdf/ros2_control/README.md`](./urdf/ros2_control/README.md) for more information on ROS 2 control configurations.

## Meshes

-   **Meshes Directory (`meshes/`)**
    -   Contains STL files representing the visual and collision geometry of the SUJ components.
    -   Subdirectories:
        -   `Classic/`: Meshes for the Classic SUJ model.
        -   `Si/`: Meshes for the Si SUJ model, further divided into `ECM/` and `PSM/` subdirectories.

## Controllers

-   **Controller Configuration (`suj.controllers.yaml`)**

    -   Located at [`config/suj.controllers.yaml`](./config/suj.controllers.yaml).
    -   Defines the controllers to be used with the SUJ.

-   **Custom Joint Controller Source (`suj_joint_controller.cpp`)**
    -   Located at [`src/suj_joint_controller.cpp`](./src/suj_joint_controller.cpp).
    -   Source code for the custom joint controller node.
    -   Implements specific control logic for the SUJ.

## RViz Configurations

-   **RViz Configuration (`suj_description.rviz`)**
    -   Located at [`rviz/suj_description.rviz`](./rviz/suj_description.rviz).
    -   Pre-configured settings for RViz visualization of the SUJ.

## Building the Package

Ensure you have a ROS 2 workspace set up. Clone the `suj_description` package into the `src` directory of your workspace, and then build the workspace:

```bash
colcon build
```

Source your workspace after building:

```bash
source install/setup.bash
```

## Usage Examples

### Simulate the SUJ with Controllers

To simulate the SUJ with ROS 2 control and visualize it in RViz:

```bash
ros2 launch suj_description suj_bringup.launch.py
```

-   **Launch File:** [`launch/suj_bringup.launch.py`](./launch/suj_bringup.launch.py)
-   **Launch Files README:** See [`launch/README.md`](./launch/README.md) for detailed explanations of the launch files.

### Visualize and Manipulate the SUJ with GUI

To launch the SUJ and manipulate its joints using the Joint State Publisher GUI:

```bash
ros2 launch suj_description view_robot.launch.py
```

-   **Launch File:** [`launch/view_robot.launch.py`](./launch/view_robot.launch.py)
-   **Launch Files README:** See [`launch/README.md`](./launch/README.md) for detailed explanations of the launch files.

## Additional Resources

-   **URDF README:** [`urdf/README.md`](./urdf/README.md) - Detailed documentation on the URDF files.
-   **Xacro Macros README (Classic):** [`urdf/xacros/Classic/README.md`](./urdf/xacros/Classic/README.md) - Information on Xacro macros used in the Classic SUJ description.
-   **Xacro Macros README (Si):** [`urdf/xacros/Si/README.md`](./urdf/xacros/Si/README.md) - Information on Xacro macros used in the Si SUJ description.
-   **ROS 2 Control README:** [`urdf/ros2_control/README.md`](./urdf/ros2_control/README.md) - Details about the ROS 2 control configurations.
-   **Launch Files README:** [`launch/README.md`](./launch/README.md) - Explanations of the provided launch files.
