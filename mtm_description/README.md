Here's a README file for the `mtm_description` project based on the template and the provided project structure:

# MTM Description

The Master Tool Manipulator (MTM) is a component of the da Vinci Surgical System. This package provides the URDF descriptions, launch files, and configurations necessary to simulate and visualize the MTM in ROS 2.

## Table of Contents

-   [Package Structure](#package-structure)
-   [URDF and Xacro Files](#urdf-and-xacro-files)
    -   [URDF Directory (`urdf/`)](#urdf-directory-urdf)
    -   [Main URDF File (`mtms.urdf.xacro`)](#main-urdf-file-mtmsurdfxacro)
    -   [Xacro Macros (`urdf/xacros/`)](#xacro-macros-urdfxacros)
    -   [ROS 2 Control Configurations (`urdf/ros2_control/`)](#ros-2-control-configurations-urdfros2_control)
-   [Meshes](#meshes)
-   [Controllers](#controllers)
-   [RViz Configurations](#rviz-configurations)
-   [Building the Package](#building-the-package)
-   [Usage Examples](#usage-examples)
    -   [Simulate the Robot with Controllers](#simulate-the-robot-with-controllers)
    -   [Visualize and Manipulate the Robot with GUI](#visualize-and-manipulate-the-robot-with-gui)
-   [Additional Resources](#additional-resources)

## Package Structure

```
mtm_description
├── CMakeLists.txt
├── config
│   └── mtms.controllers.yaml
├── launch
│   ├── mtms_bringup.launch.py
│   ├── README.md
│   └── view_robot.launch.py
├── meshes
│   ├── ArmParallel1.dae
│   ├── ArmParallel1.stl
│   ├── ArmParallel.dae
│   ├── ArmParallel.stl
│   ├── BottomArm.dae
│   ├── BottomArm.stl
│   ├── Link.dae
│   ├── Link.stl
│   ├── OutPitch_Shoulder.dae
│   ├── OutPitch_Shoulder.stl
│   ├── TopPanel.dae
│   ├── TopPanel.stl
│   ├── WristPitch.dae
│   ├── WristPitch.stl
│   ├── WristPlatform.dae
│   ├── WristPlatform.stl
│   ├── WristRoll.dae
│   ├── WristRoll.stl
│   ├── WristYaw.dae
│   └── WristYaw.stl
├── package.xml
├── README.md
├── resources
│   ├── mtm_omponents.blend
│   ├── mtm_wrist_pitch.png
│   ├── mtm_wrist_platform.png
│   └── mtm_wrist_yaw.png
├── rviz
│   └── mtm_description.rviz
├── src
│   └── mtms_joint_controller.cpp
└── urdf
    ├── mtms.urdf.xacro
    ├── README.md
    ├── ros2_control
    │   ├── mtm.ros2_control.xacro
    │   └── README.md
    └── xacros
        ├── common.xacro
        ├── mtm.xacro
        └── README.md
```

## URDF and Xacro Files

### URDF Directory (`urdf/`)

-   Contains the Unified Robot Description Format (URDF) files and Xacro macros defining the MTM model.
-   **README:** See [`urdf/README.md`](./urdf/README.md) for detailed information.

### Main URDF File (`mtms.urdf.xacro`)

-   Located at [`urdf/mtms.urdf.xacro`](./urdf/mtms.urdf.xacro).
-   Includes MTM definitions and integrates macros.

### Xacro Macros (`urdf/xacros/`)

-   Contains reusable macros for building the MTM's URDF.
-   Files:
    -   [`common.xacro`](./urdf/xacros/common.xacro): Common macros used across the MTM description.
    -   [`mtm.xacro`](./urdf/xacros/mtm.xacro): Main macro defining the MTM structure.
-   **README:** See [`urdf/xacros/README.md`](./urdf/xacros/README.md) for more details on Xacro macros.

### ROS 2 Control Configurations (`urdf/ros2_control/`)

-   Contains Xacro files defining the ROS 2 control interfaces for the MTM.
-   Files:
    -   [`mtm.ros2_control.xacro`](./urdf/ros2_control/mtm.ros2_control.xacro): Defines the control interfaces for the MTM.
-   **README:** See [`urdf/ros2_control/README.md`](./urdf/ros2_control/README.md) for more information on ROS 2 control configurations.

## Meshes

-   **Meshes Directory (`meshes/`)**
    -   Contains DAE and STL files representing the visual and collision geometry of the MTM's components.
    -   Key files include meshes for various parts of the arm, wrist, and other components.

## Controllers

-   **Controller Configuration (`mtms.controllers.yaml`)**

    -   Located at [`config/mtms.controllers.yaml`](./config/mtms.controllers.yaml).
    -   Defines the controllers to be used with the MTM.

-   **Custom Joint Controller Source (`mtms_joint_controller.cpp`)**
    -   Located at [`src/mtms_joint_controller.cpp`](./src/mtms_joint_controller.cpp).
    -   Source code for the custom joint controller node.
    -   Implements specific control logic for the MTM.

## RViz Configurations

-   **RViz Configuration (`mtm_description.rviz`)**
    -   Located at [`rviz/mtm_description.rviz`](./rviz/mtm_description.rviz).
    -   Pre-configured settings for RViz visualization of the MTM.

## Building the Package

Ensure you have a ROS 2 workspace set up. Clone the `mtm_description` package into the `src` directory of your workspace, and then build the workspace:

```bash
colcon build
```

Source your workspace after building:

```bash
source install/setup.bash
```

## Usage Examples

### Simulate the Robot with Controllers

To simulate the MTM with ROS 2 control and visualize it in RViz:

```bash
ros2 launch mtm_description mtms_bringup.launch.py
```

-   **Launch File:** [`launch/mtms_bringup.launch.py`](./launch/mtms_bringup.launch.py)
-   **Launch Files README:** See [`launch/README.md`](./launch/README.md) for detailed explanations of the launch files.

### Visualize and Manipulate the Robot with GUI

To launch the MTM and manipulate its joints using the Joint State Publisher GUI:

```bash
ros2 launch mtm_description view_robot.launch.py
```

-   **Launch File:** [`launch/view_robot.launch.py`](./launch/view_robot.launch.py)
-   **Launch Files README:** See [`launch/README.md`](./launch/README.md) for detailed explanations of the launch files.

## Additional Resources

-   **URDF README:** [`urdf/README.md`](./urdf/README.md) - Detailed documentation on the URDF files.
-   **Xacro Macros README:** [`urdf/xacros/README.md`](./urdf/xacros/README.md) - Information on Xacro macros used in the MTM description.
-   **ROS 2 Control README:** [`urdf/ros2_control/README.md`](./urdf/ros2_control/README.md) - Details about the ROS 2 control configurations.
-   **Launch Files README:** [`launch/README.md`](./launch/README.md) - Explanations of the provided launch files.
-   **Resources:** The `resources/` directory contains additional files such as Blender models and images that may be useful for understanding the MTM's structure.
