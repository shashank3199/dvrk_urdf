# daVinci Core Description

The da Vinci Surgical System model includes two arms (left and right). This package provides the URDF descriptions, launch files, and configurations necessary to simulate and visualize the da Vinci Surgical System's core components in ROS 2.

## Table of Contents

-   [Package Structure](#package-structure)
-   [URDF and Xacro Files](#urdf-and-xacro-files)
    -   [URDF Directory (`urdf/`)](#urdf-directory-urdf)
    -   [Main URDF File (`daVinci.urdf.xacro`)](#main-urdf-file-davinciurdfxacro)
    -   [Xacro Macros (`urdf/xacros/`)](#xacro-macros-urdfxacros)
    -   [ROS 2 Control Configurations (`urdf/ros2_control/`)](#ros-2-control-configurations-urdfros2_control)
-   [Meshes](#meshes)
-   [Controllers](#controllers)
-   [RViz Configurations](#rviz-configurations)
-   [Building the Package](#building-the-package)
-   [Dependencies](#dependencies)
-   [Usage Examples](#usage-examples)
    -   [Simulate the Robot with Controllers](#simulate-the-robot-with-controllers)
    -   [Visualize and Manipulate the Robot with GUI](#visualize-and-manipulate-the-robot-with-gui)
-   [Additional Resources](#additional-resources)

## Package Structure

```
davinci_core_description
├── CMakeLists.txt
├── config
│   └── davinci_core.controllers.yaml
├── launch
│   ├── davinci_core_bringup.launch.py
│   └── view_robot.launch.py
├── meshes
│   ├── mounting_base.stl
│   ├── outer_insertion.stl
│   ├── outer_pitch_base.stl
│   ├── outer_pitch_bottom.stl
│   ├── outer_pitch_front.stl
│   ├── outer_pitch_top.stl
│   ├── outer_yaw.stl
│   ├── slave_frame.stl
│   ├── tool_adaptor.stl
│   └── tool_asm.stl
├── package.xml
├── README.md
├── rviz
│   └── davinci_core_description.rviz
├── src
│   └── davinci_core_joint_controller.cpp
└── urdf
    ├── daVinci.urdf.xacro
    ├── README.md
    ├── ros2_control
    │   ├── daVinci.arm.ros2_control.xacro
    │   └── README.md
    └── xacros
        ├── daVinci.arm.xacro
        └── README.md
```

## URDF and Xacro Files

### URDF Directory (`urdf/`)

-   Contains the Unified Robot Description Format (URDF) files and Xacro macros defining the robot's model.
-   **README:** See [`urdf/README.md`](./urdf/README.md) for detailed information.

### Main URDF File (`daVinci.urdf.xacro`)

-   Located at [`urdf/daVinci.urdf.xacro`](./urdf/daVinci.urdf.xacro).
-   Includes robot definitions and integrates macros.

### Xacro Macros (`urdf/xacros/`)

-   Contains reusable macros for building the robot's URDF.
-   Files:
    -   [`daVinci.arm.xacro`](./urdf/xacros/daVinci.arm.xacro): Macro defining an arm of the da Vinci robot.
-   **README:** See [`urdf/xacros/README.md`](./urdf/xacros/README.md) for more details on Xacro macros.

### ROS 2 Control Configurations (`urdf/ros2_control/`)

-   Contains Xacro files defining the ROS 2 control interfaces.
-   Files:
    -   [`daVinci.arm.ros2_control.xacro`](./urdf/ros2_control/daVinci.arm.ros2_control.xacro): Defines the control interfaces for the robot's arm.
-   **README:** See [`urdf/ros2_control/README.md`](./urdf/ros2_control/README.md) for more information on ROS 2 control configurations.

## Meshes

-   **Meshes Directory (`meshes/`)**
    -   Contains STL files representing the visual and collision geometry of the robot's components.
    -   Key files include:
        -   `slave_frame.stl`: Mesh for the mounting platform.
        -   `mounting_base.stl`, `outer_yaw.stl`, `outer_pitch_base.stl`, etc.: Meshes for various robot parts.

## Controllers

-   **Controller Configuration (`davinci_core.controllers.yaml`)**

    -   Located at [`config/davinci_core.controllers.yaml`](./config/davinci_core.controllers.yaml).
    -   Defines the controllers to be used with the robot.
    -   Controllers include:
        -   `joint_state_broadcaster`: Publishes the joint states.
        -   `forward_position_controller`: Controls joint positions.

-   **Custom Joint Controller Source (`davinci_core_joint_controller.cpp`)**
    -   Located at [`src/davinci_core_joint_controller.cpp`](./src/davinci_core_joint_controller.cpp).
    -   Source code for the custom joint controller node.
    -   Implements specific control logic for the da Vinci core robot.

## RViz Configurations

-   **RViz Configuration (`davinci_core_description.rviz`)**
    -   Located at [`rviz/davinci_core_description.rviz`](./rviz/davinci_core_description.rviz).
    -   Pre-configured settings for RViz visualization.
    -   Sets up the display to visualize the robot model and its state.

## Building the Package

Ensure you have a ROS 2 workspace set up. Clone the `davinci_core_description` package into the `src` directory of your workspace, and then build the workspace:

```bash
colcon build
```

Source your workspace after building:

```bash
source install/setup.bash
```

## Dependencies

Ensure that you have the following dependencies installed:

-   **ROS 2 (Foxy/Galactic/Humble)**
-   **robot_state_publisher**
-   **joint_state_publisher_gui**
-   **ros2_control**
-   **rviz2**

## Usage Examples

### Simulate the Robot with Controllers

To simulate the da Vinci core robot with ROS 2 control and visualize it in RViz:

```bash
ros2 launch davinci_core_description davinci_core_bringup.launch.py
```

-   **Launch File:** [`launch/davinci_core_bringup.launch.py`](./launch/davinci_core_bringup.launch.py)
-   **Launch Files README:** See [`launch/README.md`](./launch/README.md) for detailed explanations of the launch files.

### Visualize and Manipulate the Robot with GUI

To launch the robot and manipulate its joints using the Joint State Publisher GUI:

```bash
ros2 launch davinci_core_description view_robot.launch.py
```

-   **Launch File:** [`launch/view_robot.launch.py`](./launch/view_robot.launch.py)
-   **Launch Files README:** See [`launch/README.md`](./launch/README.md) for detailed explanations of the launch files.

## Additional Resources

-   **URDF README:** [`urdf/README.md`](./urdf/README.md) - Detailed documentation on the URDF files.
-   **Xacro Macros README:** [`urdf/xacros/README.md`](./urdf/xacros/README.md) - Information on Xacro macros used in the robot description.
-   **ROS 2 Control README:** [`urdf/ros2_control/README.md`](./urdf/ros2_control/README.md) - Details about the ROS 2 control configurations.
-   **Launch Files README:** [`launch/README.md`](./launch/README.md) - Explanations of the provided launch files.
