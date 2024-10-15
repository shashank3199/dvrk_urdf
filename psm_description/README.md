# PSM Description Package

This package provides the URDF descriptions, launch files, and configurations necessary to simulate and visualize the Patient Side Manipulator (PSM) in ROS 2. It includes support for both Classic and Si versions of the PSM, as well as various tool configurations.

## Table of Contents

-   [Package Structure](#package-structure)
-   [URDF and Xacro Files](#urdf-and-xacro-files)
-   [Meshes](#meshes)
-   [Controllers](#controllers)
-   [RViz Configurations](#rviz-configurations)
-   [Scripts](#scripts)
-   [Building the Package](#building-the-package)
-   [Usage Examples](#usage-examples)
-   [Additional Resources](#additional-resources)

## Package Structure

```
psm_description
├── CMakeLists.txt
├── config/
├── launch/
├── meshes/
├── package.xml
├── README.md
├── rviz/
├── scripts/
├── src/
└── urdf/
```

## URDF and Xacro Files

### URDF Directory (`urdf/`)

-   Contains the Unified Robot Description Format (URDF) files and Xacro macros defining the robot's model.
-   **README:** See `urdf/README.md` for detailed information.

### Main URDF Files

-   `psm.classic.urdf.xacro`: Classic PSM configuration
-   `psm.si.urdf.xacro`: Si PSM configuration
-   `both_psms.classic.urdf.xacro`: Configuration for both Classic PSMs

### Xacro Macros

-   Located in `urdf/xacros/` directory
-   Separate directories for Classic and Si versions
-   **README:** See `urdf/xacros/Classic/README.md` and `urdf/xacros/Si/README.md` for more details

### ROS 2 Control Configurations

-   Located in `urdf/ros2_control/` directory
-   Separate directories for Classic and Si versions
-   **README:** See `urdf/ros2_control/Classic/README.md` and `urdf/ros2_control/Si/README.md` for more information

## Meshes

-   **Meshes Directory (`meshes/`)**
    -   Contains STL and DAE files for various PSM components
    -   Subdirectories:
        -   `Classic/`: Meshes for Classic PSM
        -   `Si/`: Meshes for Si PSM
        -   `P420006/`: Specific tool meshes
        -   `SF826001/`: Specific tool meshes
        -   `snake_tool/`: Snake tool meshes

## Controllers

-   **Controller Configurations**

    -   Located in `config/` directory
    -   Files:
        -   `example.both.classic.controllers.yaml`
        -   `example.psm.classic.controllers.yaml`
        -   `example.psm.si.controllers.yaml`

-   **Custom Joint Controller Source**
    -   Located at `src/psm_joint_controller.cpp`
    -   Implements specific control logic for the PSM

## RViz Configurations

-   **RViz Configuration File**
    -   Located at `rviz/psm_description.rviz`
    -   Pre-configured settings for RViz visualization

## Scripts

-   **Controller Generation Script**
    -   Located at `scripts/generate_controller.py`
    -   Utility for generating controller configurations

## Building the Package

Ensure you have a ROS 2 workspace set up. Clone the `psm_description` package into the `src` directory of your workspace, and then build the workspace:

```bash
colcon build
```

Source your workspace after building:

```bash
source install/setup.bash
```

## Usage Examples

### Launch PSM with Controllers

To launch the PSM with ROS 2 control and visualize it in RViz:

```bash
ros2 launch psm_description psm_bringup.launch.py
```

-   **Launch File:** `launch/psm_bringup.launch.py`
-   **Launch Files README:** See `launch/README.md` for detailed explanations of the launch files.

### Visualize and Manipulate the Robot with GUI

To launch the robot and manipulate its joints using the Joint State Publisher GUI:

```bash
ros2 launch psm_description view_robot.launch.py
```

-   **Launch File:** `launch/view_robot.launch.py`
-   **Launch Files README:** See `launch/README.md` for detailed explanations of the launch files.

## Additional Resources

-   **URDF README:** `urdf/README.md` - Detailed documentation on the URDF files.
-   **Xacro Macros README (Classic):** `urdf/xacros/Classic/README.md` - Information on Xacro macros used for the Classic PSM.
-   **Xacro Macros README (Si):** `urdf/xacros/Si/README.md` - Information on Xacro macros used for the Si PSM.
-   **ROS 2 Control README (Classic):** `urdf/ros2_control/Classic/README.md` - Details about the ROS 2 control configurations for Classic PSM.
-   **ROS 2 Control README (Si):** `urdf/ros2_control/Si/README.md` - Details about the ROS 2 control configurations for Si PSM.
-   **Launch Files README:** `launch/README.md` - Explanations of the provided launch files.
