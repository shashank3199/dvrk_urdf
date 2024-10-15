# Patient Cart Description

The Patient Cart Description package provides the URDF descriptions, launch files, and configurations necessary to simulate and visualize the Patient Cart of a surgical robotic system in ROS 2. This package includes models for both the classic and SI (System Intelligence) versions of the Patient Cart.

## Table of Contents

-   [Package Structure](#package-structure)
-   [URDF Files](#urdf-files)
-   [Controllers](#controllers)
-   [RViz Configurations](#rviz-configurations)
-   [Building the Package](#building-the-package)
-   [Usage Examples](#usage-examples)
    -   [Simulate the Classic Patient Cart](#simulate-the-classic-patient-cart)
    -   [Simulate the SI Patient Cart](#simulate-the-si-patient-cart)
    -   [Visualize and Manipulate the Robot with GUI](#visualize-and-manipulate-the-robot-with-gui)
-   [Additional Resources](#additional-resources)

## Package Structure

```
patient_cart_description
├── CMakeLists.txt
├── config
│   ├── patient_cart.classic.controllers.yaml
│   └── patient_cart.si.controllers.yaml
├── launch
│   ├── patient_cart_classic_bringup.launch.py
│   ├── patient_cart_si_bringup.launch.py
│   ├── README.md
│   └── view_robot.launch.py
├── package.xml
├── README.md
├── rviz
│   └── patient_cart_description.rviz
├── src
│   └── patient_cart_joint_controller.cpp
└── urdf
    ├── patient_cart.classic.urdf.xacro
    ├── patient_cart.si.urdf.xacro
    └── README.md
```

## URDF Files

-   **Classic Patient Cart URDF (`patient_cart.classic.urdf.xacro`)**

    -   Located at `urdf/patient_cart.classic.urdf.xacro`.
    -   Defines the model for the classic version of the Patient Cart.

-   **SI Patient Cart URDF (`patient_cart.si.urdf.xacro`)**

    -   Located at `urdf/patient_cart.si.urdf.xacro`.
    -   Defines the model for the SI (System Intelligence) version of the Patient Cart.

-   **URDF README**
    -   See `urdf/README.md` for detailed information on the URDF files and their structure.

## Controllers

-   **Classic Patient Cart Controllers (`patient_cart.classic.controllers.yaml`)**

    -   Located at `config/patient_cart.classic.controllers.yaml`.
    -   Defines the controllers for the classic version of the Patient Cart.

-   **SI Patient Cart Controllers (`patient_cart.si.controllers.yaml`)**

    -   Located at `config/patient_cart.si.controllers.yaml`.
    -   Defines the controllers for the SI version of the Patient Cart.

-   **Custom Joint Controller Source (`patient_cart_joint_controller.cpp`)**
    -   Located at `src/patient_cart_joint_controller.cpp`.
    -   Implements specific control logic for the Patient Cart.

## RViz Configurations

-   **RViz Configuration (`patient_cart_description.rviz`)**
    -   Located at `rviz/patient_cart_description.rviz`.
    -   Pre-configured settings for RViz visualization of the Patient Cart.

## Building the Package

Ensure you have a ROS 2 workspace set up. Clone the `patient_cart_description` package into the `src` directory of your workspace, and then build the workspace:

```bash
colcon build
```

Source your workspace after building:

```bash
source install/setup.bash
```

## Usage Examples

### Simulate the Classic Patient Cart

To simulate the classic version of the Patient Cart with ROS 2 control and visualize it in RViz:

```bash
ros2 launch patient_cart_description patient_cart_classic_bringup.launch.py
```

### Simulate the SI Patient Cart

To simulate the SI version of the Patient Cart:

```bash
ros2 launch patient_cart_description patient_cart_si_bringup.launch.py
```

### Visualize and Manipulate the Robot with GUI

To launch the robot and manipulate its joints using the Joint State Publisher GUI:

```bash
ros2 launch patient_cart_description view_robot.launch.py
```

## Additional Resources

-   **URDF README:** `urdf/README.md` - Detailed documentation on the URDF files for both classic and SI versions.
-   **Launch Files README:** `launch/README.md` - Explanations of the provided launch files and their usage.
