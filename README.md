[![ROS2 Version](https://img.shields.io/badge/ROS2-humble-orange)](https://docs.ros.org/en/humble/index.html)
![Release Version](https://img.shields.io/badge/Version-1.0.0-blue)

# dVRK URDF Description

This package provides the Unified Robot Description Format (URDF) models for the **da Vinci Research Kit (dVRK)**. It includes detailed models of the various robotic components used in the **da Vinci Surgical System**, allowing for simulation and visualization in **ROS 2**.

## Table of Contents

-   [Introduction](#introduction)
-   [Package Structure](#package-structure)
-   [Components Overview](#components-overview)
    -   [da Vinci Core Description](#da-vinci-core-description)
        -   [Subcomponents](#subcomponents)
        -   [Readme](#readme)
    -   [Patient Side Manipulator (PSM)](#patient-side-manipulator-psm)
        -   [Subcomponents](#subcomponents-1)
        -   [Readme](#readme-1)
    -   [Endoscopic Camera Manipulator (ECM)](#endoscopic-camera-manipulator-ecm)
        -   [Subcomponents](#subcomponents-2)
        -   [Readme](#readme-2)
    -   [Master Tool Manipulator (MTM)](#master-tool-manipulator-mtm)
        -   [Subcomponents](#subcomponents-3)
        -   [Readme](#readme-3)
    -   [Setup Joints (SUJ)](#setup-joints-suj)
        -   [Subcomponents](#subcomponents-4)
        -   [Readme](#readme-4)
    -   [Patient Cart](#patient-cart)
        -   [Subcomponents](#subcomponents-5)
        -   [Readme](#readme-5)
-   [Building the Packages](#building-the-packages)
-   [Usage Examples](#usage-examples)
    -   [Simulate the Robots with Controllers](#simulate-the-robots-with-controllers)
        -   [da Vinci Core Robot](#da-vinci-core-robot)
        -   [Patient Side Manipulator (PSM)](#patient-side-manipulator-psm-1)
        -   [Endoscopic Camera Manipulator (ECM)](#endoscopic-camera-manipulator-ecm-1)
        -   [Master Tool Manipulator (MTM)](#master-tool-manipulator-mtm-1)
        -   [Setup Joints (SUJ)](#setup-joints-suj-1)
        -   [Patient Cart](#patient-cart-1)
        -   [Launch Files README](#launch-files-readme)
    -   [Visualize and Manipulate the Robots with GUI](#visualize-and-manipulate-the-robots-with-gui)
        -   [da Vinci Core Robot](#da-vinci-core-robot-1)
        -   [Other Components](#other-components)
        -   [Launch Files README](#launch-files-readme-1)
-   [Dependencies](#dependencies)
-   [Additional Resources](#additional-resources)
-   [Notes](#notes)

---

## Introduction

The **dVRK URDF** models provide detailed representations of the robotic components used in the **da Vinci Surgical System**. This package combines information from multiple components to serve as a unified guide for simulation and development within **ROS 2**.

---

## Package Structure

```
dvrk_urdf
├── davinci_core_description
├── ecm_description
├── mtm_description
├── patient_cart_description
├── psm_description
├── suj_description
└── README.md
```

Each sub-package contains URDF models, launch files, configurations, meshes, and additional resources specific to each component.

---

## Components Overview

### da Vinci Core Description

The `davinci_core_description` package provides the URDF descriptions, launch files, and configurations necessary to simulate and visualize the da Vinci Surgical System's core components in ROS 2.

#### Subcomponents

-   **URDF and Xacro Files**
-   **Meshes**
-   **Controllers**
-   **RViz Configurations**

#### Readme

-   **README:** [`davinci_core_description/README.md`](./davinci_core_description/README.md)

### Patient Side Manipulator (PSM)

The `psm_description` package provides URDF models and configurations for the Patient Side Manipulator, a robotic arm that performs surgical procedures.

#### Subcomponents

-   **Classic and Si Versions**
-   **Support for Various Surgical Tools**
-   **URDF and Xacro Files**
-   **Meshes**
-   **Controllers**
-   **RViz Configurations**

#### Readme

-   **README:** [`psm_description/README.md`](./psm_description/README.md)

### Endoscopic Camera Manipulator (ECM)

The `ecm_description` package provides URDF models and configurations for the Endoscopic Camera Manipulator, which controls the endoscopic camera during surgery.

#### Subcomponents

-   **Full ECM Models and Base Models**
-   **URDF and Xacro Files**
-   **Meshes**
-   **Controllers**
-   **RViz Configurations**

#### Readme

-   **README:** [`ecm_description/README.md`](./ecm_description/README.md)

### Master Tool Manipulator (MTM)

The `mtm_description` package provides URDF models and configurations for the Master Tool Manipulator, which serves as the master control interface for the surgeon.

#### Subcomponents

-   **Left (MTML) and Right (MTMR) Manipulators**
-   **URDF and Xacro Files**
-   **Meshes**
-   **Controllers**
-   **RViz Configurations**

#### Readme

-   **README:** [`mtm_description/README.md`](./mtm_description/README.md)

### Setup Joints (SUJ)

The `suj_description` package provides URDF models and configurations for the Setup Joints, which are the mounting and positioning mechanisms for the PSMs and ECM.

#### Subcomponents

-   **Classic and Si Versions**
-   **URDF and Xacro Files**
-   **Meshes**
-   **Controllers**
-   **RViz Configurations**

#### Readme

-   **README:** [`suj_description/README.md`](./suj_description/README.md)

### Patient Cart

The `patient_cart_description` package provides URDF models and configurations for the Patient Cart, which holds the PSMs and ECM during surgical procedures.

#### Subcomponents

-   **Classic and Si Models**
-   **Integration with SUJ and Mounting Points for PSMs and ECM**
-   **URDF and Xacro Files**
-   **Controllers**
-   **RViz Configurations**

#### Readme

-   **README:** [`patient_cart_description/README.md`](./patient_cart_description/README.md)

---

## Building the Packages

Ensure you have a **ROS 2** workspace set up. Clone the `dvrk_urdf` meta-package into the `src` directory of your workspace, and then build the workspace:

```bash
colcon build
```

Source your workspace after building:

```bash
source install/setup.bash
```

---

## Usage Examples

### Simulate the Robots with Controllers

#### da Vinci Core Robot

To simulate the da Vinci core robot with **ROS 2 control** and visualize it in **RViz**:

```bash
ros2 launch davinci_core_description davinci_core_bringup.launch.py
```

-   **Launch File:** [`davinci_core_description/launch/davinci_core_bringup.launch.py`](./davinci_core_description/launch/davinci_core_bringup.launch.py)
-   **Launch Files README:** See [`davinci_core_description/launch/README.md`](./davinci_core_description/launch/README.md) for detailed explanations.

#### Patient Side Manipulator (PSM)

To simulate the PSM with **ROS 2 control** and visualize it in **RViz**:

```bash
ros2 launch psm_description psm_bringup.launch.py
```

-   **Launch File:** [`psm_description/launch/psm_bringup.launch.py`](./psm_description/launch/psm_bringup.launch.py)
-   **Launch Files README:** See [`psm_description/launch/README.md`](./psm_description/launch/README.md) for detailed explanations.

#### Endoscopic Camera Manipulator (ECM)

To simulate the ECM with **ROS 2 control** and visualize it in **RViz**:

```bash
ros2 launch ecm_description ecm_bringup.launch.py
```

-   **Launch File:** [`ecm_description/launch/ecm_bringup.launch.py`](./ecm_description/launch/ecm_bringup.launch.py)
-   **Launch Files README:** See [`ecm_description/launch/README.md`](./ecm_description/launch/README.md) for detailed explanations.

#### Master Tool Manipulator (MTM)

To simulate the MTM with **ROS 2 control** and visualize it in **RViz**:

```bash
ros2 launch mtm_description mtms_bringup.launch.py
```

-   **Launch File:** [`mtm_description/launch/mtms_bringup.launch.py`](./mtm_description/launch/mtms_bringup.launch.py)
-   **Launch Files README:** See [`mtm_description/launch/README.md`](./mtm_description/launch/README.md) for detailed explanations.

#### Setup Joints (SUJ)

To simulate the SUJ with **ROS 2 control** and visualize it in **RViz**:

```bash
ros2 launch suj_description suj_bringup.launch.py
```

-   **Launch File:** [`suj_description/launch/suj_bringup.launch.py`](./suj_description/launch/suj_bringup.launch.py)
-   **Launch Files README:** See [`suj_description/launch/README.md`](./suj_description/launch/README.md) for detailed explanations.

#### Patient Cart

To simulate the Patient Cart with **ROS 2 control** and visualize it in **RViz**:

```bash
ros2 launch patient_cart_description patient_cart_classic_bringup.launch.py
```

-   **Launch File:** [`patient_cart_description/launch/patient_cart_classic_bringup.launch.py`](./patient_cart_description/launch/patient_cart_classic_bringup.launch.py)
-   **Launch Files README:** See [`patient_cart_description/launch/README.md`](./patient_cart_description/launch/README.md) for detailed explanations.

#### Launch Files README

For detailed explanations of the launch files and their configurations, refer to the individual **Launch Files README** in each component's `launch` directory.

### Visualize and Manipulate the Robots with GUI

#### da Vinci Core Robot

To launch the da Vinci core robot and manipulate its joints using the **Joint State Publisher GUI**:

```bash
ros2 launch davinci_core_description view_robot.launch.py
```

-   **Launch File:** [`davinci_core_description/launch/view_robot.launch.py`](./davinci_core_description/launch/view_robot.launch.py)
-   **Launch Files README:** See [`davinci_core_description/launch/README.md`](./davinci_core_description/launch/README.md) for detailed explanations.

#### Other Components

To visualize and manipulate other components, use their respective `view_robot.launch.py` files:

```bash
ros2 launch [component]_description view_robot.launch.py
```

Replace `[component]` with the specific component name, such as `psm`, `ecm`, `mtm`, `suj`, or `patient_cart`.

#### Launch Files README

For detailed explanations of the visualization launch files, refer to the **Launch Files README** in each component's `launch` directory.

---

## Dependencies

Ensure that you have the following dependencies installed:

-   **ROS 2 (Foxy, Galactic, Humble, etc.)**
-   **robot_state_publisher**
-   **joint_state_publisher_gui**
-   **ros2_control**
-   **rviz2**

---

## Additional Resources

-   **da Vinci Core Description README:** [`davinci_core_description/README.md`](./davinci_core_description/README.md)
-   **PSM Description README:** [`psm_description/README.md`](./psm_description/README.md)
-   **ECM Description README:** [`ecm_description/README.md`](./ecm_description/README.md)
-   **MTM Description README:** [`mtm_description/README.md`](./mtm_description/README.md)
-   **SUJ Description README:** [`suj_description/README.md`](./suj_description/README.md)
-   **Patient Cart Description README:** [`patient_cart_description/README.md`](./patient_cart_description/README.md)

For detailed information on each component, including URDF and Xacro files, meshes, controllers, and RViz configurations, please refer to the READMEs in the respective packages.

---

[![Developer](https://img.shields.io/badge/Developer-shashank3199-red)](https://github.com/shashank3199)
