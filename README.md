# dVRK URDF Description

This README provides an overview of the da Vinci Research Kit (dVRK) Unified Robot Description Format (URDF) models. This document combines information on various robotic elements, including the Patient Side Manipulator (PSM), Endoscope Camera Manipulator (ECM), Master Tool Manipulator (MTM), Setup Joints (SUJ), and the Patient Cart.

---

## Table of Contents

- [Introduction](#introduction)
- [Components Overview](#components-overview)
  - [Patient Side Manipulator (PSM)](#patient-side-manipulator-psm)
  - [Endoscope Camera Manipulator (ECM)](#endoscope-camera-manipulator-ecm)
  - [Master Tool Manipulator (MTM)](#master-tool-manipulator-mtm)
  - [Setup Joints (SUJ)](#setup-joints-suj)
  - [Patient Cart](#patient-cart)
- [Launching the URDF Models](#launching-the-urdf-models)
- [Notes](#notes)

---

## Introduction

The dVRK URDF models provide detailed representations of the robotic components used in the da Vinci Surgical System. This README combines information from multiple components to serve as a unified guide for the `dvrk_urdf` package.

---

## Components Overview

### Patient Side Manipulator (PSM)

The PSM is a robotic arm that performs surgical procedures. It can be equipped with various tools, and its URDF model includes detailed links and joints representing its mechanical structure.

- **Tool Types**: Different surgical tools can be mounted on the PSM. Change the tool type by modifying the `tool_name` parameter in the URDF files.
- **Meshes**: PSM meshes are located in the `meshes` directory within the `psm_description` package.

### Endoscope Camera Manipulator (ECM)

The ECM controls the endoscopic camera, providing visual feedback during surgery. Its URDF model includes all necessary links and joints to simulate its movements.

- **Variants**: Full ECM models and base models can be launched separately.
- **Meshes**: ECM meshes are found in the `meshes` directory within the `ecm_description` package.

### Master Tool Manipulator (MTM)

The MTM serves as the master control interface for the surgeon, translating hand movements into robot actions.

- **Variants**: Both left (MTML) and right (MTMR) manipulators are available.
- **Meshes**: MTM meshes are stored in the `meshes` directory within the `mtm_description` package.

### Setup Joints (SUJ)

The SUJ provides the mounting and positioning mechanism for the PSMs and ECM.

- **Variants**: Classic and Si versions are available.
- **Meshes**: SUJ meshes are located in the `meshes` directory within the `suj_description` package.

### Patient Cart

The Patient Cart holds the PSMs and ECM during surgical procedures.

- **Variants**: Classic and Si models are provided.
- **Integration**: The Patient Cart URDF includes the SUJ and mounting points for the PSMs and ECM.

---

## Launching the URDF Models

Use the following ROS 2 commands to launch the URDF models:

- **PSM Classic Model**:
  ```bash
  ros2 launch psm_description urdf.launch.py file:=psm.classic.urdf.xacro
  ```
- **ECM Model**:
  ```bash
  ros2 launch ecm_description urdf.launch.py file:=ecm.urdf.xacro
  ```
- **MTM Models**:
  ```bash
  ros2 launch mtm_description urdf.launch.py file:=mtms.urdf.xacro
  ```
- **SUJ Classic Model**:
  ```bash
  ros2 launch suj_description urdf.launch.py file:=suj.classic.urdf.xacro
  ```
- **Patient Cart Classic Model**:
  ```bash
  ros2 launch patient_cart_description urdf.launch.py file:=patient_cart.classic.urdf.xacro
  ```

---

## Notes

- **Further Information**: For detailed information on each component, refer to their individual README files within their respective packages.

---
