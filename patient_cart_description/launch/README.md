# patient_cart_description

This package contains the description and launch files for the patient cart robot, including both classic and SI (Single Incision) variants.

## Launch Files

### 1. `patient_cart_classic_bringup.launch.py`

This launch file initializes the classic patient cart robot simulation with ROS 2 controllers.

#### Purpose

-   Launches the robot description and state publisher for the classic patient cart.
-   Starts the ROS 2 control node with the specified controllers.
-   Spawns the joint state broadcaster and position controllers.
-   Runs a custom joint controller node for the patient cart.
-   Launches RViz 2 with a pre-configured view.

#### Nodes Launched

1. **Robot State Publisher (`robot_state_publisher`)**

    - Publishes the robot's state based on the URDF description.
    - Parameters:
        - `robot_description`: The robot's URDF model.
        - `publish_robot_description`: `True`
        - `use_sim_time`: `True`

2. **Controller Manager (`ros2_control_node`)**

    - Manages the controllers for the robot using the ROS 2 control framework.
    - Parameters:
        - Controller configuration from [patient_cart.classic.controllers.yaml](../config/patient_cart.classic.controllers.yaml).
        - `use_sim_time`: `True`

3. **Joint State Broadcaster (`joint_state_broadcaster`)**

    - Publishes the state of all joints in the robot.
    - Loaded via the `spawner` executable.
    - Parameters:
        - `use_sim_time`: `True`

4. **Forward Position Controller (`forward_position_controller`)**

    - Controls the positions of the robot's joints.
    - Loaded via the `spawner` executable.

5. **Custom Joint Controller (`patient_cart_joint_controller`)**

    - A custom node specific to controlling the patient cart joints.
    - Parameters:
        - `use_sim_time`: `True`

6. **RViz 2 (`rviz2`)**
    - Visualizes the robot in a 3D environment.
    - Launches after the joint state broadcaster has started.
    - Uses a pre-configured RViz configuration file [patient_cart_description.rviz](../rviz/patient_cart_description.rviz).

#### Configuration Files

-   **Robot Description (`patient_cart.classic.urdf.xacro`)**

    -   Located at [patient_cart.classic.urdf.xacro](../urdf/patient_cart.classic.urdf.xacro).
    -   Defines the robot's physical and visual properties using Xacro macros.

-   **Controller Configuration (`patient_cart.classic.controllers.yaml`)**
    -   Located at [patient_cart.classic.controllers.yaml](../config/patient_cart.classic.controllers.yaml).
    -   Specifies the controllers to be loaded and their parameters.

#### Usage

To launch the classic patient cart robot with controllers:

```bash
ros2 launch patient_cart_description patient_cart_classic_bringup.launch.py
```

#### File Contents

-   **Launch File:** [patient_cart_classic_bringup.launch.py](patient_cart_classic_bringup.launch.py)
-   **RViz Configuration:** [patient_cart_description.rviz](../rviz/patient_cart_description.rviz)
-   **Custom Joint Controller Source:** [patient_cart_joint_controller.cpp](../src/patient_cart_joint_controller.cpp)

---

### 2. `patient_cart_si_bringup.launch.py`

This launch file initializes the Single Incision (SI) patient cart robot simulation with ROS 2 controllers.

#### Purpose

-   Launches the robot description and state publisher for the SI patient cart.
-   Starts the ROS 2 control node with the specified controllers.
-   Spawns the joint state broadcaster and position controllers.
-   Runs a custom joint controller node for the patient cart.
-   Launches RViz 2 with a pre-configured view.

#### Nodes Launched

1. **Robot State Publisher (`robot_state_publisher`)**

    - Publishes the robot's state based on the URDF description.
    - Parameters:
        - `robot_description`: The robot's URDF model.
        - `publish_robot_description`: `True`
        - `use_sim_time`: `True`

2. **Controller Manager (`ros2_control_node`)**

    - Manages the controllers for the robot using the ROS 2 control framework.
    - Parameters:
        - Controller configuration from [patient_cart.si.controllers.yaml](../config/patient_cart.si.controllers.yaml).
        - `use_sim_time`: `True`

3. **Joint State Broadcaster (`joint_state_broadcaster`)**

    - Publishes the state of all joints in the robot.
    - Loaded via the `spawner` executable.
    - Parameters:
        - `use_sim_time`: `True`

4. **Forward Position Controller (`forward_position_controller`)**

    - Controls the positions of the robot's joints.
    - Loaded via the `spawner` executable.

5. **Custom Joint Controller (`patient_cart_joint_controller`)**

    - A custom node specific to controlling the patient cart joints.
    - Parameters:
        - `use_sim_time`: `True`

6. **RViz 2 (`rviz2`)**
    - Visualizes the robot in a 3D environment.
    - Launches after the joint state broadcaster has started.
    - Uses a pre-configured RViz configuration file [patient_cart_description.rviz](../rviz/patient_cart_description.rviz).

#### Configuration Files

-   **Robot Description (`patient_cart.si.urdf.xacro`)**

    -   Located at [patient_cart.si.urdf.xacro](../urdf/patient_cart.si.urdf.xacro).
    -   Defines the robot's physical and visual properties using Xacro macros.

-   **Controller Configuration (`patient_cart.si.controllers.yaml`)**
    -   Located at [patient_cart.si.controllers.yaml](../config/patient_cart.si.controllers.yaml).
    -   Specifies the controllers to be loaded and their parameters.

#### Usage

To launch the SI patient cart robot with controllers:

```bash
ros2 launch patient_cart_description patient_cart_si_bringup.launch.py
```

#### File Contents

-   **Launch File:** [patient_cart_si_bringup.launch.py](patient_cart_si_bringup.launch.py)
-   **RViz Configuration:** [patient_cart_description.rviz](../rviz/patient_cart_description.rviz)
-   **Custom Joint Controller Source:** [patient_cart_joint_controller.cpp](../src/patient_cart_joint_controller.cpp)

---

### 3. `view_robot.launch.py`

This launch file allows you to visualize the patient cart robot and manually manipulate its joints using a GUI.

#### Purpose

-   Launches the robot description and state publisher.
-   Starts the Joint State Publisher GUI for manual joint control.
-   Launches RViz 2 with a pre-configured view.

#### Nodes Launched

1. **Robot State Publisher (`robot_state_publisher`)**

    - Publishes the robot's state based on the URDF description.
    - Parameters:
        - `robot_description`: The robot's URDF model.

2. **Joint State Publisher GUI (`joint_state_publisher_gui`)**

    - Provides a graphical interface to manipulate the robot's joint states.
    - Allows you to adjust joint angles and see the effect in real-time.

3. **RViz 2 (`rviz2`)**
    - Visualizes the robot in a 3D environment.
    - Uses the pre-configured RViz configuration file [patient_cart_description.rviz](../rviz/patient_cart_description.rviz).

#### Configuration Files

-   **Robot Description (URDF Xacro file)**

    -   The URDF Xacro file is specified as a launch argument.
    -   Can be either `patient_cart.classic.urdf.xacro` or `patient_cart.si.urdf.xacro`.

-   **RViz Configuration (`patient_cart_description.rviz`)**
    -   Located at [patient_cart_description.rviz](../rviz/patient_cart_description.rviz).

#### Usage

To visualize the patient cart robot and manipulate joints via GUI:

```bash
ros2 launch patient_cart_description view_robot.launch.py urdf:=patient_cart.classic.urdf.xacro
```

or

```bash
ros2 launch patient_cart_description view_robot.launch.py urdf:=patient_cart.si.urdf.xacro
```

#### File Contents

-   **Launch File:** [view_robot.launch.py](view_robot.launch.py)

---

### Additional Notes

-   The package includes two variants of the patient cart: classic and Single Incision (SI).
-   Both variants use the same custom joint controller (`patient_cart_joint_controller`).
-   The `view_robot.launch.py` file allows for easy visualization and joint manipulation of both variants.
-   Make sure to use the appropriate URDF and controller configuration files for each variant when launching or modifying the robot.
