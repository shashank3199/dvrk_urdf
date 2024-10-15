## Launch Files

### 1. `ecm_base_bringup.launch.py`

This launch file initializes the ECM (Endoscopic Camera Manipulator) base robot simulation with ROS 2 controllers, suitable for simulation and integration with other ROS 2 nodes.

#### Purpose

-   Launches the robot description and state publisher for the ECM base.
-   Starts the ROS 2 control node with the specified controllers.
-   Spawns the joint state broadcaster and position controllers.
-   Runs a custom joint controller node for the ECM base.
-   Launches RViz 2 with a pre-configured view.

#### Nodes Launched

1. **Robot State Publisher (`robot_state_publisher`)**

    - Publishes the robot's state (joint states and transforms) based on the URDF description.
    - Parameters:
        - `robot_description`: The robot's URDF model.
        - `publish_robot_description`: `True`
        - `use_sim_time`: `True`

2. **Controller Manager (`ros2_control_node`)**

    - Manages the controllers for the robot using the ROS 2 control framework.
    - Parameters:
        - Controller configuration from [ecm.base.controllers.yaml](../config/ecm.base.controllers.yaml).
        - `use_sim_time`: `True`

3. **Joint State Broadcaster (`joint_state_broadcaster`)**

    - Publishes the state of all joints in the robot.
    - Loaded via the `spawner` executable.
    - Parameters:
        - `use_sim_time`: `True`

4. **Forward Position Controller (`forward_position_controller`)**

    - Controls the positions of the robot's joints.
    - Loaded via the `spawner` executable.

5. **Custom Joint Controller (`ecm_joint_controller`)**

    - A custom node specific to controlling the ECM base joints.
    - Parameters:
        - `use_sim_time`: `True`

6. **RViz 2 (`rviz2`)**
    - Visualizes the robot in a 3D environment.
    - Launches after the joint state broadcaster has started.
    - Uses a pre-configured RViz configuration file [ecm_description.rviz](../rviz/ecm_description.rviz).

#### Configuration Files

-   **Robot Description ([ecm.base.urdf.xacro](../urdf/ecm.base.urdf.xacro))**

    -   Defines the robot's physical and visual properties using Xacro macros.

-   **Controller Configuration ([ecm.base.controllers.yaml](../config/ecm.base.controllers.yaml))**
    -   Specifies the controllers to be loaded and their parameters.

#### Usage

To launch the ECM base robot with controllers:

```bash
ros2 launch ecm_description ecm_base_bringup.launch.py
```

#### File Contents

-   **Launch File:** [ecm_base_bringup.launch.py](ecm_base_bringup.launch.py)
-   **RViz Configuration:** [ecm_description.rviz](../rviz/ecm_description.rviz)
-   **Custom Joint Controller Source:** [ecm_joint_controller.cpp](../src/ecm_joint_controller.cpp)

---

### 2. `ecm_bringup.launch.py`

This launch file initializes the full ECM robot simulation with ROS 2 controllers, including both the base and the endoscopic arm.

#### Purpose

-   Launches the robot description and state publisher for the complete ECM.
-   Starts the ROS 2 control node with the specified controllers.
-   Spawns the joint state broadcaster and position controllers.
-   Runs a custom joint controller node for the ECM.
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
        - Controller configuration from [ecm.controllers.yaml](../config/ecm.controllers.yaml).
        - `use_sim_time`: `True`

3. **Joint State Broadcaster (`joint_state_broadcaster`)**

    - Publishes the state of all joints in the robot.
    - Loaded via the `spawner` executable.
    - Parameters:
        - `use_sim_time`: `True`

4. **Forward Position Controller (`forward_position_controller`)**

    - Controls the positions of the robot's joints.
    - Loaded via the `spawner` executable.

5. **Custom Joint Controller (`ecm_joint_controller`)**

    - A custom node specific to controlling the ECM joints.
    - Parameters:
        - `use_sim_time`: `True`

6. **RViz 2 (`rviz2`)**
    - Visualizes the robot in a 3D environment.
    - Launches after the joint state broadcaster has started.
    - Uses a pre-configured RViz configuration file [ecm_description.rviz](../rviz/ecm_description.rviz).

#### Configuration Files

-   **Robot Description ([ecm.urdf.xacro](../urdf/ecm.urdf.xacro))**

    -   Defines the robot's physical and visual properties using Xacro macros.

-   **Controller Configuration ([ecm.controllers.yaml](../config/ecm.controllers.yaml))**
    -   Specifies the controllers to be loaded and their parameters.

#### Usage

To launch the full ECM robot with controllers:

```bash
ros2 launch ecm_description ecm_bringup.launch.py
```

#### File Contents

-   **Launch File:** [ecm_bringup.launch.py](ecm_bringup.launch.py)
-   **RViz Configuration:** [ecm_description.rviz](../rviz/ecm_description.rviz)
-   **Custom Joint Controller Source:** [ecm_joint_controller.cpp](../src/ecm_joint_controller.cpp)

---

### 3. `view_robot.launch.py`

This launch file allows you to visualize the ECM robot and manually manipulate its joints using a GUI.

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
    - Uses the pre-configured RViz configuration file [ecm_description.rviz](../rviz/ecm_description.rviz).

#### Configuration Files

-   **Robot Description ([ecm.urdf.xacro](../urdf/ecm.urdf.xacro) or [ecm.base.urdf.xacro](../urdf/ecm.base.urdf.xacro))**

    -   The specific URDF file used is determined by the `urdf` launch argument.

-   **RViz Configuration ([ecm_description.rviz](../rviz/ecm_description.rviz))**

#### Usage

To visualize the ECM robot and manipulate joints via GUI:

```bash
ros2 launch ecm_description view_robot.launch.py urdf:=ecm.urdf.xacro
```

You can specify either `ecm.urdf.xacro` for the full ECM or `ecm.base.urdf.xacro` for just the base.

#### File Contents

-   **Launch File:** [view_robot.launch.py](view_robot.launch.py)

#### Additional Notes

-   The `view_robot.launch.py` file uses a launch argument `urdf` to specify which URDF file to load. This allows for flexibility in viewing either the full ECM or just the base.
-   The Joint State Publisher GUI provides an interactive way to explore the robot's kinematics without needing to run the full simulation or controllers.
