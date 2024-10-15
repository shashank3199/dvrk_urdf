## Launch Files

### 1. `mtms_bringup.launch.py`

This launch file initializes the MTM (Master Tool Manipulator) robot simulation with ROS 2 controllers, suitable for simulation and integration with other ROS 2 nodes.

#### Purpose

-   Launches the robot description and state publisher.
-   Starts the ROS 2 control node with the specified controllers.
-   Spawns the joint state broadcaster and position controllers.
-   Runs a custom joint controller node for the MTM.
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
        - Controller configuration from [mtms.controllers.yaml](../config/mtms.controllers.yaml).
        - `use_sim_time`: `True`

3. **Joint State Broadcaster (`joint_state_broadcaster`)**

    - Publishes the state of all joints in the robot.
    - Loaded via the `spawner` executable.
    - Parameters:
        - `use_sim_time`: `True`

4. **Forward Position Controller (`forward_position_controller`)**

    - Controls the positions of the robot's joints.
    - Loaded via the `spawner` executable.

5. **Custom Joint Controller (`mtms_joint_controller`)**

    - A custom node specific to controlling the MTM joints.
    - Parameters:
        - `use_sim_time`: `True`

6. **RViz 2 (`rviz2`)**
    - Visualizes the robot in a 3D environment.
    - Launches after the joint state broadcaster has started.
    - Uses a pre-configured RViz configuration file [mtm_description.rviz](../rviz/mtm_description.rviz).

#### Configuration Files

-   **Robot Description (`mtms.urdf.xacro`)**

    -   Located at [urdf/mtms.urdf.xacro](../urdf/mtms.urdf.xacro).
    -   Defines the robot's physical and visual properties using Xacro macros.

-   **Controller Configuration (`mtms.controllers.yaml`)**
    -   Located at [config/mtms.controllers.yaml](../config/mtms.controllers.yaml).
    -   Specifies the controllers to be loaded and their parameters.

#### Usage

To launch the MTM robot with controllers:

```bash
ros2 launch mtm_description mtms_bringup.launch.py
```

#### File Contents

-   **Launch File:** [launch/mtms_bringup.launch.py](launch/mtms_bringup.launch.py)
-   **RViz Configuration:** [rviz/mtm_description.rviz](rviz/mtm_description.rviz)
-   **Custom Joint Controller Source:** [src/mtms_joint_controller.cpp](src/mtms_joint_controller.cpp)

---

### 2. `view_robot.launch.py`

This launch file allows you to visualize the MTM robot and manually manipulate its joints using a GUI.

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
    - Uses the pre-configured RViz configuration file [mtm_description.rviz](../rviz/mtm_description.rviz).

#### Configuration Files

-   **Robot Description (`mtms.urdf.xacro`)**

    -   Located at [urdf/mtms.urdf.xacro](../urdf/mtms.urdf.xacro).

-   **RViz Configuration (`mtm_description.rviz`)**
    -   Located at [rviz/mtm_description.rviz](../rviz/mtm_description.rviz).

#### Usage

To visualize the MTM robot and manipulate joints via GUI:

```bash
ros2 launch mtm_description view_robot.launch.py
```

---

### Additional Notes

-   Both launch files use the same URDF file (`mtms.urdf.xacro`) and RViz configuration file (`mtm_description.rviz`).
-   The `mtms_bringup.launch.py` file is more comprehensive, including controllers and a custom joint controller, while `view_robot.launch.py` is focused on visualization and manual joint manipulation.
-   The package includes mesh files (in both `.dae` and `.stl` formats) for various robot components, which are likely referenced in the URDF file for visualization.
-   There are additional resources and documentation in the `resources` and `urdf` directories that may be helpful for understanding the robot's structure and configuration.
