## Launch Files

### 1. `suj_bringup.launch.py`

This launch file initializes the SUJ (Setup Joint) robot simulation with ROS 2 controllers, suitable for simulation and integration with other ROS 2 nodes.

#### Purpose

-   Launches the robot description and state publisher.
-   Starts the ROS 2 control node with the specified controllers.
-   Spawns the joint state broadcaster and position controllers.
-   Runs a custom joint controller node for the SUJ.
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
        - Controller configuration from [suj.controllers.yaml](../config/suj.controllers.yaml).
        - `use_sim_time`: `True`

3. **Joint State Broadcaster (`joint_state_broadcaster`)**

    - Publishes the state of all joints in the robot.
    - Loaded via the `spawner` executable.
    - Parameters:
        - `use_sim_time`: `True`

4. **Forward Position Controller (`forward_position_controller`)**

    - Controls the positions of the robot's joints.
    - Loaded via the `spawner` executable.

5. **Custom Joint Controller (`suj_joint_controller`)**

    - A custom node specific to controlling the SUJ joints.
    - Parameters:
        - `use_sim_time`: `True`

6. **RViz 2 (`rviz2`)**
    - Visualizes the robot in a 3D environment.
    - Launches after the joint state broadcaster has started.
    - Uses a pre-configured RViz configuration file [suj_description.rviz](../rviz/suj_description.rviz).

#### Configuration Files

-   **Robot Description (URDF file)**

    -   Located at `urdf/[urdf_file_name]`.
    -   Defined by the `urdf` launch argument.

-   **Controller Configuration (`suj.controllers.yaml`)**
    -   Located at [suj.controllers.yaml](../config/suj.controllers.yaml).
    -   Specifies the controllers to be loaded and their parameters.

#### Usage

To launch the SUJ robot with controllers:

```bash
ros2 launch suj_description suj_bringup.launch.py urdf:=<urdf_file_name>
```

Replace `<urdf_file_name>` with the desired URDF file (e.g., [suj.classic.urdf.xacro](../urdf/suj.classic.urdf.xacro) or [suj.si.urdf.xacro](../urdf/suj.si.urdf.xacro)).

#### File Contents

-   **Launch File:** [suj_bringup.launch.py](suj_bringup.launch.py)
-   **RViz Configuration:** [suj_description.rviz](../rviz/suj_description.rviz)
-   **Custom Joint Controller Source:** [suj_joint_controller.cpp](../src/suj_joint_controller.cpp)

---

### 2. `view_robot.launch.py`

This launch file allows you to visualize the SUJ robot and manually manipulate its joints using a GUI.

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
    - Uses the pre-configured RViz configuration file [suj_description.rviz](../rviz/suj_description.rviz).

#### Configuration Files

-   **Robot Description (URDF file)**

    -   Located at `urdf/[urdf_file_name]`.
    -   Defined by the `urdf` launch argument.

-   **RViz Configuration (`suj_description.rviz`)**
    -   Located at [suj_description.rviz](../rviz/suj_description.rviz).

#### Usage

To visualize the SUJ robot and manipulate joints via GUI:

```bash
ros2 launch suj_description view_robot.launch.py urdf:=<urdf_file_name>
```

Replace `<urdf_file_name>` with the desired URDF file (e.g., [suj.classic.urdf.xacro](../urdf/suj.classic.urdf.xacro) or [suj.si.urdf.xacro](../urdf/suj.si.urdf.xacro)).

#### File Contents

-   **Launch File:** [view_robot.launch.py](view_robot.launch.py)

---

### Additional Notes

-   Both launch files use Xacro to process the URDF files, allowing for more flexible and modular robot descriptions.
-   The `suj_bringup.launch.py` file sets up a complete simulation environment with controllers, while `view_robot.launch.py` is simpler and focused on visualization and manual joint manipulation.
-   The package supports two different SUJ models: Classic and Si. Make sure to use the appropriate URDF file when launching.
-   The `meshes` directory contains STL files for both Classic and Si models, which are referenced in the URDF files for visualization.
-   The `urdf` directory contains separate URDF files for Classic and Si models, as well as modular Xacro files in the `xacros` subdirectory.
-   The `config` directory contains the controller configuration file used by the `suj_bringup.launch.py` file.
-   The `src` directory contains the source code for the custom joint controller used in the `suj_bringup.launch.py` file.
