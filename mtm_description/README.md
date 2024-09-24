# MTM: Master Tool Manipulator -

```bash
ros2 launch mtm_description urdf.launch.py file:=mtms.urdf.xacro
```

---

## MTM Xacro -

### Links -

| Link Number | Link Name                           |
|-------------|-------------------------------------|
| Link 0      | Top Panel Link                      |
| Link 1      | Outer Yaw Link                      |
| Link 2      | Back Parallel Link                  |
| Link 3      | Top Parallel Link                   |
| Link 4      | Bottom Parallel Link                |
| Link 5      | Wrist Platform Link                 |
| Link 6      | Wrist Pitch Link                    |
| Link 7      | Wrist Yaw Link                      |
| Link 8      | Wrist Roll Link                     |
| Link 9      | Wrist Platform Link (with Origins)  |
| Link 10     | Wrist Pitch Link (with Origins)     |
| Link 11     | Wrist Yaw Link (with Origins)       |
| Link 12     | Wrist Roll Link (with Origins)      |

### Joints -

| Joint Number  | Joint Name                | Associated Links (Parent to Child)| Type      |
|---------------|---------------------------|-----------------------------------|-----------|
| Joint 0       | Fixed to World            | Macro Parent Link to Link 0       | Fixed     |
| Joint 1       | Outer Yaw                 | Link 0 to Link 1                  | Revolute  |
| Joint 2       | Shoulder Pitch            | Link 1 to Link 2                  | Revolute  |
| Joint 3       | Shoulder Pitch Parallel   | Link 1 to Link 3                  | Revolute  |
| Joint 4       | Elbow Pitch               | Link 2 to Link 4                  | Revolute  |
| Joint 5       | Wrist Platform Joint      | Link 4 to Link 5                  | Revolute  |
| Joint 6       | Wrist Pitch Joint         | Link 5 to Link 6                  | Revolute  |
| Joint 7       | Wrist Yaw Joint           | Link 6 to Link 7                  | Revolute  |
| Joint 8       | Wrist Roll Joint          | Link 7 to Link 8                  | Revolute  |
| Joint 9       | Wrist Platform Joint      | Link 4 to Link 5/9                | Revolute  |
| Joint 10      | Wrist Pitch Joint         | Link 5/9 to Link 6/10             | Revolute  |
| Joint 11      | Wrist Yaw Joint           | Link 6/10 to Link 7/11            | Revolute  |
| Joint 12      | Wrist Roll Joint          | Link 7/11 to Link 8/12            | Revolute  |

---
