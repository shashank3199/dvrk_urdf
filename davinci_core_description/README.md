# daVinci Core Description -

```bash
ros2 launch davinci_core_description urdf.launch.py file:=daVinci.urdf.xacro
```

## daVinci Arm Xacro -

### Links -

| Link Number | Link Name                           |
|-------------|-------------------------------------|
| Link 0      | Mounting Base Link                  |
| Link 1      | Outer Yaw Link                      |
| Link 2      | Outer Pitch Base Link               |
| Link 3      | Outer Pitch Front Link              |
| Link 4      | Outer Pitch Bottom Link             |
| Link 5      | Outer Pitch Top Link                |
| Link 6      | Outer Insertion Link                |
| Link 7      | Tool Adaptor Link                   |
| Link 8      | Tool Link                           |

### Joints -

| Joint Number  | Joint Name                | Associated Links (Parent to Child)| Type          |
|---------------|---------------------------|-----------------------------------|---------------|
| Joint 0       | Mounting Base Joint       | Macro Parent to Link 0            | Fixed         |
| Joint 1       | Outer Yaw Joint           | Link 0 to Link 1                  | Continuous    |
| Joint 2       | Outer Pitch Base Joint    | Link 1 to Link 2                  | Continuous    |
| Joint 3       | Outer Pitch Front Joint   | Link 1 to Link 3                  | Continuous    |
| Joint 4       | Outer Pitch Bottom Joint  | Link 2 to Link 4                  | Continuous    |
| Joint 5       | Outer Pitch Top Joint     | Link 2 to Link 5                  | Continuous    |
| Joint 6       | Outer Insertion Joint     | Link 4 to Link 6                  | Continuous    |
| Joint 7       | Tool Insertion Joint      | Link 6 to Link 7                  | Prismatic     |
| Joint 8       | Tool Joint                | Link 7 to Link 8                  | Fixed         |

---