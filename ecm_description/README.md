# ECM: Endoscope Camera Manipulator -

ECM -

```bash
ros2 launch ecm_description urdf.launch.py file:=ecm.urdf.xacro
```

ECM Base -

```bash
ros2 launch ecm_description urdf.launch.py file:=ecm.base.urdf.xacro
```

---

## ECM Xacro -

### Links -

| Link Number | Link Name       |
|-------------|-----------------|
| Link 0      | Setup Base Link |
| Link 1      | Setup Link      |
| Link 2      | Base Link       |
| Link 3      | Yaw Link        |
| Link 4      | Pitch Link      |
| Link 5      | Pitch Link 1    |
| Link 6      | Pitch Link 2    |
| Link 7      | Pitch Link 3    |
| Link 8      | Insertion Link  |
| Link 9      | Roll Link       |


### Joints -

| Joint Number  | Joint Name                | Associated Links (Parent to Child)| Type          |
|---------------|---------------------------|-----------------------------------|---------------|
| Joint 0       | Fixed to World            | Macro Parent Link to Link 0       | Fixed         |
| Joint 1       | Fixed to Setup Joint      | Link 0 to Link 1                  | Fixed         |
| Joint 2       | Fixed to Base Joint       | Link 1 to Link 2                  | Fixed         |
| Joint 3       | Outer Yaw Joint           | Link 2 to Link 3                  | Revolute      |
| Joint 4       | Outer Pitch Joint         | Link 3 to Link 4                  | Revolute      |
| Joint 5       | Outer Pitch Joint 0       | Link 3 to Link 5                  | Continuous    |
| Joint 6       | Outer Pitch Joint 1       | Link 5 to Link 6                  | Continuous    |
| Joint 7       | Outer Pitch Joint 2       | Link 6 to Link 7                  | Continuous    |
| Joint 8       | Insertion Joint           | Link 4 to Link 8                  | Prismatic     |
| Joint 9       | Outer Roll Joint          | Link 8 to Link 9                  | Revolute      |

---

# ECM Base Xacro -

### Links -

| Link Number   | Link Name             |
|---------------|-----------------------|
| Link 0        | Base Link             |
| Link 1        | Yaw Link              |
| Link 2        | Pitch Front Link      |
| Link 3        | Pitch Bottom Link     |
| Link 4        | Pitch End Link        |
| Link 5        | Main Insertion Link   |
| Link 6        | Tool Link             |
| Link 7        | Pitch Top Link        |
| Link 8        | Pitch Back Link       |
| Link 9        | Remote Center Link    |
| Link 10       | End Link              |

### Joints -


| Joint Number  | Joint Name                | Associated Links (Parent to Child)| Type      |
|---------------|---------------------------|-----------------------------------|-----------|
| Joint 0       | Fixed to Setup Joint      | Link Parent to Link 0             | Fixed     |
| Joint 1       | Yaw Joint                 | Link 0 to Link 1                  | Revolute  |
| Joint 2       | Pitch Front Joint         | Link 1 to Link 2                  | Revolute  |
| Joint 3       | Pitch Bottom Joint        | Link 2 to Link 3                  | Revolute  |
| Joint 4       | Pitch End Joint           | Link 3 to Link 4                  | Revolute  |
| Joint 5       | Insertion Joint           | Link 4 to Link 5                  | Prismatic |
| Joint 6       | Roll Joint                | Link 5 to Link 6                  | Revolute  |
| Joint 7       | Pitch Top Joint           | Link 2 to Link 7                  | Revolute  |
| Joint 8       | Pitch Back Joint          | Link 1 to Link 8                  | Revolute  |
| Joint 9       | Remote Center Joint       | Link 0 to Link 9                  | Fixed     |
| Joint 10      | End Joint                 | Link 6 to Link 10                 | Fixed     |

---