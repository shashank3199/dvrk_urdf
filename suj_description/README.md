# SUJ: Setup Joint -

#### SUJ Classic -

```bash
ros2 launch suj_description urdf.launch.py file:=suj.classic.urdf.xacro
```

#### SUJ Si -

```bash
ros2 launch suj_description urdf.launch.py file:=suj.si.urdf.xacro
```

---

## SUJ Classic ECMJ -

### Links -

| Link Number | Link Name           |
|-------------|---------------------|
| Link 8      | ECM Link 0          |
| Link 9      | ECM Link 1          |
| Link 10     | ECM Link 2          |
| Link 11     | ECM Link 3          |
| Link 12     | ECM Mounting Point  |

### Joints -

| Joint Number  | Joint Name                | Associated Links (Parent to Child)| Type          |
|---------------|---------------------------|-----------------------------------|---------------|
| Joint 7       | ECM Joint 0               | Link 1 to Link 8                  | Prismatic     |
| Joint 8       | ECM Joint 1               | Link 8 to Link 9                  | Continuous    |
| Joint 9       | ECM Joint 2               | Link 9 to Link 10                 | Continuous    |
| Joint 10      | ECM Joint 3               | Link 10 to Link 11                | Continuous    |
| Joint 11      | ECM RCM Joint             | Link 11 to Link 12                | Fixed         |

---

## SUJ Classic PSM1J -

### Links -

| Link Number | Link Name           |
|-------------|---------------------|
| Link 13     | PSM1 Link 0         |
| Link 14     | PSM1 Link 1         |
| Link 15     | PSM1 Link 2         |
| Link 16     | PSM1 Link 3         |
| Link 17     | PSM1 Link 4         |
| Link 18     | PSM1 Mounting Point |

### Joints -

| Joint Number  | Joint Name                | Associated Links (Parent to Child)| Type          |
|---------------|---------------------------|-----------------------------------|---------------|
| Joint 12      | PSM1 Joint 0              | Link 1 to Link 13                 | Prismatic     |
| Joint 13      | PSM1 Joint 1              | Link 13 to Link 14                | Continuous    |
| Joint 14      | PSM1 Joint 2              | Link 14 to Link 15                | Continuous    |
| Joint 15      | PSM1 Joint 3              | Link 15 to Link 16                | Continuous    |
| Joint 16      | PSM1 Joint 4              | Link 16 to Link 17                | Continuous    |
| Joint 17      | PSM1 RCM Joint            | Link 17 to Link 18                | Fixed         |

---

## SUJ Classic PSM2J -

### Links -

| Link Number | Link Name           |
|-------------|---------------------|
| Link 2      | PSM2 Link 0         |
| Link 3      | PSM2 Link 1         |
| Link 4      | PSM2 Link 2         |
| Link 5      | PSM2 Link 3         |
| Link 6      | PSM2 Link 4         |
| Link 7      | PSM2 Mounting Point |

### Joints -

| Joint Number  | Joint Name                | Associated Links (Parent to Child)| Type          |
|---------------|---------------------------|-----------------------------------|---------------|
| Joint 1       | PSM2 Joint 0              | Link 1 to Link 2                  | Prismatic     |
| Joint 2       | PSM2 Joint 1              | Link 2 to Link 3                  | Continuous    |
| Joint 3       | PSM2 Joint 2              | Link 3 to Link 4                  | Continuous    |
| Joint 4       | PSM2 Joint 3              | Link 4 to Link 5                  | Continuous    |
| Joint 5       | PSM2 Joint 4              | Link 5 to Link 6                  | Continuous    |
| Joint 6       | PSM2 RCM Joint            | Link 6 to Link 7                  | Fixed         |

---

## SUJ Classic PSM3J -

### Links -

| Link Number | Link Name           |
|-------------|---------------------|
| Link 19     | PSM3 Link 0         |
| Link 20     | PSM3 Link 1         |
| Link 21     | PSM3 Link 2         |
| Link 22     | PSM3 Link 3         |
| Link 23     | PSM3 Link 4         |
| Link 24     | PSM3 Mounting Point |

### Joints -

| Joint Number  | Joint Name                | Associated Links (Parent to Child)| Type          |
|---------------|---------------------------|-----------------------------------|---------------|
| Joint 18      | PSM3 Joint 0              | Link 1 to Link 19                 | Prismatic     |
| Joint 19      | PSM3 Joint 1              | Link 19 to Link 20                | Continuous    |
| Joint 20      | PSM3 Joint 2              | Link 20 to Link 21                | Continuous    |
| Joint 21      | PSM3 Joint 3              | Link 21 to Link 22                | Continuous    |
| Joint 22      | PSM3 Joint 4              | Link 22 to Link 23                | Continuous    |
| Joint 23      | PSM3 RCM Joint            | Link 23 to Link 24                | Fixed         |

---

## SUJ Si PSM12 Xacro -

### Links -

| Link Number | Link Name                       |
|-------------|---------------------------------|
| Link 2      | PSM 1/2 Joint Link 0            |
| Link 3      | PSM 1/2 Joint Link 1            |
| Link 4      | PSM 1/2 Joint Link 2            |
| Link 5      | PSM 1/2 Joint Link 3            |
| Link 6      | PSM 1/2 Joint Mounting Point    |

### Joints -

| Joint Number  | Joint Name                    | Associated Links (Parent to Child)| Type          |
|---------------|-------------------------------|-----------------------------------|---------------|
| Joint 1       | PSM 1/2 Joint Prismatic Joint | Link 1 to Link 2                  | Prismatic     |
| Joint 2       | PSM 1/2 Joint Revolute Joint  | Link 2 to Link 3                  | Revolute      |
| Joint 3       | PSM 1/2 Joint Revolute Joint  | Link 3 to Link 4                  | Revolute      |
| Joint 4       | PSM 1/2 Joint Revolute Joint  | Link 4 to Link 5                  | Revolute      |
| Joint 5       | PSM 1/2 Joint Fixed Joint     | Link 3 to Link 6                  | Fixed         |

---

## SUJ Si PSM3 Xacro -

### Links -

| Link Number | Link Name           |
|-------------|---------------------|
| Link 7      | PSM3 Link 0         |
| Link 8      | PSM3 Link 1         |
| Link 9      | PSM3 Link 2         |
| Link 10     | PSM3 Link 3         |
| Link 11     | PSM3 Link 4         |
| Link 12     | PSM3 Mounting Point |

### Joints -

| Joint Number  | Joint Name                | Associated Links (Parent to Child)| Type          |
|---------------|---------------------------|-----------------------------------|---------------|
| Joint 6       | PSM3 Joint 0              | Link 1 to Link 7                  | Prismatic     |
| Joint 7       | PSM3 Joint 1              | Link 7 to Link 8                  | Revolute      |
| Joint 8       | PSM3 Joint 2              | Link 8 to Link 9                  | Revolute      |
| Joint 9       | PSM3 Joint 3              | Link 9 to Link 10                 | Revolute      |
| Joint 10      | PSM3 Joint 4              | Link 10 to Link 11                | Revolute      |
| Joint 11      | PSM3 Fixed Joint          | Link 11 to Link 12                | Fixed         |

---

## SUJ Si ECM Xacro -

### Links -

| Link Number | Link Name           |
|-------------|---------------------|
| Link 13     | ECM Link 0          |
| Link 14     | ECM Link 1          |
| Link 15     | ECM Link 2          |
| Link 16     | ECM Link 3          |
| Link 17     | ECM Mounting Point  |

### Joints -

| Joint Number  | Joint Name                | Associated Links (Parent to Child)| Type          |
|---------------|---------------------------|-----------------------------------|---------------|
| Joint 12      | ECM Joint 0               | Link 1 to Link 13                 | Prismatic     |
| Joint 13      | ECM Joint 1               | Link 13 to Link 14                | Revolute      |
| Joint 14      | ECM Joint 2               | Link 14 to Link 15                | Revolute      |
| Joint 15      | ECM Joint 3               | Link 15 to Link 16                | Revolute      |
| Joint 16      | ECM Fixed Joint           | Link 15 to Link 17                | Fixed         |

---