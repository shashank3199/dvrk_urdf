# PSM: Patient Side Manipulator -

Both PSMs Classic -

```bash
ros2 launch psm_description urdf.launch.py file:=both_psms.classic.urdf.xacro
```

PSM Classic -

Change the type of tool that needs to be mounted on the PSM in the [psm.classic.urdf.xacro](./urdf/psm.classic.urdf.xacro) -

```xml
    <!-- Tool Arguments -->
    <!-- Select tool_name from blade, caudier_blade, caudier, sca_blade, sca, snake -->
    <xacro:property name="tool_name" default="sca" />
```

```bash
ros2 launch psm_description urdf.launch.py file:=psm.classic.urdf.xacro
```

PSM Si -

Change the type of tool that needs to be mounted on the PSM in the [psm.si.urdf.xacro](./urdf/psm.si.urdf.xacro) -

```xml
    <!-- Tool Arguments -->
    <!-- Select tool_name from P420006, SF826001 -->
    <xacro:property name="tool_name" default="P420006" />
```

```bash
ros2 launch psm_description urdf.launch.py file:=psm.si.urdf.xacro
```

---

## PSM Base Xacro -

### Links -

| Link Number | Link Name                           |
|-------------|-------------------------------------|
| Link 0      | PSM Base Link                       |
| Link 1      | Outer Yaw Link                      |
| Link 2      | Outer Pitch Link                    |
| Link 2-1    | Outer Pitch Back Link               |
| Link 2-2    | Outer Pitch Front Link              |
| Link 2-3    | Outer Pitch Bottom Link             |
| Link 2-4    | Outer Pitch Top Link                |
| Link 2-5    | Outer Insertion Link                |
| Link 3      | Tool Main Link                      |
| Link 4      | Tool Wrist Link                     |
| Link 4-1    | Tool Wrist Shaft Link               |

### Joints -

| Joint Number  | Joint Name                | Associated Links (Parent to Child)| Type          |
|---------------|---------------------------|-----------------------------------|---------------|
| Joint 0       | Fixed Joint               | Parent Link to Link 0             | Fixed         |
| Joint 1       | Outer Yaw Joint           | Link 0 to Link 1                  | Revolute      |
| Joint 2       | Outer Pitch Joint         | Link 1 to Link 2                  | Revolute      |
| Joint 2-1     | Outer Pitch 1 Joint       | Link 1 to Link 2-1                | Continuous    |
| Joint 2-2     | Outer Pitch 2 Joint       | Link 1 to Link 2-2                | Continuous    |
| Joint 2-3     | Outer Pitch 3 Joint       | Link 2-1 to Link 2-3              | Continuous    |
| Joint 2-4     | Outer Pitch 4 Joint       | Link 2-1 to Link 2-4              | Continuous    |
| Joint 2-5     | Outer Pitch 5 Joint       | Link 2-3 to Link 2-5              | Continuous    |
| Joint 3       | Outer Insertion Joint     | Link 2 to Link 3                  | Prismatic     |
| Joint 4       | Outer Roll Joint          | Link 3 to Link 4                  | Revolute      |
| Joint 4-1     | Outer Roll Shaft Joint    | Link 4 to Link 4-1                | Fixed         |

---

## PSM Tool Blade Xacro -

### Links -

| Link Number | Link Name                           |
|-------------|-------------------------------------|
| Link 5      | Tool Wrist Sca Link                 |
| Link 6      | Tool Wrist Sca Shaft Link           |
| Link 7-0    | Outer Open Angle Virtual Link       |
| Link 8      | Tool Tip Link                       |

### Joints -

| Joint Number  | Joint Name                | Associated Links (Parent to Child)| Type      |
|---------------|---------------------------|-----------------------------------|-----------|
| Joint 5       | Outer Wrist Pitch Joint   | Link 4-1 to Link 5                | Revolute  |
| Joint 6       | Outer Wrist Yaw Joint     | Link 5 to Link 6                  | Revolute  |
| Joint 7-0     | Outer Open Angle 1 Joint  | Link 6 to Link 7-0                | Revolute  |
| Joint 7-1     | Tool Tip Joint            | Link 7-0 to Link 8                | Fixed     |

---

## PSM Tool Caudier Blade Xacro -

### Links -

| Link Number | Link Name                           |
|-------------|-------------------------------------|
| Link 5      | Tool Wrist Caudier Link             |
| Link 6      | Tool Wrist Caudier Shaft Link       |
| Link 7-0    | Outer Open Angle Virtual Link       |
| Link 6-3    | Tool Tip Link                       |

### Joints -

| Joint Number  | Joint Name                | Associated Links (Parent to Child)| Type      |
|---------------|---------------------------|-----------------------------------|-----------|
| Joint 5       | Outer Wrist Pitch Joint   | Link 4-1 to Link 5                | Revolute  |
| Joint 6       | Tool Wrist Yaw Joint      | Link 5 to Link 6                  | Revolute  |
| Joint 7-0     | Outer Open Angle 1        | Link 6 to Link 7-0                | Fixed     |
| Joint 7-1     | Tool Tip Joint            | Link 6 to Link 6-3                | Fixed     |

---

## PSM Tool Caudier Xacro -

### Links -

| Link Number | Link Name                           |
|-------------|-------------------------------------|
| Link 5      | Tool Wrist Caudier Link             |
| Link 6      | Tool Wrist Caudier Shaft Link       |
| Link 7-0    | Outer Open Angle Virtual Link       |
| Link 7-1    | Outer Open Angle 1 Link             |
| Link 7-2    | Outer Open Angle 2 Link             |
| Link 6-3    | Tool Wrist Caudier EE Link          |

### Joints -

| Joint Number  | Joint Name                | Associated Links (Parent to Child)| Type      |
|---------------|---------------------------|-----------------------------------|-----------|
| Joint 5       | Outer Wrist Pitch Joint   | Link 4-1 to Link 5                | Revolute  |
| Joint 6       | Tool Wrist Yaw Joint      | Link 5 to Link 6                  | Revolute  |
| Joint 7-0     | Outer Open Angle 1        | Link 6 to Link 7-0                | Revolute  |
| Joint 7-1     | Outer Open Angle 1        | Link 6 to Link 7-1                | Revolute  |
| Joint 7-2     | Outer Open Angle 2        | Link 6 to Link 7-2                | Revolute  |
| Joint 7-3     | Tool Tip Joint            | Link 6 to Link 6-3                | Fixed     |

---

## PSM Tool SCA Blade Xacro -

### Links -

| Link Number | Link Name                           |
|-------------|-------------------------------------|
|Link 5       | Tool Wrist Sca Link                 |
|Link 6       | Tool Wrist Sca Shaft Link           |
|Link 7-0     | Outer Open Angle Virtual Link       |
|Link 8       | Tool Tip Link                       |

### Joints -

| Joint Number  | Joint Name                | Associated Links (Parent to Child)| Type      |
|---------------|---------------------------|-----------------------------------|-----------|
| Joint 5       | Outer Wrist Pitch Joint   | Link 4-1 to Link 5                | Revolute  |
| Joint 6       | Outer Wrist Yaw Joint     | Link 5 to Link 6                  | Revolute  |
| Joint 7-0     | Outer Open Angle 1 Joint  | Link 6 to Link 7-0                | Fixed     |
| Joint 7-1     | Tool Tip Joint            | Link 7-0 to Link 8                | Fixed     |

---

## PSM Tool SCA Xacro -

### Links -

| Link Number | Link Name                           |
|-------------|-------------------------------------|
| Link 5      | Tool Wrist Sca Link                 |
| Link 6      | Tool Wrist Sca Shaft Link           |
| Link 7-0    | Outer Open Angle Virtual Link       |
| Link 7-1    | Outer Open Angle 1 Link             |
| Link 7-2    | Outer Open Angle 2 Link             |
| Link 8      | Tool Tip Link                       |

### Joints -

| Joint Number  | Joint Name                | Associated Links (Parent to Child)| Type      |
|---------------|---------------------------|-----------------------------------|-----------|
| Joint 5       | Outer Wrist Pitch Joint   | Link 4-1 to Link 5                | Revolute  |
| Joint 6       | Outer Wrist Yaw Joint     | Link 5 to Link 6                  | Revolute  |
| Joint 7-0     | Outer Open Angle Joint    | Link 6 to Link 7-0                | Revolute  |
| Joint 7-1     | Outer Open Angle 1 Joint  | Link 6 to Link 7-1                | Revolute  |
| Joint 7-2     | Outer Open Angle 2 Joint  | Link 6 to Link 7-2                | Revolute  |
| Joint 7-3     | Tool Tip Joint            | Link 6 to Link 8                  | Fixed     |

---

## PSM Tool Snake Xacro -

### Links -

| Link Number | Link Name                           |
|-------------|-------------------------------------|
| Link 6      | Tool Snake Disc 1 Link              |
| Link 7      | Tool Snake Disc 2 Link              |
| Link 8      | Tool Snake Disc 3 Link              |
| Link 9      | Tool Snake End Link                 |
| Link 11-0   | Outer Open Angle Virtual Link       |
| Link 11-1   | Outer Open Angle 1 Link             |
| Link 11-2   | Outer Open Angle 2 Link             |

### Joints -

| Joint Number  | Joint Name                | Associated Links (Parent to Child)| Type      |
|---------------|---------------------------|-----------------------------------|-----------|
| Joint 6       | Outer Wrist Pitch 1 Joint | Link 4-1 to Link 6                | Revolute  |
| Joint 7       | Outer Wrist Yaw 1 Joint   | Link 6 to Link 7                  | Revolute  |
| Joint 8       | Outer Wrist Yaw 2 Joint   | Link 7 to Link 8                  | Revolute  |
| Joint 9       | Outer Wrist Pitch 2 Joint | Link 8 to Link 9                  | Revolute  |
| Joint 11-0    | Outer Open Angle Joint    | Link 9 to Link 11-0               | Revolute  |
| Joint 11-1    | Outer Open Angle 1 Joint  | Link 9 to Link 11-1               | Revolute  |
| Joint 11-2    | Outer Open Angle 2 Joint  | Link 9 to Link 11-2               | Revolute  |

---

## PSM Si Base Xacro -

### Links -

| Link Number | Link Name           |
|-------------|---------------------|
| Link 0      | Link 0              |
| Link 1      | Link 1              |
| Link 2      | Link 2              |
| Link 3      | Link 3              |
| Link 4      | Link 4              |
| Link 5      | Tool Parent Link    |
| Link 6      | RCM Link            |

### Joints -

| Joint Number  | Joint Name                        | Associated Links (Parent to Child)| Type          |
|---------------|-----------------------------------|-----------------------------------|---------------|
| Joint 0       | Fixed Joint                       | Macro Parent Link to Link 0       | Fixed         |
| Joint 1       | Yaw Joint                         | Link 0 to Link 1                  | Revolute      |
| Joint 2       | Pitch Joint                       | Link 1 to Link 2                  | Revolute      |
| Joint 3       | Pitch Joint                       | Link 2 to Link 3                  | Revolute      |
| Joint 4       | Pitch Joint                       | Link 3 to Link 4                  | Revolute      |
| Joint 5       | Outer Insertion Reference Joint   | Link 4 to Link 5                  | Fixed         |
| Joint 6       | RCM Joint                         | Link 4 to Link 6                  | Fixed         |

---

## PSM Tool P420006 Xacro -

### Links -

| Link Number | Link Name                   |
|-------------|-----------------------------|
| Link 0      | Tool Main Link              |
| Link 1      | Tool Wrist Link             |
| Link 2      | Tool Wrist Shaft Link       |
| Link 3      | Tool Wrist SCA Link         |
| Link 4      | Tool Wrist SCA Shaft Link   |
| Link 5      | Tool Wrist SCA EE Link      |
| Link 6      | Tool Wrist SCA EE Link 1    |
| Link 7      | Tool Wrist SCA EE Link 2    |

### Joints -

| Joint Number  | Joint Name                        | Associated Links (Parent to Child)| Type          |
|---------------|-----------------------------------|-----------------------------------|---------------|
| Joint 0       | Insertion Joint                   | Parent Link to Link 0             | Prismatic     |
| Joint 1       | Roll Joint                        | Link 0 to Link 1                  | Revolute      |
| Joint 2       | Roll Shaft Joint                  | Link 1 to Link 2                  | Fixed         |
| Joint 3       | Wrist Pitch Joint                 | Link 2 to Link 3                  | Revolute      |
| Joint 4       | Wrist Yaw Joint                   | Link 3 to Link 4                  | Revolute      |
| Joint 5       | Jaw Joint                         | Link 4 to Link 5                  | Revolute      |
| Joint 6       | Jaw 1 Joint                       | Link 4 to Link 6                  | Revolute      |
| Joint 7       | Jaw 2 Joint                       | Link 4 to Link 7                  | Revolute      |

---

## PSM Tool SF826001 Xacro -

### Links -

| Link Number | Link Name                   |
|-------------|-----------------------------|
| Link 0      | Tool Main Link              |
| Link 1      | Tool Roll Link              |

### Joints -

| Joint Number  | Joint Name                        | Associated Links (Parent to Child)| Type          |
|---------------|-----------------------------------|-----------------------------------|---------------|
| Joint 0       | Insertion Joint                   | Parent Link to Link 0             | Prismatic     |
| Joint 1       | Roll Joint                        | Link 0 to Link 1                  | Revolute      |

---