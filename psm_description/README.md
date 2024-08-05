# PSM: Patient Side Manipulator -

PSM_exp -

```bash
ros2 launch psm_description urdf.launch.py file:=psm_exp.urdf.xacro
```

Both PSMs -

```bash
ros2 launch psm_description urdf.launch.py file:=both_psms.urdf.xacro
```

PSM -

Uncomment the type of tool that needs to be mounted on the PSM in the [psm.urdf.xacro](./urdf/psm.urdf.xacro) -

```xml
<?xml version="1.0"?>
<robot name="dvrk_psm" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- Macros -->
    <xacro:include filename="$(find psm_description)/urdf/psm.tool.xacro" />

    <link name="world" />

    <!--
        <xacro:psm_snake prefix="" parent_link="world" xyz="-0.25 0.0 0.5" rpy="0.0 0.0 ${PI}" />
        <xacro:psm_sca prefix="" parent_link="world" xyz="-0.25 0.0 0.5" rpy="0.0 0.0 ${PI}" />
        <xacro:psm_sca_blade prefix="" parent_link="world" xyz="-0.25 0.0 0.5" rpy="0.0 0.0 ${PI}" />
        <xacro:psm_caudier prefix="" parent_link="world" xyz="0.25 0.0 0.5" rpy="0.0 0.0 -${PI}" />
        <xacro:psm_caudier_blade prefix="" parent_link="world" xyz="-0.25 0.0 0.5" rpy="0.0 0.0 ${PI}"
        />
    -->

</robot>
```

```bash
ros2 launch psm_description urdf.launch.py file:=psm.urdf.xacro
```
