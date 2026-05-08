# Publish Limb Targets

## FR
```bash
ros2 topic pub --once /fr/target geometry_msgs/msg/Point "{x: -0, y: 0.02088, z: -0.078}"
```

## FL
```bash
ros2 topic pub --once /fl/target geometry_msgs/msg/Point "{x: -0, y: 1, z: -0.094}"
```

## BR
```bash
ros2 topic pub --once /br/target geometry_msgs/msg/Point "{x: -0, y: 0.02088, z: -0.00}"
```

## BL
```bash
ros2 topic pub --once /bl/target geometry_msgs/msg/Point "{x: 0.0, y: 0.04088, z: -0.03}"
```

```bash
#!/usr/bin/env bash
fr_haa=0; fr_hfe=0; fr_kfe=-0
fl_haa=0.3; fl_hfe=0; fl_kfe=-0
br_haa=0; br_hfe=0; br_kfe=-0
bl_haa=0; bl_hfe=0; bl_kfe=0

data=($fr_haa $fr_hfe $fr_kfe $fl_haa $fl_hfe $fl_kfe $br_haa $br_hfe $br_kfe $bl_haa $bl_hfe $bl_kfe)
IFS=,
ros2 topic pub --once /joint_group_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [${data[*]}]}"

```