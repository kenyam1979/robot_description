
# robot_discription

This includes robot's URDF, config and launch files.

## Folder

```
.
│
├── config
│   ├── ekf.yaml                            # EKF config for localization
│   ├── mapper_params_online_async.yaml     # SLAM config
│   └── ros_gz_bridge.yaml                  # Bridge config for Gazebo Ingition (Fortress)
│
├── launch
│   ├── display.launch.py                   # Rviz2 launch
│   ├── online_async.launch.py              # SLAM launch
│   ├── robot.launch.py                     # Robot TF publish
│   ├── test_ign.launch.py                  # Gazebo Ignition (Fortress) launch
│   └── test.launch.py                      # Gazebo Classic launch
│
├── rviz
│   └── nav2_default_view.rviz              # RViz2 config for navigation
│
├── src
│   └── description
│       ├── robot_description_ign.urdf      # Robot definition for Gazebo Ignition (Fortress)
│       └── robot_description.urdf          # Robot definition for Gazebo Classic
│
├── world
│   ├── map_ign.sdf                         # Map for Gazebo Ignition (Fortress)
│   └── map.sdf                             # Map for Gazebo Classic
│
├── CMakeLists.txt
├── package.xml
└── README.md
```