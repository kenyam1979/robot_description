
# robot_discription

This includes robot's URDF, config and launch files.

## Folder

```
.
├── CMakeLists.txt
├── config
│   ├── ekf.yaml                            # EKF config for localization
│   ├── mapper_params_online_async.yaml     # Config for SLAM
│   └── test_ekf.yaml                       # EKF config for localization on GAZEBO
├── launch
│   ├── display.launch.py                   # Rviz2 launch
│   ├── online_async.launch.py              # SLAM launch
│   ├── robot.launch.py                     # Robot TF publish
│   └── test.launch.py                      # Gazebo launch
├── package.xml
├── rviz
│   ├── nav2_default_view.rviz              # RViz2 config for navigation
│   └── urdf_config.rviz
├── src
│   └── description
│       └── robot_description.urdf          # Robot definition
├── world
|   └── map.sdf
└── README.md
```