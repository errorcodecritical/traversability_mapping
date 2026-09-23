# Traversability Mapping ROS2

This is a ROS 2 port of the original ROS 1 `traversability_mapping` package. It creates occupancy maps for uneven terrain, including forest environments, and is designed for use with Nav2 (Navigation Stack 2).

This package was tested on the BunkerMini robot with a Hesai QT128C2X LiDAR.

## Packages

- `traversability_mapping` filters lidar data, builds an elevation/traversability map, publishes a standard occupancy grid, and provides PRM/path-planning visualization.
- `elevation_msgs` defines `OccupancyElevation`, which combines a `nav_msgs/OccupancyGrid` with per-cell elevation and cost arrays.

## Key Features

- Elevation mapping from point clouds
- Occupancy-grid map generation
- Traversability calculation based on slope and height differences
- Dynamic map updating with Kalman filtering
- Nav2 integration through a standard `nav_msgs/msg/OccupancyGrid`

## Runtime pipeline

1. `traversability_filter` subscribes to `/hesai/points_decimated` and publishes filtered clouds.
2. `traversability_map` consumes the filtered cloud and publishes:
   - `/occupancy_map_local` (`nav_msgs/OccupancyGrid`) for Nav2
   - `/occupancy_map_local_height` (`elevation_msgs/OccupancyElevation`)
3. `traversability_prm` consumes the elevation map and publishes PRM/path visualization.
4. `traversability_path` can generate trajectory-library paths from elevation-map goals.

The default frames are `map`, `base_link`, and `hesai_lidar`. All topics, frames, sensor/map geometry, filtering thresholds, traversability weights, occupancy fusion, PRM settings, and trajectory settings are configured in:

```text
traversability_mapping/config/traversability_mapping.yaml
```

`config/nav2_costmap.yaml` contains example Nav2 StaticLayer configuration using `/occupancy_map_local`.

## Build

Inside a ROS 2 Jazzy workspace:

```bash
source /opt/ros/jazzy/setup.bash
cd /ros2_ws
colcon build --symlink-install --packages-select elevation_msgs traversability_mapping
source install/setup.bash
```

The workspace must provide PCL, OpenCV, `cv_bridge`, `pcl_conversions`, TF2, and the listed ROS 2 message packages.

### ROS 2 dependencies

- `rclcpp`
- `tf2_ros`
- `tf2_geometry_msgs`
- `sensor_msgs`
- `geometry_msgs`
- `nav_msgs`
- `cv_bridge`
- `image_transport`
- `pcl_ros`
- `pcl_conversions`
- `laser_geometry`
- Nav2 packages when using Nav2 costmaps

### External libraries

- PCL
- OpenCV
- Eigen3
- Boost

## Launch

```bash
ros2 launch traversability_mapping traversability_mapping.launch.py
```

Use simulation time when required:

```bash
ros2 launch traversability_mapping traversability_mapping.launch.py use_sim_time:=true
```

The lidar driver and a valid TF tree must be running separately. At minimum, the configured input cloud topic and transforms from `map` to `hesai_lidar` and `base_link` must be available.

## Installation

Clone this repository into the `src` directory of a ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone <repository_url>
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Topics

### Published

- `/occupancy_map_local` (`nav_msgs/msg/OccupancyGrid`): Local occupancy map for Nav2.
- `/occupancy_map_local_height` (`elevation_msgs/msg/OccupancyElevation`): Occupancy map with elevation and cost data.
- `/elevation_pointcloud` (`sensor_msgs/msg/PointCloud2`): Elevation map for visualization.
- `/filtered_pointcloud` (`sensor_msgs/msg/PointCloud2`): Filtered point cloud consumed by the mapper.

### Subscribed

- `/hesai/points_decimated` (`sensor_msgs/msg/PointCloud2`): Default raw lidar input.
- `/prm_goal` (`geometry_msgs/msg/PoseStamped`): Optional PRM goal input.

Topics, frames, sensor geometry, filtering thresholds, traversability weights, occupancy fusion, PRM settings, and trajectory settings are configured in `config/traversability_mapping.yaml`.

## Nav2 Integration

The mapping node publishes a standard `nav_msgs/msg/OccupancyGrid` on `/occupancy_map_local`. Nav2 can consume this through a `nav2_costmap_2d::StaticLayer`.

An example configuration is provided in `config/nav2_costmap.yaml`. Enable transient-local map subscription and set the StaticLayer `map_topic` to `/occupancy_map_local`.

## Cite *Traversability Mapping*

Please cite the following paper if you use this code:

```bibtex
@inproceedings{bayesian2018shan,
  title={Bayesian Generalized Kernel Inference for Terrain Traversability Mapping},
  author={Shan, Tixiao and Wang, Jinkun and Englot, Brendan and Doherty, Kevin},
  booktitle={In Proceedings of the 2nd Annual Conference on Robot Learning},
  year={2018}
}
```
