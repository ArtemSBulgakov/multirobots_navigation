# Multirobots Navigation

Based on [nav2_bringup](https://github.com/ros-planning/navigation2/tree/1.1.8/nav2_bringup).

## Running

```bash
colcon build
ros2 launch multirobots_navigation multirobots_navigation_launch.py
```
To add a keep out area:
```bash
mkdir -p ~/tutorials_ws/src
cd ~/tutorials_ws/src
git clone https://github.com/ros-planning/navigation2_tutorials.git
cd ~/tutorials_ws
colcon build --symlink-install --packages-select nav2_costmap_filters_demo
source ~/tutorials_ws/install/setup.bash
```
Run in a separate terminal before launching robots:
```bash
ros2 launch nav2_costmap_filters_demo costmap_filter_info.launch.py params_file:=src/navigation2_tutorials/nav2_costmap_filters_demo/params/keepout_params.yaml mask:=src/navigation2_tutorials/nav2_costmap_filters_demo/maps/keepout_mask.yaml
```
