# Installation
## Clone the repository
```bash
sudo apt-get install ros-humble-grid-map-cv ros-humble-grid-map-msgs ros-humble-grid-map-ros ros-humble-grid-map-sdf libmpfr-dev libpcap-dev ros-humble-grid-map-demos

# install pinocchio
git clone --recurse-submodules git@github.com:xu-yang16/pinocchio.git
# install hpp-fcl
git clone --recurse-submodules git@github.com:xu-yang16/hpp-fcl.git
# Clone ocs2_robotic_assets in ros2_ws/src
git clone git@github.com:xu-yang16/ocs2_robotic_assets.git
# Clone plane_segmentation_ros2 in ros2_ws/src
git clone git@github.com:xu-yang16/plane_segmentation_ros2.git
```
## Test convex plane decomposition
```bash
colcon build --packages-up-to convex_plane_decomposition_ros
# test
ros2 run convex_plane_decomposition_ros convex_plane_decomposition_ros_TestShapeGrowing
ros2 launch convex_plane_decomposition_ros demo.launch.py
```

## test ocs2_ros2
```bash
colcon build --packages-up-to ocs2_double_integrator_ros
source install/setup.bash
ros2 launch ocs2_double_integrator_ros double_integrator.launch.py

colcon build --packages-up-to ocs2_cartpole_ros
source install/setup.bash
ros2 launch ocs2_cartpole_ros cartpole.launch.py

colcon build --packages-up-to ocs2_legged_robot_ros
source install/setup.bash
ros2 launch ocs2_legged_robot_ros legged_robot_ddp.launch.py
ros2 launch ocs2_legged_robot_ros legged_robot_ipm.launch.py
ros2 launch ocs2_legged_robot_ros legged_robot_sqp.launch.py

colcon build --packages-up-to ocs2_mobile_manipulator_ros
source install/setup.bash
ros2 launch ocs2_mobile_manipulator_ros manipulator_franka.launch.py

colcon build --packages-up-to ocs2_anymal_loopshaping_mpc
source install/setup.bash
ros2 launch ocs2_anymal_loopshaping_mpc perceptive_mpc_demo.launch.py
```