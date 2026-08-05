cmake_minimum_required(VERSION 3.0.2)
project(quad_uav_gazebo)

find_package(catkin REQUIRED)

catkin_package()

# Install ROS Python nodes and let catkin rewrite their shebang when needed.
catkin_install_python(PROGRAMS
  scripts/cloud_to_world.py
  scripts/gt_to_px4.py
  scripts/rc_sim.py
  scripts/vel_to_world.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/clean_env.sh
  scripts/rspx4.sh
  scripts/run_utils.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Keep Gazebo resources available from the package share directory in an
# install space as well as directly from the source space.
install(DIRECTORY
  models
  worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
