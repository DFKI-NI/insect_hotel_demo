cmake_minimum_required(VERSION 3.0.2)
project(insect_hotel_bringup)

# add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS)

catkin_package()

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  nodes/simulate_worker_gazebo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)