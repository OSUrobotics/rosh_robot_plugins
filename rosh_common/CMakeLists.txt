cmake_minimum_required(VERSION 2.8.3)
project(rosh_common)
find_package(catkin REQUIRED COMPONENTS std_msgs sensor_msgs nav_msgs)
catkin_package(CATKIN_DEPENDS std_msgs sensor_msgs nav_msgs)
catkin_python_setup()