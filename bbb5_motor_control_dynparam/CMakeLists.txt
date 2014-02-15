cmake_minimum_required(VERSION 2.8.3)
project(bbb5_motor_control_dynparam)

#add dynamic reconfigure api
find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/planner.cfg
  #...
)


catkin_package(CATKIN_DEPENDS rospy std_msgs)
