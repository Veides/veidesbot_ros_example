cmake_minimum_required(VERSION 3.0.2)
project(veidesbot_platform)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package()

install(PROGRAMS
   nodes/veidesbot_platform_node
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
