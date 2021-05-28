cmake_minimum_required(VERSION 3.0.2)
project(veidesbot_platform)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)

catkin_python_setup()

add_message_files(
   DIRECTORY
   msg
   FILES
   ShutdownMethod.msg
)

add_service_files(
  DIRECTORY
  srv
  FILES
  ShutdownMethodCompleted.srv
)

generate_messages()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
   nodes/veidesbot_platform_node
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
