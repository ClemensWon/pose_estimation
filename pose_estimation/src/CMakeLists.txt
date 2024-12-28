cmake_minimum_required(VERSION 3.1.3)
project(src)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  # ... other dependencies ...
  controller_manager_msgs
)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
