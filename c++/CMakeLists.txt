################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(mercury_sdk)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mercury_sdk
)

################################################################################
# Build
################################################################################
include_directories(
  include/mercury_sdk
  ${catkin_INCLUDE_DIRS}
)

  add_library(mercury_sdk
    src/mercury_sdk/packet_handler.cpp
    src/mercury_sdk/protocol_packet_handler.cpp
    src/mercury_sdk/group_sync_read.cpp
    src/mercury_sdk/group_sync_write.cpp
    src/mercury_sdk/group_bulk_read.cpp
    src/mercury_sdk/group_bulk_write.cpp
    src/mercury_sdk/port_handler.cpp
  )
if(APPLE)
  target_sources(mercury_sdk PRIVATE src/mercury_sdk/port_handler_mac.cpp)
else()
  target_sources(mercury_sdk PRIVATE src/mercury_sdk/port_handler_linux.cpp)
endif()

add_dependencies(mercury_sdk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mercury_sdk ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS mercury_sdk
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

################################################################################
# Test
################################################################################
