################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(mercury_sdk)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-but-set-variable -Wno-unused-parameter -Wno-unused-variable)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include/${PROJECT_NAME}
)

set(MERCURY_SDK_SOURCES
  src/mercury_sdk/packet_handler.cpp
  src/mercury_sdk/protocol1_packet_handler.cpp
  src/mercury_sdk/protocol2_packet_handler.cpp
  src/mercury_sdk/group_sync_read.cpp
  src/mercury_sdk/group_sync_write.cpp
  src/mercury_sdk/group_bulk_read.cpp
  src/mercury_sdk/group_bulk_write.cpp
  src/mercury_sdk/port_handler.cpp
)

if(APPLE)
  add_library(mercury_sdk SHARED
    ${MERCURY_SDK_SOURCES}
    src/mercury_sdk/port_handler_mac.cpp
  )
elseif(WIN32)
  add_library(mercury_sdk SHARED
    ${MERCURY_SDK_SOURCES}
    src/mercury_sdk/port_handler_windows.cpp
  )
else()
  add_library(mercury_sdk SHARED
    ${MERCURY_SDK_SOURCES}
    src/mercury_sdk/port_handler_linux.cpp
  )
endif()

################################################################################
# Install
################################################################################
install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
