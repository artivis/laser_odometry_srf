cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)
project(laser_odometry_srf)

find_package(catkin REQUIRED COMPONENTS
  laser_odometry_core srf_laser_odometry
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS laser_odometry_core srf_laser_odometry
)

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

###############
##   Build   ##
###############

add_library(${PROJECT_NAME} src/laser_odometry_srf.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Requires C++11
if(CMAKE_VERSION VERSION_LESS "3.1")
  set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-std=c++11")
else()
  set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
  set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)
  set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_EXTENSIONS OFF)
endif()

################
##  Install   ##
################

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES laser_odometry_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)