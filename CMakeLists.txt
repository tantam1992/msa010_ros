cmake_minimum_required(VERSION 3.0.2)
project(msa010_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find dependencies
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_ros
  cv_bridge
  tf2_ros
  tf
  image_geometry
)

generate_dynamic_reconfigure_options(cfg/PCLFilter.cfg)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs pcl_ros cv_bridge tf2_ros tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(point_cloud_processing_node src/pcl_filter.cpp)
add_dependencies(point_cloud_processing_node ${PROJECT_NAME}_gencfg)
target_link_libraries(point_cloud_processing_node
  ${catkin_LIBRARIES}
  pcl_common
  pcl_io
  pcl_filters
)

add_executable(depth_image_to_point_cloud_node src/pc_publisher.cpp)
target_link_libraries(depth_image_to_point_cloud_node
  ${catkin_LIBRARIES}
)

add_executable(cliff_detector_node src/cliff_detector.cpp)
target_link_libraries(cliff_detector_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(laser_scan_filter_node src/laser_scan_filter.cpp)
target_link_libraries(laser_scan_filter_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  scripts/cliff_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/msa010_ros_driver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
install(TARGETS point_cloud_processing_node depth_image_to_point_cloud_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )