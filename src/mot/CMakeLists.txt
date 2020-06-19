cmake_minimum_required(VERSION 2.8.3)
project(mot)
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  pcl_conversions
  velodyne_pointcloud
  cv_bridge
)

catkin_package(CATKIN_DEPENDS
	sensor_msgs
	velodyne_pointcloud
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(Eigen3 REQUIRED)

#SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")
SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

link_directories(${PCL_LIBRARY_DIRS})

#Ground Plane Fitter
# find_package(PCL 1.8 REQUIRED)
# add_definitions(${PCL_DEFINITIONS})

add_executable( ground_removal
src/ground_fit.cpp
)

add_executable( road_fit
src/road_fit.cpp
)
add_executable( clustering
src/clustering.cpp
)

target_link_libraries(ground_removal
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${PCAP_INCLUDE_DIR} 
)

target_link_libraries(road_fit
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
  ${PCAP_INCLUDE_DIR} 

)

target_link_libraries(clustering
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${PCAP_INCLUDE_DIR} 
 
)
