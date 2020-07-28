cmake_minimum_required(VERSION 2.8.3)
project(artificial_potential_fields)

find_package(catkin REQUIRED COMPONENTS roscpp roslib dynamic_reconfigure std_msgs sensor_msgs geometry_msgs tf pcl_conversions pcl_ros laser_geometry)
find_package(PCL 1.2 REQUIRED)
find_package(gazebo REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

generate_dynamic_reconfigure_options(cfg/setAPF.cfg)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(teleop_key src/teleop_key.cpp)
target_link_libraries(teleop_key ${catkin_LIBRARIES})

add_executable(teleop_joy src/teleop_joy.cpp)
target_link_libraries(teleop_joy ${catkin_LIBRARIES})

add_executable(apf src/APF.cpp)
target_link_libraries(apf ${catkin_LIBRARIES})
add_dependencies(apf ${PROJECT_NAME}_gencfg)

add_executable(cloud_viewer src/cloud_viewer.cpp)
target_link_libraries(cloud_viewer ${PCL_LIBRARIES} ${catkin_LIBRARIES})
