cmake_minimum_required(VERSION 3.0.2)
project(tracking_tester)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    tf2_msgs
    camera_apps_msgs
    )
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tracking_tester
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(tracking_tester_node src/tracking_tester_node.cpp src/tracking_tester.cpp src/hungarian.cpp)
add_dependencies(tracking_tester_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
    tracking_tester_node
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    )

