cmake_minimum_required(VERSION 2.8.3)
project(visual-mcl)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    apriltag_ros
    image_transport
    cv_bridge
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs apriltag_ros image_transport cv_bridge
    DEPENDS OpenCV Eigen3
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

# add_executable(${PROJECT_NAME}_node
#     src/particle_filter.cpp
# )

target_link_libraries(${PROJECT_NAME}-node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
