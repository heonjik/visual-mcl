cmake_minimum_required(VERSION 2.8.3)
project(visual-mcl)

# Find other packages needed to build
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    apriltag_ros
    cv_bridge
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Specify package build info export
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ParticleFilter
    CATKIN_DEPENDS roscpp std_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs apriltag_ros cv_bridge
    DEPENDS OpenCV Eigen3
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

# Libraries/Executables to build
file(GLOB_RECURSE SOURCE src/*.cpp)

add_library(ParticleFilter src/particle_filter.cpp)

add_executable(${PROJECT_NAME}-node src/particle_filter.cpp)

target_link_libraries(ParticleFilter
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}-node
    ParticleFilter
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
