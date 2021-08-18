cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_accumulator)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        filters
        nodelet
        roscpp
        sensor_msgs
        rviz
        pcl_ros
        pointcloud_accumulator_msgs
        cartographer_ros_msgs
        geometry_msgs
        tf2_geometry_msgs
        )

find_package(PCL REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES pointcloud_accumulator
        CATKIN_DEPENDS filters nodelet roscpp sensor_msgs rviz pointcloud_accumulator_msgs cartographer_ros_msgs geometry_msgs tf2_geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

set(CMAKE_AUTOMOC ON)

if(rviz_QT_VERSION VERSION_LESS "5")
    message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
    ## pull in all required include dirs, define QT_LIBRARIES, etc.
    include(${QT_USE_FILE})
else()
    message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
    ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
    set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)


set(HEADER_FILES
        include/pointcloud_accumulator/pointcloud_accumulator.h
        include/pointcloud_accumulator/ikd_Tree.h
        include/pointcloud_accumulator/PointXYZRGBT.h
        )

set(SRC_FILES
        src/pointcloud_accumulator.cpp
        src/ikd_Tree.cpp
        src/pointcloud_accumulator_nodelet.cpp
        src/pointcloud_accumulator_node.cpp
        )

add_library(${PROJECT_NAME} ${SRC_FILES} ${HEADER_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pointcloud_accumulator_node src/pointcloud_accumulator_node.cpp)
target_link_libraries(pointcloud_accumulator_node ${PROJECT_NAME})


install(TARGETS
        ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )

install(FILES
        nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
