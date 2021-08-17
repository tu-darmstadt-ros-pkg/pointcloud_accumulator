cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_accumulator_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        std_msgs
        message_generation
        )

################################################
## Declare ROS messages, services and actions ##
################################################


add_service_files(
        FILES
        SavePointCloud.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS std_msgs message_runtime
)
