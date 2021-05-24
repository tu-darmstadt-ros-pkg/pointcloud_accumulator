#include <ros/ros.h>
#include <nodelet/loader.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_accumulator_node");

    nodelet::Loader nodelet;
    const nodelet::M_string& remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    const std::string& nodelet_name = ros::this_node::getName();
    ROS_INFO_STREAM("Started " << nodelet_name << " nodelet.");
    nodelet.load(nodelet_name, "pointcloud_accumulator/PointcloudAccumulatorNodelet", remap, nargv);

    ros::spin();
    return 0;
}