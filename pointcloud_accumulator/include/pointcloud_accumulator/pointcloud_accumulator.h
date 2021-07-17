#ifndef POINTCLOUD_ACCUMULATOR_H
#define POINTCLOUD_ACCUMULATOR_H

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pointcloud_accumulator/ikd_Tree.h>
#include <pointcloud_accumulator_msgs/SavePointCloud.h>
#include <std_srvs/Trigger.h>

namespace pointcloud_accumulator
{

    class PointcloudAccumulator
    {
    public:
        PointcloudAccumulator(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
        virtual void init();
        void publish_pointcloud();

    private:
        void callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
        bool savePointcloud(pointcloud_accumulator_msgs::SavePointCloud::Request &req,
                            pointcloud_accumulator_msgs::SavePointCloud::Response &res);
        bool resetPointcloud(std_srvs::Trigger::Request  &req,
                             std_srvs::Trigger::Response &res);

        std::string static_frame;
        double downsample_resolution;

        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;

        ros::Publisher pub;
        ros::Subscriber sub;
        ros::ServiceServer save_map_service;
        ros::ServiceServer reset_map_service;

        KD_TREE* kd_tree;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;

    };
} // namespace pointcloud_accumulator

#endif