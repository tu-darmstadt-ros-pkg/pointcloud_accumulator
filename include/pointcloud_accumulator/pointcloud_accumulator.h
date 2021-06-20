#ifndef POINTCLOUD_ACCUMULATOR_H
#define POINTCLOUD_ACCUMULATOR_H

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pointcloud_accumulator/ikd_Tree.h>


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
        std::string static_frame;
        double downsample_resolution;

        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;

        ros::Publisher pub;
        ros::Subscriber sub;

        int counter;

        KD_TREE* kd_tree;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;

    };
} // namespace pointcloud_accumulator

#endif