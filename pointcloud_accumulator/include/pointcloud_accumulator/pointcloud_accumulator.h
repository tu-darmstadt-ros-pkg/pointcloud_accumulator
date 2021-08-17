#ifndef POINTCLOUD_ACCUMULATOR_H
#define POINTCLOUD_ACCUMULATOR_H

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pointcloud_accumulator/ikd_Tree.h>
#include <pointcloud_accumulator_msgs/SavePointCloud.h>
#include <std_srvs/Trigger.h>
#include <cartographer_ros_msgs/SubmapList.h>
#include <cartographer_ros_msgs/StampedSubmapEntry.h>


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
        bool reset();
        int calculate_adaptive_increment(double add_duration, int adaptive_param);
        void submap_announcements(const cartographer_ros_msgs::StampedSubmapEntry::ConstPtr& msg);
        void submap_update(const cartographer_ros_msgs::SubmapList::ConstPtr& msg);

        std::string static_frame;
        double downsample_resolution;
        bool use_submaps;
        double add_duration;
        int adaptive_incr;
        std::vector<cartographer_ros_msgs::StampedSubmapEntry> submap_list;

        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;

        ros::Publisher pub;
        ros::Subscriber sub;
        ros::Subscriber submap_announcement_sub;
        ros::Subscriber submap_update_sub;

        ros::ServiceServer save_map_service;
        ros::ServiceServer reset_map_service;

        KD_TREE* kd_tree;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;

    };
} // namespace pointcloud_accumulator

#endif