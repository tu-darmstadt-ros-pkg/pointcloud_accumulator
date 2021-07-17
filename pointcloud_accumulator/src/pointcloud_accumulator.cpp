#include <pointcloud_accumulator/pointcloud_accumulator.h>
#include <rviz/default_plugin/point_cloud_transformers.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>


namespace pointcloud_accumulator
{

PointcloudAccumulator::PointcloudAccumulator(const ros::NodeHandle& nh, const ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh), tfListener(tfBuffer){
    pnh.param<double>("downsample_resolution", downsample_resolution, 0.1);
    pnh.param<std::string>("static_frame", static_frame, "odom");
}

void PointcloudAccumulator::init(){

    kd_tree = new KD_TREE(0.3, 0.6, downsample_resolution);

    sub = nh_.subscribe<sensor_msgs::PointCloud2>("cloud_in", 1, &PointcloudAccumulator::callback, this);
    pub = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("cloud_out", 1, false);

    save_map_service = pnh_.advertiseService("save_pointcloud", &PointcloudAccumulator::savePointcloud, this);
    reset_map_service = pnh_.advertiseService("reset_pointcloud", &PointcloudAccumulator::resetPointcloud, this);

    PointVector p;
    PointType pt;
    pt.x = 0; pt.y = 0; pt.z = 0;
    p.push_back(pt);
    kd_tree->Build(p);
    }

void PointcloudAccumulator::callback(const sensor_msgs::PointCloud2::ConstPtr& msg){

    if(!tfBuffer.canTransform(static_frame, msg->header.frame_id, msg->header.stamp, ros::Duration(0.01))){
        ROS_DEBUG("Can't transform from %s to %s!", msg->header.frame_id.c_str(), static_frame.c_str());
        return;
    }
    sensor_msgs::PointCloud2Ptr cloud(new sensor_msgs::PointCloud2);
    pcl_ros::transformPointCloud(static_frame, *msg, *cloud, tfBuffer);

    //Transmit point cloud data to the ikd-tree
    int32_t xi = rviz::findChannelIndex(msg, "x");
    int32_t yi = rviz::findChannelIndex(msg, "y");
    int32_t zi = rviz::findChannelIndex(msg, "z");
    int32_t rgbi = rviz::findChannelIndex(msg, "rgb");


    const uint32_t xoff = msg->fields[xi].offset;
    const uint32_t yoff = msg->fields[yi].offset;
    const uint32_t zoff = msg->fields[zi].offset;
    const uint32_t rgboff = msg->fields[rgbi].offset;

    const uint8_t type = msg->fields[xi].datatype;
    const uint32_t point_step = msg->point_step;

    PointVector points;

    for(size_t i = 0; i < cloud->width*msg->height; i++){
        PointType p;
        p.x = rviz::valueFromCloud<float>(cloud, xoff, type, point_step, i);
        p.y = rviz::valueFromCloud<float>(cloud, yoff, type, point_step, i);
        p.z = rviz::valueFromCloud<float>(cloud, zoff, type, point_step, i);
        p.rgb = rviz::valueFromCloud<float>(cloud, rgboff, type, point_step, i);

        points.push_back(p);
    }

    kd_tree->Add_Points(points, true);

    }

void PointcloudAccumulator::publish_pointcloud(){

    PointVector points;
    kd_tree->flatten(kd_tree->Root_Node, points, NOT_RECORD);

    sensor_msgs::PointCloud2Ptr msg(new sensor_msgs::PointCloud2);
    msg->header.stamp = ros::Time::now();
    msg->header.frame_id = static_frame;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->width = points.size();
    cloud->height = 1;
    pcl_conversions::toPCL(msg->header, cloud->header);
    cloud->is_dense = true;
    for (size_t i = 0; i < points.size(); i++) {
        cloud->points.push_back(points[i]);
    }
    pub.publish(cloud);
}

bool PointcloudAccumulator::savePointcloud(pointcloud_accumulator_msgs::SavePointCloud::Request  &req,
                                               pointcloud_accumulator_msgs::SavePointCloud::Response &res){
    PointVector points;
    kd_tree->flatten(kd_tree->Root_Node, points, NOT_RECORD);

    sensor_msgs::PointCloud2Ptr msg(new sensor_msgs::PointCloud2);
    msg->header.stamp = ros::Time::now();
    msg->header.frame_id = static_frame;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->height = 1;
    pcl_conversions::toPCL(msg->header, cloud->header);
    cloud->is_dense = true;
    cloud->width = points.size();

    for (size_t i = 0; i < points.size(); i++) {
        cloud->points.push_back(points[i]);
    }

    std::string file_name = req.file_path + "/" + req.file_name + ".pcd";
    pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2);

    pcl::toPCLPointCloud2(*cloud, *cloud2);
    pcl::io::savePCDFile(file_name, *cloud2);
    ROS_INFO("Saved Point Cloud");
    res.success = true;
    return true;
}

    bool PointcloudAccumulator::resetPointcloud(std_srvs::Trigger::Request  &req,
                                                std_srvs::Trigger::Response &res){

        kd_tree = new KD_TREE(0.3, 0.6, downsample_resolution);

        PointVector p;
        PointType pt;
        pt.x = 0; pt.y = 0; pt.z = 0;
        p.push_back(pt);
        kd_tree->Build(p);
        ROS_INFO("Reset Point Cloud");
        res.success = true;
        return true;
    }

}