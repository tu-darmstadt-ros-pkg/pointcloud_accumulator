#include <pointcloud_accumulator/pointcloud_accumulator.h>
#include <rviz/default_plugin/point_cloud_transformers.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>


namespace pointcloud_accumulator
{

PointcloudAccumulator::PointcloudAccumulator(const ros::NodeHandle& nh, const ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh), tfListener(tfBuffer){

    }

void PointcloudAccumulator::init(){

    kd_tree = new KD_TREE(0.3, 0.6, 0.15);
    sub = nh_.subscribe<sensor_msgs::PointCloud2>("cloud_in", 100, &PointcloudAccumulator::callback, this);
    pub = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("cloud_out", 100, false);

    //TODO Build initial tree with some more sophisticated method than some dummy point/empty point
    PointVector p;
    PointType pt;
    pt.x = 0; pt.y = 0; pt.z = 0;
    p.push_back(pt);
    kd_tree->Build(p);

    }

void PointcloudAccumulator::callback(const sensor_msgs::PointCloud2::ConstPtr& msg){

    //TODO Transformation into some static frame required because time information is lost
    if(!tfBuffer.canTransform("world", msg->header.frame_id, msg->header.stamp, ros::Duration(0.01))){
        ROS_INFO("Can't transform!");
        return;
    }
    sensor_msgs::PointCloud2Ptr cloud(new sensor_msgs::PointCloud2);
    pcl_ros::transformPointCloud("world", *msg, *cloud, tfBuffer);

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

    ROS_INFO("Added %ld Points to the ikd-tree", points.size());
    kd_tree->Add_Points(points, true);

    }

void PointcloudAccumulator::publish_pointcloud(){

    PointVector points;
    kd_tree->flatten(kd_tree->Root_Node, points);
    ROS_INFO("ikd-tree size %ld", points.size());

    sensor_msgs::PointCloud2Ptr msg(new sensor_msgs::PointCloud2);
    msg->header.stamp = ros::Time::now();
    msg->header.frame_id = "world";

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
}