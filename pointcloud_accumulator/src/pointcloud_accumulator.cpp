#include <pointcloud_accumulator/pointcloud_accumulator.h>
#include <rviz/default_plugin/point_cloud_transformers.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace pointcloud_accumulator
{

PointcloudAccumulator::PointcloudAccumulator(const ros::NodeHandle& nh, const ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh), tfListener(tfBuffer){
    pnh.param<double>("downsample_resolution", downsample_resolution, 0.1);
    pnh.param<std::string>("static_frame", static_frame, "odom");
    pnh.param<bool>("use_cartographer_submaps", use_submaps, false);

    add_duration = 1.0;
    adaptive_incr = 2;
}

void PointcloudAccumulator::init(){

    sub = nh_.subscribe<sensor_msgs::PointCloud2>("cloud_in", 0, &PointcloudAccumulator::callback, this);
    if(use_submaps){
        submap_announcement_sub = nh_.subscribe<cartographer_ros_msgs::StampedSubmapEntry>("submap_announcement", 1, &PointcloudAccumulator::submap_announcements, this);
        submap_update_sub = nh_.subscribe<cartographer_ros_msgs::SubmapList>("submap_list", 1, &PointcloudAccumulator::submap_update, this);
    }

    pub = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("cloud_out", 1, false);

    save_map_service = pnh_.advertiseService("save_pointcloud", &PointcloudAccumulator::savePointcloud, this);
    reset_map_service = pnh_.advertiseService("reset_pointcloud", &PointcloudAccumulator::resetPointcloud, this);

    reset();
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

    adaptive_incr = calculate_adaptive_increment(add_duration, adaptive_incr);

    for(size_t i = 0; i < cloud->width*msg->height; i = i + adaptive_incr){
        PointType p;
        p.x = rviz::valueFromCloud<float>(cloud, xoff, type, point_step, i);
        p.y = rviz::valueFromCloud<float>(cloud, yoff, type, point_step, i);
        p.z = rviz::valueFromCloud<float>(cloud, zoff, type, point_step, i);

        if(rgbi == -1){
            p.r = 255;
            p.b = 255;
            p.g = 255;
        }else{
            p.rgb = rviz::valueFromCloud<float>(cloud, rgboff, type, point_step, i);
        }
        p.stamp = msg->header.stamp;

        points.push_back(p);
    }
    auto t1 = ros::Time::now().toSec();
    kd_tree->Add_Points(points, true);
    auto t2 = ros::Time::now().toSec();
    add_duration = t2 - t1;
    }

void PointcloudAccumulator::submap_announcements(const cartographer_ros_msgs::StampedSubmapEntry::ConstPtr& msg){

        submap_list.push_back(*msg);
    }

void PointcloudAccumulator::submap_update(const cartographer_ros_msgs::SubmapList::ConstPtr& msg){

        for(int i = 0; i < msg->submap.size(); i++){
            //Check whether pose of submap changed
            if(msg->submap[i].pose != submap_list[i].submap.pose){

                std::string submap = "submap_" + std::to_string(i);
                PointVector points;
                PointVector submap_points;
                kd_tree->flatten(kd_tree->Root_Node, points, NOT_RECORD);
                //Extract points belonging to submap_i
                for (PointVector::iterator it = points.begin() ; it != points.end(); ++it){
                    if(it->stamp > submap_list[i].header.stamp && (i == submap_list.size() - 1 || it->stamp < submap_list[i+1].header.stamp)){
                        submap_points.push_back(*it);
                    }
                }
                //Timestamp of old transform
                ros::Time old = ros::Time::now() - ros::Duration(3.0);
                if(tfBuffer.canTransform(submap, static_frame, old, ros::Duration(0.1))){

                    kd_tree->Delete_Points(submap_points);

                    //Shift points to the updated submap frame
                    for (PointVector::iterator it = submap_points.begin() ; it != submap_points.end() ; ++it){
                        geometry_msgs::PointStamped p;
                        p.point.x = it->x;
                        p.point.y = it->y;
                        p.point.z = it->z;
                        p.header.frame_id = static_frame;
                        tfBuffer.transform(p, p, submap, old, static_frame);
                        tfBuffer.transform(p, p, static_frame, ros::Time(0), static_frame);
                        it->x = p.point.x;
                        it->y = p.point.y;
                        it->z = p.point.z;
                    }
                    kd_tree->Add_Points(submap_points, true);
                    submap_list[i].submap = msg->submap[i];
                }
            }
        }
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
        res.success = reset();
        ROS_INFO("Reset Point Cloud");
        return res.success;
    }

    bool PointcloudAccumulator::reset(){
        kd_tree = new KD_TREE(0.3, 0.6, downsample_resolution);

        PointVector p;
        PointType pt;
        pt.x = 0; pt.y = 0; pt.z = 0;
        p.push_back(pt);
        kd_tree->Build(p);
        return true;
}

    int PointcloudAccumulator::calculate_adaptive_increment(double add_duration, int adaptive_param){
        int incr = 0.5 * adaptive_param * (add_duration + 1);
        if (incr >= 1){
            return incr;
        } else{
            return 1;
        }
    }

}