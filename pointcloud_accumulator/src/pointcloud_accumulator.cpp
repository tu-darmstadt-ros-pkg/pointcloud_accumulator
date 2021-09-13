#include <pointcloud_accumulator/pointcloud_accumulator.h>
#include <rviz/default_plugin/point_cloud_transformers.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>

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
        float x = rviz::valueFromCloud<float>(cloud, xoff, type, point_step, i);

        if(!std::isnan(x) && x != 0.0) {
            p.x = x;
            p.y = rviz::valueFromCloud<float>(cloud, yoff, type, point_step, i);
            p.z = rviz::valueFromCloud<float>(cloud, zoff, type, point_step, i);

            if (rgbi == -1) {
                p.r = 255;
                p.b = 255;
                p.g = 255;
            } else {
                p.rgb = rviz::valueFromCloud<float>(cloud, rgboff, type, point_step, i);
            }
            p.stamp = msg->header.stamp;

            points.push_back(p);
        }
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

        PointVector points;
        kd_tree->flatten(kd_tree->Root_Node, points, NOT_RECORD);

        for(size_t submap_index = 0; submap_index < msg->submap.size(); submap_index++){
            int internal_index = -1;
            for(size_t j = 0; j < submap_list.size(); j++){
                if(submap_list[j].submap.submap_index == submap_index){
                    internal_index = j;
                    break;
                }
            }
            if(internal_index == -1){
                continue;
            }

            geometry_msgs::Pose old_submap = submap_list[internal_index].submap.pose;
            geometry_msgs::Pose new_submap = msg->submap[submap_index].pose;

            //Check whether pose of submap changed substantially
            if(sqrt(pow(old_submap.position.x - new_submap.position.x, 2)
                    + pow(old_submap.position.y - new_submap.position.y, 2)
                    + pow(old_submap.position.z - new_submap.position.z, 2)
                    + pow(old_submap.orientation.x - new_submap.orientation.x, 2)
                    + pow(old_submap.orientation.y - new_submap.orientation.y, 2)
                    + pow(old_submap.orientation.z - new_submap.orientation.z, 2)
                    + pow(old_submap.orientation.w - new_submap.orientation.w, 2)) > 0.1){

                std::string submap = "submap_" + std::to_string(submap_index);

                //Create transform between world, old submap pose and new pose
                //Transform world -> old pose
                tf2::Transform world_old;
                geometry_msgs::TransformStamped t_world_old;
                tf2::fromMsg(old_submap, world_old);
                t_world_old.transform = tf2::toMsg(world_old);

                //Transform new pose -> world
                tf2::Transform world_new;
                geometry_msgs::TransformStamped t_new_world;
                tf2::fromMsg(new_submap, world_new);
                t_new_world.transform = tf2::toMsg(world_new.inverse());

                PointVector submap_points;

                //Extract points belonging to submap_i
                for (PointVector::iterator it = points.begin() ; it != points.end(); ++it){
                    if(it->stamp > submap_list[internal_index].header.stamp &&
                            (internal_index == submap_list.size() - 1 || it->stamp < submap_list[internal_index+1].header.stamp)){
                        submap_points.push_back(*it);
                    }
                }

                kd_tree->Delete_Points(submap_points);

                //Shift points to the updated submap frame
                for (PointVector::iterator it = submap_points.begin() ; it != submap_points.end() ; ++it){
                    geometry_msgs::Point p;
                    p.x = it->x;
                    p.y = it->y;
                    p.z = it->z;

                    tf2::doTransform(p, p, t_world_old);
                    tf2::doTransform(p, p, t_new_world);

                    it->x = p.x;
                    it->y = p.y;
                    it->z = p.z;
                }

                kd_tree->Add_Points(submap_points, true);
                submap_list[internal_index].submap = msg->submap[submap_index];
            }
        }
    }


void PointcloudAccumulator::publish_pointcloud(){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    PointVector points;
    kd_tree->flatten(kd_tree->Root_Node, points, NOT_RECORD);

    constructPclCloud(points, cloud);
    pub.publish(cloud);
}

void PointcloudAccumulator::constructPclCloud(PointVector points, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud){

    sensor_msgs::PointCloud2Ptr msg(new sensor_msgs::PointCloud2);
    msg->header.stamp = ros::Time::now();
    msg->header.frame_id = static_frame;

    cloud->width = points.size();
    cloud->height = 1;
    pcl_conversions::toPCL(msg->header, cloud->header);
    cloud->is_dense = false;
    for (size_t i = 0; i < points.size(); i++) {
        cloud->points.push_back(points[i]);
    }
}

bool PointcloudAccumulator::savePointcloud(pointcloud_accumulator_msgs::SavePointCloud::Request  &req,
                                               pointcloud_accumulator_msgs::SavePointCloud::Response &res){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    PointVector points;
    kd_tree->flatten(kd_tree->Root_Node, points, NOT_RECORD);
    constructPclCloud(points, cloud);

    std::string file_name = req.file_path + "/" + req.file_name + ".pcd";
    pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2);

    pcl::toPCLPointCloud2(*cloud, *cloud2);
    pcl::io::savePCDFile(file_name, *cloud2);
    ROS_INFO("Saved Point Cloud");
    res.success = true;
    return res.success;
}

bool PointcloudAccumulator::resetPointcloud(std_srvs::Trigger::Request  &req,
                                                std_srvs::Trigger::Response &res){
    res.success = reset();
    ROS_INFO("Reset Point Cloud");
    return res.success;
}

bool PointcloudAccumulator::reset(){
    kd_tree = new KD_TREE(0.01, 0.52, downsample_resolution);

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