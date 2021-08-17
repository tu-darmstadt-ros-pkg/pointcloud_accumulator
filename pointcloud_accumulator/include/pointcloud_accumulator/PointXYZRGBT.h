#ifndef POINTCLOUD_ACCUMULATOR_POINTXYZRGBT_H
#define POINTCLOUD_ACCUMULATOR_POINTXYZRGBT_H

#include <pcl_ros/point_cloud.h>


class PointXYZRGBT : public pcl::PointXYZRGB{
public:
    ros::Time stamp;
};


#endif //POINTCLOUD_ACCUMULATOR_POINTXYZRGBT_H
