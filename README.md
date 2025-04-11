> [!NOTE]
> This repository is no longer maintained. A ROS2 alternative can be found here: [tu-darmstadt-ros-pkg/hector_pointcloud_tools](https://github.com/tu-darmstadt-ros-pkg/hector_pointcloud_tools)

# Point Cloud Accumulator
## Overview
ROS Nodelet to build an [ikd-Tree](https://github.com/hku-mars/ikd-Tree) from incoming PointCloud messages to build a downsampled cloud of the environment.

## Install
To install clone the repository in a [ROS workspace](http://wiki.ros.org/catkin/workspaces)
```
git clone https://github.com/tu-darmstadt-ros-pkg/pointcloud_accumulator.git
```
and build
```
catkin build pointcloud_accumulator
```

## Parameters
In `pointcloud_accumulator/launch/accumulator.launch` several parameters can be set:
* **`cloud_in`** The input point cloud topic.
* **`cloud_out`** The topic where the downsampled cloud gets published.
* **`nodelet_manager`** Nodelet manager for the point cloud topic.
* **`resolution`** The resolution with which the tree gets downsampled, i.e. the side length of one voxel.
* **`frame`** The *static* tf frame into which all points get transformed.
* **`update_rate`** The rate (in Hz) with which the cloud gets published.

## Services
* **`save_pointcloud`** Saves the current state of the point cloud as a `.pcd`-file. Call the service with 
```
rosservice call /pointcloud_accumulator_node/save_pointcloud "<file_path>" "<file_name>"
```
For example
```
rosservice call /pointcloud_accumulator_node/save_pointcloud "$(rospack find pointcloud_accumulator)/saved_clouds" "pc1"
```

* **`reset_pointcloud`** Resets the accumulated point cloud. Call the service with 
```
rosservice call /pointcloud_accumulator_node/reset_pointcloud
```

## Launch
Launch the nodelet using

```roslaunch pointcloud_accumulator node.launch```

## Acknowledgement

The project is based on [ikd-Tree](https://github.com/hku-mars/ikd-Tree), see the corrsponding paper for details: *Cai, Yixi, Wei Xu, and Fu Zhang. "ikd-tree: An incremental kd tree for robotic applications." arXiv preprint arXiv:2102.10808 (2021).*
