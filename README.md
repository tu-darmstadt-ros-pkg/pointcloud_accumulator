# Point Cloud Accumulator
Nodelet to build an ikd-Tree from incoming PointCloud messages to build a downsampled cloud of the environment.

To install just clone

```git clone git@git.sim.informatik.tu-darmstadt.de:hector/pointcloud_accumulator.git```

and build

```hector make pointcloud_accumulator```

After that, set the appropriate input cloud and the nodelet manager in launch/accumulator.launch. The static frame into which all points of the tree are transformed can be set via `static_frame`, the resolution of the tree, i.e. the length of one voxel, can be set via `downsample_resolution`.
Then launch using

```roslaunch pointcloud_accumulator accumulator.launch```
