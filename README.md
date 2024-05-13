# msa010_ros
This package contains the ROS driver of MaixSense-A010 Depth Camera, and the related post-processing tools.  
Fisrt, the depth image is read from the sensor through UART. Then, the depth image is converted into point cloud. After that, the point cloud is filtered to remove the noise. Finally, the [point cloud is converted to laser scan](http://wiki.ros.org/pointcloud_to_laserscan) and add to the costmap.  
## PCL Filters 
### Input Parameters 
* **sub_topic:** Input PointCloud2 topic. 
* **mode:** noise/ground, choose denoising mode or remove ground plane mode.
#### Voxel Grid Filter (Unused)
* **LeafSize:** The voxel grid leaf size.
####  Statistical Outlier Removal Filter
* **MeanK:** The number of points (k) to use for mean distance estimation​. 
* **StddevMulThresh:** The standard deviation multiplier threshold​.
#### Radius Outlier Removal Filter
* **RadiusSearch:** The sphere radius that is to be used for determining the k-nearest neighbors for filtering​.
* **MinNeighborsInRadius​:** The minimum number of neighbors that a point needs to have in the given search radius in order to be considered an inlier​.
#### SAC Segmentation​ (For Ground Plane Removal Only)
* **MaxIterations:** The maximum number of iterations before giving up​.
* **DistanceThreshold:** The distance to the model threshold​.
