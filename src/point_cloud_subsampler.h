#include <ros/ros.h>
#include <ros/console.h>
#include <signal.h>
#include <iostream>
// PCL specific includes

// Include PointCloud message that we subscribe to
#include <sensor_msgs/PointCloud2.h>
// Include pass through filter
#include <pcl/filters/passthrough.h>
// Include voxel grid filter
#include <pcl/filters/voxel_grid.h>
// Include SOR filter
#include <pcl/filters/statistical_outlier_removal.h>
// Include library for convering from message to PCL::PointCloud
#include <pcl_conversions/pcl_conversions.h>


/* Truncating pointcloud on the interval [min, max] across the specified axis
Arguments:
cloud       cloud to filter
axis        axis to filter across
min         lower limit
max         upper limit*/
void pass_through(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string axis,
                                                        double min, double max){
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName(axis);
  pass.setFilterLimits(min, max);
  pass.filter(*cloud);
}
