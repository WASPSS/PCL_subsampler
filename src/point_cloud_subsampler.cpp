/*
* pass_through
* voxel_grid
* statistical_outlier_removal
*/

#include <ros/ros.h>
#include <ros/console.h>
#include <signal.h>
#include <iostream>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree.h>

ros::Publisher pub;
std::string pass_through_axis = "z";
double pass_through_min_limit = 0.0;
double pass_through_max_limit = 2.0;
double voxel_leaf_size = 0.1;
int sor_mean_k = 50;
double sor_stdev_thresh = 1.0;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

  ros::param::getCached("/PCL_cloud_subsampler/pass_through/filter_limit/min", pass_through_min_limit);
  ros::param::getCached("/PCL_cloud_subsampler/pass_through/filter_limit/max", pass_through_max_limit);
  ros::param::getCached("/PCL_cloud_subsampler/voxel_grid/leaf_size", voxel_leaf_size);
  ros::param::getCached("/PCL_cloud_subsampler/statistical_outlier_removal/mean_k", sor_mean_k);
  ros::param::getCached("/PCL_cloud_subsampler/statistical_outlier_removal/stddev_mult_thresh", sor_stdev_thresh);


  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);

  if(pcl_pc2.data.size()!=0){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 output;
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

    // 1. Including PassThrough filter here. Removes areas that we are not interested in
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName(pass_through_axis);
    pass.setFilterLimits(pass_through_min_limit, pass_through_max_limit);
    pass.filter(*cloud);


    // 2. Downsampling with VoxelGrid
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud(cloud);
    vox.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    vox.filter(*cloud);

    // 3. Statistical outlier removal
    pcl::StatisticalOutlierRemoval <pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK (sor_mean_k);
    sor.setStddevMulThresh(sor_stdev_thresh);
    sor.filter(*cloud);

    pcl::toROSMsg (*cloud, output);

    // Publish the data.
    pub.publish (output);
  }
}


  // ctrl-c handler in order to save parameters in PCL_shape_classifier_params.yaml
  void mySigintHandler(int sig){
    // Dump all parameters in objecd_detection_params.yaml
    ros::param::set("/PCL_cloud_subsampler/pass_through/filter_limit/min", pass_through_min_limit);
    ros::param::set("/PCL_cloud_subsampler/pass_through/filter_limit/max", pass_through_max_limit);
    ros::param::set("/PCL_cloud_subsampler/voxel_grid/leaf_size", voxel_leaf_size);
    ros::param::set("/PCL_cloud_subsampler/statistical_outlier_removal/mean_k", sor_mean_k);
    ros::param::set("/PCL_cloud_subsampler/statistical_outlier_removal/stddev_mult_thresh", sor_stdev_thresh);
    system("rosparam dump -v ~/catkin_ws/src/turtlebot_point_cloud_subsampler/paramerters/PCL_cloud_subsampler.yaml /PCL_cloud_subsampler");
    ros::shutdown();
  }

  int main (int argc, char** argv)
  {
    // Initialize ROS
    system("rosparam load ~/catkin_ws/src/turtlebot_point_cloud_subsampler/paramerters/PCL_cloud_subsampler.yaml /PCL_cloud_subsampler");
    ros::init (argc, argv, "point_cloud_subsampler", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    // In case of ctrl-c handle that with mySigintHandler
    signal(SIGINT, mySigintHandler);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/camera/depth/filtered_points", 1);

    // Spin
    ros::spin ();
  }
