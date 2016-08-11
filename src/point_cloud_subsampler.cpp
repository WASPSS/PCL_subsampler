#include "point_cloud_subsampler.h"

// initiate and declare global variables
ros::Publisher pub;
std::string pass_through_axis = "z";
double pass_through_min_limit = 0.0;
double pass_through_max_limit = 2.0;
double voxel_leaf_size = 0.1;
int sor_mean_k = 50;
double sor_stdev_thresh = 1.0;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input){

  // Updating parameters from the Parameter server
  ros::param::getCached("/PCL_cloud_subsampler/pass_through/filter_limit/min", pass_through_min_limit);
  ros::param::getCached("/PCL_cloud_subsampler/pass_through/filter_limit/max", pass_through_max_limit);
  ros::param::getCached("/PCL_cloud_subsampler/voxel_grid/leaf_size", voxel_leaf_size);
  ros::param::getCached("/PCL_cloud_subsampler/statistical_outlier_removal/mean_k", sor_mean_k);
  ros::param::getCached("/PCL_cloud_subsampler/statistical_outlier_removal/stddev_mult_thresh", sor_stdev_thresh);

  // Create a container for the output data.
  sensor_msgs::PointCloud2 output;

  // converting from sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

  pass_through(cloud, pass_through_axis, pass_through_min_limit, pass_through_max_limit);


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
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/camera/depth_registered/filtered_points", 1);

  // Spin
  ros::spin ();
}
