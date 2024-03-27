#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <msa010_ros/PCLFilterConfig.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <sensor_msgs/PointCloud2.h>

#include <string>

using namespace std;

class PointCloudProcessor
{
public:
  PointCloudProcessor(ros::NodeHandle nh)
    : nh_(nh)
  {
    nh_.getParam("point_cloud_processing_node/sub_topic", sub_topic);
    nh_.getParam("point_cloud_processing_node/LeafSize", LeafSize);
    nh_.getParam("point_cloud_processing_node/MeanK", MeanK);
    nh_.getParam("point_cloud_processing_node/StddevMulThresh", StddevMulThresh);
    nh_.getParam("point_cloud_processing_node/RadiusSearch", RadiusSearch);
    nh_.getParam("point_cloud_processing_node/MinNeighborsInRadius", MinNeighborsInRadius);

    // Create a subscriber for the PointCloud2 message
    cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(sub_topic, 1, &PointCloudProcessor::pointCloudCallback, this);

    // Create a publisher for the processed PointCloud2 message
    processed_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("processed_point_cloud", 1);

    // ground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("ground", 1);
    // non_ground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("non_ground", 1);

    // Initialize filters
    voxel_.setLeafSize(LeafSize, LeafSize, LeafSize);
    sor_.setMeanK(MeanK);
    sor_.setStddevMulThresh(StddevMulThresh);
    ror_.setRadiusSearch(RadiusSearch);
    ror_.setMinNeighborsInRadius(MinNeighborsInRadius);

    server_cb_ = boost::bind(&PointCloudProcessor::reconfigureCallback, this, _1, _2);
    server_.setCallback(server_cb_);

    // Initialize the point cloud object
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  }


  void reconfigureCallback(msa010_ros::PCLFilterConfig& config, uint32_t level) {
    // Update filter parameters based on dynamic reconfigure values
    voxel_.setLeafSize(config.LeafSize, config.LeafSize, config.LeafSize);
    sor_.setMeanK(config.MeanK);
    sor_.setStddevMulThresh(config.StddevMulThresh);
    ror_.setRadiusSearch(config.RadiusSearch);
    ror_.setMinNeighborsInRadius(config.MinNeighborsInRadius);
  }


  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    // Convert PointCloud2 message to PCL PointCloud
    pcl::fromROSMsg(*msg, *cloud_);
    pcl::removeNaNFromPointCloud(*cloud_, *cloud_, indices_);

    /*
    {
    // Estimate the ground plane model using RANSAC
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(50);
    seg.setDistanceThreshold(0.01); // Set the distance threshold according to your requirements
    seg.setInputCloud(cloud_);
    seg.segment(*inliers, *coefficients);

    // Create separate point clouds to store the ground and non-ground points
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Iterate over each point in the cloud and assign it to the ground or non-ground point cloud
    for (size_t i = 0; i < cloud_->size(); ++i) {
      if (std::find(inliers->indices.begin(), inliers->indices.end(), i) != inliers->indices.end()) {
        // Ground point
        ground_cloud->push_back(cloud_->at(i));
      } else {
        // Non-ground point
        non_ground_cloud->push_back(cloud_->at(i));
      }
    }

    // Convert filtered PointCloud back to PointCloud2 messages
    sensor_msgs::PointCloud2 ground_msg;
    pcl::toROSMsg(*ground_cloud, ground_msg);
    ground_msg.header = msg->header;

    sensor_msgs::PointCloud2 non_ground_msg;
    pcl::toROSMsg(*non_ground_cloud, non_ground_msg);
    non_ground_msg.header = msg->header;

    // Publish the ground and non-ground PointCloud2 messages
    ground_pub_.publish(ground_msg);
    non_ground_pub_.publish(non_ground_msg);
    }
    */

    /*
    // Perform voxel grid downsampling
    voxel_.setInputCloud(cloud_);
    voxel_.filter(*cloud_);
    */
    
    // Perform statistical outlier removal
    sor_.setInputCloud(cloud_);
    sor_.filter(*cloud_);

    // Perform radius outlier removal
    ror_.setInputCloud(cloud_);
    ror_.filter(*cloud_);

    // Convert filtered PointCloud back to PointCloud2 message
    pcl::toROSMsg(*cloud_, filtered_msg_);
    filtered_msg_.header = msg->header;

    // Publish the processed PointCloud2 message
    processed_pub_.publish(filtered_msg_);
  }


private:
  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub_;
  ros::Publisher processed_pub_;

  // ros::Publisher ground_pub_;
  // ros::Publisher non_ground_pub_;

  float LeafSize;
  int MeanK;
  double StddevMulThresh;
  double RadiusSearch;
  int MinNeighborsInRadius;
  string sub_topic;

  std::vector<int> indices_;

  pcl::VoxelGrid<pcl::PointXYZ> voxel_;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_;
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

  sensor_msgs::PointCloud2 filtered_msg_;

  dynamic_reconfigure::Server<msa010_ros::PCLFilterConfig> server_;
  dynamic_reconfigure::Server<msa010_ros::PCLFilterConfig>::CallbackType server_cb_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_cloud_processing_node");
  ros::NodeHandle nh;

  PointCloudProcessor processor(nh);

  ROS_INFO("PCL Filtering Node Started.");

  ros::spin();

  return 0;
}