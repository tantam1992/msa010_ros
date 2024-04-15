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
    nh_.param<std::string>("point_cloud_processing_node/sub_topic", sub_topic, "depth/points");

    nh_.param("point_cloud_processing_node/LeafSize", LeafSize, 0.05);
    nh_.param("point_cloud_processing_node/MeanK", MeanK, 20);
    nh_.param("point_cloud_processing_node/StddevMulThresh", StddevMulThresh, 1.0);
    nh_.param("point_cloud_processing_node/RadiusSearch", RadiusSearch, 0.1);
    nh_.param("point_cloud_processing_node/MinNeighborsInRadius", MinNeighborsInRadius, 5);
    nh_.param("point_cloud_processing_node/MaxIterations", MaxIterations, 20);
    nh_.param("point_cloud_processing_node/DistanceThreshold", DistanceThreshold, 0.01);

    nh_.param<std::string>("point_cloud_processing_node/mode", mode, "noise");

    // Create parameter server
    server_cb_ = boost::bind(&PointCloudProcessor::reconfigureCallback, this, _1, _2);
    server_.setCallback(server_cb_);

    // Initialize filters
    voxel_.setLeafSize(LeafSize, LeafSize, LeafSize);
    sor_.setMeanK(MeanK);
    sor_.setStddevMulThresh(StddevMulThresh);
    ror_.setRadiusSearch(RadiusSearch);
    ror_.setMinNeighborsInRadius(MinNeighborsInRadius);

    seg_.setOptimizeCoefficients(true);
    seg_.setModelType(pcl::SACMODEL_PLANE);
    seg_.setMethodType(pcl::SAC_RANSAC);
    seg_.setMaxIterations(MaxIterations);
    seg_.setDistanceThreshold(DistanceThreshold); 

    // Initialize the point cloud object
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    if (mode == "noise")
    {
      cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(sub_topic, 1, &PointCloudProcessor::denoisingCallback, this);
      processed_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("processed_point_cloud", 1);
    }
    else if (mode == "ground") 
    {
      ground_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
      non_ground_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
      coefficients_.reset(new pcl::ModelCoefficients);
      inliers_.reset(new pcl::PointIndices);

      cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(sub_topic, 1, &PointCloudProcessor::removeGroundCallback, this);
      ground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("ground", 1);
      non_ground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("non_ground", 1);
    }

  }


  void reconfigureCallback(msa010_ros::PCLFilterConfig& config, uint32_t level) {
    // Update filter parameters based on dynamic reconfigure values
    voxel_.setLeafSize(config.LeafSize, config.LeafSize, config.LeafSize);
    sor_.setMeanK(config.MeanK);
    sor_.setStddevMulThresh(config.StddevMulThresh);
    ror_.setRadiusSearch(config.RadiusSearch);
    ror_.setMinNeighborsInRadius(config.MinNeighborsInRadius);

    seg_.setMaxIterations(config.MaxIterations);
    seg_.setDistanceThreshold(config.DistanceThreshold); 
  }


  void denoisingCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    // Convert PointCloud2 message to PCL PointCloud
    pcl::fromROSMsg(*msg, *cloud_);
    pcl::removeNaNFromPointCloud(*cloud_, *cloud_, indices_);

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


  void removeGroundCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    // Convert PointCloud2 message to PCL PointCloud
    pcl::fromROSMsg(*msg, *cloud_);
    pcl::removeNaNFromPointCloud(*cloud_, *cloud_, indices_);

    // Estimate the ground plane model using RANSAC
    seg_.setInputCloud(cloud_);
    seg_.segment(*inliers_, *coefficients_);    

    // Iterate over each point in the cloud and assign it to the ground or non-ground point cloud
    for (size_t i = 0; i < cloud_->size(); ++i) {
      if (std::find(inliers_->indices.begin(), inliers_->indices.end(), i) != inliers_->indices.end()) {
        // Ground point
        ground_cloud_->push_back(cloud_->at(i));
      } else {
        // Non-ground point
        non_ground_cloud_->push_back(cloud_->at(i));
      }
    }

    // Perform statistical outlier removal
    sor_.setInputCloud(cloud_);
    sor_.filter(*cloud_);
    
    // Perform radius outlier removal
    ror_.setInputCloud(non_ground_cloud_);
    ror_.filter(*non_ground_cloud_);

    // Convert filtered PointCloud back to PointCloud2 messages
    pcl::toROSMsg(*ground_cloud_, ground_msg_);
    ground_msg_.header = msg->header;

    pcl::toROSMsg(*non_ground_cloud_, non_ground_msg_);
    non_ground_msg_.header = msg->header;

    // Publish the ground and non-ground PointCloud2 messages
    ground_pub_.publish(ground_msg_);
    non_ground_pub_.publish(non_ground_msg_);

    ground_cloud_->clear();
    non_ground_cloud_->clear();
  }


private:
  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub_;
  ros::Publisher processed_pub_;

  ros::Publisher ground_pub_;
  ros::Publisher non_ground_pub_;

  double LeafSize;
  int MeanK;
  double StddevMulThresh;
  double RadiusSearch;
  int MinNeighborsInRadius;
  int MaxIterations;
  double DistanceThreshold;

  string sub_topic;
  string mode;

  std::vector<int> indices_;

  pcl::VoxelGrid<pcl::PointXYZ> voxel_;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_;
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror_;

  pcl::ModelCoefficients::Ptr coefficients_;
  pcl::PointIndices::Ptr inliers_;
  pcl::SACSegmentation<pcl::PointXYZ> seg_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud_;

  sensor_msgs::PointCloud2 filtered_msg_;
  sensor_msgs::PointCloud2 ground_msg_;
  sensor_msgs::PointCloud2 non_ground_msg_;
  
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