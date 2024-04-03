#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;


class laserScanFilter
{
public:
  laserScanFilter(ros::NodeHandle nh)
    : nh_(nh)
  {
    nh_.param<std::string>("laser_scan_filter_node/scan_input", scan_input_, "scan");
    nh_.param<std::string>("laser_scan_filter_node/scan_output", scan_output_, "scan_cutoff");
    nh_.param("laser_scan_filter_node/range_min", range_min_, 0.2);
    nh_.param("laser_scan_filter_node/range_max", range_max_, 10.0);

    init_msg_ = false;

    scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(scan_input_, 1, &laserScanFilter::scanCallback, this);
    scan_filtered_pub_ = nh_.advertise<sensor_msgs::LaserScan>(scan_output_, 1);
  }


  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    if (!init_msg_) 
    {
      filtered_scan_.angle_min = msg->angle_min;
      filtered_scan_.angle_max = msg->angle_max;
      filtered_scan_.angle_increment = msg->angle_increment;
      filtered_scan_.time_increment = msg->time_increment;
      filtered_scan_.scan_time = msg->scan_time;
      filtered_scan_.range_min = range_min_;
      filtered_scan_.range_max = range_max_;
      init_msg_ = true;
    }
    
    filtered_scan_.header = msg->header;
    filtered_scan_.ranges = msg->ranges;

    for (unsigned int i=0; i < filtered_scan_.ranges.size(); i++) 
    {
      if (filtered_scan_.ranges[i] <= range_min_)
      {
        filtered_scan_.ranges[i] = -std::numeric_limits<float>::infinity();

      }
      else if (filtered_scan_.ranges[i] >= range_max_)
      {
        filtered_scan_.ranges[i] = std::numeric_limits<float>::infinity();
      }
    }

    scan_filtered_pub_.publish(filtered_scan_);
  }


private:
  ros::NodeHandle nh_;
  ros::Subscriber scan_sub_;
  ros::Publisher scan_filtered_pub_;

  string scan_input_;
  string scan_output_;
  double range_min_;
  double range_max_;

  sensor_msgs::LaserScan filtered_scan_;

  bool init_msg_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_scan_filter");
  ros::NodeHandle nh;

  laserScanFilter laser_scan_filter(nh);

  ROS_INFO("Laser Scan Filter Start.");

  ros::spin();

  return 0;
}