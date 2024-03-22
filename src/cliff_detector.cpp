#include <ros/ros.h>
#include <tf/tf.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>

#include <math.h>

using namespace std;


class cliffDetector
{
public:
  cliffDetector(ros::NodeHandle nh)
    : nh_(nh)
  {
    nh_.param<std::string>("cliff_detector/img_topic", img_topic_, "depth/image_raw");
    nh_.param<std::string>("cliff_detector/cam_info_topic", cam_info_topic_, "depth/camera_info");

    nh_.param("cliff_detector/cam_height", cam_height_, 0.0);
    nh_.param("cliff_detector/cam_angle", cam_angle_, 0.0);

    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/imu/data", 1, &cliffDetector::imuCallback, this);
    cam_info_sub_ = nh_.subscribe<sensor_msgs::CameraInfo>(cam_info_topic_, 1, &cliffDetector::cameraInfoCallback, this);
    dep_img_sub_ = nh_.subscribe<sensor_msgs::Image>(img_topic_, 1, &cliffDetector::depthImageCallback, this);
    
    seg_pub_ = nh_.advertise<sensor_msgs::Image>("depth/seg", 1);
    cliff_pub_ = nh_.advertise<sensor_msgs::Image>("depth/cliff", 1);
    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("depth/scan", 1);
  }


  double lengthOfVector(const cv::Point3d& vec) const 
  {
    return sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
  }


  double angleBetweenRays(const cv::Point3d& ray1, const cv::Point3d& ray2) const 
  {
    double dot = ray1.x*ray2.x + ray1.y*ray2.y + ray1.z*ray2.z;
    return acos(dot / (lengthOfVector(ray1) * lengthOfVector(ray2)));
  }


  void fieldOfView(double & min, double & max, double x1, double y1,
                   double xc, double yc, double x2, double y2) 
  {
    cv::Point2d raw_pixel_left(x1, y1);
    cv::Point2d rect_pixel_left = camera_model_.rectifyPoint(raw_pixel_left);
    cv::Point3d left_ray = camera_model_.projectPixelTo3dRay(rect_pixel_left);

    cv::Point2d raw_pixel_right(x2, y2);
    cv::Point2d rect_pixel_right = camera_model_.rectifyPoint(raw_pixel_right);
    cv::Point3d right_ray = camera_model_.projectPixelTo3dRay(rect_pixel_right);

    cv::Point2d raw_pixel_center(xc, yc);
    cv::Point2d rect_pixel_center = camera_model_.rectifyPoint(raw_pixel_center);
    cv::Point3d center_ray = camera_model_.projectPixelTo3dRay(rect_pixel_center);

    min = -angleBetweenRays(center_ray, right_ray);
    max = angleBetweenRays(left_ray, center_ray);
  }


  // TODO: vertical_fov
  void calcDeltaAngleForImgRows(double vertical_fov) 
  {
    const unsigned img_height = camera_model_.fullResolution().height;

    delta_row_.resize(img_height);

    // Angle between ray and optical center
    for(unsigned i = 0; i < img_height; i++) {
      delta_row_[i] = vertical_fov * (i - camera_model_.cy() - 0.5) / (img_height - 1);
    }
  }


  void calcGroundDistancesForImgRows() 
  {
    const unsigned img_height = camera_model_.fullResolution().height;
    const double alpha = cam_angle_ * M_PI / 180.0; // Sensor tilt angle in radians

    dist_to_ground_.resize(img_height);

    // Calculations for each row of image
    for (unsigned i = 0; i < img_height; i++) {
      // Angle between ray and optical center
      if ((delta_row_[i] + alpha) > 0) {
        dist_to_ground_[i] = cam_height_ * sin(M_PI/2 - delta_row_[i])
            / cos(M_PI/2 - delta_row_[i] - alpha);
      }
      else {
        dist_to_ground_[i] = 100.0;
      }
    }
  }


  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) 
  {
    imu_quat_ = tf::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w); 
  }


  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
  {
    camera_model_.fromCameraInfo(msg);

    cam_info_sub_.shutdown();
  }


  void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
  {
    tf::Matrix3x3(imu_quat_).getRPY(roll_, pitch_, yaw_);
  }


private:
  ros::NodeHandle nh_;

  ros::Subscriber imu_sub_;
  ros::Subscriber cam_info_sub_;
  ros::Subscriber dep_img_sub_;

  ros::Publisher seg_pub_;
  ros::Publisher cliff_pub_;
  ros::Publisher scan_pub_;

  image_geometry::PinholeCameraModel camera_model_;

  std::string img_topic_; 
  std::string cam_info_topic_;

  double cam_height_;
  double cam_angle_;

  std::vector<double> delta_row_;
  std::vector<double> dist_to_ground_;

  tf::Quaternion imu_quat_; 
  double roll_, pitch_, yaw_; 
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cliff_detector");
  ros::NodeHandle nh;

  cliffDetector cliff_detector(nh);

  ROS_INFO("Cliff Detector Start.");

  ros::spin();

  return 0;
}