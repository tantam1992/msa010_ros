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
    nh_.param<std::string>("cliff_detector/frame_id", frame_id_, "dep_cam_laser_link");

    nh_.param("cliff_detector/cam_height", cam_height_, 0.0);
    nh_.param("cliff_detector/cam_angle", cam_angle_, 0.0);
    nh_.param("cliff_detector/cliff_threshold", cliff_threshold_, 0.0);
    nh_.param("cliff_detector/img_freq", img_freq_, 0.0);
    nh_.param("cliff_detector/range_min", range_min_, 0.0);
    nh_.param("cliff_detector/range_max", range_max_, 0.0);
    nh_.param("cliff_detector/cam_x", cam_x_, 0.0);
    nh_.param("cliff_detector/cam_y", cam_y_, 0.0);
    nh_.param("cliff_detector/cam_z", cam_z_, 0.0);

    nh_.param("cliff_detector/row_upper", row_upper_, 0.0);
    nh_.param("cliff_detector/col_left", col_left_, 0.0);
    nh_.param("cliff_detector/col_right", col_right_, 0.0);

    nh_.param("cliff_detector/skip_row_upper", skip_row_upper_, 0.0);
    nh_.param("cliff_detector/skip_row_bottom", skip_row_bottom_, 0.0);

    nh_.param("cliff_detector/tilt_compensation", tilt_compensation_, 0.0);
    tilt_compensation_ = tilt_compensation_ * M_PI / 180.0;

    kernel_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    trans_.setOrigin(tf::Vector3(cam_x_, cam_y_, cam_z_));          
    q_.setRPY(0.0, (cam_angle_ * M_PI / 180.0), 0.0);
    trans_.setRotation(q_);

    camera_info_received_ = false;

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


  void calcDeltaAngleForImgRows(double vertical_fov) 
  {
    // Angle between ray and optical center
    for(unsigned i = 0; i < img_height_; i++) 
    {
      delta_row_[i] = vertical_fov * (i - cy_ - 0.5) / (img_height_ - 1);
    }
  }


  std::vector<double> calcGroundDistancesForImgRows(double cam_height, double cam_angle) 
  {
    std::vector<double> dist_to_ground(img_height_, 0.0);
    double alpha = cam_angle * M_PI / 180.0; // Sensor tilt angle in radians

    // Calculations for each row of image
    for (unsigned i = 0; i < img_height_; i++) 
    {
      // Angle between ray and optical center
      if ((delta_row_[i] + alpha) > 0) 
      {
        dist_to_ground[i] = cam_height * sin(M_PI/2 - delta_row_[i]) / cos(M_PI/2 - delta_row_[i] - alpha);
      }
      else 
      {
        dist_to_ground[i] = 100.0;
      }
    }

    return dist_to_ground;
  }


  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) 
  {
    imu_quat_ = tf::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w); 
  }


  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
  {
    camera_model_.fromCameraInfo(msg);

    // For camera not rotate
    /*
    img_height_ = camera_model_.fullResolution().height;
    img_width_ = camera_model_.fullResolution().width;
    fx_ = camera_model_.fx();
    fy_ = camera_model_.fy();
    cx_ = camera_model_.cx();
    cy_ = camera_model_.cy();
    */

    // For camera rotate 90deg (clockwise)
    img_height_ = camera_model_.fullResolution().width;
    img_width_ = camera_model_.fullResolution().height;
    fx_ = camera_model_.fy();
    fy_ = camera_model_.fx();
    cx_ = camera_model_.fullResolution().height - camera_model_.cy();
    cy_ = camera_model_.cx();

    delta_row_.resize(img_height_);
    dist_to_ground_.resize(img_height_);
    dist_to_ground_init_.resize(img_height_);

    fieldOfView(v_angle_min_, v_angle_max_, cx_, 0, cx_, cy_, cx_, img_height_ -1);
    vertical_fov_ = v_angle_max_ - v_angle_min_;

    calcDeltaAngleForImgRows(vertical_fov_);

    dist_to_ground_init_ = calcGroundDistancesForImgRows(cam_height_, cam_angle_);

    fieldOfView(h_angle_min_, h_angle_max_, 0, cy_, cx_, cy_, img_width_ - 1, cy_);
    horizontal_fov_ = h_angle_max_ - h_angle_min_;

    scan_msg_.header.frame_id = frame_id_;
    scan_msg_.angle_min = h_angle_min_;
    scan_msg_.angle_max = h_angle_max_;
    scan_msg_.angle_increment = (scan_msg_.angle_max - scan_msg_.angle_min) / (img_width_ - 1);
    scan_msg_.time_increment = 0.0;
    scan_msg_.scan_time = 1 / img_freq_;
    scan_msg_.range_min = range_min_;
    scan_msg_.range_max = range_max_;

    camera_info_received_ = true;

    cam_info_sub_.shutdown();
  }


  void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
  {
    if (!camera_info_received_)
    {
      ROS_WARN("Camera calibration parameters not received yet.");
      return;
    }

    tf::Matrix3x3(imu_quat_).getRPY(roll_, pitch_, yaw_);

    if (abs(pitch_) > tilt_compensation_)
    {
      extra_height_ = cam_x_ * sin(pitch_);
      dist_to_ground_ = calcGroundDistancesForImgRows(cam_height_ - extra_height_, cam_angle_ + (pitch_ * 180 / M_PI));
    }
    else
    {
      dist_to_ground_ = dist_to_ground_init_;
    }

    try
    {
      cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat img = cv_ptr_->image;
    cv::rotate(img, img, cv::ROTATE_90_CLOCKWISE);

    cv::Mat img_cliff = cv::Mat::zeros(100, 100, CV_8UC1);
    cv::Mat img_edge = cv::Mat::zeros(100, 100, CV_8UC1);
    
    for (int col = col_left_; col < col_right_; col++)
    {
      for (int row = img_height_ - 1; row >= row_upper_; row--)
      {
        if (img.at<uchar>(row, col) == 255)
        {
          img_cliff.at<uchar>(row, col) = 127;
        }
        else
        {
          double dst = pow((img.at<uchar>(row, col) / 5.1), 2) / 1000;
          if (dst < (dist_to_ground_[row] + cliff_threshold_))
          {
            img_cliff.at<uchar>(row, col) = 255;
          }
          else
          {
            img_cliff.at<uchar>(row, col) = 0;
          }
        }
      }
    }

    cv::dilate(img_cliff, img_cliff, kernel_, cv::Point(-1, -1), 3);
    cv::erode(img_cliff, img_cliff, kernel_, cv::Point(-1, -1), 3);

    std::vector<double> p_dst(100, 0.0);

    for (int col = col_left_; col < col_right_; col++)
    {
      for (int row = img_height_ - 1 - skip_row_bottom_; row >= row_upper_ + skip_row_upper_; row--)
      {
        if (img_cliff.at<uchar>(row, col) == 255 && (img_cliff.at<uchar>(row - 1, col) == 0 || img_cliff.at<uchar>(row - 1, col) == 127))
        {
          img_edge.at<uchar>(row, col) = 255;

          double u = (col - cx_) / fx_;
          double v = (row - cy_) / fy_;
          double dst = pow((img.at<uchar>(row, col) / 5.1), 2) / 1000;

          double x = dst;
          double y = -dst * u;
          double z = -dst * v;

          p_.setOrigin(tf::Vector3(x, y, z));

          p_trans_ = trans_ * p_;

          p_dst[col] = sqrt(pow(p_trans_.getOrigin().x(), 2) + pow(p_trans_.getOrigin().y(), 2));

          break;
        }
      }
    }

    scan_msg_.header.stamp = msg->header.stamp;

    scan_msg_.ranges.resize(p_dst.size());
    for (int i = 0; i < p_dst.size(); i++) {
        scan_msg_.ranges[i] = p_dst[p_dst.size() - i - 1];
    }

    scan_pub_.publish(scan_msg_);

    img_cliff_msg = cv_bridge::CvImage(std_msgs::Header(), "8UC1", img_cliff).toImageMsg();
    img_cliff_msg->header = msg->header;
    seg_pub_.publish(img_cliff_msg);

    img_edge_msg = cv_bridge::CvImage(std_msgs::Header(), "8UC1", img_edge).toImageMsg();
    img_edge_msg->header = msg->header;
    cliff_pub_.publish(img_edge_msg);
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

  double img_height_, img_width_, fx_, fy_, cx_, cy_; 
  bool camera_info_received_;

  cv_bridge::CvImagePtr cv_ptr_;

  std::string img_topic_; 
  std::string cam_info_topic_;
  std::string frame_id_;
  double cam_height_, cam_angle_;
  double cliff_threshold_;
  double img_freq_, range_min_, range_max_;
  double cam_x_, cam_y_, cam_z_;
  double row_upper_, col_left_, col_right_;
  double skip_row_upper_, skip_row_bottom_;

  double tilt_compensation_;
  double extra_height_; 

  double v_angle_min_, v_angle_max_, h_angle_min_, h_angle_max_;
  double vertical_fov_, horizontal_fov_;

  std::vector<double> delta_row_;
  std::vector<double> dist_to_ground_, dist_to_ground_init_;

  cv::Mat kernel_;

  tf::Quaternion imu_quat_; 
  double roll_, pitch_, yaw_; 

  tf::Transform p_, trans_, p_trans_;
  tf::Quaternion q_;

  sensor_msgs::ImagePtr img_cliff_msg;
  sensor_msgs::ImagePtr img_edge_msg;

  sensor_msgs::LaserScan scan_msg_;
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