#ifndef LIDARSAVE_HPP_
#define LIDARSAVE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <opencv2/opencv.hpp>

#define RAD2DEG(x) ((x)*180./M_PI)

class LIDARSAVED : public rclcpp::Node
{
public:
  LIDARSAVED();
  cv::VideoWriter writer;
private:
  void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_info_sub;
};

#endif // LIDARSAVE_HPP_