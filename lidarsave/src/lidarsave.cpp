#include "lidarsave/lidarsave.hpp"
#include <cmath>
#include <opencv2/opencv.hpp>

LIDARSAVED::LIDARSAVED() : Node("lidarsave_node")
{
  writer.open("lidarsave.mp4", cv::VideoWriter::fourcc('D','I','V','X'), 30, cv::Size(500, 500));
  if (!writer.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Could not open the output video file for write");
    return;
  }
  lidar_info_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(), std::bind(&LIDARSAVED::scanCb, this, std::placeholders::_1));
}

void LIDARSAVED::scanCb(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  int count = static_cast<int>(scan->scan_time / scan->time_increment);
  cv::Mat img(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
  RCLCPP_INFO(this->get_logger(), "I heard a laser scan %s[%d]", scan->header.frame_id.c_str(), count);
  RCLCPP_INFO(this->get_logger(), "angle_range : [%f, %f]", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

  for (int i = 0; i < count; i++)
  {
    float degree = scan->angle_min + scan->angle_increment * i;
    float distance = scan->ranges[i];
    if (distance >= scan->range_min && distance <= scan->range_max)
    {
      int x = 250 + static_cast<int>(distance * 50 * sin(degree));
      int y = 250 - static_cast<int>(distance * 50 * cos(degree));
      cv::circle(img, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), -1);
    }
  }
  writer.write(img);
  cv::imshow("Lidar save", img);
  cv::waitKey(10);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LIDARSAVED>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}