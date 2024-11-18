#include <glog/logging.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "include/slam.h"
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  google::InitGoogleLogging(argv[0]);
  google::SetLogDestination(google::GLOG_INFO, "/root/gamma/log/log.txt");
  auto slam_ptr = std::make_shared<cartographer::slam::Slam>();
  rclcpp::spin(slam_ptr);
  rclcpp::shutdown();
  return 0;
}