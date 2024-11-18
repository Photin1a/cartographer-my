#ifndef CARTOGRAPHER_SLAM_H_
#define CARTOGRAPHER_SLAM_H_
#include "cartographer_msgs/srv/command.hpp"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "cartographer/mapping/internal/2d/global_trajectory_builder_2d.h"

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "cartographer/transform/transform.h"

#include <tf2_ros/transform_broadcaster.h>

// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>

namespace cartographer {
namespace slam {

constexpr char kMapsDirectory[] = "/root/gamma/maps/";
class Slam:public rclcpp::Node {
  using Command = cartographer_msgs::srv::Command;
  using SensorId = cartographer::mapping::SensorId;
  using SensorType = SensorId::SensorType;
  using InsertionResult = cartographer::mapping::InsertionResult;

 public:
  explicit Slam();
  ~Slam();
  Slam(const Slam&) = delete;
  Slam& operator=(const Slam&) = delete;
  void SetCommandCallBack(const Command::Request::SharedPtr req, Command::Response::SharedPtr res);
  void PublishNodes();
  void PublishSubmaps();

  double m_resolution = 0.5;
  void GetSubmaps();
  std::unique_ptr<nav_msgs::msg::OccupancyGrid>  PublishGridmap();
  std::unique_ptr<nav_msgs::msg::OccupancyGrid> CreateOccupancyGridMsg(const cartographer::io::PaintSubmapSlicesResult &painted_slices, const double resolution, const std::string &frame_id, const rclcpp::Time &time);

  void ImuCallback(sensor_msgs::msg::Imu::ConstPtr imu_data);
  void OdomCallback(nav_msgs::msg::Odometry::ConstPtr odom_data);
  void ScanCallback(sensor_msgs::msg::LaserScan::ConstPtr scan_data);

  void LocalSlamResultCallback(int tra_id, common::Time time, transform::Rigid3d local_pose,sensor::RangeData range_data_in_local);

 private:
  std::unique_ptr<mapping::MapBuilder> map_builder_ptr_;
  cartographer::mapping::GlobalTrajectoryBuilder2D *trajectory_builder_;
  int traj_id_;

  std::map<cartographer::mapping::SubmapId, cartographer::io::SubmapSlice> m_submap_slices;
  std::string m_last_frame_id;
  rclcpp::Time m_last_timestamp;

  // 目前这个变换时写死的
  cartographer::transform::Rigid3d scan_to_tracking_;
  
  // std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::set<cartographer::mapping::SensorId> sensor_ids_;

  rclcpp::Service<cartographer_msgs::srv::Command>::SharedPtr command_srv_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr submaps_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr submap_poses_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr node_poses_publisher_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

};
}  // namespace slam
}  // namespace cartographer
#endif