#include "include/slam.h"
#include "cartographer/io/submap_painter.h"

namespace cartographer {
namespace slam {
Slam::Slam():rclcpp::Node("cartographer_node") {
  // tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  Eigen::Vector3d trans;  
  trans << 0,0,0;
  auto rot = Eigen::Quaterniond(0, 0, 0, 1);
  scan_to_tracking_ = cartographer::transform::Rigid3d(trans,rot); // tracking = scan

  map_builder_ptr_ = std::make_unique<mapping::MapBuilder>();
  command_srv_ = this->create_service<Command>("command", std::bind(&Slam::SetCommandCallBack,this,std::placeholders::_1,std::placeholders::_2));
  submaps_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", 100);
  submap_poses_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("submap_poses",100);
  node_poses_publisher_ = create_publisher<nav_msgs::msg::Path>("node_poses", 100);

  laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan",1,std::bind(&Slam::ScanCallback,this,std::placeholders::_1));
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu",1,std::bind(&Slam::ImuCallback,this,std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom",1,std::bind(&Slam::OdomCallback,this,std::placeholders::_1));

  sensor_ids_.insert(SensorId{SensorType::RANGE, "echoes"});// laser 2d
  sensor_ids_.insert(SensorId{SensorType::IMU, "imu"});// IMU
  sensor_ids_.insert(SensorId{SensorType::ODOMETRY, "odom"});// IMU

  traj_id_ = -1;
  map_builder_ptr_->AddTrajectoryBuilder(sensor_ids_,[this](int tra_id,const common::Time time,transform::Rigid3d local_pose,
                         sensor::RangeData range_data_in_local,std::unique_ptr<const InsertionResult> res){
    this->LocalSlamResultCallback(tra_id, time, local_pose,range_data_in_local);
    traj_id_ = tra_id;
  });

  if(traj_id_!=-1){
    trajectory_builder_ = map_builder_ptr_->GetTrajectoryBuilder(traj_id_);
  }
};

/**
 * @brief  回调函数
 * @param id                   轨迹id
 * @param time                 时间
 * @param local_pose           局部地图上的位置
 * @param range_data_in_local  
 */
void Slam::LocalSlamResultCallback(int tra_id, common::Time time, transform::Rigid3d local_pose,sensor::RangeData range_data_in_local){
    cartographer::transform::Rigid3d local2global = map_builder_ptr_->pose_graph()->GetLocalToGlobalTransform(tra_id);
    cartographer::transform::Rigid3d pose3d = local2global * local_pose;
    std::cout<<"pose: x"<<pose3d.translation().x()<<" y "<<pose3d.translation().y()<<" angle "<<pose3d.rotation().z()<<std::endl;
}

// ros::Time ToRos(::cartographer::common::Time time) {
//   int64_t uts_timestamp = ::cartographer::common::ToUniversal(time);
//   int64_t ns_since_unix_epoch =
//       (uts_timestamp -
//        ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds *
//            10000000ll) *
//       100ll;
//   ::ros::Time ros_time;
//   ros_time.fromNSec(ns_since_unix_epoch);
//   return ros_time;
// }

cartographer::common::Time FromRos(const rclcpp::Time& time) {
  // The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0 +0000",
  // exactly 719162 days before the Unix epoch.
  return ::cartographer::common::FromUniversal(
      (time.seconds() +
       cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds) *
          10000000ll +
      (time.nanoseconds() + 50) / 100);  // + 50 to get the rounding correct.
}
void Slam::ImuCallback(sensor_msgs::msg::Imu::ConstPtr imu_data){
  if(trajectory_builder_ == nullptr) return;
  sensor::ImuData carto_imu;
  carto_imu.time = FromRos(imu_data->header.stamp);
  carto_imu.linear_acceleration << imu_data->linear_acceleration.x,imu_data->linear_acceleration.y,imu_data->linear_acceleration.z;
  carto_imu.angular_velocity << imu_data->angular_velocity.x,imu_data->angular_velocity.y,imu_data->angular_velocity.z;
  trajectory_builder_->AddSensorData("imu",carto_imu);
}

// TODO
void Slam::OdomCallback(nav_msgs::msg::Odometry::ConstPtr odom_data){
  if(trajectory_builder_ == nullptr) return;
  sensor::OdometryData carto_odom;
  carto_odom.time = FromRos(odom_data->header.stamp);
  // carto_odom

}

void Slam::ScanCallback(sensor_msgs::msg::LaserScan::ConstPtr scan_data){
  cartographer::sensor::PointCloudWithIntensities point_cloud;
  auto time = FromRos(scan_data->header.stamp);
  float angle = scan_data->angle_min;
  // 把所有的激光点打上相对时间戳（相对于当前帧的第一个点）
  for (size_t i = 0; i < scan_data->ranges.size(); ++i){
    const auto& echo = scan_data->ranges[i];
    if(scan_data->angle_min <= echo && echo <= scan_data->angle_max){
      const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
      cartographer::sensor::TimedRangefinderPoint point;
      point.position = rotation * (echo * Eigen::Vector3f::UnitX());
      point.time = i * scan_data->time_increment;
      point_cloud.points.push_back(point);
      if (!scan_data->intensities.empty()){
          auto echo_intensitie = scan_data->intensities[i];
          point_cloud.intensities.push_back(echo_intensitie);
      }
      else{
          point_cloud.intensities.push_back(0.f);
      }
    }
    angle += scan_data->angle_increment;
  }

  if(!point_cloud.points.empty()){
    auto duration = point_cloud.points.back().time;
    time += cartographer::common::FromSeconds(duration); //得到最后一个echo的绝对时间
    for (auto &point : point_cloud.points)
    {
        point.time -= duration;
    }
  }else{
    return;
  }

  //将laser下的点云转换到tracking下，然后把点云添加进去
  trajectory_builder_->AddSensorData("echoes",cartographer::sensor::TimedPointCloudData{time,scan_to_tracking_.translation().cast<float>(),
                                                cartographer::sensor::TransformTimedPointCloud(point_cloud.points,scan_to_tracking_.cast<float>())});
}

Slam::~Slam(){};

void Slam::SetCommandCallBack(const Command::Request::SharedPtr req,Command::Response::SharedPtr res) {
  res->message = "Failed to set command.";
  if (req->command == "load") {
    std::string filename = std::string(kMapsDirectory) + std::to_string(req->filename) + ".pbstream";
    map_builder_ptr_->LoadStateFromFile(filename, false);
    // PublishSubmaps();
    res->message = "Set command "+req->command +" "+ filename +" successfully.";
  }
};

std::unique_ptr<nav_msgs::msg::OccupancyGrid>
Slam::CreateOccupancyGridMsg(const cartographer::io::PaintSubmapSlicesResult &painted_slices,
                                const double resolution, const std::string &frame_id, const rclcpp::Time &time){
  auto occupancy_grid = std::make_unique<nav_msgs::msg::OccupancyGrid>();

  const int width = cairo_image_surface_get_width(painted_slices.surface.get());
  const int height = cairo_image_surface_get_height(painted_slices.surface.get());
  auto now = rclcpp::Time();

  occupancy_grid->header.stamp = time;
  occupancy_grid->header.frame_id = frame_id;
  occupancy_grid->info.map_load_time = time;
  occupancy_grid->info.resolution = resolution;
  occupancy_grid->info.width = width;
  occupancy_grid->info.height = height;
  occupancy_grid->info.origin.position.x = -painted_slices.origin.x() * resolution;
  occupancy_grid->info.origin.position.y = (-height + painted_slices.origin.y()) * resolution;
  occupancy_grid->info.origin.position.z = 0.;
  occupancy_grid->info.origin.orientation.w = 1.;
  occupancy_grid->info.origin.orientation.x = 0.;
  occupancy_grid->info.origin.orientation.y = 0.;
  occupancy_grid->info.origin.orientation.z = 0.;

  const uint32_t* pixel_data = reinterpret_cast<uint32_t*>(cairo_image_surface_get_data(painted_slices.surface.get()));
  occupancy_grid->data.reserve(width * height);
  for (int y = height - 1; y >= 0; --y) {
      for (int x = 0; x < width; ++x) {
          const uint32_t packed = pixel_data[y * width + x];
          const unsigned char color = packed >> 16;
          const unsigned char observed = packed >> 8;
          const int value = observed == 0 ? -1 : cartographer::common::RoundToInt((1. - color / 255.) * 100.);
          CHECK_LE(-1, value);
          CHECK_GE(100, value);
          occupancy_grid->data.push_back(value);
      }
  }
  return occupancy_grid;
}

/**
 * @brief  可视化地图数据
 * @return 返回栅格地图数据
 */
std::unique_ptr<nav_msgs::msg::OccupancyGrid>  Slam::PublishGridmap()
{
  // 获取所有子地图
  GetSubmaps();
  // 绘制地图
  auto painted_slices = cartographer::io::PaintSubmapSlices(m_submap_slices, m_resolution);
  // 绘制地图转换为栅格地图格式
  return CreateOccupancyGridMsg(painted_slices, m_resolution, m_last_frame_id, m_last_timestamp);
}

void Slam::GetSubmaps(){
  auto submaps_id_pose =  map_builder_ptr_->pose_graph()->GetAllSubmapPoses();
  if(submaps_id_pose.empty()){
      return;
  }
  std::set<cartographer::mapping::SubmapId> submap_ids_to_delete;
  for(const auto& pair:m_submap_slices){
      submap_ids_to_delete.insert(pair.first);
  }
  for (const auto& submap_id_pose : submaps_id_pose)
  {
      submap_ids_to_delete.erase(submap_id_pose.id);
      auto& submap_slice = m_submap_slices[submap_id_pose.id];
      submap_slice.pose = submap_id_pose.data.pose;
      submap_slice.metadata_version = submap_id_pose.data.version;
      if(submap_slice.surface != nullptr && submap_slice.version == submap_id_pose.data.version)
      {// need not to update map
          continue;
      }
      cartographer::mapping::proto::SubmapQuery::Response response_proto;
      map_builder_ptr_->SubmapToProto(submap_id_pose.id,&response_proto);
      submap_slice.version = response_proto.submap_version();
      for (const auto& texture_proto : response_proto.textures())
      {

      }
      submap_slice.width = response_proto.textures().begin()->width();
      submap_slice.height = response_proto.textures().begin()->height();
      auto pose = response_proto.textures().begin()->slice_pose();
      submap_slice.slice_pose = cartographer::transform::ToRigid3(response_proto.textures().begin()->slice_pose());
      submap_slice.resolution = response_proto.textures().begin()->resolution();
      submap_slice.cairo_data.clear();

      const std::string compressed_cells(response_proto.textures().begin()->cells().begin(),response_proto.textures().begin()->cells().end());
      const auto fetched_texture = cartographer::io::UnpackTextureData(
              compressed_cells,
              submap_slice.width,
              submap_slice.height);
      submap_slice.surface = cartographer::io::DrawTexture(
              fetched_texture.intensity, fetched_texture.alpha,
              submap_slice.width, submap_slice.height,
              &submap_slice.cairo_data);
  }
  for (const auto& id : submap_ids_to_delete)
  {
      m_submap_slices.erase(id);
  }
  m_last_frame_id = "map";
  m_last_timestamp = rclcpp::Time();
}


// void Slam::PublishNodes() {}

// void Slam::PublishSubmaps() {
//   mapping::MapById<mapping::SubmapId, mapping::SubmapPose> submap_poses =
//       map_builder_ptr_->pose_graph()->GetAllSubmapPoses();
//   // mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose>
//   //     trajectory_poses =
//   //         map_builder_ptr_->pose_graph()->GetTrajectoryNodePoses();
//   for (auto&& submap_id_pose : submap_poses) {
//     geometry_msgs::msg::Pose submap_pose =
//         transform::Rigid3dToGeometryPose(submap_id_pose.data.pose);
//     // LOG(ERROR) << "Submap pose 1:(" << submap_id_pose.data.pose << ").";
//     mapping::proto::SubmapQuery::Response proto;
//     std::string error =
//         map_builder_ptr_->SubmapToProto(submap_id_pose.id, &proto);
//     if (proto.textures().size() > 1) {
//       LOG(ERROR) << "Proto textures size is (" << proto.textures().size()
//                  << "),continue.";
//       continue;
//     }
//     if (proto.textures().empty()) {
//       LOG(ERROR) << "Proto textures is empty,continue.";
//       continue;
//     }
//     for (auto&& texture : proto.textures()) {
//       nav_msgs::msg::OccupancyGrid grid;
//       grid.header.stamp = rclcpp::Time();
//       grid.header.frame_id = "map";
//       grid.info.resolution = texture.resolution();
//       grid.info.width = texture.width();
//       grid.info.height = texture.height();
//       grid.info.origin = submap_pose;
//       transform::Rigid3d slice_pose = transform::ToRigid3(texture.slice_pose());
//       grid.info.origin = transform::Rigid3dToGeometryPose(slice_pose.inverse());
//       // grid.info.origin.position.x = 0.;
//       // grid.info.origin.position.y = 0.;
//       // grid.info.origin.position.y = 0.;
//       // grid.info.origin.position.z = 0.;
//       // grid.info.origin.orientation.w = 1.;
//       // grid.info.origin.orientation.x = 0.;
//       // grid.info.origin.orientation.y = 0.;
//       // grid.info.origin.orientation.z = 0.;
//       std::string cells;
//       ::cartographer::common::FastGunzipString(texture.cells(), &cells);
//       const int num_pixels = texture.width() * texture.height();
//       CHECK_EQ(cells.size(), 2 * num_pixels);
//       // LOG(ERROR) << "id:" << submap_id_pose.id << ",Cells size:("
//       //            << cells.size() << "),num_pixels:(" << num_pixels << ").";
//       for (int i = 0; i < texture.height(); ++i) {
//         for (int j = 0; j < texture.width(); ++j) {
//           int intensity = cells[(i * texture.width() + j) * 2];
//           if (intensity > 0)
//             grid.data.push_back(intensity);
//           else
//             grid.data.push_back(-1);
//         }
//       }
//       submaps_publisher_->publish(grid);
//       geometry_msgs::msg::PoseStamped pose_stamped;
//       pose_stamped.header = grid.header;
//       pose_stamped.pose = submap_pose;
//       submap_poses_publisher_->publish(pose_stamped);
//       mapping::MapById<mapping::NodeId, mapping::TrajectoryNodePose>
//           node_poses = map_builder_ptr_->pose_graph()->GetNodePosesBySubmapId(
//               submap_id_pose.id);

//       nav_msgs::msg::Path path;
//       path.header.stamp = rclcpp::Time();
//       path.header.frame_id = "map";
//       LOG(ERROR) << "node_poses size:(" << node_poses.size() << ").";
//       for (auto&& node_pose : node_poses) {
//         pose_stamped.header.stamp = rclcpp::Time();
//         pose_stamped.header.frame_id = "map";
//         pose_stamped.pose =
//             transform::Rigid3dToGeometryPose(node_pose.data.global_pose);
//         path.poses.push_back(pose_stamped);
//       }
//       node_poses_publisher_->publish(path);
//       sleep(1);
//     }
//   }
}  // namespace slam
}  // namespace cartographer