/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/map_builder.h"

#include "absl/memory/memory.h"
#include "absl/types/optional.h"
#include "cartographer/common/time.h"
#include "cartographer/io/internal/mapping_state_serialization.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
// #include "cartographer/io/serialization_format_migration.h"
#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"
#include "cartographer/mapping/internal/2d/pose_graph_2d.h"
// #include "cartographer/mapping/internal/3d/local_trajectory_builder_3d.h"
// #include "cartographer/mapping/internal/3d/pose_graph_3d.h"
// #include "cartographer/mapping/internal/collated_trajectory_builder.h"
#include "cartographer/mapping/internal/global_trajectory_builder.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/sensor/internal/collator.h"
// #include "cartographer/sensor/internal/trajectory_collator.h"
#include "Eigen/Geometry"
// #include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/id.h"
// #include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/internal/pose_graph_data.h"
// #include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
// #include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/submaps.h"
// #include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/transform/rigid_transform.h"
// #include "cartographer/mapping/map_builder_interface.h"
// #include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/internal/2d/pose_graph_2d.h"
#include "cartographer/mapping/internal/pose_graph_data.h"
// #include "cartographer/mapping/proto/map_builder_options.pb.h"
// #include "cartographer/sensor/collator_interface.h"
#include "cartographer/sensor/internal/collator.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {
// proto::MapBuilderOptions CreateMapBuilderOptions(
//     common::LuaParameterDictionary* const parameter_dictionary) {
//   proto::MapBuilderOptions options;
//   options.set_use_trajectory_builder_2d(
//       parameter_dictionary->GetBool("use_trajectory_builder_2d"));
//   //   options.set_use_trajectory_builder_3d(
//   //       parameter_dictionary->GetBool("use_trajectory_builder_3d"));
//   options.set_num_background_threads(
//       parameter_dictionary->GetNonNegativeInt("num_background_threads"));
//   options.set_collate_by_trajectory(
//       parameter_dictionary->GetBool("collate_by_trajectory"));
//   *options.mutable_pose_graph_options() = CreatePoseGraphOptions(
//       parameter_dictionary->GetDictionary("pose_graph").get());
//   //   CHECK_NE(options.use_trajectory_builder_2d(),
//   //            options.use_trajectory_builder_3d());
//   return options;
// }
namespace {

using mapping::proto::SerializedData;

std::vector<std::string> SelectRangeSensorIds(
    const std::set<SensorId>& expected_sensor_ids) {
  std::vector<std::string> range_sensor_ids;
  for (const SensorId& sensor_id : expected_sensor_ids) {
    if (sensor_id.type == SensorId::SensorType::RANGE) {
      range_sensor_ids.push_back(sensor_id.id);
    }
  }
  return range_sensor_ids;
}

// void MaybeAddPureLocalizationTrimmer(
//     const int trajectory_id,
//     const proto::TrajectoryBuilderOptions& trajectory_options,
//     PoseGraph* pose_graph) {
//   if (trajectory_options.pure_localization()) {
//     LOG(WARNING)
//         << "'TrajectoryBuilderOptions::pure_localization' field is
//         deprecated. "
//            "Use 'TrajectoryBuilderOptions::pure_localization_trimmer'
//            instead.";
//     pose_graph->AddTrimmer(std::make_unique<PureLocalizationTrimmer>(
//         trajectory_id, 3 /* max_submaps_to_keep */));
//     return;
//   }
//   if (trajectory_options.has_pure_localization_trimmer()) {
//     pose_graph->AddTrimmer(std::make_unique<PureLocalizationTrimmer>(
//         trajectory_id,
//         trajectory_options.pure_localization_trimmer().max_submaps_to_keep()));
//   }
// }

// void MaybeAddPureLocalizationTrimmer(const int trajectory_id,
//                                      PoseGraph* pose_graph) {
//   if (trajectory_options.pure_localization()) {
//     LOG(WARNING)
//         << "'TrajectoryBuilderOptions::pure_localization' field is
//         deprecated. "
//            "Use 'TrajectoryBuilderOptions::pure_localization_trimmer'
//            instead.";
//     pose_graph->AddTrimmer(std::make_unique<PureLocalizationTrimmer>(
//         trajectory_id, 3 /* max_submaps_to_keep */));
//     return;
//   }
//   if (trajectory_options.has_pure_localization_trimmer()) {
//     pose_graph->AddTrimmer(std::make_unique<PureLocalizationTrimmer>(
//         trajectory_id,
//         trajectory_options.pure_localization_trimmer().max_submaps_to_keep()));
//   }
// }

}  // namespace

MapBuilder::MapBuilder() : thread_pool_(kBackgroundThreadsCount) {
  pose_graph_2d_ = std::make_unique<PoseGraph2D>(
      std::make_unique<optimization::OptimizationProblem2D>(), &thread_pool_);
  sensor_collator_ = std::make_unique<sensor::Collator>();
}

// MapBuilder::MapBuilder(const proto::MapBuilderOptions& options)
//     : options_(options), thread_pool_(options.num_background_threads()) {
//   // CHECK(options.use_trajectory_builder_2d() ^
//   //       options.use_trajectory_builder_3d());
//   if (options.use_trajectory_builder_2d()) {
//     pose_graph_2d_ = std::make_unique<PoseGraph2D>(
//         options_.pose_graph_options(),
//         std::make_unique<optimization::OptimizationProblem2D>(
//             options_.pose_graph_options().optimization_problem_options()),
//         &thread_pool_);
//   }
//   // if (options.use_trajectory_builder_3d()) {
//   //   pose_graph_ = std::make_unique<PoseGraph3D>(
//   //       options_.pose_graph_options(),
//   //       std::make_unique<optimization::OptimizationProblem3D>(
//   //           options_.pose_graph_options().optimization_problem_options()),
//   //       &thread_pool_);
//   // }
//   // if (options.collate_by_trajectory()) {
//   //   sensor_collator_ = std::make_unique<sensor::TrajectoryCollator>();
//   // } else {
//   sensor_collator_ = std::make_unique<sensor::Collator>();
//   // }
// }

int MapBuilder::AddTrajectoryBuilder(
    const std::set<SensorId>& expected_sensor_ids,
    // const proto::TrajectoryBuilderOptions& trajectory_options,
    LocalSlamResultCallback local_slam_result_callback) {
  const int trajectory_id = trajectory_builders_.size();
  // absl::optional<MotionFilter> pose_graph_odometry_motion_filter;
  std::optional<MotionFilter> pose_graph_odometry_motion_filter;
  // if (trajectory_options.has_pose_graph_odometry_motion_filter()) {
  //   LOG(INFO) << "Using a motion filter for adding odometry to the pose
  //   graph."; pose_graph_odometry_motion_filter.emplace(
  //       MotionFilter(trajectory_options.pose_graph_odometry_motion_filter()));
  // }

  // if (options_.use_trajectory_builder_3d()) {
  //   std::unique_ptr<LocalTrajectoryBuilder3D> local_trajectory_builder;
  //   if (trajectory_options.has_trajectory_builder_3d_options()) {
  //     local_trajectory_builder = std::make_unique<LocalTrajectoryBuilder3D>(
  //         trajectory_options.trajectory_builder_3d_options(),
  //         SelectRangeSensorIds(expected_sensor_ids));
  //   }
  //   DCHECK(dynamic_cast<PoseGraph3D*>(pose_graph_.get()));
  //   trajectory_builders_.push_back(std::make_unique<CollatedTrajectoryBuilder>(
  //       trajectory_options, sensor_collator_.get(), trajectory_id,
  //       expected_sensor_ids,
  //       CreateGlobalTrajectoryBuilder3D(
  //           std::move(local_trajectory_builder), trajectory_id,
  //           static_cast<PoseGraph3D*>(pose_graph_.get()),
  //           local_slam_result_callback, pose_graph_odometry_motion_filter)));
  // } else {
  // std::shared_ptr<LocalTrajectoryBuilder2D> local_trajectory_builder_2d =
  //     std::make_shared<LocalTrajectoryBuilder2D>(
  //         trajectory_options.trajectory_builder_2d_options(),
  //         SelectRangeSensorIds(expected_sensor_ids));
  std::shared_ptr<LocalTrajectoryBuilder2D> local_trajectory_builder_2d =
      std::make_shared<LocalTrajectoryBuilder2D>(
          SelectRangeSensorIds(expected_sensor_ids));

  std::shared_ptr<GlobalTrajectoryBuilder2D> gloabal_trajectory_builder_2d =
      std::make_shared<GlobalTrajectoryBuilder2D>(
          local_trajectory_builder_2d, trajectory_id, pose_graph_2d_.get(),
          local_slam_result_callback, pose_graph_odometry_motion_filter);

  trajectory_builders_.push_back(std::move(gloabal_trajectory_builder_2d));

  //   return std::make_unique<
  //       GlobalTrajectoryBuilder<LocalTrajectoryBuilder2D,
  //       mapping::PoseGraph2D>>( std::move(local_trajectory_builder),
  //       trajectory_id, pose_graph, local_slam_result_callback,
  //       pose_graph_odometry_motion_filter);

  // }
  // DCHECK(dynamic_cast<PoseGraph2D*>(pose_graph_.get()));
  // trajectory_builders_.push_back(std::make_unique<CollatedTrajectoryBuilder>(
  //     trajectory_options, sensor_collator_.get(), trajectory_id,
  //     expected_sensor_ids,
  //     CreateGlobalTrajectoryBuilder2D(
  //         std::move(local_trajectory_builder), trajectory_id,
  //         static_cast<PoseGraph2D*>(pose_graph_.get()),
  //         local_slam_result_callback, pose_graph_odometry_motion_filter)));
  // }
  // MaybeAddPureLocalizationTrimmer(trajectory_id, trajectory_options,
  //                                 pose_graph_2d_.get());

  // if (trajectory_options.has_initial_trajectory_pose()) {
  //   const auto& initial_trajectory_pose =
  //       trajectory_options.initial_trajectory_pose();
  //   pose_graph_2d_->SetInitialTrajectoryPose(
  //       trajectory_id, initial_trajectory_pose.to_trajectory_id(),
  //       transform::ToRigid3(initial_trajectory_pose.relative_pose()),
  //       common::FromUniversal(initial_trajectory_pose.timestamp()));
  //   }
  // proto::TrajectoryBuilderOptionsWithSensorIds options_with_sensor_ids_proto;
  // for (const auto& sensor_id : expected_sensor_ids) {
  //   *options_with_sensor_ids_proto.add_sensor_id() = ToProto(sensor_id);
  // }
  // *options_with_sensor_ids_proto.mutable_trajectory_builder_options() =
  //     trajectory_options;
  // all_trajectory_builder_options_.push_back(options_with_sensor_ids_proto);
  // CHECK_EQ(trajectory_builders_.size(),
  // all_trajectory_builder_options_.size());
  return trajectory_id;
}

// int MapBuilder::AddTrajectoryForDeserialization(
//     const proto::TrajectoryBuilderOptionsWithSensorIds&
//         options_with_sensor_ids_proto) {
//   const int trajectory_id = trajectory_builders_.size();
//   trajectory_builders_.emplace_back();
//   //
//   all_trajectory_builder_options_.push_back(options_with_sensor_ids_proto);
//   // CHECK_EQ(trajectory_builders_.size(),
//   // all_trajectory_builder_options_.size());
//   return trajectory_id;
// }

int MapBuilder::AddTrajectoryForDeserialization() {
  const int trajectory_id = trajectory_builders_.size();
  trajectory_builders_.emplace_back();
  // all_trajectory_builder_options_.push_back(options_with_sensor_ids_proto);
  // CHECK_EQ(trajectory_builders_.size(),
  // all_trajectory_builder_options_.size());
  return trajectory_id;
}

void MapBuilder::FinishTrajectory(const int trajectory_id) {
  sensor_collator_->FinishTrajectory(trajectory_id);
  pose_graph_2d_->FinishTrajectory(trajectory_id);
}

MapById<SubmapId, ::cartographer::mapping::SubmapData>
MapBuilder::GetAllSubmapData() {
  return pose_graph_2d_->GetAllSubmapData();
}

std::string MapBuilder::SubmapToProto(
    const SubmapId& submap_id, proto::SubmapQuery::Response* const response) {
  if (submap_id.trajectory_id < 0 ||
      submap_id.trajectory_id >= num_trajectory_builders()) {
    return "Requested submap from trajectory " +
           std::to_string(submap_id.trajectory_id) + " but there are only " +
           std::to_string(num_trajectory_builders()) + " trajectories.";
  }

  const auto submap_data = pose_graph_2d_->GetSubmapData(submap_id);
  if (submap_data.submap == nullptr) {
    return "Requested submap " + std::to_string(submap_id.submap_index) +
           " from trajectory " + std::to_string(submap_id.trajectory_id) +
           " but it does not exist: maybe it has been trimmed.";
  }
  submap_data.submap->ToResponseProto(submap_data.pose, response);
  return "";
}

void MapBuilder::SerializeState(bool include_unfinished_submaps,
                                io::ProtoStreamWriter* const writer) {
  io::WritePbStream(*pose_graph_2d_,
                    // all_trajectory_builder_options_,
                    writer, include_unfinished_submaps);
}

bool MapBuilder::SerializeStateToFile(bool include_unfinished_submaps,
                                      const std::string& filename) {
  io::ProtoStreamWriter writer(filename);
  io::WritePbStream(*pose_graph_2d_,
                    // all_trajectory_builder_options_,
                    &writer, include_unfinished_submaps);
  return (writer.Close());
}

std::map<int, int> MapBuilder::LoadState(io::ProtoStreamReader* const reader,
                                         bool load_frozen_state) {
  io::ProtoStreamDeserializer deserializer(reader);
  proto::PoseGraph pose_graph_proto = deserializer.pose_graph();
  // const auto& all_builder_options_proto =
  //     deserializer.all_trajectory_builder_options();
  // LOG(ERROR) << "Trajectory size:(" << pose_graph_proto.trajectory_size()
  //            << ").";
  std::map<int, int> trajectory_remapping;
  for (int i = 0; i < pose_graph_proto.trajectory_size(); ++i) {
    auto& trajectory_proto = *pose_graph_proto.mutable_trajectory(i);
    // const auto& options_with_sensor_ids_proto =
    //     all_builder_options_proto.options_with_sensor_ids(i);
    const int new_trajectory_id = AddTrajectoryForDeserialization();
    CHECK(trajectory_remapping
              .emplace(trajectory_proto.trajectory_id(), new_trajectory_id)
              .second)
        << "Duplicate trajectory ID: " << trajectory_proto.trajectory_id();
    trajectory_proto.set_trajectory_id(new_trajectory_id);
    if (load_frozen_state) {
      pose_graph_2d_->FreezeTrajectory(new_trajectory_id);
    }
  }

  // Apply the calculated remapping to constraints in the pose graph proto.
  for (auto& constraint_proto : *pose_graph_proto.mutable_constraint()) {
    constraint_proto.mutable_submap_id()->set_trajectory_id(
        trajectory_remapping.at(constraint_proto.submap_id().trajectory_id()));
    constraint_proto.mutable_node_id()->set_trajectory_id(
        trajectory_remapping.at(constraint_proto.node_id().trajectory_id()));
  }

  MapById<SubmapId, transform::Rigid3d> submap_poses;
  for (const proto::Trajectory& trajectory_proto :
       pose_graph_proto.trajectory()) {
    for (const proto::Trajectory::Submap& submap_proto :
         trajectory_proto.submap()) {
      // LOG(ERROR) << "Submap:(" << trajectory_proto.trajectory_id() << ","
      //            << submap_proto.submap_index() << "),pose:("
      //            << transform::Project2D(
      //                   transform::ToRigid3(submap_proto.pose()))
      //            << ").";
      submap_poses.Insert(SubmapId{trajectory_proto.trajectory_id(),
                                   submap_proto.submap_index()},
                          transform::ToRigid3(submap_proto.pose()));
    }
  }

  MapById<NodeId, transform::Rigid3d> node_poses;
  for (const proto::Trajectory& trajectory_proto :
       pose_graph_proto.trajectory()) {
    for (const proto::Trajectory::Node& node_proto : trajectory_proto.node()) {
      // LOG(ERROR) << "Node:(" << trajectory_proto.trajectory_id() << ","
      //            << node_proto.node_index() << "),pose:("
      //            <<
      //            transform::Project2D(transform::ToRigid3(node_proto.pose()))
      //            << ").";
      node_poses.Insert(
          NodeId{trajectory_proto.trajectory_id(), node_proto.node_index()},
          transform::ToRigid3(node_proto.pose()));
    }
  }

  // Set global poses of landmarks.
  // for (const auto& landmark : pose_graph_proto.landmark_poses()) {
  //   pose_graph_->SetLandmarkPose(landmark.landmark_id(),
  //                                transform::ToRigid3(landmark.global_pose()),
  //                                true);
  // }

  // if (options_.use_trajectory_builder_3d()) {
  //   CHECK_NE(deserializer.header().format_version(),
  //            io::kFormatVersionWithoutSubmapHistograms)
  //       << "The pbstream file contains submaps without rotational histograms.
  //       "
  //          "This can be converted with the 'pbstream migrate' tool, see the "
  //          "Cartographer documentation for details. ";
  // }

  SerializedData proto;
  while (deserializer.ReadNextSerializedData(&proto)) {
    switch (proto.data_case()) {
      case SerializedData::kPoseGraph:
        LOG(ERROR) << "Found multiple serialized `PoseGraph`. Serialized "
                      "stream likely corrupt!.";
        break;
      // case SerializedData::kAllTrajectoryBuilderOptions:
      //   LOG(ERROR) << "Found AllTrajectoryBuilderOptions";
      //   //   LOG(ERROR) << "Found multiple serialized "
      //   //                 "`AllTrajectoryBuilderOptions`. Serialized stream
      //   //                 likely " "corrupt!.";
      //   break;
      case SerializedData::kSubmap: {
        proto.mutable_submap()->mutable_submap_id()->set_trajectory_id(
            trajectory_remapping.at(
                proto.submap().submap_id().trajectory_id()));
        const SubmapId submap_id(proto.submap().submap_id().trajectory_id(),
                                 proto.submap().submap_id().submap_index());
        pose_graph_2d_->AddSubmapFromProto(submap_poses.at(submap_id),
                                           proto.submap());
        break;
      }
      case SerializedData::kNode: {
        proto.mutable_node()->mutable_node_id()->set_trajectory_id(
            trajectory_remapping.at(proto.node().node_id().trajectory_id()));
        const NodeId node_id(proto.node().node_id().trajectory_id(),
                             proto.node().node_id().node_index());
        const transform::Rigid3d& node_pose = node_poses.at(node_id);
        pose_graph_2d_->AddNodeFromProto(node_pose, proto.node());
        break;
      }
      case SerializedData::kTrajectoryData: {
        proto.mutable_trajectory_data()->set_trajectory_id(
            trajectory_remapping.at(proto.trajectory_data().trajectory_id()));
        pose_graph_2d_->SetTrajectoryDataFromProto(proto.trajectory_data());
        break;
      }
      case SerializedData::kImuData: {
        if (load_frozen_state) break;
        LOG(ERROR) << "AddImuData";
        pose_graph_2d_->AddImuData(
            trajectory_remapping.at(proto.imu_data().trajectory_id()),
            sensor::FromProto(proto.imu_data().imu_data()));
        break;
      }
      case SerializedData::kOdometryData: {
        if (load_frozen_state) break;
        LOG(ERROR) << "AddOdometryData";
        pose_graph_2d_->AddOdometryData(
            trajectory_remapping.at(proto.odometry_data().trajectory_id()),
            sensor::FromProto(proto.odometry_data().odometry_data()));
        break;
      }
      // case SerializedData::kFixedFramePoseData: {
      //   LOG(ERROR) << "kFixedFramePoseData";
      //   //   if (load_frozen_state) break;
      //   //   pose_graph_->AddFixedFramePoseData(
      //   //       trajectory_remapping.at(
      //   //           proto.fixed_frame_pose_data().trajectory_id()),
      //   //       sensor::FromProto(
      //   //           proto.fixed_frame_pose_data().fixed_frame_pose_data()));
      //   break;
      // }
      // case SerializedData::kLandmarkData: {
      //   LOG(ERROR) << "kLandmarkData";
      //   //   if (load_frozen_state) break;
      //   //   pose_graph_->AddLandmarkData(
      //   // trajectory_remapping.at(proto.landmark_data().trajectory_id()),
      //   //       sensor::FromProto(proto.landmark_data().landmark_data()));
      //   break;
      // }
      default:
        LOG(WARNING) << "Skipping unknown message type in stream:("
                     //  << proto.GetTypeName() << "),(" << proto.data_case()
                     << ").";
    }
  }

  if (load_frozen_state) {
    // LOG(ERROR) << "load_frozen_state:(" << load_frozen_state << ").";
    // Add information about which nodes belong to which submap.
    // This is required, even without constraints.
    for (const proto::PoseGraph::Constraint& constraint_proto :
         pose_graph_proto.constraint()) {
      if (constraint_proto.tag() !=
          proto::PoseGraph::Constraint::INTRA_SUBMAP) {
        continue;
      }
      pose_graph_2d_->AddNodeToSubmap(
          NodeId{constraint_proto.node_id().trajectory_id(),
                 constraint_proto.node_id().node_index()},
          SubmapId{constraint_proto.submap_id().trajectory_id(),
                   constraint_proto.submap_id().submap_index()});
    }
  } else {
    // When loading unfrozen trajectories, 'AddSerializedConstraints' will
    // take care of adding information about which nodes belong to which
    // submap.
    pose_graph_2d_->AddSerializedConstraints(
        FromProto(pose_graph_proto.constraint()));
  }
  CHECK(reader->eof());
  return trajectory_remapping;
}

std::map<int, int> MapBuilder::LoadStateFromFile(
    const std::string& state_filename, const bool load_frozen_state) {
  const std::string suffix = ".pbstream";
  if (state_filename.substr(
          std::max<int>(state_filename.size() - suffix.size(), 0)) != suffix) {
    LOG(WARNING) << "The file containing the state should be a "
                    ".pbstream file.";
  }
  LOG(INFO) << "Loading saved state '" << state_filename << "'...";
  io::ProtoStreamReader stream(state_filename);
  return LoadState(&stream, load_frozen_state);
}

// std::unique_ptr<MapBuilder> CreateMapBuilder(
//     const proto::MapBuilderOptions& options) {
//   return std::make_unique<MapBuilder>(options);
// }

}  // namespace mapping
}  // namespace cartographer
