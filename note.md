# Cartographer build notes

## 1 平台搭建
克隆cartograher
```bash
git clone https://github.com/cartographer-project/cartographer.git
```

### 1.2 依赖
absl

编译
```bash
cmake build
cd build
cmake ..
make
```

## 2 CMakeLists.txt编译优化
### 2.1 删除去WIN32分支和MSVC分支
### 2.2 删除GRPC
删除GRPC是cartographer做的另外一套接口，是通过服务来调用的。我们使用ros的接口，所以不使用这个。
* CMakeList.txt和cartographer-config.cmake.in中的GRPC相关删除
* 同时文中提ALL_GRPC_FILES在file(...)，也就是说cartographer/cloud文件夹中的文件，我们一起删除
```bash
rm cartographer/cloud -rf
```
* 删除Lua文件
```
rm ./configuration_files/map_builder_server.lua
```

### 2.3 删除PROMETHEUS
PROMETHEUS（普罗米修斯）

### 2.4 删除Sphinx分支
```c
find_package(Sphinx)
if(SPHINX_FOUND)
  add_subdirectory("docs")
endif()
```
删除docs文件夹

### 2.5 删除TEST相关

```
find . -name fake_*
./cartographer/io/fake_file_writer.cc
./cartographer/io/fake_file_writer_test.cc
./cartographer/io/fake_file_writer.h
./cartographer/mapping/internal/testing/fake_trimmable.h
```

```
find . -name *test_helpers* 
./cartographer/sensor/internal/test_helpers.h
./cartographer/io/internal/testing/test_helpers.h
./cartographer/io/internal/testing/test_helpers.cc
./cartographer/common/internal/testing/lua_parameter_dictionary_test_helpers.h
./cartographer/mapping/internal/testing/test_helpers.h
./cartographer/mapping/internal/testing/test_helpers.cc
./cartographer/transform/rigid_transform_test_helpers.h
```

```
find . -name mock_*
./cartographer/mapping/internal/testing/mock_pose_graph.h
./cartographer/mapping/internal/testing/mock_trajectory_builder.h
./cartographer/mapping/internal/testing/mock_map_builder.h
```

```
find . -name *_test.*
./cartographer/sensor/map_by_time_test.cc
./cartographer/sensor/point_cloud_test.cc
./cartographer/sensor/compressed_point_cloud_test.cc
./cartographer/sensor/landmark_data_test.cc
./cartographer/sensor/internal/voxel_filter_test.cc
./cartographer/sensor/internal/trajectory_collator_test.cc
./cartographer/sensor/internal/collator_test.cc
./cartographer/sensor/internal/ordered_multi_queue_test.cc
./cartographer/sensor/range_data_test.cc
./cartographer/io/probability_grid_points_processor_test.cc
./cartographer/io/points_processor_pipeline_builder_test.cc
./cartographer/io/proto_stream_test.cc
./cartographer/io/proto_stream_deserializer_test.cc
./cartographer/io/
./cartographer/io/internal/in_memory_proto_stream_test.cc
./cartographer/common/fixed_ratio_sampler_test.cc
./cartographer/common/math_test.cc
./cartographer/common/thread_pool_test.cc
./cartographer/common/internal/blocking_queue_test.cc
./cartographer/common/internal/rate_timer_test.cc
./cartographer/common/task_test.cc
./cartographer/common/configuration_files_test.cc
./cartographer/common/lua_parameter_dictionary_test.cc
./cartographer/mapping/pose_extrapolator_test.cc
./cartographer/mapping/pose_graph_test.cc
./cartographer/mapping/id_test.cc
./cartographer/mapping/submaps_test.cc
./cartographer/mapping/pose_graph_trimmer_test.cc
./cartographer/mapping/2d/probability_grid_test.cc
./cartographer/mapping/2d/range_data_inserter_2d_test.cc
./cartographer/mapping/2d/xy_index_test.cc
./cartographer/mapping/2d/submap_2d_test.cc
./cartographer/mapping/2d/map_limits_test.cc
./cartographer/mapping/probability_values_test.cc
./cartographer/mapping/3d/submap_3d_test.cc
./cartographer/mapping/3d/hybrid_grid_test.cc
./cartographer/mapping/3d/range_data_inserter_3d_test.cc
./cartographer/mapping/map_builder_test.cc
./cartographer/mapping/imu_tracker_test.cc
./cartographer/mapping/internal/optimization/cost_functions/spa_cost_function_2d_test.cc
./cartographer/mapping/internal/optimization/cost_functions/landmark_cost_function_2d_test.cc
./cartographer/mapping/internal/optimization/cost_functions/landmark_cost_function_3d_test.cc
./cartographer/mapping/internal/optimization/optimization_problem_3d_test.cc
./cartographer/mapping/internal/trajectory_connectivity_state_test.cc
./cartographer/mapping/internal/2d/overlapping_submaps_trimmer_2d_test.cc
./cartographer/mapping/internal/2d/tsdf_2d_test.cc
./cartographer/mapping/internal/2d/ray_to_pixel_mask_test.cc
./cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d_test.cc
./cartographer/mapping/internal/2d/scan_matching/occupied_space_cost_function_2d_test.cc
./cartographer/mapping/internal/2d/scan_matching/interpolated_tsdf_2d_test.cc
./cartographer/mapping/internal/2d/scan_matching/tsdf_match_cost_function_2d_test.cc
./cartographer/mapping/internal/2d/scan_matching/correlative_scan_matcher_test.cc
./cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d_test.cc
./cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d_test.cc
./cartographer/mapping/internal/2d/pose_graph_2d_test.cc
./cartographer/mapping/internal/2d/tsdf_range_data_inserter_2d_test.cc
./cartographer/mapping/internal/2d/tsd_value_converter_test.cc
./cartographer/mapping/internal/2d/normal_estimation_2d_test.cc
./cartographer/mapping/internal/3d/scan_matching/real_time_correlative_scan_matcher_3d_test.cc
./cartographer/mapping/internal/3d/scan_matching/interpolated_grid_test.cc
./cartographer/mapping/internal/3d/scan_matching/precomputation_grid_3d_test.cc
./cartographer/mapping/internal/3d/scan_matching/fast_correlative_scan_matcher_3d_test.cc
./cartographer/mapping/internal/3d/scan_matching/ceres_scan_matcher_3d_test.cc
./cartographer/mapping/internal/3d/scan_matching/rotation_delta_cost_functor_3d_test.cc
./cartographer/mapping/internal/3d/scan_matching/intensity_cost_function_3d_test.cc
./cartographer/mapping/internal/3d/scan_matching/rotational_scan_matcher_test.cc
./cartographer/mapping/internal/3d/pose_graph_3d_test.cc
./cartographer/mapping/internal/3d/local_trajectory_builder_3d_test.cc
./cartographer/mapping/internal/connected_components_test.cc
./cartographer/mapping/internal/range_data_collator_test.cc
./cartographer/mapping/internal/constraints/constraint_builder_2d_test.cc
./cartographer/mapping/internal/constraints/constraint_builder_3d_test.cc
./cartographer/mapping/internal/motion_filter_test.cc
./cartographer/mapping/trajectory_node_test.cc
./cartographer/mapping/value_conversion_tables_test.cc
./cartographer/transform/transform_test.cc
./cartographer/transform/transform_interpolation_buffer_test.cc
./cartographer/transform/timestamped_transform_test.cc
./cartographer/transform/rigid_transform_test.cc
```

```
find . -name fake_* -delete
find . -name *test_helpers*  -delete
find . -name mock_* -delete
find . -name *_test.* -delete
```

修改CMakeLists.txt

```
# google_enable_testing()
#file(GLOB_RECURSE TEST_LIBRARY_HDRS "cartographer/fake_*.h" "cartographer/*test_helpers*.h" "cartographer/mock_*.h")
#file(GLOB_RECURSE TEST_LIBRARY_SRCS "cartographer/fake_*.cc" "cartographer/*test_helpers*.cc" "cartographer/mock_*.cc")
# file(GLOB_RECURSE ALL_TESTS "cartographer/*_test.cc")
  # list(REMOVE_ITEM TEST_LIBRARY_HDRS ${ALL_DOTFILES})
  # list(REMOVE_ITEM TEST_LIBRARY_SRCS ${ALL_DOTFILES})
  # list(REMOVE_ITEM ALL_TESTS ${ALL_DOTFILES})
# list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_TESTS})
# list(REMOVE_ITEM ALL_LIBRARY_HDRS ${TEST_LIBRARY_HDRS})
# list(REMOVE_ITEM ALL_LIBRARY_SRCS ${TEST_LIBRARY_SRCS})
# set(TEST_LIB
#   cartographer_test_library
# )
# add_library(${TEST_LIB} ${TEST_LIBRARY_HDRS} ${TEST_LIBRARY_SRCS})
# target_include_directories(${TEST_LIB} SYSTEM PRIVATE
#   "${GMOCK_INCLUDE_DIRS}")
# target_link_libraries(${TEST_LIB} PUBLIC ${GMOCK_LIBRARY})
# target_link_libraries(${TEST_LIB} PUBLIC ${PROJECT_NAME})
# set_target_properties(${TEST_LIB} PROPERTIES
#   COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

# foreach(ABS_FIL ${ALL_TESTS})
#   file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
#   get_filename_component(DIR ${REL_FIL} DIRECTORY)
#   get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
#   # Replace slashes as required for CMP0037.
#   string(REPLACE "/" "." TEST_TARGET_NAME "${DIR}/${FIL_WE}")
#   google_test("${TEST_TARGET_NAME}" ${ABS_FIL})
#   # if(${BUILD_GRPC})
#   #   target_link_libraries("${TEST_TARGET_NAME}" PUBLIC grpc++)
#   #   target_link_libraries("${TEST_TARGET_NAME}" PUBLIC async_grpc)
#   # endif()
#   # if(${BUILD_PROMETHEUS})
#   #   target_link_libraries("${TEST_TARGET_NAME}" PUBLIC ${ZLIB_LIBRARIES})
#   #   target_link_libraries("${TEST_TARGET_NAME}" PUBLIC prometheus-cpp-core)
#   #   target_link_libraries("${TEST_TARGET_NAME}" PUBLIC prometheus-cpp-pull)
#   # endif()
#   target_link_libraries("${TEST_TARGET_NAME}" PUBLIC ${TEST_LIB})
# endforeach()
```