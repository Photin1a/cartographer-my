cmake_minimum_required(VERSION 3.2)
project(cartographer_core)
set(CMAKE_CXX_STANDARD 17)
cmake_policy(SET CMP0057 NEW)

add_compile_options(-Wno-error=deprecated)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")  #std::thread  must be to linux   
set(CMAKE_EXPORT_COMPILE_COMMANDS on)

set(BUILD_TEST off)
set(BUILD_SLAM on)

message("USE ROS2")
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cartographer_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


include("${PROJECT_SOURCE_DIR}/cmake/functions.cmake")
google_initialize_cartographer_project()
find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(Eigen3 REQUIRED)
find_package(Protobuf 3.0.0 REQUIRED)
find_library(CAIRO_LIBRARIES cairo)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS} ${GTest_INCLUDE_DIRS} ${Glog_INCLUDE_DIRS} 
${EIGEN3_INCLUDE_DIR} ${CERES_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIR})

file(GLOB_RECURSE CARTO_LIBRARY_HDRS "cartographer/*.h" "cartographer/*.hpp")
file(GLOB_RECURSE CARTO_LIBRARY_SRCS "cartographer/*.cc")
file(GLOB_RECURSE ALL_PROTOS "cartographer/*.proto")
# file(GLOB_RECURSE INTERNAL_HDRS "cartographer/*/internal/*.h")

#将probuf文件转为.h .cc文件
set(ALL_PROTO_SRCS)
set(ALL_PROTO_HDRS)
foreach(ABS_FIL ${ALL_PROTOS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})  #得到相对路径文件
  get_filename_component(DIR ${REL_FIL} DIRECTORY)  #从相对路径文件提取路径部分
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE) #...提取文件名
  list(APPEND ALL_PROTO_SRCS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
  list(APPEND ALL_PROTO_HDRS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")
  add_custom_command(
    OUTPUT "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc"
          "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h"
    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --cpp_out  ${PROJECT_BINARY_DIR} -I
      ${PROJECT_SOURCE_DIR} ${ABS_FIL}
    DEPENDS ${ABS_FIL}
    COMMENT "Running C++ protocol buffer compiler on ${ABS_FIL}"
    VERBATIM
  )
endforeach()

#标记文件为自动生产文件，不应该被手动编辑
set_source_files_properties(${ALL_PROTO_SRCS} ${ALL_PROTO_HDRS} PROPERTIES GENERATED TRUE)
list(APPEND CARTO_LIBRARY_HDRS ${ALL_PROTO_HDRS})
list(APPEND CARTO_LIBRARY_SRCS ${ALL_PROTO_SRCS})

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  cartographer_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
)

if(${BUILD_TEST})
  add_executable(test_node ./src/test_node.cc ${CARTO_LIBRARY_SRCS})
  target_link_libraries(test_node ${EIGEN3_LIBRARIES} ${CERES_LIBRARIES} ${PROTOBUF_LIBRARY}
    ${Boost_LIBRARIES} ${CAIRO_LIBRARIES} pthread gtest glog gflags)
  ament_target_dependencies(test_node ${dependencies})
  install(TARGETS test_node DESTINATION lib/${PROJECT_NAME})
endif()

if(${BUILD_SLAM})
  add_executable(slam_node ./src/slam_node.cc ./src/slam.cc ${CARTO_LIBRARY_SRCS})
  target_link_libraries(slam_node ${EIGEN3_LIBRARIES} ${CERES_LIBRARIES} ${PROTOBUF_LIBRARY}
    ${Boost_LIBRARIES} ${CAIRO_LIBRARIES} pthread glog gflags )
  ament_target_dependencies(slam_node ${dependencies})
  install(TARGETS slam_node DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

endif()

ament_package()