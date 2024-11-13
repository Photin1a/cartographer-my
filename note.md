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

## 2 编译优化
### 2.1 删除去WIN32分支和MSVC分支
### 2.2 删除GRPC
删除GRPC是cartographer做的另外一套接口，是通过服务来调用的。我们使用ros的接口，所以不使用这个。
* CMakeList.txt和cartographer-config.cmake.in中的GRPC相关删除
* 同时文中提ALL_GRPC_FILES在file(...)，也就是说cartographer/cloud文件夹中的文件，我们一起删除
```bash
rm cartographer/cloud -rf
```