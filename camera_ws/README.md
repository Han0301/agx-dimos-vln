# 相机与数据源工作空间

复用现有 D455、X3 驱动和 C++ RGB-D 压缩器。实时采集与 bag 回灌提供同一套原生输入话题，算法不需要切换协议。

## 启动

从项目根目录执行：

```bash
bash camera_ws/start_cameras.sh --camera d455
bash camera_ws/start_cameras.sh --camera x3 d455
bash sh/play_bag.sh --bag data/bags/有效bag目录
```

`sh/play_bag.sh` 只播放 bag 原话题和 /clock；独立压缩入口为 `bash camera_ws/start_compressor.sh`，默认等待来源并检测 /clock 或 rosbag2 发布者，检查/纠正已有压缩器时钟，确认输入时间一致后复用。不能与真实数据源同时运行。源参数见 `config/camera_stack.conf`、项目根 `config/bag.yaml`；详细行为见 [数据源约定](../doc/architecture/data-sources.md)。

`camera_ws/start_cameras.sh` 是唯一实时采集入口；`camera_ws/start.sh` 仅为相机独立部署包保留 SDK bootstrap，打包脚本仍使用它；整个项目初始化统一使用 sh/setup/install.sh。

## 环境与维护

ROS Jazzy、librealsense、兼容库在 vendor，已有安装产物在 install，源码在 third_party/src。迁移机器需恢复匹配架构的运行时；X3 另需用户取得的官方 CameraSDK。bag 模式另需参数指定的 rosbag2 环境，仅播放器子进程加载它。

项目根目录统一安装/检查：

```bash
bash sh/setup/install.sh --camera d455
bash sh/setup/install.sh --camera all --sdk-archive /path/to/CameraSDK.zip
bash sh/setup/install.sh --check-only
```

内部构建/SDK 工具在 `sh/setup/`，部署打包与检查在 `sh/maintenance/`，会话辅助在 `sh/internal/`。SDK 不随普通部署包发布。需要制作相机运行时包时，在本目录运行 `bash sh/maintenance/package_upload.sh`。

## 数据职责

- D455 原始彩色和对齐深度经 C++ 压缩后发布既有 `/compressed` 话题；没有 Python RGB-D 编码器。
- 驱动的实时标定直接 remap 为 `/realsense_camera_info`，DimOS 不重复发布。
- 本体桥负责 world→base_link 和相机安装变换，驱动负责相机内部 TF；bag 只恢复实际录制且选定的数据。
- 原始深度和对齐深度有不同几何；X3 全景是四方向图的前置数据，不作为重复节点删除。X3 IMU 滤波默认关闭。
- metadata 是逐帧采集信息，最小推理不依赖；无订阅者时当前驱动不发送。
- compressed_rgbd_node 与 compressed_rgbd_tx 是同一 C++ 进程的接收和网络发布上下文，不是两套编码器。

真实数据、软件环境和模型通信分别验收，不用话题名或进程存活代替数据到达。
