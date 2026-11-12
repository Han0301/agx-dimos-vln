# 参数维护约定

- [base.yaml](base.yaml) 启动本体通信并默认关闭运动，[base_motion.yaml](base_motion.yaml) 用于显式启用 Go2 运动，[empty.yaml](empty.yaml) 用于无设备生命周期检查。当前 schema 为 version 2，模块开关只有 `body`。
- 基座不托管算法子进程、不解码或转发图像；插件在 `plugins/<name>/` 维护配置并独立启动。
- 旧 version 1 配置需要以当前 base.yaml 为模板迁移：删除 algorithms、rgbd_bridge、camera.depth_scale 与图像/标定话题字段。保留 body、ROS、安装外参和本体话题的实际参数；加载器不会静默忽略旧字段。
- `topics` 配置本体使用的 odom、lidar、cmd_vel、tf、lowstate、imu。`/odom` 为 PoseStamped，`/imu` 为 Imu，姿态取自 Go2 lowstate 的 RPY。旧配置缺少 `topics.imu` 时默认使用 `/imu`。
- 相机尺寸和采集参数在 camera_ws/config，标定由相机驱动直接发布 `/realsense_camera_info`；bag 使用录制的同一接口。
- `camera.translation/pitch/mount_frame` 仅描述本体到相机根的安装变换；本体负责 world→base_link→camera_link，驱动负责相机内部 TF。
- DDS 共用 sh/xnavdp_ros2；接口契约看 [interfaces.yaml](interfaces.yaml)，不建立中间话题。
- 独立插件目录、入口和参数见 [插件索引](plugins/README.md)，新增插件不修改基座运行时。
- [vln_preview.yaml](profiles/vln_preview.yaml) 装配 Web/CLI 预览，[vln_motion.yaml](profiles/vln_motion.yaml) 在 Web 进程内装配持续规划与 tracker。
- 录制/回放共用 bag.yaml；录制输出固定为 data/bags/record_<时间>_<唯一后缀>/，仅记录指定话题。播放只生成 bag 话题与 /clock，不管理压缩器或模型。
