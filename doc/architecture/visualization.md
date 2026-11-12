# 可选 XNavDP RViz

入口：`bash sh/run_rviz.sh`。仅显式启动时加载 RViz 与可视化辅助节点，不进入本体、相机、回放或推理默认启动。

参考源为 XNavDP/sh/run_rviz.sh 及 config/xnavdp.rviz。背景、Fixed Frame=base_link、网格、坐标轴、轨迹颜色与宽度、点云、彩色图、Orbit 视角和窗口尺寸沿用原配置。

服务端与最小版统一使用 /xnavdp/trajectory 和 /xnavdp/result；xnavdp.rviz 与参考配置保持一致。/result 是原生 JSON，只有独立可视化辅助将其画成 /xnavdp/result_markers；该显示话题不用于推理、控制或默认录制。候选、评分、目标的颜色、线宽和排列沿用原实现，耗时文字使用真实 roundtrip_seconds。
- 主配置：config/plugins/xnavdp/xnavdp.rviz。
- 运行环境：同目录 rviz.yaml，显式配置 ros_setup；RViz 在独立 ROS 环境运行，不污染基座或相机环境。
- 深度点云辅助沿用参考投影实现，订阅原始对齐深度和唯一 /realsense_camera_info，输出仅供显示的 /xnavdp/depth/points。
- TF 辅助先等待已有 TF；仅缺失时按 base.yaml 安装外参补齐相机变换，并按 /odom 补齐世界到本体变换。没有真实 odom 时不猜测世界位姿。已有 TF 不重复发布；来源切换或机器人 TF 后续接入时重新启动可视化。
- 原始 RGB、深度、标定用于图像和点云显示。只有压缩图的 bag 不会因此生成另一套原始图；仍可查看原生模型路径与候选。

默认启动时检测 /clock；先启动 bag 回放，再启动 RViz。需要提前启动用 --time-source sim，实时用 --time-source wall。参数检查用 --dry-run，不创建 ROS 节点或窗口。

```bash
bash sh/run_rviz.sh
bash sh/run_rviz.sh --time-source sim
```

退出只清理本次可视化创建的辅助进程与 RViz，不停止数据源、压缩器、推理或服务端，不 reset 模型，不发送运动指令。数据录制与 RViz 独立；不因开启显示就追加录制话题。

## 自由探索记忆视图

在项目根目录从独立终端启动 `bash sh/run_rviz_mem.sh`。可加 `--time-source wall` 或 `--time-source sim` 显式选择时钟，`--dry-run` 只打印命令。使用与 VLN 相同的 ROS domain 和时钟，Fixed Frame 按实际里程计坐标系设置，默认 `world`。

- `/vln/exploration/map` 显示上层选点使用的占据图。
- `/vln/exploration/markers` 显示当前位姿、已检测朝向、选中前沿、短程航点和冷却中的前沿。搜索结束或取消后清除叠加层。
- XNavDP 的原生路径、结果标记、RGB 和深度点云仍可在此视图查看。

探索数据由 `bash sh/start_vln_motion.sh --exploration-memory` 的找物任务发布。运行入口与日志对照见 [自由探索测试](../skills/vln-operations/SKILL.md#自由探索测试)。
