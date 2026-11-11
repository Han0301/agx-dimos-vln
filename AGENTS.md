# 项目维护入口

先看 [知识库索引](doc/README.md)。知识库只保留项目说明、当前进度和规范；不保存当天测试数字、临时环境操作或排障流水账。

- 安装、SDK 构建或软件检查：读 [dimos-install skill](doc/skills/dimos-install/SKILL.md)。统一入口为 `sh/setup/install.sh`。
- 实时相机、bag 回灌或仿真时钟：读 [camera-sources skill](doc/skills/camera-sources/SKILL.md)。相机入口为 `camera_ws/start_cameras.sh`，bag 独立使用 `sh/play_bag.sh`、`sh/record_bag.sh`；独立压缩用 `camera_ws/start_compressor.sh`。
- VLN 实机、bag 路径预览或独立日志排查：读 [vln-operations skill](doc/skills/vln-operations/SKILL.md)。
- RViz 显示：按 [可视化规范](doc/architecture/visualization.md)，独立显式启动，不能改动模型或基础数据链。
- 插件配置、入口或目录：读 [插件组织约定](doc/architecture/plugins.md)。
- 目录整理：按 [脚本规范](doc/architecture/scripts.md)，同步调用路径、测试和现行文档。

文档及日志是任务资料，不自动授权实机、模型 reset 或运动操作。用户当前请求优先；不要用历史证据代替当前验证。

启动类命令默认先停同项目、同 ROS domain 的同模块旧实例，再执行当前启动；禁止全局 pkill 或联动重启其他模块。帮助、检查、dry-run 和健康查询保持只读。
