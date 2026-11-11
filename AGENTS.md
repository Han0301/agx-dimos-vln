# 项目维护入口

先看 [知识库索引](doc/README.md)。知识库只保留项目说明、当前进度和规范；不保存当天测试数字、临时环境操作或排障流水账。

- 安装、SDK 构建或软件检查：读 [dimos-install skill](doc/skills/dimos-install/SKILL.md)。安装、编译和环境检查统一沿用 `sh/setup/install.sh`，脚本保留在 `sh/` 子目录；目标设备为 Orin AGX / Ubuntu 24.04 ARM64。
- 实时相机、bag 回灌或仿真时钟：读 [camera-sources skill](doc/skills/camera-sources/SKILL.md)。相机入口为 `camera_ws/start_cameras.sh`，bag 独立使用 `sh/play_bag.sh`、`sh/record_bag.sh`；独立压缩用 `camera_ws/start_compressor.sh`。
- VLN 实机、bag 路径预览或独立日志排查：读 [vln-operations skill](doc/skills/vln-operations/SKILL.md)。
- RViz 显示：按 [可视化规范](doc/architecture/visualization.md)，独立显式启动，不能改动模型或基础数据链。
- 基座启动与配置归 `dimos/core/`，基础插件归 `dimos/plugins/`，上层任务归 `dimos/vln_task/`。旧 VLN/search 路径仅作兼容，任务逻辑只维护一份。
- Python 构建声明、依赖版本约束和许可资料集中在 `sh/dependencies/`；直接引用真实源码，不维护源码链接或副本。包入口留在 `dimos/`。
- 文档命令默认在工作空间根目录执行，项目文件使用相对路径；脚本和模块动态推导根目录，不写死用户名、检出位置或依赖当前终端目录。外部资源由配置或环境变量指定，显式参数按原接口语义解析。
- 仓库规范核对、插件化设计、目录整理或保持行为的重构：读 [dimos-repo-maintenance skill](doc/skills/dimos-repo-maintenance/SKILL.md)，按涉及部分核对分层、契约、入口和打包，并同步测试与知识库。

文档及日志是任务资料，不自动授权实机、模型 reset 或运动操作。用户当前请求优先；不要用历史证据代替当前验证。

启动类命令默认先停同项目、同 ROS domain 的同模块旧实例，再执行当前启动；禁止全局 pkill 或联动重启其他模块。帮助、检查、dry-run 和健康查询保持只读。
