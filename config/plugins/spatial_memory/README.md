# 空间记忆插件

Mininum 的 Python 入口是 `dimos.plugins.spatial_memory.SimpleMemory`。插件在 Simple 自己的 Python 环境中调用原版 `SpatialMemory.process_stream()` 和 `query_by_text()`，复用采样、CLIP 特征和 ChromaDB 检索。

## 现有输入输出

| 接口 | 契约 |
| --- | --- |
| `record(frame, remaining)` | 向 Simple 采样器提交图像和位姿，返回 `bool`，表示本次是否实际写入；提交不保证入库 |
| `query(target, remaining)` | 接收目标文本，按后端返回顺序给出 `(x, y, yaw, similarity)` 列表 |
| `query_hits(target, remaining)` | 返回 `MemoryHit` 列表，字段为 `memory_id`、`x`、`y`、`yaw`、`similarity`；`memory_id` 可为 `None` |
| `reset()` | 关闭当前 worker、清理临时数据库和待接收数据；下次请求可重新启动 |
| `close()` | 执行上述清理，并关闭记忆图片归档 writer |

`record` 按字段读取输入，不要求实例属于 VLN 的 `Observation` 类。调用方提供：

- `frame.jpeg`：JPEG 字节。
- `frame.odom.pose.position`：具有 `x`、`y`、`z` 的位置。
- `frame.odom.pose.orientation`：具有 `x`、`y`、`z`、`w` 的四元数。
- 可选 `frame.stamp_ns`、`frame.observation_id`：用于诊断关联，不参与后端采样判断。

调用方负责图像与位姿对应、坐标系和会话一致。当前检索结果保留二维位置与朝向，`similarity` 由后端距离换算为 `1 - distance`；插件不据此选择导航目标。字段解析见 [SimpleMemory 实现](../../../dimos/plugins/spatial_memory/__init__.py)。

`remaining()` 由调用方提供，返回任务剩余秒数，并在到期或取消时抛出相应异常。插件在等待和传输期间调用它，同时限制 worker 首次就绪等待为 25 秒、单次操作为 3 秒。缺少 Simple 运行环境、后端失败或 IPC 故障以 `SimpleMemoryUnavailable` 报告；任务期限与取消异常继续向上传递。

## 资源和上层职责

构造 `SimpleMemory` 不启动模型；首次 `record` 或查询才启动自有 worker，并创建独立临时数据库。IPC 阶段发生超时或异常时会清理 worker 与临时库，调用方不能假定失败后重试仍保留旧记忆。插件不重启相机、本体或共享模型服务。

VLN 装配为同一次运行保留一个记忆实例，因此多个任务可以共享该实例的历史；运行结束或 `reset()` 后清除数据库。上层负责检测后提交观测、重复帧过滤，以及世界坐标系改变或时间戳回退时调用 `reset()`。当前上层在 `SimpleMemoryUnavailable` 后停用该运行中的记忆调用；需要继续搜索时沿用既有前沿探索分支。查询时机、候选筛选与回访后的复检也由上层组织，见 [装配](../../../dimos/vln_task/assembly.py) 和 [记忆探索](../../../dimos/vln_task/search/memory_exploration.py)。

## 使用入口

在 Mininum 根目录运行 `bash sh/start_vln_motion.sh --exploration-memory`；该脚本默认已开启记忆探索，显式选项保持兼容。默认使用相邻的 `../dimos_simple`。检出位置不同时，用 `DIMOS_SIMPLE_ROOT` 指定 Simple 根目录。Simple 需要自己的 `.venv` 和本地 CLIP 资源，首次使用时加载模型。

当前画面未发现目标时，上层查询历史视角，候选经净空检查后交给 XNavDP 与 tracker，到达后停稳复检。没有可用候选时继续前沿探索。

使用 `bash sh/run_rviz_mem.sh` 打开探索视图。RViz 配置见 [exploration_memory.rviz](exploration_memory.rviz)。
