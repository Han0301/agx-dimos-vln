# 空间记忆插件

Mininum 的 Python 入口是 `dimos.plugins.spatial_memory.SimpleMemory`。插件在 Simple 自己的 Python 环境中调用原版 `SpatialMemory.process_stream()` 和 `query_by_text()`，复用采样、CLIP 特征和 ChromaDB 检索。

- `record(frame, remaining)` 接收含 JPEG 和里程计位姿的观测，返回本次是否写入。
- `query(target, remaining)` 返回按相似度排列的历史拍摄位置和朝向 `(x, y, yaw, similarity)`。
- `reset()` 清除当前里程计会话的记忆，`close()` 关闭子进程。记忆在同一次 VLN 运行中的任务之间共享，使用独立临时数据库。

在 Mininum 根目录运行 `bash sh/start_vln_motion.sh --exploration-memory`。默认使用相邻的 `../dimos_simple`。检出位置不同时，用 `DIMOS_SIMPLE_ROOT` 指定 Simple 根目录。Simple 需要自己的 `.venv` 和本地 CLIP 资源，首次使用时加载模型。

当前画面未发现目标时，上层查询历史视角，候选经净空检查后交给 XNavDP 与 tracker，到达后停稳复检。没有可用候选时继续前沿探索。

使用 `bash sh/run_rviz_mem.sh` 打开探索视图。RViz 配置见 [exploration_memory.rviz](exploration_memory.rviz)。
