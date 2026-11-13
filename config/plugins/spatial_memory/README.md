# 空间记忆插件

Mininum 的 Python 入口是 `dimos.plugins.spatial_memory.SimpleMemory`。插件在 Simple 自己的 Python 环境中调用原版 `SpatialMemory.process_stream()` 和 `query_by_text()`，复用 CLIP 特征提取、向量入库和 ChromaDB 检索。首帧可直接入库。后续帧相对上次入库须间隔至少 1 秒，并满足位置变化至少 5 cm 或航向变化超过 15°。

- `record(jpeg, pose, remaining, ...)` 接收 JPEG 字节、XYZ/XYZW 位姿和调用预算。调用方提供距离、角度和间隔阈值，可传采样组、采样时间戳和观测 ID。返回本次是否写入。无采样组时按所有成功写入的三维位姿和单调时钟去重。有采样组时按组内平面位姿和输入时间戳去重。
- `query(target, remaining)` 返回按相似度排列的历史拍摄位置和朝向 `(x, y, yaw, similarity)`。
- `reset()` 清除当前里程计会话的记忆，`close()` 关闭子进程。记忆在同一次 VLN 运行中的任务之间共享，使用独立临时数据库。

在 Mininum 根目录运行 `bash sh/start_vln_motion.sh --exploration-memory`。默认使用相邻的 `../dimos_simple`。检出位置不同时，用 `DIMOS_SIMPLE_ROOT` 指定 Simple 根目录。Simple 需要自己的 `.venv` 和本地 CLIP 资源，首次使用时加载模型。

CLIP 默认使用 TensorRT FP32。Orin 需要可访问的 CUDA GPU、CUDA runtime、TensorRT Python 绑定（支持系统安装）和首次构建所需的 `trtexec`。引擎按模型文件和 TensorRT 版本缓存在 `${XDG_CACHE_HOME:-~/.cache}/dimos-mininum/spatial-memory/`，首次启动可能需要额外的构建时间。GPU 初始化或推理报错时，worker 切换到 ONNX Runtime CPU 并重试本次推理。设置 `DIMOS_MEMORY_BACKEND=cpu` 可直接选择 CPU。开启运行日志时，worker 就绪事件记录初始后端，回退原因写入 worker 的 stderr 日志。

当前画面未发现目标时，上层查询历史视角，候选按相似度和回访记录筛选，历史点交给 XNavDP 规划，tracker 在发速度前检查碰撞，到达后停稳复检。没有可用候选时继续前沿探索。

使用 `bash sh/run_rviz_mem.sh` 打开探索视图。RViz 配置见 [exploration_memory.rviz](exploration_memory.rviz)。
