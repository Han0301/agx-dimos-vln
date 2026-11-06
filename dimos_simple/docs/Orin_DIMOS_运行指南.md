# 运行指南与快速调试

## 启动命令

```bash
# 终端1: 启动两个相机
'/home/deepvln/insta360_ws/start_cameras.sh'

# 终端2: 独立记录多方面日志
'/home/deepvln/insta360_ws/start_debug_logging.sh'

# 终端3：启动dimos
# 在本工作空间根目录执行
source ./sh/activate_vln.sh

dimos --robot-ip 192.168.123.161 --navigation-policy xnavdp run unitree-go2-vln-local
```

网页交互打开

```text
http://127.0.0.1:5556
```

测试用例：

```textile
find a fire extinguisher box and walk up to it until you are about 0.8 meters away
```

xnavdp测试：一直发布前方3m的目标点，测试其是否能脱困

```bash
# 在本工作空间根目录执行
source ./sh/activate_vln.sh
python /home/deepvln/Documents/Codex/2026-11-01/new-chat-2/xnavdp_goal_3m_test.py
```



## 一、当前配置

- Orin 地址：`192.168.10.110`
- Go2 地址：`192.168.10.78`（无线），`192.168.123.161`有线）
- 5090服务器：`10.10.10.56`
- Ollama：`10.10.10.56:11434`
- Qwen3：`10.10.10.56:8001`
- NavDP：`10.10.10.56:8888`
- RealSense：D455
- RealSense 分辨率：`848 x 480`
- DIMOS 配置文件：`$DIMOS_ROOT/dimos/agents/skills/config/vln_config.yaml`
