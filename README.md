# -# rmtc_bot 参赛程序运行说明（供比赛主办方工作人员）

- 程序名称：rmtc_bot（ROS 2 节点：rmtc_bot_node）
- 编程语言：C++17
- 适用比赛：RMTC 第一阶段（单人采集）与第二阶段（四方对抗）

---

## 1. 目录结构

```
contestant_ws/
├── run_contestant.sh                  # 一键启动脚本（推荐使用）
├── maps/                              # 本地测试用地图（地图编辑器导出格式）
│   ├── stage1-20x20.json
│   └── stage2-24x24.json
├── tools/                             # 本地测试工具（仅调试用，非参赛程序）
│   ├── mock_referee.py                # 本地模拟裁判
│   ├── visualizer.py                  # 比赛实时可视化
│   └── check_map.py                   # 地图文件校验
└── src/rmtc_bot/                      # 参赛程序包
    ├── CMakeLists.txt
    ├── package.xml
    ├── config/
    │   └── decision_config.json       # 现场唯一允许修改的配置
    ├── launch/
    │   └── rmtc_bot.launch.py         # 备选启动方式（ros2 launch）
    ├── include/rmtc_bot/              # 头文件
    │   ├── perception.hpp             # 感知数据结构
    │   ├── world_memory.hpp           # 地图记忆
    │   ├── planner.hpp                # BFS 寻路
    │   ├── strategy.hpp               # 决策策略
    │   ├── llm_advisor.hpp            # AI 网关调用
    │   ├── http_client.hpp            # 自带 HTTP 客户端
    │   └── json_lite.hpp              # 自带 JSON 解析
    └── src/
        ├── main.cpp                   # ROS 节点主程序
        ├── perception.cpp
        ├── world_memory.cpp
        ├── planner.cpp
        ├── strategy.cpp
        ├── llm_advisor.cpp
        └── http_client.cpp
```

## 2. 运行环境与所需依赖

| 项目 | 要求 |
| --- | --- |
| 操作系统 | Ubuntu 22.04（x86_64） |
| ROS 2 | ROS 2 Humble（`/opt/ros/humble`） |
| ROS 包依赖 | rclcpp、std_msgs（均为 Humble 自带） |
| 编译工具 | gcc/g++ 11、CMake、colcon（随 ROS 2 安装） |
| 第三方 C++ 库 | **无**。JSON 解析与 HTTP 客户端均为程序自带实现，构建全程不需要联网 |
| 网络 | 与裁判端组播互通（ROS_DOMAIN_ID 一致）；与 AI 网关网络可达 |

> `tools/` 下的 Python 测试工具依赖 python3 与 rclpy（Humble 自带），仅用于本地调试，比赛现场不运行。

## 3. 构建方法

将 `contestant_ws` 目录整体拷贝到虚拟机后，执行：

```bash
cd ~/contestant_ws
chmod +x run_contestant.sh        # 从 Windows 复制后需恢复可执行位
colcon build --symlink-install
source install/setup.bash
```

编译成功输出 `Summary: 1 package finished`。用 `--symlink-install` 编译后，修改 `config/decision_config.json` 无需重新编译。

## 4. 启动目录与完整启动命令

启动目录：`~/contestant_ws`（脚本会自行定位工作空间并 source ROS 环境，从任意目录执行亦可）。

```bash
cd ~/contestant_ws
./run_contestant.sh A "我的队伍" http://127.0.0.1:8080
```

参数：第 1 个为席位号（A/B/C/D），第 2 个为注册队伍名，第 3 个为 AI 网关地址。

等价备选方式（ros2 launch）：

```bash
ros2 launch rmtc_bot rmtc_bot.launch.py team:=A name:=我的队伍 gateway:=http://127.0.0.1:8080 token:=practice-A
```

启动行为：程序启动后立即发布注册信息，之后每 2 秒重发一次（应对裁判端晚启动）；启动时对网关执行一次 `GET /healthz` 健康检查（仅打印状态，不影响启动）。建议先启动裁判端，再启动选手程序。

## 5. 第一、第二阶段启动与切换

程序**默认自动识别阶段（--stage 0），无需人工切换**，识别规则：

- 地图尺寸 ≤ 20×20 且视野内没有敌方机器人 → 第一阶段；
- 视野内出现敌人，或地图尺寸 > 20×20 → 第二阶段；
- 本场（同一 session_id）内阶段只升不降；换场后重新识别。

第一阶段行为约束：只提交 `move` / `wait` 动作（即使策略计算出攻击也会被程序拦截）。第二阶段解锁 `attack` / `defend` 并启用 AI 网关调用。

如需强制指定阶段（调试用），可直接运行节点：

```bash
source install/setup.bash
ros2 run rmtc_bot rmtc_bot_node --team A --stage 1     # 强制第一阶段
ros2 run rmtc_bot rmtc_bot_node --team A --stage 2     # 强制第二阶段
```

## 6. 席位、ROS 域、AI 网关地址与令牌配置

### 6.1 席位

席位号是启动脚本的第 1 个参数（A/B/C/D），决定本席位使用的全部话题：

- 订阅：`/team_<席位>/perception`
- 发布：`/team_<席位>/action`、`/team_<席位>/registration`
- 另订阅官方公共话题：`/referee/mission`、`/referee/event_text`、`/referee/team_registry`

每个席位一个进程，各占一个终端。程序只订阅本席位与官方公共话题，不读取其他席位感知。

### 6.2 ROS 域与中间件

启动脚本自动设置，可被环境变量覆盖：

| 环境变量 | 默认值 | 说明 |
| --- | --- | --- |
| `ROS_DOMAIN_ID` | 27 | 必须与裁判端一致 |
| `ROS_LOCALHOST_ONLY` | 0（强制） | 跨机通信必需 |
| `RMW_IMPLEMENTATION` | rmw_fastrtps_cpp | 与裁判端保持一致 |

### 6.3 AI 网关地址

启动脚本第 3 个参数，或用环境变量 `RMTC_AI_URL` 覆盖，默认 `http://127.0.0.1:8080`。正式比赛使用裁判提供的网关地址。

### 6.4 令牌

环境变量 `RMTC_TEAM_TOKEN`，默认 `practice-<席位>`（练习）。正式比赛前：

```bash
export RMTC_TEAM_TOKEN=正式令牌
./run_contestant.sh B "队伍名" http://正式网关地址
```

令牌仅用于 AI 网关请求鉴权（`Authorization: Bearer`），不写入任何仓库文件。

### 6.5 现场唯一可修改配置

`src/rmtc_bot/config/decision_config.json` 中的 `decision_type`，可选 `collect` / `balanced` / `aggressive`（当前为 `balanced`）。其余代码按规则封存不改动。

## 7. 程序主要模块与运行逻辑

### 7.1 模块划分

| 模块 | 文件 | 职责 |
| --- | --- | --- |
| ROS 主节点 | src/main.cpp | 话题收发、注册、tick 驱动、阶段识别、动作合规拦截 |
| 感知解析 | perception.cpp | 解析感知 JSON，解码 9×9 相对坐标网格 |
| 世界记忆 | world_memory.cpp | 维护地图尺寸、障碍、资源、敌人与撞墙记忆 |
| 寻路 | planner.cpp | BFS 最短路径，输出第一步方向 |
| 决策策略 | strategy.cpp | 决策链，产出每个 Tick 的动作 |
| AI 顾问 | llm_advisor.cpp | 后台线程调用官方 AI 网关，解读任务/随机事件 |
| 基础库 | json_lite.hpp / http_client.cpp | 自带 JSON 解析与 POSIX socket HTTP 客户端 |

### 7.2 主循环运行逻辑

```
启动
 ├─ 发布注册信息（每 2 秒重发）
 ├─ GET /healthz 网关健康检查（仅打印）
 └─ 循环：
     收到感知 → session_id 变化？→ 清空地图记忆与状态，重新识别阶段
             → tick 回退？→ 丢弃（过期感知不用旧 Tick 提交）
             → 同一 tick 已决策？→ 重发缓存动作，不重复决策
             → 解析感知，更新地图记忆
             → Strategy::decide() 按决策链产出动作
             → 第一阶段强制只允许 move / wait
             → 发布 /team_<席位>/action（原样回填 tick 与 session_id）
```

### 7.3 决策链（优先级递减）

1. 战斗（仅第二阶段）：同排同列距离 1~2 格——能击杀则攻击；预计被打死但防御后能活则防御；残血（HP≤20）且敌人在 2 格内则撤退；
2. AI 建议目标：采用 LLM 返回的 go 坐标，avoid 区域加入寻路排除；
3. 任务坐标：比赛后段前往 reach 类任务点；
4. 采集资源：按"距离 − 高价值奖励"排序拾取；
5. 主动接敌（仅第二阶段）：血量领先 20 以上且距离在追击范围内时靠近；
6. 探索：BFS 找最近未去过的可达格；
7. 等待：以上均不可行时 wait。

### 7.4 AI 网关调用要点

- 请求：`POST /v1/chat/completions`（OpenAI 兼容格式），`Authorization: Bearer <令牌>`；
- 模型 `deepseek-flash`；显式关闭思考（`"thinking":{"type":"disabled"}`）；`response_format` 为 json_object；`max_tokens` 512；
- 同一时刻最多 1 个在途请求；常规请求最小间隔 12 秒；收到新任务/事件时立即请求；
- 结果按场次与 Tick 校验，越界/非法建议全部丢弃，超过 60 Tick 自动失效；
- 网关不可用、超时或额度耗尽（429）时自动降级为本地策略，比赛照常进行；
- 第二阶段程序会持续重试直到至少一次 AI 调用成功（规则要求）。

### 7.5 协议合规要点

- 每个 Tick 最多提交一个动作，动作 JSON 原样回填最新感知的 tick 与 session_id；
- 同一 tick 收到重复感知不重复决策，重发上一条动作；
- 检测到 session_id 变化视为新场次，全部状态重置；
- 第一阶段只提交 move / wait；
- 受保护状态下不发起攻击。
