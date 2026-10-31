# RMTC 参赛程序使用手册（rmtc_bot）

这是一个可以直接编译运行的 ROS 2 Humble 参赛程序，覆盖两个阶段：

- 第一阶段（单人采集）：只用 `move` / `wait`，优先捡资源、探索未知区域。
- 第二阶段（四方对抗）：按“能击杀就攻击、会被打死就防御、残血拉开距离”的优先级行动，并接入官方 AI 网关解读自然语言任务/随机事件。

程序不依赖任何第三方 C++ 库（JSON 解析和 HTTP 客户端都是自带的），所以 `colcon build` 不需要联网装包。

> 重要：代码是在 Windows 上写的，核心逻辑已用离线自测验证过，但**还没有在 ROS 2 环境里编译过**。拿到虚拟机后的第一件事就是 `colcon build`，见下文。

---

## 1. 目录结构

```
contestant_ws/
├── README.md                      # 本文件
├── run_contestant.sh              # 一键启动脚本（推荐用它启动）
├── tools/
│   ├── mock_referee.py            # 本地模拟裁判，无官方环境也能自测
│   └── visualizer.py              # 比赛实时可视化（图形窗口，无依赖可退回终端字符画）
└── src/rmtc_bot/
    ├── CMakeLists.txt
    ├── package.xml
    ├── config/
    │   └── decision_config.json   # 现场唯一允许改的配置（decision_type）
    ├── launch/
    │   └── rmtc_bot.launch.py
    ├── include/rmtc_bot/
    │   ├── json_lite.hpp          # 自带 JSON 解析/生成
    │   ├── http_client.hpp        # 自带 HTTP 客户端（POSIX socket）
    │   ├── perception.hpp         # 感知 JSON 的数据结构
    │   ├── world_memory.hpp       # 地图记忆（障碍/资源/敌人）
    │   ├── planner.hpp            # BFS 寻路
    │   ├── strategy.hpp           # 决策逻辑
    │   └── llm_advisor.hpp        # AI 网关调用
    └── src/
        ├── main.cpp               # ROS 2 节点：话题收发 + 主循环
        ├── http_client.cpp
        ├── perception.cpp         # 感知解析 + 9x9 相对坐标解码
        ├── world_memory.cpp
        ├── planner.cpp
        ├── strategy.cpp           # 策略主体（想改行为主要改这里）
        └── llm_advisor.cpp        # 后台线程调用 LLM，不阻塞主循环
```

---

## 2. 最快跑通（虚拟机内）

### 2.1 检查环境

```bash
source /opt/ros/humble/setup.bash
ros2 --version
```

能打印版本号就说明 ROS 2 正常。没有的话先按课程/官方教程装好 Humble。

### 2.2 编译

把 `contestant_ws` 整个目录拷进虚拟机（例如 `~/contestant_ws`），然后：

```bash
cd ~/contestant_ws
chmod +x run_contestant.sh   # 从 Windows 复制过来后，可执行位可能丢失
colcon build --symlink-install
source install/setup.bash
```

> 建议先复制到 Ubuntu 家目录再编译。直接在共享目录（如 `/mnt/d/...`）里 `colcon build` 通常也能成功，但速度慢，且 `--symlink-install` 的软链接在共享盘上偶发异常。

编译成功会看到 `Summary: 1 package finished`。如果报错，先看第 8 节排错表。

### 2.3 不开官方环境，先用模拟裁判自测

打开**两个终端**，都先执行：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=27
export ROS_LOCALHOST_ONLY=0
```

终端 1（模拟裁判，第一阶段）：

```bash
cd ~/contestant_ws
python3 tools/mock_referee.py --stage 1 --ticks 100 --tick-ms 100
```

终端 2（你的程序）：

```bash
cd ~/contestant_ws
./run_contestant.sh A "测试队" http://127.0.0.1:8080
```

模拟裁判会打印每 20 tick 的状态和最终得分，程序会打印每个 tick 的决策。看到 `tick=... pos=[x,y] ...` 并且裁判给出分数就说明链路打通了。

第二阶段自测（4 个机器人，建议加快 tick）：

```bash
# 终端 1
python3 tools/mock_referee.py --stage 2 --ticks 500 --tick-ms 100 --seed 7

# 终端 2
./run_contestant.sh A "测试队" http://127.0.0.1:8080
```

想测试随机事件传给 AI 的效果，可以加 `--event "北方出现补给，采集效率翻倍"`。

#### 实时可视化（可选）

再开一个终端（同样先 source ROS 并设好环境变量），就能实时看到比赛过程：

```bash
python3 tools/visualizer.py --map maps/stage2-24x24.json
```

- 有 matplotlib（`pip install matplotlib`）就弹图形窗口：灰块=障碍、黄点=普通资源、绿点=高级资源、字母=各队机器人（带 HP，圆圈=防守中），标题栏显示 tick。
- 没有 matplotlib 会自动退回终端字符画，零依赖。
- `--map` 预载本地地图文件能看到全图；不加 `--map` 也能用（只显示各队已探明的部分，配合官方练习裁判时只能这样）。
- 双人对战时它同时订阅 A/B 的感知，两个机器人都会显示。

#### 用地图编辑器画的地图来跑（推荐）

`maps/` 目录下有两张用官方《地图编辑器》格式生成的地图：

- `maps/stage1-20x20.json` —— 第一阶段单人图（20×20）
- `maps/stage2-24x24.json` —— 第二阶段对战图（24×24，A/B/C/D 四角出生）

```bash
# 终端 1（第二阶段，用地图编辑器导出的地图）
python3 tools/mock_referee.py --stage 2 --map maps/stage2-24x24.json
```

`--map` 直接吃地图编辑器「下载 JSON」导出的文件（geometry 或 practice 格式都行）。想自己画图：双击 `地图编辑器.html` → 画好 → 下载 JSON 放进 `maps/` → 先跑 `python3 tools/check_map.py maps/你的地图.json` 校验（越界/重叠/资源数量/连通性/中心任务点），再用 `--map` 启动。

#### 双人模拟对战（PvP）

裁判支持多个真人席位：默认 A 是真人，加 `--human B`（可重复）把其他席位也交给真人控制，剩余席位仍由内置陪练 AI 顶替。协议与正式比赛一致：每个席位只能收到自己的 `/team_X/perception`，动作只能发到自己的 `/team_X/action`，裁判同时裁决双方。

同一台虚拟机（先跑通，最简单），开三个终端：

```bash
# 终端 1（裁判）
python3 tools/mock_referee.py --stage 2 --map maps/stage2-24x24.json --human B

# 终端 2（你的 A 队）
./run_contestant.sh A "测试队" http://127.0.0.1:8080

# 终端 3（对手的 B 队）
./run_contestant.sh B "朋友队" http://127.0.0.1:8080
```

两台机器联机：两台虚拟机都改成桥接网络（同一局域网），`ROS_DOMAIN_ID=27`、`ROS_LOCALHOST_ONLY=0`（run_contestant.sh 已自动设置）。先各自跑 `ros2 multicast send` / `ros2 multicast receive` 验证组播发现，再 `ros2 topic list` 能看到对方席位的话题。然后一方机器跑裁判+自己的席位，另一方只跑自己的席位。裁判会打印双方每 20 tick 的战况和最终比分。

> 模拟裁判只验证“协议/移动/采集/战斗/得分”这条链路，不需要真实 AI 网关。网关不通时程序会自动退化为纯本地策略，不会崩。

---

## 3. 接官方裁判测试端

按官方《裁判测试端使用教程》启动测试端（通常是 `start_reference.sh`），然后：

```bash
# 每个席位一个终端；token 用官方练习令牌
./run_contestant.sh A "我的队伍" http://127.0.0.1:8080
```

- `A` 是席位号，正式比赛按裁判分配的席位改。
- 默认 token 是 `practice-A`（即 `practice-` + 席位），正式 token 用环境变量覆盖：
  ```bash
  export RMTC_TEAM_TOKEN=正式令牌
  ./run_contestant.sh A "我的队伍"
  ```
- 网关地址默认 `http://127.0.0.1:8080`，正式地址若不同，用第三个参数或 `RMTC_AI_URL` 覆盖。
- 启动时会自动请求 `GET /healthz` 做健康检查，并打印结果。

也可以用 launch 启动：

```bash
ros2 launch rmtc_bot rmtc_bot.launch.py team:=A name:=我的队伍 gateway:=http://127.0.0.1:8080 token:=practice-A
```

程序行为要点：

- 启动后立即发布注册信息，之后每 2 秒重发一次（应对裁判端晚启动）。
- 每个 Tick 只发一个动作，`tick` 和 `session_id` 原样回显最新感知。
- 同一 tick 重复收到感知时，不重复决策，直接重发上一条动作。
- 检测到 `session_id` 变化视为新场次，地图记忆和状态全部重置。
- 第一阶段强制只允许 `move` / `wait`，即使策略算出攻击也会被拦下。

---

## 4. 启动参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--team` / `-t` | `A` | 席位号，决定话题 `/team_A/...` |
| `--name` / `-n` | `我的队伍` | 注册显示名称 |
| `--gateway` / `-g` | `http://127.0.0.1:8080` | 统一 AI 网关地址 |
| `--token` | 环境变量 `RMTC_TEAM_TOKEN` | 席位令牌 |
| `--model` | `deepseek-flash` | 本届统一模型，一般不用改 |
| `--stage` | `0` | `0`=自动：小地图（≤20）且视野内无敌人判为第一阶段，否则第二阶段；本场只升不降 |
| `--decision-config` | 无 | `decision_config.json` 路径 |
| `--min-interval-ms` | `12000` | 两次常规 AI 请求的最小间隔 |
| `--max-tokens` | `512` | 与官方请求示例一致，够放一段 JSON 建议 |
| `--no-ai` | 关 | 完全关闭 AI，只用本地策略（调试用） |
| `--verbose` | 关 | 每个 Tick 都打印决策（默认每 10 tick 打一次） |

环境变量：

- `ROS_DOMAIN_ID`：默认脚本设为 `27`，必须和裁判端一致。
- `ROS_LOCALHOST_ONLY`：必须为 `0`，否则跨进程/跨机收不到消息。
- `RMW_IMPLEMENTATION`：默认 `rmw_fastrtps_cpp`，与裁判端保持一致。
- `RMTC_TEAM_TOKEN`：席位令牌。
- `RMTC_MODEL`：模型名，默认 `deepseek-flash`。
- `RMTC_MAX_TOKENS`：输出上限，默认 512。

---

## 5. 策略说明（想改行为看这里）

决策入口是 `Strategy::decide()`，按以下优先级依次尝试，前一条能产出动作就直接返回：

1. **战斗判断（仅第二阶段）**
   - 找出同一行/列、距离 1~2 格、血量最低的敌人。
   - 自己处于受保护状态时不能攻击，会跳过。
   - 预计挨打会死、但防御后能活（`hp <= 来袭伤害` 且 `hp > 伤害一半`）时 **防御**。
   - 能击杀（或只是能打到）且不致命时 **攻击**。
   - 自己血量 ≤20 且敌人在 2 格内时，往反方向 **撤退**。
2. **AI 建议目标**：LLM 返回 `go` 坐标就过去；`hold` 模式到达后原地等待。
3. **任务坐标**：解析出 `reach` 类坐标后，在阶段后 3/5 的时间去踩点。
4. **采集资源**：按“距离 - 高价值奖励”排序，去最近且 150 tick 内见过的资源。
5. **主动接敌（仅第二阶段）**：有把握（血量领先 20 以上）且距离在追击范围内时靠近敌人。
6. **探索**：BFS 找最近的、没去过且可达的格子。
7. **等待**：都做不到就 `wait`，保证每 tick 都有合法动作。

寻路细节：

- 每 tick 做两次 BFS：一次排除障碍和“上一步撞墙”的格子，一次再排除 AI 建议的 `avoid` 区域；第一次失败自动用第二次。
- 敌人的格子本身不可通行，但“走到敌人旁边”通过 `adjacent_target()` 单独处理。
- 撞墙或移动失败的格子会被记住 8 tick，避免反复撞。

可调参数（都在 `src/strategy.cpp` 顶部或对应 LOGIC 段）：

| 想调整 | 改哪里 |
| --- | --- |
| 整体风格 | `config/decision_config.json` 的 `decision_type`：`collect` / `balanced` / `aggressive` |
| 追击距离 | LOGIC-5 的 `chase_range`（aggressive 5，balanced 2，fight 模式 6） |
| 资源新鲜度 | LOGIC-4 的 `150` tick |
| 撤退血量阈值 | LOGIC-1 的 `p.self.hp <= 20` |
| AI 请求频率 | `--min-interval-ms`，默认 12 秒 |
| AI 提示词 | `src/llm_advisor.cpp` 的 `request_once()` / system prompt |

`decision_type` 是现场唯一允许修改的配置项。用 `--symlink-install` 编译时，直接改 `src/rmtc_bot/config/decision_config.json` 就生效；否则改完后重新 `colcon build`。改动后按裁判要求登记。

---

## 6. AI 网关接入

- 启动时请求 `GET /healthz`，只打印状态，不影响启动。
- 后台线程调用 `POST /v1/chat/completions`，请求体是 OpenAI 兼容格式，带 `Authorization: Bearer <token>`。
- 提示词里包含地图尺寸、自身坐标/血量、比分、任务文本、事件文本、敌人和资源概况，要求模型只返回 JSON：
  ```json
  {"mode":"collect|reach|fight|hold|survive","go":[x,y],"avoid":[[x1,y1,x2,y2]],"note":"..."}
  ```
- 返回内容会被严格校验：越界坐标、非法 mode、格式错误的矩形全部丢弃，坏建议不会导致崩溃。
- 后台结果带场次和 Tick 标记：换场后旧结果直接丢弃，超过 60 Tick 的建议自动失效，符合协议“异步结果不得跨场次/跨 Tick 使用”的要求。
- 收到新的任务公告或随机事件时，会立即强制请求一次，不受 12 秒间隔限制。
- 第二阶段开始时，如果还没有一次成功调用，会持续重试直到成功（避免 `M=0`）。
- 第一阶段不调用 AI，把额度留给第二阶段；任务/事件文本仍会缓存，进入第二阶段后一起发送。
- 所有网络调用都是非阻塞的：AI 超时或失败时，主循环照常按本地策略行动。
- 只有一个后台线程，同一时刻最多 1 个在途请求，满足“每席位最多 2 个在途调用”的限制；请求超时 2 秒，网关读取超时为 5 秒，留有余量。
- 只有 2xx 成功才消耗额度；遇到 `429 quota_exceeded` 说明额度耗尽，程序会继续用本地策略，不会卡住比赛。
- **新规（10-29 公布）**：模型统一 `deepseek-flash`；网关除 `model` 外原样转发请求字段，DeepSeek 思考默认开启且网关不会自动关闭，所以程序在请求体顶层显式发送 `"thinking":{"type":"disabled"}` 关闭思考，并发送 `"response_format":{"type":"json_object"}`（系统提示词已包含“JSON”字样，满足 json_object 模式要求）；`max_tokens` 默认 512 与官方请求示例一致。请求走 `POST /v1/chat/completions`、带 `Authorization: Bearer <本场令牌>`，并带 `Content-Length` 头（不用分块传输），遵守配额、并发与请求大小限制。

---

## 7. 比赛规则要点（写策略时对照）

- 第一阶段：100 ms/Tick、100 Tick，只能 `move`/`wait`，得分 `40 × R1 / 200`。
- 第二阶段：500 ms/Tick、500 Tick，得分 `30 × min(R2/120,1) + 15 × min(D/500,1) + 5 × M`。
- 攻击：同行或同列、距离 1~2 格，命中最近敌人，20 伤害（对方防御时 10，受保护时 0）。
- 防御只对当 Tick 有效，不叠加；受保护机器人不能攻击。
- 死亡后 3 Tick 复活，复活后 2 Tick 受保护；死亡会损失有效伤害。
- 资源：普通 10 分 ×12、高级 20 分 ×4，走过即拾取，不刷新。
- 第二阶段至少要有 1 次成功的 AI 调用，否则 `M=0`。

---

## 8. 排错表

| 现象 | 可能原因 | 处理 |
| --- | --- | --- |
| `colcon: command not found` | 没 source ROS | `source /opt/ros/humble/setup.bash` |
| `Package 'rmtc_bot' not found` | 没 source 工作空间 | `source install/setup.bash` |
| 编译找不到 `rclcpp` | 环境没 source 或装错版本 | 确认 `/opt/ros/humble` 存在后重新开终端 |
| 编译卡死/内存不足 | 并行编译太多 | `colcon build --parallel-workers 1` |
| 启动后收不到感知 | 域/网络配置不一致 | 双方 `ROS_DOMAIN_ID=27`、`ROS_LOCALHOST_ONLY=0`、同一个 `RMW_IMPLEMENTATION` |
| 裁判端看不到注册 | 裁判晚启动 | 等 2 秒自动重发；确认话题 `/team_A/registration` 有数据：`ros2 topic echo /team_A/registration` |
| 动作全部被判无效 | `tick/session_id` 没回显 | 本程序自动回显；若自己改过代码，检查是否用了旧 tick |
| AI 一直 401 | token 不对 | 覆盖 `RMTC_TEAM_TOKEN`，确认席位与 token 对应 |
| `healthz` 不是 200 | 网关没启动/地址错 | 浏览器或 `curl http://127.0.0.1:8080/healthz` 检查 |
| 第二阶段 AI 调用失败 | 网络/配额/格式 | 看日志里的失败原因；至少保证成功 1 次，否则 `M=0` |
| AI 返回 429 | 额度耗尽，或同席位并发请求 | 程序只有一个在途请求；若 `quota_exceeded` 则只能降级为本地策略 |
| 第一阶段只会 move/wait | 设计如此 | 规则不允许攻击/防御 |
| 打不到敌人 | 不同行/列或距离超 2 | 用 `--verbose` 看坐标，策略会自动靠近 |
| 程序启动即退出 | 参数或配置 JSON 错误 | 看终端报错；`decision_config.json` 必须是合法 JSON |

常用观察命令：

```bash
ros2 node list
ros2 topic list
ros2 topic echo /team_A/perception
ros2 topic echo /team_A/action
ros2 topic hz /team_A/perception
```

官方练习包的日志默认在 `~/.rmtc-practice/`，其中 `communication.jsonl` 记录通信、`ai.jsonl` 记录 AI 调用。排查问题时先看自己的程序日志，再看这两个文件；向裁判反馈时附上阶段、席位、Tick 和错误内容，**不要附令牌**。

---

## 9. 赛前检查清单（照做一遍）

- [ ] 虚拟机内 `colcon build` 通过，`install/setup.bash` 能 source。
- [ ] 两个阶段都已在默认 Tick 节奏（100 ms / 500 ms）下完整跑通。
- [ ] “收到感知 → 提交动作 → 收到回执（`last_action`）”三个环节都验证过。
- [ ] 第二阶段用真实 LLM 验证过任务和事件处理，而不是只验证模拟接口。
- [ ] 暂停恢复、重开比赛、选手进程重启后都能恢复正常控制（程序会重新注册、按新 `session_id` 清缓存）。
- [ ] `ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY=0`、`RMW_IMPLEMENTATION` 与裁判端一致。
- [ ] 席位号、队伍名、token 正确，注册话题有数据。
- [ ] 网关 `/healthz` 返回 200，且第二阶段日志里出现至少一次 AI 调用成功。
- [ ] 模拟裁判第一/第二阶段都能跑满并给出分数。
- [ ] `decision_config.json` 里的 `decision_type` 已确定并按裁判要求登记/封存。
- [ ] 断网/网关挂掉时程序仍能正常行动（可以拔网线或填错网关地址试一次）。
- [ ] 比赛当天按“裁判端 → 选手程序”的顺序启动；程序晚启动也没关系（会自动重注册）。
- [ ] 代码和材料在截止时间（10-31 20:00）前提交，具体提交方式以官方通知为准。

---

## 10. 目前还不能确定的信息，之后怎么接

下面这些要等主办方发布/现场公布，代码已经做了对应设计，**不需要改逻辑，只需要改参数或补数据**：

1. **比赛地图、障碍和资源位置**
   代码不写死任何坐标，地图尺寸、障碍、资源都从感知实时更新。官方公布固定地图后，如果想针对性优化，可以把地图画进 `tools/mock_referee.py` 做专项训练，或在 `strategy.cpp` 里给关键区域加权重。比赛现场仍以感知为准。

2. **随机事件的文本格式**
   不做关键词解析（规则允许的方式是由官方 LLM 解读）。比赛前用 `mock_referee.py --event "..."` 模拟各种说法，确认 AI 返回的 JSON 能被正确采用即可。

3. **正式席位、token、域名、网关地址、模型名**
   全部通过参数/环境变量传入，代码里只有练习默认值。现场按裁判给的值设置：
   ```bash
   export RMTC_TEAM_TOKEN=正式令牌
   ./run_contestant.sh B "队名" http://正式网关地址
   ```

4. **`decision_type` 的最终选择**
   现场只允许改这一个字段。建议先在模拟环境里对比 `collect` / `balanced` / `aggressive` 的得分，再定稿并登记。

5. **比赛流程细节（调试时间、封存方式、提交材料格式）**
   以裁判通知为准；程序侧不受影响。

6. **对手的真实行为**
   未知，所以默认用 `balanced`：不主动送死，但有击杀机会不放过。

---

## 11. 三天安排建议

- **10-28（今天）**：拷进虚拟机 → `colcon build` → 模拟裁判跑通第一阶段和第二阶段 → 看一眼日志确认决策合理。
- **10-29**：接官方裁判测试端联调；确认 AI 网关能成功调用；分别试三种 `decision_type`，记录得分；处理编译/网络问题。
- **10-30**：稳定性测试（换 `--seed`、跑满 500 tick、断网测试）；确定 `decision_type`；准备提交材料。
- **10-31 20:00 前**：提交代码和材料。
- **11-01 比赛日**：提前启动裁判端，按第 9 节清单逐项检查，再启动程序。

---

## 12. ROS 2 最小知识（够用版）

- **节点（node）**：一个独立进程，本程序就是一个节点 `rmtc_bot`。
- **话题（topic）**：节点之间传消息的“广播频道”。本程序订阅 `/team_A/perception`、`/referee/mission`、`/referee/event_text`、`/referee/team_registry`，发布 `/team_A/action`、`/team_A/registration`。
- **消息（message）**：本比赛所有消息都是 `std_msgs/msg/String`，真正的数据是字符串里的 JSON。
- **QoS**：消息的可靠性和缓存策略。感知/动作要求可靠、只保留最新一条；注册和任务要求可靠且“后启动的订阅者也能收到最后一条”（transient local）；事件保留最近 128 条。代码里已经按协议设置好，不用动。
- 常用命令：`ros2 run` 启动节点、`ros2 topic list/echo/hz` 看数据、`ros2 node list` 看节点。

---

## 13. 已验证 / 未验证

已验证（Windows 离线自测）：

- JSON 解析与生成（含中文、转义、数组/对象）。
- 9×9 相对网格解码、感知 JSON 解析。
- BFS 距离与第一步方向。
- 攻击（同排 2 格残血敌人）、防御（低血量将被击杀）、第一阶段动作限制、越界 AI 建议不崩溃。

未验证（需要你在虚拟机里确认）：

- 在 ROS 2 Humble 下的实际编译（`colcon build`）。
- 与官方裁判测试端的联调、QoS 匹配。
- 真实 AI 网关的响应格式和配额。
- 长时间运行（500 tick）稳定性。

如果编译报错，把完整报错贴出来即可定位；绝大多数会是环境没 source 或路径问题。
