// ============================================================================
//  world_model.cpp —— 世界记忆与寻路实现
// ============================================================================

#include "rmtc_bot/world_model.hpp"

#include <algorithm>
#include <queue>
#include <sstream>

namespace rmtc {

WorldModel::WorldModel() {
    eventLines_.clear();
}

// ---------------------------------------------------------------- 场次管理

bool WorldModel::syncSession(const Perception& p) {
    bool newSession = false;

    // 【协议 §4】收到新的 session_id 时清空上一场的规划、定位和动作缓存
    if (sessionId_ != p.sessionId) {
        reset();
        sessionId_ = p.sessionId;
        newSession = true;
    }

    // 地图尺寸以本场感知为准（协议 §8：不同大轮可用不同地图）
    if (p.mapW > 0 && p.mapH > 0) {
        mapW_ = p.mapW;
        mapH_ = p.mapH;
    }

    // 同一 Tick 的重复消息只处理一次（协议：按 tick 去重，不按接收次数计数）。
    // 恢复比赛时裁判可能重发相同 Tick，此时不能重复推进记忆。
    if (p.hasTick && p.tick == lastTick_) {
        return newSession;
    }
    if (p.hasTick) {
        lastTick_ = p.tick;
    }
    return newSession;
}

void WorldModel::reset() {
    obstacles_.clear();
    resources_.clear();
    enemies_.clear();
    visited_.clear();
    seenCells_.clear();
    reachTargets_.clear();
    eventLines_.clear();
    eventHints_.clear();
    lastTick_ = -1;
    // 注意：sessionId_ 和 mapW_/mapH_ 由调用方（syncSession）重新设置
}

// ---------------------------------------------------------------- 吸收感知

void WorldModel::observe(const Perception& p) {
    if (p.mapW > 0 && p.mapH > 0) {
        mapW_ = p.mapW;
        mapH_ = p.mapH;
    }

    int tick = p.hasTick ? p.tick : lastTick_;

    // ---- 障碍：静态的，长期记住 ----
    for (const Entity& e : p.entities) {
        if (e.type == EntityType::Obstacle && e.hasPos) {
            obstacles_.insert(e.pos);
        }
    }

    // ---- 资源 ----
    for (const Entity& e : p.entities) {
        if (!e.isResource() || !e.hasPos) {
            continue;
        }
        ResourceMemo memo;
        memo.high = e.isHigh();
        memo.value = (e.value > 0) ? e.value : (memo.high ? 20 : 10);
        memo.seenTick = tick;
        memo.id = e.id;

        auto it = resources_.find(e.pos);
        if (it != resources_.end()) {
            // 之前记过：保留"已被捡走"标记，避免重复去捡
            memo.taken = it->second.taken;
        }
        resources_[e.pos] = memo;
    }

    // ---- 敌车：会动，只记"最后看到的位置 + 当时 Tick" ----
    for (const Entity& e : p.entities) {
        if (e.type != EntityType::Robot || !e.hasPos) {
            continue;
        }
        std::string key = e.id;
        if (key.empty()) {
            key = "robot_" + (e.robotId.empty() ? e.team : e.robotId);
        }
        EnemyMemo m;
        m.pos = e.pos;
        m.hasPos = true;
        m.hp = e.hp;
        m.team = e.team.empty() ? e.robotId : e.team;
        m.defending = e.defending;
        m.protectedSpawn = e.protectedSpawn;
        m.seenTick = tick;
        enemies_[key] = m;
    }
    // 注意：阵亡期间感知不提供周围实体（规则 §4.1），
    // 此时 enemies_ 不会被更新，seenTick 自然变旧，liveEnemies 会过滤掉。

    // ---- 自己的位置 ----
    if (p.self.hasPos) {
        visited_.insert(p.self.pos);

        // 【规则 §4.2】资源在存活机器人位于其格子时自动收集，不需要交互动作。
        // 所以自己踩过的格子上，资源肯定已经进自己口袋了，标记为已捡走。
        if (p.self.alive) {
            auto it = resources_.find(p.self.pos);
            if (it != resources_.end() && !it->second.taken) {
                it->second.taken = true;
            }
        }

        // 把以自身为中心的 9x9 记为"看过的格子"，用于探索未知区域
        for (int dy = -4; dy <= 4; ++dy) {
            for (int dx = -4; dx <= 4; ++dx) {
                seenCells_.insert(Cell(p.self.pos.x + dx, p.self.pos.y + dy));
            }
        }
    }

    // ---- 任务里的 reach 目标（练习基准是 [12,12]，但必须从感知读，不能写死）----
    reachTargets_.clear();
    for (const Mission& m : p.missions) {
        if (m.type == "reach" && m.hasPos) {
            reachTargets_.push_back(m.pos);
        }
    }

    // ---- 事件公告：按行去重记录 ----
    // 【协议 §3】重复收到公告不表示事件重复触发，所以这里去重只是为了省 token，
    // 不改变事件语义。
    if (!p.eventText.empty()) {
        std::istringstream iss(p.eventText);
        std::string line;
        while (std::getline(iss, line)) {
            // 去掉首尾空白
            size_t b = line.find_first_not_of(" \t\r");
            size_t e = line.find_last_not_of(" \t\r");
            if (b == std::string::npos) {
                continue;
            }
            std::string clean = line.substr(b, e - b + 1);
            addEventLine(clean);
        }
    }

    prune(tick);
}

void WorldModel::addEventLine(const std::string& line) {
    if (line.empty()) {
        return;
    }
    for (const std::string& old : eventLines_) {
        if (old == line) {
            return;   // 已记录过
        }
    }
    // 容量上限 64 条，防止长时间跑占用过多内存和 token
    if (eventLines_.size() >= 64) {
        eventLines_.pop_front();
    }
    eventLines_.push_back(line);
}

// ---------------------------------------------------------------- 查询

bool WorldModel::inMap(const Cell& c) const {
    if (mapW_ <= 0 || mapH_ <= 0) {
        return true;   // 地图尺寸未知时不阻拦，交给裁判的 E_OOB 回执兜底
    }
    return c.x >= 0 && c.x < mapW_ && c.y >= 0 && c.y < mapH_;
}

bool WorldModel::isObstacle(const Cell& c) const {
    return obstacles_.find(c) != obstacles_.end();
}

bool WorldModel::isBlocked(const Cell& c) const {
    if (!inMap(c)) {
        return true;
    }
    return isObstacle(c);
}

bool WorldModel::isSeen(const Cell& c) const {
    return seenCells_.find(c) != seenCells_.end();
}

std::vector<EnemyMemo> WorldModel::liveEnemies(int tick) const {
    std::vector<EnemyMemo> out;
    if (tick < 0) {
        return out;
    }
    for (const auto& kv : enemies_) {
        const EnemyMemo& m = kv.second;
        if (!m.hasPos || m.seenTick < 0) {
            continue;
        }
        if (tick - m.seenTick <= kEnemyMemoryTicks) {
            out.push_back(m);
        }
    }
    // 近的排前面，方便策略优先处理威胁
    std::stable_sort(out.begin(), out.end(),
                     [](const EnemyMemo& a, const EnemyMemo& b) { return a.seenTick > b.seenTick; });
    return out;
}

std::vector<std::pair<Cell, ResourceMemo>> WorldModel::pendingResources() const {
    std::vector<std::pair<Cell, ResourceMemo>> out;
    for (const auto& kv : resources_) {
        if (!kv.second.taken) {
            out.push_back(kv);
        }
    }
    // 价值高的排前面；同价值按坐标排序，保证结果稳定（相同输入必得相同输出）
    std::stable_sort(out.begin(), out.end(),
                     [](const std::pair<Cell, ResourceMemo>& a,
                        const std::pair<Cell, ResourceMemo>& b) {
                         if (a.second.value != b.second.value) {
                             return a.second.value > b.second.value;
                         }
                         return a.first < b.first;
                     });
    return out;
}

void WorldModel::prune(int tick) {
    if (tick < 0) {
        return;
    }
    // 敌人记忆太久就删掉，避免 map 无限增长
    for (auto it = enemies_.begin(); it != enemies_.end();) {
        if (it->second.seenTick >= 0 &&
            tick - it->second.seenTick > kEnemyMemoryTicks * 4) {
            it = enemies_.erase(it);
        } else {
            ++it;
        }
    }
}

// ---------------------------------------------------------------- 寻路（BFS）

namespace {

// BFS 内部：返回从 start 到 goal 的完整路径（含首尾），无路返回空
std::vector<Cell> bfsImpl(const Cell& start, const Cell& goal,
                          const std::set<Cell>& obstacles,
                          int mapW, int mapH) {
    std::vector<Cell> path;
    if (start == goal) {
        path.push_back(start);
        return path;
    }

    auto inMap = [&](const Cell& c) -> bool {
        if (mapW <= 0 || mapH <= 0) {
            return c.x >= 0 && c.y >= 0;   // 尺寸未知时只挡负坐标
        }
        return c.x >= 0 && c.x < mapW && c.y >= 0 && c.y < mapH;
    };

    std::map<Cell, Cell> cameFrom;      // 记录每个格子是从哪来的，用于回溯路径
    std::set<Cell> closed;
    std::queue<Cell> q;

    q.push(start);
    closed.insert(start);

    const int kMaxExpand = 4096;        // 安全阀：防止极端情况下爆内存
    int expanded = 0;

    while (!q.empty() && expanded < kMaxExpand) {
        Cell cur = q.front();
        q.pop();
        ++expanded;

        for (int i = 0; i < 4; ++i) {
            int dx = 0, dy = 0;
            dirDelta(static_cast<Dir>(i), dx, dy);
            Cell nxt(cur.x + dx, cur.y + dy);

            if (!inMap(nxt)) continue;
            if (obstacles.find(nxt) != obstacles.end()) continue;   // 障碍不能进
            if (closed.find(nxt) != closed.end()) continue;

            closed.insert(nxt);
            cameFrom[nxt] = cur;

            if (nxt == goal) {
                // 回溯路径
                path.push_back(goal);
                Cell back = goal;
                while (cameFrom.find(back) != cameFrom.end()) {
                    back = cameFrom[back];
                    path.push_back(back);
                    if (back == start) break;
                }
                std::reverse(path.begin(), path.end());
                return path;
            }
            q.push(nxt);
        }
    }
    return path;   // 空 = 无路可走
}

}  // namespace

Dir WorldModel::bfsNextStep(const Cell& start, const Cell& goal, Cell& nextStep) const {
    nextStep = start;
    std::vector<Cell> path = bfsImpl(start, goal, obstacles_, mapW_, mapH_);
    if (path.size() < 2) {
        return Dir::None;   // 已经在目标格，或无路可走
    }
    nextStep = path[1];
    return directionBetween(start, path[1]);
}

std::vector<Cell> WorldModel::bfsPath(const Cell& start, const Cell& goal) const {
    return bfsImpl(start, goal, obstacles_, mapW_, mapH_);
}

// ---------------------------------------------------------------- 给大模型看的摘要

std::string WorldModel::snapshotText(const Perception& p, size_t maxEvents) const {
    std::ostringstream os;

    os << "当前 Tick=" << p.tick << "，场次=" << p.sessionId << "\n";
    os << "我方席位=" << p.self.team
       << "，位置=" << (p.self.hasPos ? p.self.pos.str() : std::string("<无>"))
       << "，HP=" << p.self.hp
       << "，存活=" << (p.self.alive ? "是" : "否")
       << "，防御中=" << (p.self.defending ? "是" : "否")
       << "，出生保护=" << (p.self.protectedSpawn ? "是" : "否")
       << "，复活倒计时=" << p.self.respawnIn << "\n";

    os << "计分：资源价值=" << static_cast<int>(p.score.resourceValue)
       << "，有效伤害=" << static_cast<int>(p.score.effectiveDamage)
       << "，已完成任务数=" << p.score.missionCompleted
       << "，无效动作=" << p.score.invalidActions << "\n";

    if (!p.missionText.empty()) {
        os << "任务描述：" << p.missionText << "\n";
    }

    std::vector<Entity> robots = p.robotList();
    std::vector<Entity> res = p.resourceList();
    std::vector<Entity> obs = p.obstacleList();
    os << "本帧视野：敌车 " << robots.size() << " 台，资源 " << res.size()
       << " 个，障碍 " << obs.size() << " 格\n";

    size_t shown = 0;
    for (const Entity& e : robots) {
        if (shown >= 4) break;
        os << "  敌车 " << (e.robotId.empty() ? e.team : e.robotId)
           << " 相对位移=(" << e.relDx << "," << e.relDy << ")"
           << " 绝对坐标≈" << (e.hasPos ? e.pos.str() : std::string("<无>"))
           << " HP=" << e.hp
           << " 防御=" << (e.defending ? "是" : "否")
           << " 出生保护=" << (e.protectedSpawn ? "是" : "否") << "\n";
        ++shown;
    }
    shown = 0;
    for (const Entity& e : res) {
        if (shown >= 6) break;
        os << "  资源 " << e.id << " 价值=" << e.value
           << " 相对位移=(" << e.relDx << "," << e.relDy << ")"
           << " 绝对坐标≈" << (e.hasPos ? e.pos.str() : std::string("<无>")) << "\n";
        ++shown;
    }

    os << "记忆：已知未采集资源 " << pendingResources().size()
       << " 个，已知障碍 " << obstacles_.size() << " 格，最近见过的敌车 "
       << liveEnemies(p.tick).size() << " 台\n";
    if (!reachTargets_.empty()) {
        os << "记忆：任务坐标目标 ";
        for (size_t i = 0; i < reachTargets_.size(); ++i) {
            if (i) os << " ";
            os << reachTargets_[i].str();
        }
        os << "\n";
    }

    if (eventLines_.empty()) {
        os << "近期事件公告原文：（暂无）\n";
    } else {
        os << "近期事件公告原文：\n";
        size_t total = eventLines_.size();
        size_t begin = (total > maxEvents) ? (total - maxEvents) : 0;
        for (size_t i = begin; i < total; ++i) {
            os << "  " << eventLines_[i] << "\n";
        }
    }

    return os.str();
}

}  // namespace rmtc
