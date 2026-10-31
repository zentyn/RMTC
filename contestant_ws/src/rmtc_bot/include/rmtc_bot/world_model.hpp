// ============================================================================
//  world_model.hpp —— 世界记忆与寻路
// ============================================================================
//
//  【为什么需要这个模块】
//  协议 §3 明确："map 仅地图尺寸；障碍与资源的全局分布不在感知中"。
//  也就是说裁判只给你以自己为中心的 9x9 视野，出了这 9x9 你什么都不知道。
//
//  规则 §4.1 又允许："允许记忆和推断此前合法获得的信息，
//  但不能把推断当作裁判提供的实时真值。"
//
//  所以策略是：
//    * 障碍、资源位置 —— 相对静态，长期记住（自己踩过的资源要标记为已捡走）；
//    * 敌人位置 —— 会动！只记"最后看到时的坐标 + 当时 Tick"，
//      用时按"可能已经移动"保守处理，绝不当成它现在还在那儿。
//
//  【三条硬规矩】
//  1. 换场（session_id 变了）必须清空全部记忆（协议 §4）；
//  2. 自己的位置永远以 self.position 为准，绝不自己累加位移（规则 §4.1：
//     "选手直接采用当前坐标，不累加位移"）；
//  3. 事件公告里的坐标是绝对坐标，可以记；但禁止预设固定事件时间表。
// ============================================================================

#pragma once

#include <deque>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "rmtc_bot/perception.hpp"

namespace rmtc {

// 记忆里的一条资源
struct ResourceMemo {
    int value = 10;
    bool high = false;
    bool taken = false;        // 是否已被捡走（自己踩过就算捡走）
    int seenTick = -1;         // 最后一次看到它时的 Tick
    std::string id;
};

// 记忆里的一台敌车
struct EnemyMemo {
    Cell pos;
    bool hasPos = false;
    int hp = -1;
    std::string team;
    bool defending = false;
    bool protectedSpawn = false;
    int seenTick = -1;         // 关键：最后一次看到时的 Tick，用来判断记忆新鲜度
};

class WorldModel {
public:
    // 敌人位置记忆最多信任几个 Tick 之前的信息。
    // 第二阶段 500ms/Tick，敌人每 Tick 走 1 格，超过这个 Tick 数就认为位置不可靠。
    static const int kEnemyMemoryTicks = 12;

    WorldModel();

    // ---------------------------------------------------------------- 场次管理

    // 每帧先调这个。发现 session_id 变了就清空记忆。
    // 返回 true 表示"这是新一场的第一帧"。
    bool syncSession(const Perception& p);

    // 清空本场全部记忆（换场、重开、程序重启后都要调）
    void reset();

    // ---------------------------------------------------------------- 吸收感知

    // 把一帧感知吸收进记忆
    void observe(const Perception& p);

    // ---------------------------------------------------------------- 查询

    bool inMap(const Cell& c) const;          // 是否在地图内
    bool isBlocked(const Cell& c) const;      // 是否已知不可进入（越界或障碍）
    bool isObstacle(const Cell& c) const;     // 是否已知是障碍

    // 最近还见过的敌车（按记忆新鲜度过滤）
    std::vector<EnemyMemo> liveEnemies(int tick) const;

    // 还没被捡走的资源（按价值降序）
    std::vector<std::pair<Cell, ResourceMemo>> pendingResources() const;

    // 任务里的 reach 目标坐标
    const std::vector<Cell>& reachTargets() const { return reachTargets_; }

    // 已探索过的格子（每帧把自身 9x9 记下来），用于"往没去过的地方探索"
    bool isSeen(const Cell& c) const;
    const std::set<Cell>& seenCells() const { return seenCells_; }

    size_t obstacleCount() const { return obstacles_.size(); }
    size_t enemyCount() const { return enemies_.size(); }

    // ---------------------------------------------------------------- 寻路

    // BFS 最短路。返回从 start 到 goal 的第一步方向；无路可走返回 Dir::None。
    // nextStep 输出下一步要走的格子。
    //
    // 【为什么用 BFS】地图是格子、每 Tick 只能走 1 格、不许斜走，
    // 所有边权都是 1，BFS 就是最短路，实现简单且不会出错。
    Dir bfsNextStep(const Cell& start, const Cell& goal, Cell& nextStep) const;

    // 求到 goal 的完整路径（含起点和终点），无路返回空。调试和可视化用。
    std::vector<Cell> bfsPath(const Cell& start, const Cell& goal) const;

    // ---------------------------------------------------------------- 事件与摘要

    // 记录一条事件公告（自动去重）
    void addEventLine(const std::string& line);
    const std::deque<std::string>& eventLines() const { return eventLines_; }

    // 大模型解析公告后得到的"结构化提示"。
    // 注意：这是模型输出经校验后的结果，不是裁判给的真值，只能作为参考。
    void setEventHints(const std::vector<std::string>& hints) { eventHints_ = hints; }
    const std::vector<std::string>& eventHints() const { return eventHints_; }

    // 生成一段"人话"状态摘要，用于发给大模型做高层决策。
    // 只包含协议允许的公开信息 + 自己的记忆推断，不含任何地图真值。
    std::string snapshotText(const Perception& p, size_t maxEvents = 6) const;

    // 清理过期记忆，防止长时间跑越来越慢
    void prune(int tick);

    int mapW() const { return mapW_; }
    int mapH() const { return mapH_; }
    const std::string& sessionId() const { return sessionId_; }
    int lastTick() const { return lastTick_; }

private:
    std::string sessionId_;
    int mapW_ = 0;
    int mapH_ = 0;
    int lastTick_ = -1;

    std::set<Cell> obstacles_;
    std::map<Cell, ResourceMemo> resources_;
    std::map<std::string, EnemyMemo> enemies_;
    std::set<Cell> visited_;      // 自己踩过的格子
    std::set<Cell> seenCells_;    // 自己看过的格子（9x9 视野累积）
    std::vector<Cell> reachTargets_;
    std::deque<std::string> eventLines_;
    std::vector<std::string> eventHints_;
};

}  // namespace rmtc
