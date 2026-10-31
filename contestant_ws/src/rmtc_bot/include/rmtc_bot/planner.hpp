// 网格 BFS：一次算出距离场，用来同时评估资源/任务/敌人目标
#pragma once

#include <functional>
#include <vector>

#include "rmtc_bot/perception.hpp"

namespace rmtc {

struct BfsField {
  int width = 0;
  int height = 0;
  std::vector<int> dist;
  std::vector<int> parent;

  bool inside(Vec2 c) const {
    return c.x >= 0 && c.y >= 0 && c.x < width && c.y < height;
  }
  int at(Vec2 c) const {
    if (!inside(c)) return -1;
    return dist[static_cast<std::size_t>(c.y * width + c.x)];
  }
  bool reachable(Vec2 c) const { return at(c) >= 0; }

  // 沿父指针回溯，得到从 start 走向 goal 的第一步
  Vec2 first_step(Vec2 goal) const;
};

BfsField bfs_field(Vec2 start, int width, int height,
                   const std::function<bool(Vec2)>& passable);

// 通用“最近满足条件格”的搜索，返回第一步
struct BfsResult {
  bool found = false;
  Vec2 goal;
  Vec2 first_step;
  int steps = 0;
};

BfsResult bfs_first_step(Vec2 start, int width, int height,
                         const std::function<bool(Vec2)>& passable,
                         const std::function<bool(Vec2)>& is_goal);

}  // namespace rmtc
