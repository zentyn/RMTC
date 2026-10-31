// 跨 Tick 的地图记忆：障碍、资源、敌人、已探索区域
#pragma once

#include <map>
#include <string>
#include <vector>

#include "rmtc_bot/perception.hpp"

namespace rmtc {

struct ResourceMemory {
  Vec2 pos;
  int value = 0;
  int last_seen_tick = -1;
};

struct EnemyMemory {
  Vec2 pos;
  int hp = 100;
  int last_seen_tick = -1;
};

class WorldMemory {
 public:
  void configure(int width, int height);
  void reset();
  void update(const Perception& p);

  void set_avoid(const std::vector<Rect>& rects) { avoid_ = rects; }
  void clear_avoid() { avoid_.clear(); }
  const std::vector<Rect>& avoid() const { return avoid_; }

  bool inside(Vec2 cell) const {
    return cell.x >= 0 && cell.y >= 0 && cell.x < width_ && cell.y < height_;
  }
  bool blocked(Vec2 cell, int tick) const;
  bool in_avoid(Vec2 cell) const;
  bool visited(Vec2 cell) const;
  bool ever_visited(Vec2 cell) const;

  // 移动失败后学习：该格短期不可通行
  void mark_move_blocked(Vec2 cell, int tick);

  const std::map<std::string, ResourceMemory>& resources() const { return resources_; }
  const std::map<std::string, EnemyMemory>& enemies() const { return enemies_; }

  Vec2 spawn() const { return spawn_; }
  bool has_spawn() const { return has_spawn_; }

  int width() const { return width_; }
  int height() const { return height_; }

 private:
  int index(Vec2 c) const { return c.y * width_ + c.x; }

  int width_ = 24;
  int height_ = 24;
  std::vector<int> obstacle_tick_;
  std::vector<int> learned_block_tick_;
  std::vector<int> visited_tick_;
  std::map<std::string, ResourceMemory> resources_;
  std::map<std::string, EnemyMemory> enemies_;
  std::vector<Rect> avoid_;
  Vec2 spawn_;
  bool has_spawn_ = false;
};

}  // namespace rmtc
