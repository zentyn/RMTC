#include "rmtc_bot/world_memory.hpp"

#include <algorithm>
#include <cstdlib>

namespace rmtc {
namespace {

constexpr int kObstacleMemoryTicks = 200;   // 视野外障碍的保守记忆
constexpr int kLearnedBlockTicks = 8;       // 移动失败学习的短期阻挡
constexpr int kResourceMemoryTicks = 400;   // 资源不刷新，记忆可长

int chebyshev(Vec2 a, Vec2 b) {
  return std::max(std::abs(a.x - b.x), std::abs(a.y - b.y));
}

}  // namespace

void WorldMemory::configure(int width, int height) {
  width_ = width > 0 ? width : 24;
  height_ = height > 0 ? height : 24;
  const std::size_t cells = static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
  obstacle_tick_.assign(cells, -1);
  learned_block_tick_.assign(cells, -1);
  visited_tick_.assign(cells, -1);
  resources_.clear();
  enemies_.clear();
  avoid_.clear();
  has_spawn_ = false;
  spawn_ = Vec2{0, 0};
}

void WorldMemory::reset() { configure(width_, height_); }

void WorldMemory::update(const Perception& p) {
  if (p.map_width != width_ || p.map_height != height_) {
    configure(p.map_width, p.map_height);
  }

  if (!has_spawn_ && (!p.self.position_reason.empty())) {
    spawn_ = p.self.pos;
    has_spawn_ = true;
  }

  if (!inside(p.self.pos)) return;
  visited_tick_[static_cast<std::size_t>(index(p.self.pos))] = p.tick;

  for (const Entity& e : p.entities) {
    if (!e.has_pos || !inside(e.pos)) continue;
    switch (e.type) {
      case EntityType::Obstacle:
        obstacle_tick_[static_cast<std::size_t>(index(e.pos))] = p.tick;
        break;
      case EntityType::Robot:
        if (e.id == p.self.robot_id || (!e.team.empty() && e.team == p.self.team)) continue;
        enemies_[e.id] = EnemyMemory{e.pos, e.hp, p.tick};
        break;
      case EntityType::ResourceNormal:
      case EntityType::ResourceHigh:
        resources_[e.id] = ResourceMemory{e.pos, e.value, p.tick};
        break;
      default:
        break;
    }
  }

  // 视野内没再出现的障碍：临时墙已消失或记忆有误，立即清除
  const int radius = p.vision_radius;
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      Vec2 cell{p.self.pos.x + dx, p.self.pos.y + dy};
      if (!inside(cell)) continue;
      std::size_t idx = static_cast<std::size_t>(index(cell));
      if (obstacle_tick_[idx] >= 0 && obstacle_tick_[idx] != p.tick) {
        obstacle_tick_[idx] = -1;
      }
    }
  }

  // 视野内没再出现的资源：已被捡走
  for (auto it = resources_.begin(); it != resources_.end();) {
    ResourceMemory& r = it->second;
    if (r.last_seen_tick != p.tick && chebyshev(r.pos, p.self.pos) <= radius) {
      it = resources_.erase(it);
      continue;
    }
    if (p.tick - r.last_seen_tick > kResourceMemoryTicks) {
      it = resources_.erase(it);
      continue;
    }
    ++it;
  }
}

bool WorldMemory::blocked(Vec2 cell, int tick) const {
  if (!inside(cell)) return true;
  const std::size_t idx = static_cast<std::size_t>(index(cell));
  const int obs = obstacle_tick_[idx];
  if (obs >= 0 && tick - obs <= kObstacleMemoryTicks) return true;
  const int learned = learned_block_tick_[idx];
  if (learned >= 0 && tick - learned <= kLearnedBlockTicks) return true;
  return false;
}

bool WorldMemory::in_avoid(Vec2 cell) const {
  for (const Rect& r : avoid_) {
    if (r.contains(cell)) return true;
  }
  return false;
}

bool WorldMemory::visited(Vec2 cell) const {
  if (!inside(cell)) return false;
  return visited_tick_[static_cast<std::size_t>(index(cell))] >= 0;
}

bool WorldMemory::ever_visited(Vec2 cell) const { return visited(cell); }

void WorldMemory::mark_move_blocked(Vec2 cell, int tick) {
  if (!inside(cell)) return;
  learned_block_tick_[static_cast<std::size_t>(index(cell))] = tick;
}

}  // namespace rmtc
