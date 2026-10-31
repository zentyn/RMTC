#include "rmtc_bot/planner.hpp"

#include <deque>

namespace rmtc {
namespace {

constexpr int kDx[4] = {1, -1, 0, 0};
constexpr int kDy[4] = {0, 0, 1, -1};

inline std::size_t idx_of(Vec2 c, int width) {
  return static_cast<std::size_t>(c.y * width + c.x);
}

}  // namespace

Vec2 BfsField::first_step(Vec2 goal) const {
  if (!inside(goal)) return goal;
  std::size_t goal_idx = idx_of(goal, width);
  if (parent[goal_idx] < 0) return goal;  // goal 就是起点或不可达
  std::size_t cur = goal_idx;
  while (parent[cur] >= 0) {
    std::size_t p = static_cast<std::size_t>(parent[cur]);
    if (parent[p] < 0) {
      Vec2 step;
      step.x = static_cast<int>(cur % static_cast<std::size_t>(width));
      step.y = static_cast<int>(cur / static_cast<std::size_t>(width));
      return step;
    }
    cur = p;
  }
  return goal;
}

BfsField bfs_field(Vec2 start, int width, int height,
                   const std::function<bool(Vec2)>& passable) {
  BfsField field;
  field.width = width;
  field.height = height;
  if (width <= 0 || height <= 0) return field;
  const std::size_t cells = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  field.dist.assign(cells, -1);
  field.parent.assign(cells, -1);
  if (start.x < 0 || start.y < 0 || start.x >= width || start.y >= height) return field;

  std::deque<Vec2> queue;
  field.dist[idx_of(start, width)] = 0;
  queue.push_back(start);

  while (!queue.empty()) {
    Vec2 cur = queue.front();
    queue.pop_front();
    const int cur_dist = field.dist[idx_of(cur, width)];
    for (int k = 0; k < 4; ++k) {
      Vec2 next{cur.x + kDx[k], cur.y + kDy[k]};
      if (next.x < 0 || next.y < 0 || next.x >= width || next.y >= height) continue;
      const std::size_t next_idx = idx_of(next, width);
      if (field.dist[next_idx] >= 0) continue;
      if (!passable(next)) continue;
      field.dist[next_idx] = cur_dist + 1;
      field.parent[next_idx] = static_cast<int>(idx_of(cur, width));
      queue.push_back(next);
    }
  }
  return field;
}

BfsResult bfs_first_step(Vec2 start, int width, int height,
                         const std::function<bool(Vec2)>& passable,
                         const std::function<bool(Vec2)>& is_goal) {
  BfsResult result;
  if (width <= 0 || height <= 0) return result;
  if (start.x < 0 || start.y < 0 || start.x >= width || start.y >= height) return result;

  const std::size_t cells = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  std::vector<int> dist(cells, -1);
  std::vector<int> parent(cells, -1);

  std::deque<Vec2> queue;
  dist[idx_of(start, width)] = 0;
  queue.push_back(start);

  bool hit = false;
  Vec2 found;
  while (!queue.empty() && !hit) {
    Vec2 cur = queue.front();
    queue.pop_front();
    if (cur != start && is_goal(cur)) {
      hit = true;
      found = cur;
      break;
    }
    for (int k = 0; k < 4; ++k) {
      Vec2 next{cur.x + kDx[k], cur.y + kDy[k]};
      if (next.x < 0 || next.y < 0 || next.x >= width || next.y >= height) continue;
      const std::size_t next_idx = idx_of(next, width);
      if (dist[next_idx] >= 0) continue;
      if (!passable(next)) continue;
      dist[next_idx] = dist[idx_of(cur, width)] + 1;
      parent[next_idx] = static_cast<int>(idx_of(cur, width));
      queue.push_back(next);
    }
  }
  if (!hit) return result;

  result.found = true;
  result.goal = found;
  result.steps = dist[idx_of(found, width)];

  std::size_t cur = idx_of(found, width);
  while (parent[cur] >= 0) {
    std::size_t p = static_cast<std::size_t>(parent[cur]);
    if (parent[p] < 0) {
      result.first_step.x = static_cast<int>(cur % static_cast<std::size_t>(width));
      result.first_step.y = static_cast<int>(cur / static_cast<std::size_t>(width));
      return result;
    }
    cur = p;
  }
  result.first_step = found;
  return result;
}

}  // namespace rmtc
