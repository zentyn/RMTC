#include "rmtc_bot/strategy.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <initializer_list>
#include <set>

#include "rmtc_bot/planner.hpp"

namespace rmtc {
namespace {

ActionCmd make_wait() {
  ActionCmd cmd;
  cmd.type = "wait";
  return cmd;
}

ActionCmd make_move(const char* direction) {
  ActionCmd cmd;
  cmd.type = "move";
  cmd.direction = direction;
  return cmd;
}

ActionCmd make_attack(const char* direction) {
  ActionCmd cmd;
  cmd.type = "attack";
  cmd.direction = direction;
  return cmd;
}

ActionCmd make_defend() {
  ActionCmd cmd;
  cmd.type = "defend";
  return cmd;
}

bool is_completed(const Perception& p, const std::string& id) {
  return std::find(p.score.mission_completed_ids.begin(), p.score.mission_completed_ids.end(),
                   id) != p.score.mission_completed_ids.end();
}

int manhattan(Vec2 a, Vec2 b) { return std::abs(a.x - b.x) + std::abs(a.y - b.y); }

struct AttackOption {
  bool valid = false;
  const char* direction = nullptr;
  int target_hp = 100;
  int distance = 99;
};

// 所有“本 Tick 能打到”的方向里，优先能击杀的目标
AttackOption best_attack(const Perception& p, const std::vector<const Entity*>& enemies) {
  AttackOption best;
  for (const Entity* e : enemies) {
    if (e->is_protected) continue;  // 保护期内打不掉
    int dx = e->pos.x - p.self.pos.x;
    int dy = e->pos.y - p.self.pos.y;
    const char* dir = nullptr;
    int dist = 0;
    if (dx == 0 && dy != 0) {
      dir = dy > 0 ? "down" : "up";
      dist = std::abs(dy);
    } else if (dy == 0 && dx != 0) {
      dir = dx > 0 ? "right" : "left";
      dist = std::abs(dx);
    }
    if (dir == nullptr || dist < 1 || dist > 2) continue;

    bool better = !best.valid;
    if (!better) {
      bool kill = e->hp > 0 && e->hp <= 20;
      bool best_kill = best.target_hp > 0 && best.target_hp <= 20;
      if (kill != best_kill) {
        better = kill;
      } else if (e->hp != best.target_hp) {
        better = e->hp < best.target_hp;
      } else {
        better = dist < best.distance;
      }
    }
    if (better) {
      best.valid = true;
      best.direction = dir;
      best.target_hp = e->hp;
      best.distance = dist;
    }
  }
  return best;
}

// 能在本 Tick 打到我们的敌人数（同行/列 1-2 格）
int incoming_attackers(const Perception& p, const std::vector<const Entity*>& enemies) {
  int count = 0;
  for (const Entity* e : enemies) {
    if (e->is_protected) continue;  // 保护期内不能攻击
    int dx = std::abs(e->pos.x - p.self.pos.x);
    int dy = std::abs(e->pos.y - p.self.pos.y);
    if ((dx == 0 && (dy == 1 || dy == 2)) || (dy == 0 && (dx == 1 || dx == 2))) {
      ++count;
    }
  }
  return count;
}

const Entity* nearest_enemy(const Perception& p, const std::vector<const Entity*>& enemies) {
  const Entity* best = nullptr;
  int best_d = 1 << 30;
  for (const Entity* e : enemies) {
    int d = manhattan(p.self.pos, e->pos);
    if (d < best_d) {
      best_d = d;
      best = e;
    }
  }
  return best;
}

const char* away_direction(const Perception& p, WorldMemory& mem, Vec2 from) {
  const int dx[4] = {1, -1, 0, 0};
  const int dy[4] = {0, 0, 1, -1};
  const char* names[4] = {"right", "left", "down", "up"};
  const char* best = nullptr;
  int best_score = manhattan(p.self.pos, from);
  for (int k = 0; k < 4; ++k) {
    Vec2 next{p.self.pos.x + dx[k], p.self.pos.y + dy[k]};
    if (!mem.inside(next) || mem.blocked(next, p.tick)) continue;
    int score = manhattan(next, from);
    if (score > best_score) {
      best_score = score;
      best = names[k];
    }
  }
  return best;
}

// 敌人所在格不能进（争格双输），所以追人时瞄准它旁边最近的可达格
bool adjacent_target(const BfsField& field, Vec2 enemy, Vec2& out) {
  const int dx[4] = {1, -1, 0, 0};
  const int dy[4] = {0, 0, 1, -1};
  int best = 1 << 30;
  bool found = false;
  for (int k = 0; k < 4; ++k) {
    Vec2 c{enemy.x + dx[k], enemy.y + dy[k]};
    int d = field.at(c);
    if (d <= 0) continue;
    if (d < best) {
      best = d;
      out = c;
      found = true;
    }
  }
  return found;
}

}  // namespace

DecisionType decision_type_from_string(const std::string& s) {
  if (s == "collect") return DecisionType::Collect;
  if (s == "aggressive") return DecisionType::Aggressive;
  return DecisionType::Balanced;
}

void Strategy::reset() {
  last_attempt_dir_.clear();
  last_attempt_tick_ = -1;
  last_pos_ = Vec2{0, 0};
  stay_ticks_ = 0;
}

ActionCmd Strategy::decide(const Perception& p, WorldMemory& mem, const LlmAdvice& advice,
                           int stage) {
  ActionCmd cmd = decide_impl(p, mem, advice, stage);
  if (cmd.type == "move") {
    last_attempt_dir_ = cmd.direction;
    last_attempt_tick_ = p.tick;
  }
  return cmd;
}

ActionCmd Strategy::decide_impl(const Perception& p, WorldMemory& mem, const LlmAdvice& advice,
                                int stage) {
  if (!p.self.alive) {
    return make_wait();
  }

  if (p.self.pos == last_pos_) {
    ++stay_ticks_;
  } else {
    stay_ticks_ = 0;
    last_pos_ = p.self.pos;
  }

  // 上一 Tick 移动没成功：记住那一格，短期绕开（不依赖具体错误码）
  if (p.self.last_action.valid && !p.self.last_action.moved &&
      p.self.last_action.tick == last_attempt_tick_ && !last_attempt_dir_.empty()) {
    Vec2 blocked = p.self.pos;
    if (last_attempt_dir_ == "up") blocked.y -= 1;
    if (last_attempt_dir_ == "down") blocked.y += 1;
    if (last_attempt_dir_ == "left") blocked.x -= 1;
    if (last_attempt_dir_ == "right") blocked.x += 1;
    mem.mark_move_blocked(blocked, p.tick);
  }

  // AI 建议只在一段时间内有效，避免旧事件/旧目标长期干扰
  const bool advice_fresh =
      advice.has_advice && (advice.stamp <= 0 || (p.tick >= advice.stamp &&
                                                  p.tick - advice.stamp <= 60));
  if (advice_fresh) {
    mem.set_avoid(advice.avoid);
  } else {
    mem.clear_avoid();
  }

  std::vector<const Entity*> enemies;
  std::set<std::pair<int, int>> enemy_cells;
  for (const Entity& e : p.entities) {
    if (e.type != EntityType::Robot) continue;
    if (e.team == p.self.team) continue;
    if (e.hp == 0) continue;
    enemies.push_back(&e);
    enemy_cells.insert({e.pos.x, e.pos.y});
  }

  auto passable = [&](bool use_avoid, Vec2 c) {
    if (!mem.inside(c)) return false;
    if (mem.blocked(c, p.tick)) return false;
    if (use_avoid && mem.in_avoid(c)) return false;
    if (enemy_cells.count({c.x, c.y})) return false;  // 争格会双输，绕开
    return true;
  };

  const int w = mem.width();
  const int h = mem.height();
  BfsField field = bfs_field(p.self.pos, w, h, [&](Vec2 c) { return passable(true, c); });
  BfsField field_plain = bfs_field(p.self.pos, w, h, [&](Vec2 c) { return passable(false, c); });

  auto move_to = [&](Vec2 target) -> std::optional<ActionCmd> {
    for (const BfsField* f : {&field, &field_plain}) {
      if (f->at(target) <= 0) continue;
      Vec2 step = f->first_step(target);
      if (step == p.self.pos) continue;
      const char* dir = direction_name(Vec2{step.x - p.self.pos.x, step.y - p.self.pos.y});
      if (dir != nullptr) return make_move(dir);
    }
    return std::nullopt;
  };

  LlmMode mode = advice_fresh ? advice.mode : LlmMode::None;

  // LOGIC-1: 第二阶段先判断能不能打、要不要防
  if (stage >= 2) {
    AttackOption attack = best_attack(p, enemies);
    int attackers = incoming_attackers(p, enemies);
    double incoming = 20.0 * static_cast<double>(attackers);

    if (p.self.is_protected) {
      // 出生保护期间不能攻击，直接赶路
    } else if (p.self.hp <= incoming && p.self.hp > incoming * 0.5 && attackers > 0) {
      return make_defend();  // 防御后 20->10，能活下来
    } else if (attack.valid) {
      return make_attack(attack.direction);
    }

    // 残血且敌人贴身：先脱离
    const Entity* nearest = nearest_enemy(p, enemies);
    if (nearest != nullptr && p.self.hp <= 20 && manhattan(p.self.pos, nearest->pos) <= 2) {
      const char* dir = away_direction(p, mem, nearest->pos);
      if (dir != nullptr) return make_move(dir);
    }
  }

  // LOGIC-2: LLM 指定的目标/占据点
  if (advice_fresh && advice.go.has_value()) {
    Vec2 go = *advice.go;
    if (go == p.self.pos) {
      if (mode == LlmMode::Hold) return make_wait();
    } else {
      auto mv = move_to(go);
      if (mv.has_value()) return *mv;
    }
  }

  // LOGIC-3: 任务里的 reach 坐标，比赛后段再专门跑
  const int stage_length = stage >= 2 ? 500 : 100;
  for (const Mission& m : p.missions) {
    if (m.type != "reach" || !m.has_position) continue;
    if (is_completed(p, m.id)) continue;
    if (p.tick < stage_length * 3 / 5) continue;
    auto mv = move_to(m.position);
    if (mv.has_value()) return *mv;
  }

  // LOGIC-4: 采集资源
  Vec2 best_target{0, 0};
  bool has_target = false;
  double best_score = 1e9;
  for (const auto& kv : mem.resources()) {
    const ResourceMemory& r = kv.second;
    int d = field.at(r.pos);
    if (d < 0) d = field_plain.at(r.pos);
    if (d <= 0) continue;  // d==0 表示自己正站在资源上：采集是自动的，不必再“走向”它
    if (p.tick - r.last_seen_tick > 150) continue;
    double bonus = r.value >= 20 ? 4.0 : 0.0;
    if (decision_type_ == DecisionType::Collect) bonus *= 1.5;
    double score = static_cast<double>(d) - bonus;
    if (score < best_score) {
      best_score = score;
      best_target = r.pos;
      has_target = true;
    }
  }
  if (has_target) {
    auto mv = move_to(best_target);
    if (mv.has_value()) return *mv;
  }

  // LOGIC-5: 主动接敌（仅第二阶段）
  if (stage >= 2 && !p.self.is_protected) {
    const Entity* nearest = nearest_enemy(p, enemies);
    if (nearest != nullptr && p.self.hp > 30) {
      int d = manhattan(p.self.pos, nearest->pos);
      int chase_range = decision_type_ == DecisionType::Aggressive ? 5 : 2;
      bool winning = p.self.hp > nearest->hp + 20;
      if (mode == LlmMode::Fight) chase_range = 6;
      if (d <= chase_range && (winning || mode == LlmMode::Fight)) {
        Vec2 target;
        if (adjacent_target(field, nearest->pos, target)) {
          auto mv = move_to(target);
          if (mv.has_value()) return *mv;
        }
      }
    }
  }

  // LOGIC-6: 探索没去过的格子
  BfsResult explore = bfs_first_step(
      p.self.pos, w, h, [&](Vec2 c) { return passable(true, c); },
      [&](Vec2 c) { return !mem.visited(c) && !mem.in_avoid(c); });
  if (explore.found) {
    const char* dir = direction_name(Vec2{explore.first_step.x - p.self.pos.x,
                                          explore.first_step.y - p.self.pos.y});
    if (dir != nullptr) return make_move(dir);
  }

  return make_wait();
}

}  // namespace rmtc
