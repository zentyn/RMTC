// 感知 JSON 的数据结构与解析
#pragma once

#include <string>
#include <vector>

namespace rmtc {

struct Vec2 {
  int x = 0;
  int y = 0;
};

inline bool operator==(const Vec2& a, const Vec2& b) { return a.x == b.x && a.y == b.y; }
inline bool operator!=(const Vec2& a, const Vec2& b) { return !(a == b); }

struct Rect {
  int x1 = 0;
  int y1 = 0;
  int x2 = 0;
  int y2 = 0;

  bool contains(Vec2 p) const {
    return p.x >= x1 && p.x <= x2 && p.y >= y1 && p.y <= y2;
  }
};

enum class EntityType { Robot, ResourceNormal, ResourceHigh, Obstacle, Unknown };

struct Entity {
  EntityType type = EntityType::Unknown;
  std::string id;
  std::string team;
  std::string robot_id;
  Vec2 pos;
  bool has_pos = false;
  int hp = -1;
  int value = 0;
  bool defending = false;
  bool is_protected = false;
};

struct Mission {
  std::string id;
  std::string type;
  int target = 0;
  Vec2 position;
  bool has_position = false;
};

struct Score {
  double resource_value = 0.0;
  double effective_damage = 0.0;
  int mission_completed = 0;
  int invalid_actions = 0;
  std::vector<std::string> mission_completed_ids;
};

struct LastAction {
  bool valid = false;
  int tick = -1;
  std::string action;
  bool moved = false;
  std::string effective_direction;
  std::string error;
};

struct SelfState {
  std::string team;
  std::string robot_id;
  Vec2 pos;
  std::string position_reason;
  int hp = 100;
  bool alive = true;
  bool defending = false;
  int respawn_in = 0;
  bool is_protected = false;
  LastAction last_action;
};

struct Perception {
  int tick = 0;
  std::string session_id;
  int vision_radius = 4;
  SelfState self;
  std::vector<Entity> entities;
  Score score;
  std::string mission_text;
  std::vector<Mission> missions;
  int map_width = 24;
  int map_height = 24;
  std::string event_text;
};

// 解析 /team_X/perception 的 data 字段，失败时 error 给出原因
bool parse_perception(const std::string& payload, Perception& out, std::string& error);

// 由两格位移得到协议方向名 up/down/left/right，非法时返回 nullptr
const char* direction_name(Vec2 delta);

// 9x9 相对位置图 -> 相对偏移；失败返回 false
bool decode_relative_grid(const std::string& grid, Vec2& offset);

const char* entity_type_name(EntityType type);

}  // namespace rmtc
