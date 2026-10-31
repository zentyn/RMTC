#include "rmtc_bot/perception.hpp"

#include <cmath>
#include <cstdlib>
#include <sstream>

#include "rmtc_bot/json_lite.hpp"

namespace rmtc {
namespace {

EntityType entity_type_from(const std::string& s) {
  if (s == "robot") return EntityType::Robot;
  if (s == "resource_normal") return EntityType::ResourceNormal;
  if (s == "resource_high") return EntityType::ResourceHigh;
  if (s == "obstacle") return EntityType::Obstacle;
  return EntityType::Unknown;
}

bool parse_vec2(const jlite::Value& v, Vec2& out) {
  if (!v.is_array() || v.items().size() < 2) return false;
  const jlite::Value& x = v[static_cast<std::size_t>(0)];
  const jlite::Value& y = v[static_cast<std::size_t>(1)];
  if (!x.is_number() || !y.is_number()) return false;
  out.x = static_cast<int>(std::lround(x.as_number()));
  out.y = static_cast<int>(std::lround(y.as_number()));
  return true;
}

void parse_last_action(const jlite::Value& v, LastAction& out) {
  if (!v.is_object()) return;
  out.valid = true;
  out.tick = v.get_int("tick", -1);
  out.action = v.get_string("action");
  out.moved = v.get_bool("moved", false);
  out.effective_direction = v.get_string("effective_direction");
  out.error = v.get_string("error");
}

}  // namespace

bool decode_relative_grid(const std::string& grid, Vec2& offset) {
  std::vector<std::string> rows;
  std::string line;
  for (char c : grid) {
    if (c == '\n') {
      rows.push_back(line);
      line.clear();
    } else if (c != '\r') {
      line.push_back(c);
    }
  }
  rows.push_back(line);
  if (rows.size() < 9) return false;
  for (int r = 0; r < 9; ++r) {
    const std::string& row = rows[static_cast<std::size_t>(r)];
    if (row.size() < 9) return false;
    for (int c = 0; c < 9; ++c) {
      if (row[static_cast<std::size_t>(c)] == '2') {
        offset.x = c - 4;
        offset.y = r - 4;
        return true;
      }
    }
  }
  offset.x = 0;
  offset.y = 0;
  return true;
}

bool parse_perception(const std::string& payload, Perception& out, std::string& error) {
  jlite::Value root;
  std::string json_error;
  if (!jlite::Value::parse(payload, root, &json_error)) {
    error = "E_PARSE: " + json_error;
    return false;
  }
  if (!root.is_object()) {
    error = "E_SCHEMA: root is not object";
    return false;
  }

  out = Perception();
  out.tick = root.get_int("tick", -1);
  if (out.tick < 0) {
    error = "E_SCHEMA: missing tick";
    return false;
  }
  out.session_id = root.get_string("session_id");
  if (out.session_id.empty()) {
    error = "E_SCHEMA: missing session_id";
    return false;
  }
  out.vision_radius = root.get_int("vision_radius", 4);

  const jlite::Value& self = root["self"];
  if (!self.is_object()) {
    error = "E_SCHEMA: missing self";
    return false;
  }
  out.self.team = self.get_string("team");
  out.self.robot_id = self.get_string("robot_id", out.self.team);
  if (!parse_vec2(self["position"], out.self.pos)) {
    error = "E_SCHEMA: missing self.position";
    return false;
  }
  out.self.position_reason = self.get_string("position_reason");
  out.self.hp = self.get_int("hp", 100);
  out.self.alive = self.get_bool("alive", true);
  out.self.defending = self.get_bool("defending", false);
  out.self.respawn_in = self.get_int("respawn_in", 0);
  out.self.is_protected = self.get_bool("protected", false);
  parse_last_action(self["last_action"], out.self.last_action);

  const jlite::Value& entities = root["visible_entities"];
  if (entities.is_array()) {
    for (const jlite::Value& item : entities.items()) {
      Entity e;
      e.type = entity_type_from(item.get_string("type"));
      if (e.type == EntityType::Unknown) continue;
      e.id = item.get_string("id");
      e.team = item.get_string("team");
      e.robot_id = item.get_string("robot_id");
      e.hp = item.get_int("hp", -1);
      e.value = item.get_int("value", 0);
      e.defending = item.get_bool("defending", false);
      e.is_protected = item.get_bool("protected", false);

      const jlite::Value& pos = item["position"];
      if (!pos.is_string()) continue;
      Vec2 offset;
      if (!decode_relative_grid(pos.as_string(), offset)) continue;
      e.pos.x = out.self.pos.x + offset.x;
      e.pos.y = out.self.pos.y + offset.y;
      e.has_pos = true;
      out.entities.push_back(e);
    }
  }

  const jlite::Value& score = root["score"];
  if (score.is_object()) {
    out.score.resource_value = score.get_number("resource_value", 0.0);
    out.score.effective_damage = score.get_number("effective_damage", 0.0);
    out.score.mission_completed = score.get_int("mission_completed", 0);
    out.score.invalid_actions = score.get_int("invalid_actions", 0);
    const jlite::Value& ids = score["mission_completed_ids"];
    if (ids.is_array()) {
      for (const jlite::Value& id : ids.items()) {
        if (id.is_string()) out.score.mission_completed_ids.push_back(id.as_string());
      }
    }
  }

  out.mission_text = root.get_string("mission_text");

  const jlite::Value& missions = root["missions"];
  if (missions.is_array()) {
    for (const jlite::Value& item : missions.items()) {
      Mission m;
      m.id = item.get_string("id");
      m.type = item.get_string("type");
      m.target = item.get_int("target", 0);
      m.has_position = parse_vec2(item["position"], m.position);
      out.missions.push_back(m);
    }
  }

  const jlite::Value& map = root["map"];
  if (map.is_object()) {
    out.map_width = map.get_int("width", 24);
    out.map_height = map.get_int("height", 24);
  }
  if (out.map_width <= 0) out.map_width = 24;
  if (out.map_height <= 0) out.map_height = 24;

  out.event_text = root.get_string("event_text");
  return true;
}

const char* direction_name(Vec2 delta) {
  if (delta.x == 1 && delta.y == 0) return "right";
  if (delta.x == -1 && delta.y == 0) return "left";
  if (delta.x == 0 && delta.y == 1) return "down";
  if (delta.x == 0 && delta.y == -1) return "up";
  return nullptr;
}

const char* entity_type_name(EntityType type) {
  switch (type) {
    case EntityType::Robot: return "robot";
    case EntityType::ResourceNormal: return "resource_normal";
    case EntityType::ResourceHigh: return "resource_high";
    case EntityType::Obstacle: return "obstacle";
    default: return "unknown";
  }
}

}  // namespace rmtc
