#include "rmtc_bot/llm_advisor.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <sstream>

#include "rmtc_bot/http_client.hpp"
#include "rmtc_bot/json_lite.hpp"

namespace rmtc {
namespace {

const char* kSystemPrompt =
    "你是 RoboMaster 图灵对决比赛的机器人决策助手。"
    "你只输出一个 JSON 对象，不要输出解释文字、不要用代码块。"
    "JSON 字段："
    "\"mode\"：必须是 collect / reach / fight / hold / survive 之一；"
    "\"go\"：[x,y] 整数坐标或 null，表示接下来优先前往的格子；"
    "\"avoid\"：矩形数组，每个矩形是 [x1,y1,x2,y2]，表示要避开的绝对坐标区域（含边界），没有就写 []；"
    "\"note\"：一句话中文说明。"
    "地图坐标 x 向右增大，y 向下增大，范围由用户消息给出。"
    "决定要保守：看不清危险时优先 collect。";

long long now_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

std::string join_url(const std::string& base, const std::string& path) {
  std::string b = base;
  while (!b.empty() && b.back() == '/') b.pop_back();
  return b + path;
}

LlmMode mode_from_string(const std::string& s) {
  if (s == "collect") return LlmMode::Collect;
  if (s == "reach") return LlmMode::Reach;
  if (s == "fight") return LlmMode::Fight;
  if (s == "hold") return LlmMode::Hold;
  if (s == "survive") return LlmMode::Survive;
  return LlmMode::None;
}

std::string extract_json_object(const std::string& text) {
  std::size_t begin = text.find('{');
  std::size_t end = text.rfind('}');
  if (begin == std::string::npos || end == std::string::npos || end <= begin) return "";
  return text.substr(begin, end - begin + 1);
}

}  // namespace

LlmAdvisor::LlmAdvisor(const LlmConfig& cfg) : cfg_(cfg) {}

LlmAdvisor::~LlmAdvisor() { stop(); }

void LlmAdvisor::start() {
  if (thread_.joinable()) return;
  stop_ = false;
  thread_ = std::thread([this] { worker(); });
}

void LlmAdvisor::stop() {
  if (!thread_.joinable()) return;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
    pending_.reset();
  }
  cv_.notify_all();
  thread_.join();
}

void LlmAdvisor::maybe_request(const Perception& p, const std::string& mission_text,
                               const std::string& event_text, bool force, const char* reason) {
  if (!enabled()) return;
  const long long now = now_ms();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!force && now - last_request_ms_ < cfg_.min_interval_ms) return;
    last_request_ms_ = now;
  }

  std::ostringstream oss;
  oss << "地图 " << p.map_width << "x" << p.map_height << "，坐标范围 x:0-" << (p.map_width - 1)
      << " y:0-" << (p.map_height - 1) << "。\n";
  oss << "当前 tick=" << p.tick << "，session=" << p.session_id << "。\n";
  oss << "本车位置 [" << p.self.pos.x << "," << p.self.pos.y << "]，HP=" << p.self.hp;
  oss << (p.self.alive ? "（存活）" : "（阵亡）");
  oss << (p.self.is_protected ? "（出生保护中）" : "") << "。\n";
  oss << "已得分：资源价值=" << p.score.resource_value
      << "，有效伤害=" << p.score.effective_damage
      << "，完成任务数=" << p.score.mission_completed << "。\n";

  if (!mission_text.empty()) oss << "任务公告：" << mission_text << "\n";
  if (!event_text.empty()) oss << "随机事件公告：" << event_text << "\n";

  int enemy_count = 0;
  int resource_count = 0;
  int nearest_enemy_dist = 99;
  for (const Entity& e : p.entities) {
    if (e.type == EntityType::Robot && e.team != p.self.team) {
      ++enemy_count;
      int d = std::abs(e.pos.x - p.self.pos.x) + std::abs(e.pos.y - p.self.pos.y);
      nearest_enemy_dist = std::min(nearest_enemy_dist, d);
    } else if (e.type == EntityType::ResourceNormal || e.type == EntityType::ResourceHigh) {
      ++resource_count;
    }
  }
  oss << "视野内敌人 " << enemy_count << " 个";
  if (enemy_count > 0) oss << "，最近敌人曼哈顿距离 " << nearest_enemy_dist;
  oss << "；视野内资源 " << resource_count << " 个。\n";
  oss << "请给出接下来一段时间的高层决策（" << reason << "）。";

  Job job;
  job.user_content = oss.str();
  job.reason = reason;
  job.session_id = p.session_id;
  job.tick = p.tick;
  job.map_width = p.map_width;
  job.map_height = p.map_height;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_ = job;
  }
  cv_.notify_one();
}

void LlmAdvisor::set_session(const std::string& session_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (session_id == current_session_) return;
  current_session_ = session_id;
  advice_ = LlmAdvice{};
  pending_.reset();
  last_error_.clear();
  successes_ = 0;
  failures_ = 0;
}

void LlmAdvisor::worker() {
  while (true) {
    Job job;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this] { return stop_ || pending_.has_value(); });
      if (stop_) return;
      job = *pending_;
      pending_.reset();
    }
    request_once(job);
  }
}

bool LlmAdvisor::request_once(const Job& job) {
  jlite::Value messages = jlite::Value::make_array();
  jlite::Value sys = jlite::Value::make_object();
  sys.set("role", jlite::Value("system"));
  sys.set("content", jlite::Value(kSystemPrompt));
  messages.push_back(sys);
  jlite::Value user = jlite::Value::make_object();
  user.set("role", jlite::Value("user"));
  user.set("content", jlite::Value(job.user_content));
  messages.push_back(user);

  jlite::Value body = jlite::Value::make_object();
  body.set("messages", messages);
  body.set("stream", jlite::Value(false));
  body.set("temperature", jlite::Value(0.2));
  if (cfg_.max_tokens > 0) body.set("max_tokens", jlite::Value(cfg_.max_tokens));
  if (!cfg_.model.empty()) body.set("model", jlite::Value(cfg_.model));
  // 官方网关按 DeepSeek 官方接口转发：思考默认开启，必须显式关闭；
  // response_format=json_object 要求提示词包含 JSON 字样（系统提示已满足）
  jlite::Value thinking = jlite::Value::make_object();
  thinking.set("type", jlite::Value("disabled"));
  body.set("thinking", thinking);
  jlite::Value response_format = jlite::Value::make_object();
  response_format.set("type", jlite::Value("json_object"));
  body.set("response_format", response_format);

  HttpResult resp = HttpClient::post_json(join_url(cfg_.gateway, "/v1/chat/completions"),
                                          cfg_.token, body.dump(), cfg_.timeout_ms);
  if (!resp.ok()) {
    ++failures_;
    std::ostringstream oss;
    oss << "http status " << resp.status;
    if (!resp.error.empty()) oss << " (" << resp.error << ")";
    if (!resp.body.empty()) oss << " body=" << resp.body.substr(0, 200);
    std::lock_guard<std::mutex> lock(mutex_);
    last_error_ = oss.str();
    return false;
  }

  jlite::Value root;
  std::string err;
  if (!jlite::Value::parse(resp.body, root, &err)) {
    ++failures_;
    std::lock_guard<std::mutex> lock(mutex_);
    last_error_ = "response not json: " + err;
    return false;
  }

  const jlite::Value& content_value = root["choices"][static_cast<std::size_t>(0)]["message"]["content"];
  std::string content;
  if (content_value.is_string()) {
    content = content_value.as_string();
  } else {
    // 兼容 content 为分段数组的返回
    if (content_value.is_array()) {
      for (const jlite::Value& part : content_value.items()) {
        if (part["text"].is_string()) content += part["text"].as_string();
      }
    }
  }
  if (content.empty()) {
    ++failures_;
    std::lock_guard<std::mutex> lock(mutex_);
    last_error_ = "empty model content";
    return false;
  }

  jlite::Value content_json;
  if (!jlite::Value::parse(content, content_json, &err)) {
    std::string extracted = extract_json_object(content);
    if (extracted.empty() || !jlite::Value::parse(extracted, content_json, &err)) {
      ++failures_;
      std::lock_guard<std::mutex> lock(mutex_);
      last_error_ = "model content not json: " + content.substr(0, 160);
      return false;
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (job.session_id != current_session_) {
      // 迟到的上一场结果直接丢弃，不能用于当前场次
      return false;
    }
    apply_content(content_json, job.map_width, job.map_height, job.tick);
  }
  ++successes_;
  return true;
}

// 把模型输出转成受限的 LlmAdvice；坐标越界直接丢弃
void LlmAdvisor::apply_content(const jlite::Value& v, int w, int h, int tick) {
  if (!v.is_object()) return;

  LlmAdvice advice;
  advice.has_advice = true;
  advice.stamp = tick;
  advice.mode = mode_from_string(v.get_string("mode", "collect"));
  advice.note = v.get_string("note");

  const jlite::Value& go = v["go"];
  if (go.is_array() && go.items().size() >= 2) {
    Vec2 p;
    p.x = static_cast<int>(go[static_cast<std::size_t>(0)].as_number(-1));
    p.y = static_cast<int>(go[static_cast<std::size_t>(1)].as_number(-1));
    if (p.x >= 0 && p.y >= 0) {
      if (w > 0 && h > 0 && (p.x >= w || p.y >= h)) {
        // 越界，丢弃
      } else {
        advice.go = p;
      }
    }
  }

  const jlite::Value& avoid = v["avoid"];
  if (avoid.is_array()) {
    for (const jlite::Value& item : avoid.items()) {
      if (!item.is_array() || item.items().size() < 4) continue;
      Rect r;
      r.x1 = static_cast<int>(item[static_cast<std::size_t>(0)].as_number(-1));
      r.y1 = static_cast<int>(item[static_cast<std::size_t>(1)].as_number(-1));
      r.x2 = static_cast<int>(item[static_cast<std::size_t>(2)].as_number(-1));
      r.y2 = static_cast<int>(item[static_cast<std::size_t>(3)].as_number(-1));
      if (r.x1 < 0 || r.y1 < 0 || r.x2 < 0 || r.y2 < 0) continue;
      if (r.x2 < r.x1) std::swap(r.x1, r.x2);
      if (r.y2 < r.y1) std::swap(r.y1, r.y2);
      if (w > 0 && h > 0 && (r.x1 >= w || r.y1 >= h)) continue;
      r.x2 = std::min(r.x2, w > 0 ? w - 1 : r.x2);
      r.y2 = std::min(r.y2, h > 0 ? h - 1 : r.y2);
      advice.avoid.push_back(r);
    }
  }
  advice_ = advice;
}

LlmAdvice LlmAdvisor::advice() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return advice_;
}

std::string LlmAdvisor::last_error() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_error_;
}

}  // namespace rmtc
