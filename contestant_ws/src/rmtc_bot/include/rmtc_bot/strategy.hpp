// 两阶段决策：第一阶段纯采集，第二阶段任务+资源+对抗
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "rmtc_bot/perception.hpp"
#include "rmtc_bot/world_memory.hpp"

namespace rmtc {

enum class DecisionType { Collect, Balanced, Aggressive };
DecisionType decision_type_from_string(const std::string& s);

enum class LlmMode { None, Collect, Reach, Fight, Hold, Survive };

// LLM 只给高层建议，逐 Tick 执行仍由本地策略完成
struct LlmAdvice {
  bool has_advice = false;
  int stamp = -1;
  LlmMode mode = LlmMode::None;
  std::optional<Vec2> go;
  std::vector<Rect> avoid;
  std::string note;
};

struct ActionCmd {
  std::string type = "wait";
  std::string direction;
};

class Strategy {
 public:
  void reset();
  void set_decision_type(DecisionType t) { decision_type_ = t; }
  ActionCmd decide(const Perception& p, WorldMemory& mem, const LlmAdvice& advice, int stage);

 private:
  ActionCmd decide_impl(const Perception& p, WorldMemory& mem, const LlmAdvice& advice, int stage);

  DecisionType decision_type_ = DecisionType::Balanced;
  std::string last_attempt_dir_;
  int last_attempt_tick_ = -1;
  Vec2 last_pos_{0, 0};
  int stay_ticks_ = 0;
};

}  // namespace rmtc
