// 统一 AI 网关联调：异步调用 + 输出校验，绝不阻塞 Tick 循环
#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "rmtc_bot/json_lite.hpp"
#include "rmtc_bot/perception.hpp"
#include "rmtc_bot/strategy.hpp"

namespace rmtc {

struct LlmConfig {
  std::string gateway = "http://127.0.0.1:8080";
  std::string token;
  std::string model = "deepseek-flash";
  int timeout_ms = 2000;
  int min_interval_ms = 12000;
  int max_tokens = 512;
  bool enabled = true;
};

class LlmAdvisor {
 public:
  explicit LlmAdvisor(const LlmConfig& cfg);
  ~LlmAdvisor();

  void start();
  void stop();

  // 发起一次请求（内部限频，不阻塞）
  void maybe_request(const Perception& p, const std::string& mission_text,
                     const std::string& event_text, bool force, const char* reason);

  // 切换场次：丢弃上一场的建议和在途结果
  void set_session(const std::string& session_id);

  LlmAdvice advice() const;
  long successes() const { return successes_.load(); }
  long failures() const { return failures_.load(); }
  std::string last_error() const;
  bool enabled() const { return cfg_.enabled && !cfg_.token.empty(); }

 private:
  struct Job {
    std::string user_content;
    std::string reason;
    std::string session_id;
    int tick = 0;
    int map_width = 24;
    int map_height = 24;
  };

  void worker();
  bool request_once(const Job& job);
  void apply_content(const jlite::Value& content, int w, int h, int tick);

  LlmConfig cfg_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<Job> pending_;
  LlmAdvice advice_;
  std::string last_error_;
  std::string current_session_;
  bool stop_ = false;
  std::thread thread_;

  std::atomic<long> successes_{0};
  std::atomic<long> failures_{0};
  long long last_request_ms_ = 0;
};

}  // namespace rmtc
