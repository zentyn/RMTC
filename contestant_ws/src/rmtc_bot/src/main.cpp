// RMTC 选手程序主节点
// 订阅 /team_X/perception，发布 /team_X/action；可选接统一 AI 网关。
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

#include "rmtc_bot/http_client.hpp"
#include "rmtc_bot/json_lite.hpp"
#include "rmtc_bot/llm_advisor.hpp"
#include "rmtc_bot/perception.hpp"
#include "rmtc_bot/strategy.hpp"
#include "rmtc_bot/world_memory.hpp"

namespace {

struct Options {
  std::string team = "A";
  std::string name = "我的队伍";
  std::string gateway = "http://127.0.0.1:8080";
  std::string token;
  std::string model;
  std::string decision_config;
  std::string decision_type = "balanced";
  int stage = 0;  // 0 = 自动
  int min_interval_ms = 12000;
  int max_tokens = 512;
  bool no_ai = false;
  bool verbose = false;
  bool help = false;
};

void print_usage() {
  std::printf(
      "用法: rmtc_bot_node --team A --name 队名 [选项]\n"
      "  --team, -t A/B/C/D      本场席位（默认 A）\n"
      "  --name, -n 队名         注册显示名称\n"
      "  --gateway, -g URL       AI 网关地址（默认 http://127.0.0.1:8080）\n"
      "  --token TOKEN           席位令牌，缺省读环境变量 RMTC_TEAM_TOKEN\n"
      "  --model NAME            指定模型名（默认 deepseek-flash）\n"
      "  --stage 0|1|2           阶段，0=按地图尺寸自动判断\n"
      "  --decision-config PATH  decision_config.json 路径\n"
      "  --min-interval-ms N     LLM 最小调用间隔（默认 12000）\n"
      "  --max-tokens N          按赛前公布的输出限制设置（默认 512）\n"
      "  --no-ai                 关闭 AI 调用，只用本地策略\n"
      "  --verbose               打印每个 Tick 的决策\n"
      "  --help, -h              显示本帮助\n");
}

Options parse_args(int argc, char** argv) {
  Options opt;
  const char* token_env = std::getenv("RMTC_TEAM_TOKEN");
  if (token_env != nullptr) opt.token = token_env;
  const char* gateway_env = std::getenv("RMTC_AI_URL");
  if (gateway_env != nullptr && gateway_env[0] != '\0') opt.gateway = gateway_env;

  auto take_value = [&](int& i, const std::string& arg, const std::string& key) -> std::string {
    std::size_t eq = arg.find('=');
    if (eq != std::string::npos) return arg.substr(eq + 1);
    if (i + 1 < argc) return argv[++i];
    return "";
  };

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    std::string head = arg.substr(0, arg.find('='));
    if (head == "--team" || head == "-t") {
      opt.team = take_value(i, arg, head);
    } else if (head == "--name" || head == "-n") {
      opt.name = take_value(i, arg, head);
    } else if (head == "--gateway" || head == "-g") {
      opt.gateway = take_value(i, arg, head);
    } else if (head == "--token") {
      opt.token = take_value(i, arg, head);
    } else if (head == "--model") {
      opt.model = take_value(i, arg, head);
    } else if (head == "--stage") {
      opt.stage = std::atoi(take_value(i, arg, head).c_str());
    } else if (head == "--decision-config") {
      opt.decision_config = take_value(i, arg, head);
    } else if (head == "--min-interval-ms") {
      opt.min_interval_ms = std::atoi(take_value(i, arg, head).c_str());
    } else if (head == "--max-tokens") {
      opt.max_tokens = std::atoi(take_value(i, arg, head).c_str());
    } else if (head == "--no-ai") {
      opt.no_ai = true;
    } else if (head == "--verbose") {
      opt.verbose = true;
    } else if (head == "--help" || head == "-h") {
      opt.help = true;
    }
  }

  if (!opt.decision_config.empty()) {
    std::ifstream in(opt.decision_config);
    if (in) {
      std::stringstream buffer;
      buffer << in.rdbuf();
      jlite::Value cfg;
      std::string err;
      if (jlite::Value::parse(buffer.str(), cfg, &err) && cfg.is_object()) {
        std::string type = cfg.get_string("decision_type");
        if (!type.empty()) opt.decision_type = type;
      }
    }
  }
  return opt;
}

std::string build_action_json(const rmtc::Perception& p, const rmtc::ActionCmd& cmd) {
  jlite::Value v = jlite::Value::make_object();
  v.set("tick", jlite::Value(p.tick));
  v.set("session_id", jlite::Value(p.session_id));
  v.set("action", jlite::Value(cmd.type));
  if (!cmd.direction.empty()) v.set("direction", jlite::Value(cmd.direction));
  return v.dump();
}

std::string build_registration_json(const Options& opt) {
  jlite::Value v = jlite::Value::make_object();
  v.set("team", jlite::Value(opt.team));
  v.set("name", jlite::Value(opt.name));
  return v.dump();
}

}  // namespace

class BotNode : public rclcpp::Node {
 public:
  explicit BotNode(const Options& opt) : rclcpp::Node("rmtc_bot"), opt_(opt) {
    const std::string team = opt_.team;
    team_ = team;
    stage_ = opt_.stage;

    auto qos_stream = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    auto qos_latch = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    auto qos_event = rclcpp::QoS(rclcpp::KeepLast(128)).reliable().transient_local();

    action_pub_ = create_publisher<std_msgs::msg::String>("/team_" + team_ + "/action", qos_stream);
    reg_pub_ = create_publisher<std_msgs::msg::String>("/team_" + team_ + "/registration", qos_latch);
    perception_sub_ = create_subscription<std_msgs::msg::String>(
        "/team_" + team_ + "/perception", qos_stream,
        [this](const std_msgs::msg::String::SharedPtr msg) { on_perception(msg); });
    mission_sub_ = create_subscription<std_msgs::msg::String>(
        "/referee/mission", qos_latch,
        [this](const std_msgs::msg::String::SharedPtr msg) { on_mission(msg); });
    event_sub_ = create_subscription<std_msgs::msg::String>(
        "/referee/event_text", qos_event,
        [this](const std_msgs::msg::String::SharedPtr msg) { on_event(msg); });
    registry_sub_ = create_subscription<std_msgs::msg::String>(
        "/referee/team_registry", qos_latch,
        [this](const std_msgs::msg::String::SharedPtr msg) { on_registry(msg); });

    memory_.configure(24, 24);
    strategy_.set_decision_type(rmtc::decision_type_from_string(opt_.decision_type));

    rmtc::LlmConfig cfg;
    cfg.gateway = opt_.gateway;
    cfg.token = opt_.token;
    cfg.model = opt_.model;
    if (cfg.model.empty()) cfg.model = "deepseek-flash";
    cfg.min_interval_ms = opt_.min_interval_ms;
    cfg.max_tokens = opt_.max_tokens;
    cfg.enabled = !opt_.no_ai;
    advisor_ = std::make_unique<rmtc::LlmAdvisor>(cfg);

    publish_registration();
    timer_ = create_wall_timer(std::chrono::milliseconds(2000), [this] { on_timer(); });

    if (advisor_->enabled()) {
      std::string base = opt_.gateway;
      while (!base.empty() && base.back() == '/') base.pop_back();
      const std::string health = base + "/healthz";
      rmtc::HttpResult ping = rmtc::HttpClient::get(health, 800);
      RCLCPP_INFO(get_logger(), "AI 网关 %s -> status=%d%s", health.c_str(), ping.status,
                  ping.ok() ? "" : "（不可用，将回退本地策略）");
      advisor_->start();
    } else {
      RCLCPP_INFO(get_logger(), "AI 调用已关闭，使用纯本地策略");
    }

    RCLCPP_INFO(get_logger(), "席位 %s，队名 %s，阶段 %s，决策类型 %s", team_.c_str(),
                opt_.name.c_str(), stage_ == 0 ? "自动" : std::to_string(stage_).c_str(),
                opt_.decision_type.c_str());
  }

  ~BotNode() override {
    if (advisor_) advisor_->stop();
  }

 private:
  void publish_registration() {
    std_msgs::msg::String msg;
    msg.data = build_registration_json(opt_);
    reg_pub_->publish(msg);
  }

  void on_perception(const std_msgs::msg::String::SharedPtr msg) {
    rmtc::Perception p;
    std::string err;
    if (!rmtc::parse_perception(msg->data, p, err)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "感知解析失败: %s", err.c_str());
      return;
    }

    if (p.session_id != session_) {
      session_ = p.session_id;
      memory_.configure(p.map_width, p.map_height);
      strategy_.reset();
      if (opt_.stage == 0) stage_ = 0;  // 每场重新识别阶段
      if (advisor_) advisor_->set_session(p.session_id);
      last_tick_ = -1;
      last_action_json_.clear();
      want_first_ai_ = true;
      RCLCPP_INFO(get_logger(), "新场次 %s，地图 %dx%d", session_.c_str(), p.map_width,
                  p.map_height);
    }

    if (p.tick < last_tick_) {
      return;  // 过期感知，不能再用旧 Tick 提交动作
    }

    if (p.tick == last_tick_ && !last_action_json_.empty()) {
      // 同一 Tick 的重复感知不是新时间步，重复提交缓存动作即可
      publish_action_raw(last_action_json_);
      return;
    }

    if (opt_.stage == 0) {
      // 小地图且没有敌人 → 第一阶段；看到敌人或大地图 → 第二阶段。
      // 一旦升级到第二阶段，本场不再降级，避免敌人离开视野后停止战斗。
      bool enemy_visible = false;
      for (const rmtc::Entity& e : p.entities) {
        if (e.type == rmtc::EntityType::Robot && e.team != p.self.team && e.hp > 0) {
          enemy_visible = true;
          break;
        }
      }
      const bool small_map = p.map_width <= 20 && p.map_height <= 20;
      const int detected = (small_map && !enemy_visible) ? 1 : 2;
      if (detected > stage_) {
        stage_ = detected;
        RCLCPP_INFO(get_logger(), "阶段识别为 %d（地图 %dx%d，敌人%s）", stage_, p.map_width,
                    p.map_height, enemy_visible ? "可见" : "不可见");
      }
    }

    memory_.update(p);

    const std::string event_text = p.event_text.empty() ? event_text_ : p.event_text;
    if (!p.event_text.empty() && p.event_text != event_text_) {
      event_text_ = p.event_text;
      want_event_ai_ = true;
    }
    if (!p.mission_text.empty()) mission_text_ = p.mission_text;

    if (advisor_ && advisor_->enabled() && stage_ >= 2) {
      if (want_first_ai_) {
        advisor_->maybe_request(p, mission_text_, event_text, true, "开场规划");
        want_first_ai_ = false;
      } else if (want_event_ai_) {
        advisor_->maybe_request(p, mission_text_, event_text, true, "理解随机事件");
        want_event_ai_ = false;
      } else {
        advisor_->maybe_request(p, mission_text_, event_text, false, "周期更新");
      }
    }

    rmtc::ActionCmd cmd = strategy_.decide(p, memory_, advisor_ ? advisor_->advice() : rmtc::LlmAdvice{},
                                           stage_);
    if (stage_ == 1 && cmd.type != "move" && cmd.type != "wait") {
      cmd.type = "wait";  // 第一阶段只允许 MOVE / WAIT
      cmd.direction.clear();
    }

    const std::string action_json = build_action_json(p, cmd);
    publish_action_raw(action_json);
    last_tick_ = p.tick;
    last_action_json_ = action_json;
    last_perception_ = p;

    if (opt_.verbose || p.tick % 10 == 0) {
      RCLCPP_INFO(get_logger(), "tick=%d pos=[%d,%d] hp=%d %s %s 资源%d 伤害%.0f 任务%d", p.tick,
                  p.self.pos.x, p.self.pos.y, p.self.hp, cmd.type.c_str(), cmd.direction.c_str(),
                  static_cast<int>(memory_.resources().size()), p.score.effective_damage,
                  p.score.mission_completed);
    }
  }

  void publish_action_raw(const std::string& payload) {
    std_msgs::msg::String msg;
    msg.data = payload;
    action_pub_->publish(msg);
  }

  void on_mission(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data.empty()) return;
    if (msg->data != mission_text_) {
      mission_text_ = msg->data;
      RCLCPP_INFO(get_logger(), "收到任务公告: %s", mission_text_.substr(0, 120).c_str());
      want_event_ai_ = true;  // 任务变化也请模型重新规划
    }
  }

  void on_event(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data.empty()) return;
    if (msg->data != event_text_) {
      event_text_ = msg->data;
      want_event_ai_ = true;
      RCLCPP_INFO(get_logger(), "收到随机事件: %s", event_text_.substr(0, 120).c_str());
    }
  }

  void on_registry(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "席位名单: %s",
                         msg->data.substr(0, 160).c_str());
  }

  void on_timer() {
    publish_registration();
    if (!advisor_) return;
    // 第二阶段每场必须有至少一次真实成功的 AI 调用，否则任务分 M=0
    if (stage_ >= 2 && advisor_->enabled() && advisor_->successes() == 0 &&
        advisor_->failures() < 20 && last_perception_.has_value()) {
      advisor_->maybe_request(*last_perception_, mission_text_, event_text_, true,
                              "确保本场有一次成功调用");
    }
    if (advisor_->last_error() != last_ai_error_ && !advisor_->last_error().empty()) {
      last_ai_error_ = advisor_->last_error();
      RCLCPP_WARN(get_logger(), "AI 调用失败: %s", last_ai_error_.c_str());
    }
  }

  Options opt_;
  std::string team_;
  int stage_ = 0;
  std::string session_;
  int last_tick_ = -1;
  std::string last_action_json_;
  std::string mission_text_;
  std::string event_text_;
  std::string last_ai_error_;
  bool want_first_ai_ = true;
  bool want_event_ai_ = false;

  rmtc::WorldMemory memory_;
  rmtc::Strategy strategy_;
  std::unique_ptr<rmtc::LlmAdvisor> advisor_;
  std::optional<rmtc::Perception> last_perception_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr action_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reg_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr perception_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr registry_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  Options opt = parse_args(argc, argv);
  if (opt.help) {
    print_usage();
    return 0;
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BotNode>(opt);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
