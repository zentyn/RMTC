// ============================================================================
//  ai_client.hpp —— 官方统一 AI 网关客户端（异步、不阻塞动作）
// ============================================================================
//
//  【为什么必须异步 —— 这是本模块最重要的设计约束】
//  规则 §11.1："AI 超时或输出不合适，不延长动作窗口。"
//  协议 §5："实际机器人宜异步请求并设置本地降级，不能让模型等待阻塞动作窗口。"
//
//  第二阶段 500ms/Tick。如果在这里同步等 HTTP 响应，网关读取超时是 5 秒，
//  也就是说一次卡顿就能让你连丢 10 个 Tick 的动作 —— 直接判负。
//
//  所以架构是：
//    * 后台线程负责发 HTTP 请求、收响应、解析；
//    * 主循环（ROS 回调）只调 latestAdvice()，拿"最近一次成功的结果"，
//      永远不等待网络；
//    * 网络慢或坏了 → advice 保持旧值或变无效 → 策略自动退回本地保守打法。
//
//  【为什么用原始 socket 而不是 libcurl】
//  少一个外部依赖 = 少一个环境出问题的可能。这里只需要发一个 POST、
//  收一个完整响应，几十行 socket 代码就够，而且能精确控制超时。
//
//  【协议规定的请求约束，本模块全部落实】
//  1. 请求头必须有 Content-Type: application/json
//     和 Authorization: Bearer <本场席位令牌>（协议 §5）；
//  2. 必须用 Content-Length 发送，带 Transfer-Encoding 会被返回 501；
//  3. 请求体上限 2 MiB，超过返回 413；
//  4. messages 必须是非空数组；
//  5. stream 必须省略或为 false（测试端只支持非流式）；
//  6. 每个席位令牌最多 2 个在途调用，超了会被 429 rate_limit；
//  7. 只有 2xx 才消耗额度；429 + quota_exceeded 表示额度耗尽，必须降级；
//  8. 令牌绝不能写进日志或公开仓库（红线 P3）。
// ============================================================================

#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rmtc_bot/strategy.hpp"   // 为了拿到 EventAdvice

namespace rmtc {

// 一次 AI 调用的结果
struct AiResult {
    bool ok = false;              // 是否成功（HTTP 2xx 且解析出内容）
    std::string content;          // 模型返回的文本内容
    int httpStatus = 0;           // HTTP 状态码
    std::string errorCode;        // 协议里的 code 字段，如 quota_exceeded
    std::string errorMessage;     // 错误信息（写日志用）
    bool quotaExhausted = false;  // 额度已耗尽 → 之后不再尝试，直接降级
    int elapsedMs = 0;            // 本次耗时，用于判断网关是否变慢
};

class AiClient {
public:
    // gatewayBase：完整请求地址的"基址"部分，不含 /v1/chat/completions。
    //   练习时是 http://127.0.0.1:8080
    //   正式比赛用主办方发放的地址（协议 §7.3）
    // token：本场席位令牌。练习时是 practice-A/B/C/D，正式比赛用裁判发的。
    //   【红线】令牌不得写入公开日志、不得提交进代码仓库。
    AiClient(const std::string& gatewayBase, const std::string& token);
    ~AiClient();

    // 启动后台线程。enabled=false 时不启动（用于完全不调 AI 的调试）
    void start(bool enabled = true);
    // 停止后台线程并等待退出（析构时会自动调）
    void stop();

    bool enabled() const { return enabled_; }
    bool quotaExhausted() const { return quotaExhausted_.load(); }

    // ---- 发起一次异步请求（立即返回，不阻塞）----
    // prompt 是要发给模型的完整提示词。
    // 返回 false 表示请求没能入队（已在跑、额度耗尽、未启用等），
    // 此时调用方应继续使用旧 advice。
    bool submitEventQuery(const std::string& prompt, int tick, const std::string& sessionId);

    // ---- 主循环只调这个：拿最近一次成功解析出的事件提示（永不阻塞）----
    EventAdvice latestAdvice() const;

    // ---- 统计信息（写日志、答辩用）----
    int successCount() const { return successCount_.load(); }
    int failureCount() const { return failureCount_.load(); }
    int submittedCount() const { return submittedCount_.load(); }
    std::string lastError() const;

    // 是否已经有过本场成功调用（规则 §10.2：本场必须有至少一次真实成功调用，
    // 否则任务分 M=0）。这个标记要写进日志，方便你赛前确认。
    bool hasSuccessThisSession() const { return successCount_.load() > 0; }

    // 换场时重置统计（协议 §4：换场须清空上一场缓存）
    void onNewSession();

    // 记录当前场次。异步结果回来时若场次已变，该结果必须丢弃 ——
    // 协议 §4："异步 AI 或其他后台任务中属于其他场次或早于当前 Tick 的结果
    // 不得直接作为当前动作发布。"
    void setCurrentSession(const std::string& sessionId);
    std::string currentSession() const;

    // 设置本场地图尺寸（从感知的 map 字段读）。
    // 用途：大模型可能给出越界坐标，解析时按这个尺寸过滤掉。
    void setMapSize(int w, int h) {
        mapW_.store(w);
        mapH_.store(h);
    }

    // ---- 同步调用（只用于启动自检，绝不在动作循环里用）----
    // timeoutMs：本地等待上限。协议说网关读取超时 5 秒、上游超时默认 30 秒，
    // 这里默认给 8 秒，够自检用。
    AiResult askOnce(const std::string& prompt, int timeoutMs = 8000);

    // 健康检查：GET <gateway>/healthz（协议 §5 的公开端点）
    bool healthCheck(int timeoutMs = 5000, std::string* detail = nullptr);

    // 组装发给模型的提示词。集中在这里便于你调 Prompt（规则 §2.3 允许在
    // 第一阶段结束后的统一调整窗口修改 Prompt）。
    static std::string buildEventPrompt(const std::string& snapshot,
                                        const std::string& eventText,
                                        int tick);

private:
    std::string gatewayBase_;
    std::string token_;
    bool enabled_ = true;

    // ---- 后台线程 ----
    mutable std::mutex mtx_;
    std::condition_variable cv_;
    std::thread worker_;
    bool stopFlag_ = false;

    struct PendingRequest {
        std::string prompt;
        int tick = -1;
        std::string sessionId;
    };
    // 只保留最新一条请求：旧请求即使算完也已经过期了（协议 §4：
    // 早于当前 Tick 的异步结果不得直接作为当前动作发布）
    bool hasPending_ = false;
    PendingRequest pending_;

    // ---- 最近一次结果 ----
    mutable std::mutex adviceMtx_;
    EventAdvice latestAdvice_;
    AiResult lastResult_;

    std::atomic<bool> quotaExhausted_{false};
    std::atomic<int> successCount_{0};
    std::atomic<int> failureCount_{0};
    std::atomic<int> submittedCount_{0};
    std::atomic<bool> busy_{false};

    // 当前场次（跨场过期结果必须丢弃，见协议 §4）
    mutable std::mutex sessionMtx_;
    std::string currentSession_;

    void workerLoop();

    // 解析 HTTP 响应
    static bool parseHttpResponse(const std::string& raw, AiResult& out);

    // 从 URL 里拆出 host / port / path
    static bool splitUrl(const std::string& url, std::string& host,
                         int& port, std::string& path);

    // 发一次原始 HTTP POST，返回完整响应文本
    static bool httpPost(const std::string& host, int port, const std::string& path,
                         const std::string& body, const std::vector<std::string>& headers,
                         int timeoutMs, std::string& rawResponse, std::string& err);
    static bool httpGet(const std::string& host, int port, const std::string& path,
                        int timeoutMs, std::string& rawResponse, std::string& err);
};

}  // namespace rmtc
