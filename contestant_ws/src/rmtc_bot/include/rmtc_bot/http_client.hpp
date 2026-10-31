// 极简 HTTP/1.1 客户端（仅明文 http），只用于调用裁判 AI 网关。
// 不引入 libcurl，比赛机器上少一个依赖。
#pragma once

#include <string>

namespace rmtc {

struct HttpResult {
  int status = 0;
  std::string body;
  std::string error;

  bool ok() const { return status >= 200 && status < 300; }
};

class HttpClient {
 public:
  // url 形如 http://127.0.0.1:8080/v1/chat/completions
  static HttpResult post_json(const std::string& url, const std::string& bearer_token,
                              const std::string& json_body, int timeout_ms);

  static HttpResult get(const std::string& url, int timeout_ms);
};

}  // namespace rmtc
