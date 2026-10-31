#include "rmtc_bot/http_client.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <sstream>

namespace rmtc {
namespace {

struct UrlParts {
  std::string host;
  std::string port;
  std::string path;
  bool valid = false;
};

UrlParts parse_url(const std::string& url) {
  UrlParts p;
  const std::string prefix = "http://";
  if (url.compare(0, prefix.size(), prefix) != 0) return p;
  std::string rest = url.substr(prefix.size());
  std::size_t slash = rest.find('/');
  std::string host_port = slash == std::string::npos ? rest : rest.substr(0, slash);
  p.path = slash == std::string::npos ? "/" : rest.substr(slash);
  if (p.path.empty()) p.path = "/";
  std::size_t colon = host_port.rfind(':');
  if (colon == std::string::npos) {
    p.host = host_port;
    p.port = "80";
  } else {
    p.host = host_port.substr(0, colon);
    p.port = host_port.substr(colon + 1);
  }
  p.valid = !p.host.empty() && !p.port.empty();
  return p;
}

int remaining_ms(const std::chrono::steady_clock::time_point& deadline) {
  auto now = std::chrono::steady_clock::now();
  if (now >= deadline) return 0;
  return static_cast<int>(
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
}

bool wait_fd(int fd, bool for_write, int timeout_ms) {
  if (timeout_ms <= 0) return false;
  fd_set set;
  FD_ZERO(&set);
  FD_SET(fd, &set);
  struct timeval tv;
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;
  int rc = select(fd + 1, for_write ? nullptr : &set, for_write ? &set : nullptr, nullptr, &tv);
  return rc > 0;
}

std::string lower_copy(std::string s) {
  for (char& c : s) {
    if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
  }
  return s;
}

std::string trim(const std::string& s) {
  std::size_t b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) return "";
  std::size_t e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

// 拆出 chunked 传输的正文
std::string decode_chunked(const std::string& raw) {
  std::string out;
  std::size_t pos = 0;
  while (pos < raw.size()) {
    std::size_t line_end = raw.find("\r\n", pos);
    if (line_end == std::string::npos) break;
    std::string size_line = raw.substr(pos, line_end - pos);
    std::size_t semi = size_line.find(';');
    if (semi != std::string::npos) size_line = size_line.substr(0, semi);
    unsigned long chunk = std::strtoul(size_line.c_str(), nullptr, 16);
    pos = line_end + 2;
    if (chunk == 0) break;
    if (pos + chunk > raw.size()) chunk = raw.size() - pos;
    out.append(raw, pos, chunk);
    pos += chunk + 2;
  }
  return out;
}

HttpResult request(const std::string& url, const std::string& method, const std::string& body,
                   const std::string& bearer_token, int timeout_ms) {
  HttpResult result;
  UrlParts parts = parse_url(url);
  if (!parts.valid) {
    result.error = "invalid url: " + url;
    return result;
  }

  struct addrinfo hints;
  std::memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  struct addrinfo* addr = nullptr;
  int gai = getaddrinfo(parts.host.c_str(), parts.port.c_str(), &hints, &addr);
  if (gai != 0 || addr == nullptr) {
    result.error = std::string("resolve failed: ") + gai_strerror(gai);
    return result;
  }

  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  int fd = -1;
  for (struct addrinfo* it = addr; it != nullptr; it = it->ai_next) {
    fd = ::socket(it->ai_family, it->ai_socktype, it->ai_protocol);
    if (fd < 0) continue;
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    int rc = ::connect(fd, it->ai_addr, it->ai_addrlen);
    if (rc == 0) break;
    if (errno == EINPROGRESS && wait_fd(fd, true, remaining_ms(deadline))) {
      int so_error = 0;
      socklen_t len = sizeof(so_error);
      if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &so_error, &len) == 0 && so_error == 0) break;
    }
    ::close(fd);
    fd = -1;
  }
  freeaddrinfo(addr);

  if (fd < 0) {
    result.error = "connect failed";
    return result;
  }

  int one = 1;
  ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

  std::ostringstream req;
  req << method << " " << parts.path << " HTTP/1.1\r\n";
  req << "Host: " << parts.host << ":" << parts.port << "\r\n";
  req << "Accept: application/json\r\n";
  req << "Connection: close\r\n";
  req << "User-Agent: rmtc-bot/1.0\r\n";
  if (!bearer_token.empty()) {
    req << "Authorization: Bearer " << bearer_token << "\r\n";
  }
  if (!body.empty() || method == "POST") {
    req << "Content-Type: application/json\r\n";
    req << "Content-Length: " << body.size() << "\r\n";
  }
  req << "\r\n";
  req << body;

  std::string payload = req.str();
  std::size_t sent = 0;
  while (sent < payload.size()) {
#ifdef MSG_NOSIGNAL
    const int send_flags = MSG_NOSIGNAL;  // 对端提前关闭时不要用 SIGPIPE 杀掉节点
#else
    const int send_flags = 0;
#endif
    ssize_t n = ::send(fd, payload.data() + sent, payload.size() - sent, send_flags);
    if (n > 0) {
      sent += static_cast<std::size_t>(n);
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      if (!wait_fd(fd, true, remaining_ms(deadline))) {
        result.error = "send timeout";
        ::close(fd);
        return result;
      }
      continue;
    }
    result.error = "send failed";
    ::close(fd);
    return result;
  }

  std::string raw;
  char buf[4096];
  while (true) {
    int left = remaining_ms(deadline);
    if (left <= 0) {
      result.error = "recv timeout";
      break;
    }
    ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
    if (n > 0) {
      raw.append(buf, static_cast<std::size_t>(n));
      continue;
    }
    if (n == 0) break;
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      if (!wait_fd(fd, false, left)) {
        result.error = "recv timeout";
        break;
      }
      continue;
    }
    result.error = "recv failed";
    break;
  }
  ::close(fd);

  std::size_t header_end = raw.find("\r\n\r\n");
  if (header_end == std::string::npos) {
    if (result.error.empty()) result.error = "no http header";
    return result;
  }

  std::string header = raw.substr(0, header_end);
  std::string body_raw = raw.substr(header_end + 4);

  std::istringstream hs(header);
  std::string status_line;
  std::getline(hs, status_line);
  std::size_t sp = status_line.find(' ');
  if (sp != std::string::npos) {
    result.status = std::atoi(status_line.c_str() + sp + 1);
  }

  bool chunked = false;
  long content_length = -1;
  std::string line;
  while (std::getline(hs, line)) {
    std::string l = lower_copy(trim(line));
    if (l.rfind("transfer-encoding:", 0) == 0) {
      if (l.find("chunked") != std::string::npos) chunked = true;
    } else if (l.rfind("content-length:", 0) == 0) {
      content_length = std::atol(l.substr(15).c_str());
    }
  }

  if (chunked) {
    result.body = decode_chunked(body_raw);
  } else if (content_length >= 0 &&
             static_cast<std::size_t>(content_length) <= body_raw.size()) {
    result.body = body_raw.substr(0, static_cast<std::size_t>(content_length));
  } else {
    result.body = body_raw;
  }
  return result;
}

}  // namespace

HttpResult HttpClient::post_json(const std::string& url, const std::string& bearer_token,
                                 const std::string& json_body, int timeout_ms) {
  return request(url, "POST", json_body, bearer_token, timeout_ms);
}

HttpResult HttpClient::get(const std::string& url, int timeout_ms) {
  return request(url, "GET", "", "", timeout_ms);
}

}  // namespace rmtc
