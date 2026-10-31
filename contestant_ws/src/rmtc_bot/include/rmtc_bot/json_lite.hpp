// 极简 JSON：解析 + 序列化，只依赖标准库。
// 比赛机器上不保证有 nlohmann/json，所以自带一份。
#pragma once

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace jlite {

class Value {
 public:
  enum class Type { Null, Bool, Number, String, Array, Object };

  Value() = default;
  explicit Value(bool b) : t_(Type::Bool), b_(b) {}
  explicit Value(double n) : t_(Type::Number), n_(n) {}
  explicit Value(int n) : t_(Type::Number), n_(static_cast<double>(n)) {}
  Value(const char* s) : t_(Type::String), s_(s) {}
  Value(std::string s) : t_(Type::String), s_(std::move(s)) {}

  static Value make_array() {
    Value v;
    v.t_ = Type::Array;
    return v;
  }
  static Value make_object() {
    Value v;
    v.t_ = Type::Object;
    return v;
  }

  Type type() const { return t_; }
  bool is_null() const { return t_ == Type::Null; }
  bool is_bool() const { return t_ == Type::Bool; }
  bool is_number() const { return t_ == Type::Number; }
  bool is_string() const { return t_ == Type::String; }
  bool is_array() const { return t_ == Type::Array; }
  bool is_object() const { return t_ == Type::Object; }

  bool as_bool(bool def = false) const { return t_ == Type::Bool ? b_ : def; }
  double as_number(double def = 0.0) const { return t_ == Type::Number ? n_ : def; }
  int as_int(int def = 0) const {
    return t_ == Type::Number ? static_cast<int>(std::llround(n_)) : def;
  }
  const std::string& as_string() const {
    static const std::string empty;
    return t_ == Type::String ? s_ : empty;
  }

  const std::vector<Value>& items() const {
    static const std::vector<Value> empty;
    return t_ == Type::Array ? arr_ : empty;
  }
  const std::map<std::string, Value>& members() const {
    static const std::map<std::string, Value> empty;
    return t_ == Type::Object ? obj_ : empty;
  }

  bool contains(const std::string& key) const {
    return t_ == Type::Object && obj_.find(key) != obj_.end();
  }

  const Value& operator[](const std::string& key) const {
    static const Value null_value;
    if (t_ != Type::Object) return null_value;
    auto it = obj_.find(key);
    return it == obj_.end() ? null_value : it->second;
  }
  const Value& operator[](std::size_t index) const {
    static const Value null_value;
    if (t_ != Type::Array || index >= arr_.size()) return null_value;
    return arr_[index];
  }

  Value& set(const std::string& key, Value v) {
    if (t_ != Type::Object) {
      t_ = Type::Object;
      obj_.clear();
    }
    obj_[key] = std::move(v);
    return *this;
  }
  void push_back(Value v) {
    if (t_ != Type::Array) {
      t_ = Type::Array;
      arr_.clear();
    }
    arr_.push_back(std::move(v));
  }

  std::string get_string(const std::string& key, const std::string& def = "") const {
    const Value& v = (*this)[key];
    return v.is_string() ? v.as_string() : def;
  }
  double get_number(const std::string& key, double def = 0.0) const {
    const Value& v = (*this)[key];
    return v.is_number() ? v.as_number() : def;
  }
  int get_int(const std::string& key, int def = 0) const {
    const Value& v = (*this)[key];
    return v.is_number() ? v.as_int() : def;
  }
  bool get_bool(const std::string& key, bool def = false) const {
    const Value& v = (*this)[key];
    return v.is_bool() ? v.as_bool() : def;
  }

  std::string dump() const {
    std::string out;
    out.reserve(256);
    dump_to(*this, out);
    return out;
  }

  static bool parse(const std::string& text, Value& out, std::string* error = nullptr);

 private:
  static void dump_string(const std::string& s, std::string& out) {
    out.push_back('"');
    for (unsigned char c : s) {
      switch (c) {
        case '"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\b': out += "\\b"; break;
        case '\f': out += "\\f"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:
          if (c < 0x20) {
            char buf[8];
            std::snprintf(buf, sizeof(buf), "\\u%04x", c);
            out += buf;
          } else {
            out.push_back(static_cast<char>(c));
          }
      }
    }
    out.push_back('"');
  }

  static void dump_to(const Value& v, std::string& out) {
    switch (v.t_) {
      case Type::Null: out += "null"; break;
      case Type::Bool: out += v.b_ ? "true" : "false"; break;
      case Type::Number: {
        if (!std::isfinite(v.n_)) {
          out += "0";
        } else if (v.n_ == std::floor(v.n_) && std::fabs(v.n_) < 1e15) {
          out += std::to_string(static_cast<long long>(v.n_));
        } else {
          std::ostringstream oss;
          oss.precision(15);
          oss << v.n_;
          out += oss.str();
        }
        break;
      }
      case Type::String: dump_string(v.s_, out); break;
      case Type::Array: {
        out.push_back('[');
        for (std::size_t i = 0; i < v.arr_.size(); ++i) {
          if (i) out.push_back(',');
          dump_to(v.arr_[i], out);
        }
        out.push_back(']');
        break;
      }
      case Type::Object: {
        out.push_back('{');
        bool first = true;
        for (const auto& kv : v.obj_) {
          if (!first) out.push_back(',');
          first = false;
          dump_string(kv.first, out);
          out.push_back(':');
          dump_to(kv.second, out);
        }
        out.push_back('}');
        break;
      }
    }
  }

  Type t_ = Type::Null;
  bool b_ = false;
  double n_ = 0.0;
  std::string s_;
  std::vector<Value> arr_;
  std::map<std::string, Value> obj_;
};

namespace detail {

inline void append_utf8(std::string& out, unsigned int cp) {
  if (cp <= 0x7F) {
    out.push_back(static_cast<char>(cp));
  } else if (cp <= 0x7FF) {
    out.push_back(static_cast<char>(0xC0 | (cp >> 6)));
    out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
  } else if (cp <= 0xFFFF) {
    out.push_back(static_cast<char>(0xE0 | (cp >> 12)));
    out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
    out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
  } else {
    out.push_back(static_cast<char>(0xF0 | (cp >> 18)));
    out.push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3F)));
    out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
    out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
  }
}

class Parser {
 public:
  explicit Parser(const std::string& s) : s_(s) {}

  bool run(Value& out) {
    skip_ws();
    if (!parse_value(out, 0)) return false;
    skip_ws();
    if (pos_ != s_.size()) {
      fail("trailing characters");
      return false;
    }
    return true;
  }

  const std::string& error() const { return err_; }

 private:
  void fail(const char* msg) {
    if (err_.empty()) {
      std::ostringstream oss;
      oss << "json error at offset " << pos_ << ": " << msg;
      err_ = oss.str();
    }
  }

  bool eof() const { return pos_ >= s_.size(); }
  char peek() const { return eof() ? '\0' : s_[pos_]; }

  void skip_ws() {
    while (!eof()) {
      char c = s_[pos_];
      if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
        ++pos_;
      } else {
        break;
      }
    }
  }

  bool literal(const char* lit) {
    std::size_t n = std::strlen(lit);
    if (s_.compare(pos_, n, lit) != 0) return false;
    pos_ += n;
    return true;
  }

  bool parse_value(Value& out, int depth) {
    if (depth > 64) {
      fail("nesting too deep");
      return false;
    }
    skip_ws();
    if (eof()) {
      fail("unexpected end");
      return false;
    }
    char c = peek();
    if (c == '{') return parse_object(out, depth);
    if (c == '[') return parse_array(out, depth);
    if (c == '"') {
      std::string str;
      if (!parse_string(str)) return false;
      out = Value(std::move(str));
      return true;
    }
    if (c == 't') {
      if (!literal("true")) {
        fail("bad literal");
        return false;
      }
      out = Value(true);
      return true;
    }
    if (c == 'f') {
      if (!literal("false")) {
        fail("bad literal");
        return false;
      }
      out = Value(false);
      return true;
    }
    if (c == 'n') {
      if (!literal("null")) {
        fail("bad literal");
        return false;
      }
      out = Value();
      return true;
    }
    return parse_number(out);
  }

  bool parse_number(Value& out) {
    const char* begin = s_.c_str() + pos_;
    char* end = nullptr;
    double v = std::strtod(begin, &end);
    if (end == begin) {
      fail("bad number");
      return false;
    }
    pos_ += static_cast<std::size_t>(end - begin);
    out = Value(v);
    return true;
  }

  bool parse_hex4(unsigned int& out) {
    if (pos_ + 4 > s_.size()) {
      fail("truncated \\u escape");
      return false;
    }
    unsigned int v = 0;
    for (int i = 0; i < 4; ++i) {
      char c = s_[pos_++];
      v <<= 4;
      if (c >= '0' && c <= '9') {
        v |= static_cast<unsigned int>(c - '0');
      } else if (c >= 'a' && c <= 'f') {
        v |= static_cast<unsigned int>(c - 'a' + 10);
      } else if (c >= 'A' && c <= 'F') {
        v |= static_cast<unsigned int>(c - 'A' + 10);
      } else {
        fail("bad \\u escape");
        return false;
      }
    }
    out = v;
    return true;
  }

  bool parse_string(std::string& out) {
    if (peek() != '"') {
      fail("expect string");
      return false;
    }
    ++pos_;
    out.clear();
    while (!eof()) {
      unsigned char c = static_cast<unsigned char>(s_[pos_++]);
      if (c == '"') return true;
      if (c != '\\') {
        if (c < 0x20) {
          fail("control character in string");
          return false;
        }
        out.push_back(static_cast<char>(c));
        continue;
      }
      if (eof()) {
        fail("truncated escape");
        return false;
      }
      char esc = s_[pos_++];
      switch (esc) {
        case '"': out.push_back('"'); break;
        case '\\': out.push_back('\\'); break;
        case '/': out.push_back('/'); break;
        case 'b': out.push_back('\b'); break;
        case 'f': out.push_back('\f'); break;
        case 'n': out.push_back('\n'); break;
        case 'r': out.push_back('\r'); break;
        case 't': out.push_back('\t'); break;
        case 'u': {
          unsigned int cp = 0;
          if (!parse_hex4(cp)) return false;
          if (cp >= 0xD800 && cp <= 0xDBFF) {
            if (pos_ + 1 < s_.size() && s_[pos_] == '\\' && s_[pos_ + 1] == 'u') {
              pos_ += 2;
              unsigned int low = 0;
              if (!parse_hex4(low)) return false;
              if (low >= 0xDC00 && low <= 0xDFFF) {
                cp = 0x10000 + ((cp - 0xD800) << 10) + (low - 0xDC00);
              } else {
                cp = 0xFFFD;
              }
            } else {
              cp = 0xFFFD;
            }
          } else if (cp >= 0xDC00 && cp <= 0xDFFF) {
            cp = 0xFFFD;
          }
          append_utf8(out, cp);
          break;
        }
        default: fail("bad escape"); return false;
      }
    }
    fail("unterminated string");
    return false;
  }

  bool parse_array(Value& out, int depth) {
    ++pos_;  // '['
    out = Value::make_array();
    skip_ws();
    if (peek() == ']') {
      ++pos_;
      return true;
    }
    while (true) {
      Value item;
      if (!parse_value(item, depth + 1)) return false;
      out.push_back(std::move(item));
      skip_ws();
      char c = peek();
      if (c == ',') {
        ++pos_;
        continue;
      }
      if (c == ']') {
        ++pos_;
        return true;
      }
      fail("expect ',' or ']'");
      return false;
    }
  }

  bool parse_object(Value& out, int depth) {
    ++pos_;  // '{'
    out = Value::make_object();
    skip_ws();
    if (peek() == '}') {
      ++pos_;
      return true;
    }
    while (true) {
      skip_ws();
      std::string key;
      if (!parse_string(key)) return false;
      skip_ws();
      if (peek() != ':') {
        fail("expect ':'");
        return false;
      }
      ++pos_;
      Value item;
      if (!parse_value(item, depth + 1)) return false;
      out.set(key, std::move(item));
      skip_ws();
      char c = peek();
      if (c == ',') {
        ++pos_;
        continue;
      }
      if (c == '}') {
        ++pos_;
        return true;
      }
      fail("expect ',' or '}'");
      return false;
    }
  }

  const std::string& s_;
  std::size_t pos_ = 0;
  std::string err_;
};

}  // namespace detail

inline bool Value::parse(const std::string& text, Value& out, std::string* error) {
  detail::Parser parser(text);
  Value result;
  if (!parser.run(result)) {
    if (error) *error = parser.error();
    return false;
  }
  out = std::move(result);
  return true;
}

}  // namespace jlite
