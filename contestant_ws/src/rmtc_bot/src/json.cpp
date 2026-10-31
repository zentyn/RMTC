// ============================================================================
//  json.cpp —— 轻量 JSON 解析器实现
// ============================================================================

#include "rmtc_bot/json.hpp"

#include <cctype>
#include <cstdlib>

namespace rmtc {

// 一个"永远是 Null"的静态对象，用于查找失败时返回引用（避免返回空指针）
static const Json kNullJson{};

// ---------------------------------------------------------------- Json 成员

const Json* Json::find(const std::string& key) const {
    if (type != JsonType::Object) {
        return nullptr;
    }
    auto it = obj.find(key);
    if (it == obj.end()) {
        return nullptr;
    }
    return &it->second;
}

const Json& Json::operator[](const std::string& key) const {
    const Json* p = find(key);
    return p ? *p : kNullJson;
}

const Json& Json::operator[](size_t index) const {
    if (type != JsonType::Array || index >= arr.size()) {
        return kNullJson;
    }
    return arr[index];
}

size_t Json::size() const {
    if (type == JsonType::Array) {
        return arr.size();
    }
    if (type == JsonType::Object) {
        return obj.size();
    }
    return 0;
}

bool Json::empty() const {
    return size() == 0;
}

bool Json::hasKey(const std::string& key) const {
    return find(key) != nullptr;
}

int Json::toInt(int defaultValue) const {
    switch (type) {
        case JsonType::Number: {
            // 截断小数：规则里的 tick、坐标都是整数
            return static_cast<int>(numVal);
        }
        case JsonType::Bool:
            return boolVal ? 1 : 0;
        case JsonType::String: {
            // 容错：万一裁判把数字写成字符串 "12"
            if (strVal.empty()) {
                return defaultValue;
            }
            char* end = nullptr;
            long v = std::strtol(strVal.c_str(), &end, 10);
            if (end == strVal.c_str()) {
                return defaultValue;
            }
            return static_cast<int>(v);
        }
        default:
            return defaultValue;
    }
}

double Json::toDouble(double defaultValue) const {
    switch (type) {
        case JsonType::Number:
            return numVal;
        case JsonType::Bool:
            return boolVal ? 1.0 : 0.0;
        case JsonType::String: {
            if (strVal.empty()) {
                return defaultValue;
            }
            char* end = nullptr;
            double v = std::strtod(strVal.c_str(), &end);
            if (end == strVal.c_str()) {
                return defaultValue;
            }
            return v;
        }
        default:
            return defaultValue;
    }
}

std::string Json::toStr(const std::string& defaultValue) const {
    if (type == JsonType::String) {
        return strVal;
    }
    if (type == JsonType::Number) {
        // 数字转字符串只在调试日志里用
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%g", numVal);
        return std::string(buf);
    }
    if (type == JsonType::Bool) {
        return boolVal ? "true" : "false";
    }
    return defaultValue;
}

bool Json::toBool(bool defaultValue) const {
    switch (type) {
        case JsonType::Bool:
            return boolVal;
        case JsonType::Number:
            return numVal != 0.0;
        case JsonType::String: {
            std::string s = strVal;
            for (auto& c : s) {
                c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            }
            if (s == "true" || s == "1" || s == "yes") {
                return true;
            }
            if (s == "false" || s == "0" || s == "no") {
                return false;
            }
            return defaultValue;
        }
        case JsonType::Null:
            return defaultValue;
        default:
            return defaultValue;
    }
}

bool Json::toXY(int& x, int& y) const {
    if (type != JsonType::Array || arr.size() < 2) {
        return false;
    }
    if (!arr[0].isNumber() && !arr[0].isString()) {
        return false;
    }
    if (!arr[1].isNumber() && !arr[1].isString()) {
        return false;
    }
    x = arr[0].toInt(0);
    y = arr[1].toInt(0);
    return true;
}

// ---------------------------------------------------------------- 解析器

namespace {

class Parser {
public:
    Parser(const std::string& text) : src_(text) {}

    bool parse(Json& out, std::string& err) {
        err_.clear();
        skipSpace();
        if (!parseValue(out)) {
            err = err_.empty() ? "未知解析错误" : err_;
            return false;
        }
        skipSpace();
        // 允许结尾有空白，但不允许有多余内容
        if (pos_ < src_.size()) {
            err = "JSON 结尾存在多余字符（位置 " + std::to_string(pos_) + "）";
            return false;
        }
        return true;
    }

private:
    const std::string& src_;
    size_t pos_ = 0;
    std::string err_;

    // 防止畸形输入导致无限递归（例如几万个 '['）
    static const int kMaxDepth = 64;
    int depth_ = 0;

    bool eof() const { return pos_ >= src_.size(); }
    char cur() const { return src_[pos_]; }

    bool fail(const std::string& msg) {
        if (err_.empty()) {
            err_ = msg + "（位置 " + std::to_string(pos_) + "）";
        }
        return false;
    }

    void skipSpace() {
        while (!eof()) {
            char c = cur();
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
                ++pos_;
            } else {
                break;
            }
        }
    }

    bool parseValue(Json& out) {
        if (eof()) {
            return fail("输入意外结束");
        }
        if (depth_ > kMaxDepth) {
            return fail("JSON 嵌套过深");
        }
        char c = cur();
        switch (c) {
            case '{': return parseObject(out);
            case '[': return parseArray(out);
            case '"': {
                std::string s;
                if (!parseString(s)) {
                    return false;
                }
                out.type = JsonType::String;
                out.strVal = s;
                return true;
            }
            case 't':
                if (src_.compare(pos_, 4, "true") == 0) {
                    pos_ += 4;
                    out.type = JsonType::Bool;
                    out.boolVal = true;
                    return true;
                }
                return fail("期望 true");
            case 'f':
                if (src_.compare(pos_, 5, "false") == 0) {
                    pos_ += 5;
                    out.type = JsonType::Bool;
                    out.boolVal = false;
                    return true;
                }
                return fail("期望 false");
            case 'n':
                if (src_.compare(pos_, 4, "null") == 0) {
                    pos_ += 4;
                    out.type = JsonType::Null;
                    return true;
                }
                return fail("期望 null");
            default:
                return parseNumber(out);
        }
    }

    bool parseObject(Json& out) {
        ++depth_;
        out.type = JsonType::Object;
        ++pos_;  // 吃掉 '{'
        skipSpace();
        if (!eof() && cur() == '}') {
            ++pos_;
            --depth_;
            return true;
        }
        while (true) {
            skipSpace();
            if (eof()) {
                --depth_;
                return fail("对象未闭合");
            }
            if (cur() != '"') {
                --depth_;
                return fail("对象的键必须是字符串");
            }
            std::string key;
            if (!parseString(key)) {
                --depth_;
                return false;
            }
            skipSpace();
            if (eof() || cur() != ':') {
                --depth_;
                return fail("键后缺少冒号");
            }
            ++pos_;  // 吃掉 ':'
            skipSpace();
            Json value;
            if (!parseValue(value)) {
                --depth_;
                return false;
            }
            out.obj[key] = std::move(value);
            skipSpace();
            if (eof()) {
                --depth_;
                return fail("对象未闭合");
            }
            char c = cur();
            if (c == ',') {
                ++pos_;
                continue;
            }
            if (c == '}') {
                ++pos_;
                --depth_;
                return true;
            }
            --depth_;
            return fail("对象中期望 , 或 }");
        }
    }

    bool parseArray(Json& out) {
        ++depth_;
        out.type = JsonType::Array;
        ++pos_;  // 吃掉 '['
        skipSpace();
        if (!eof() && cur() == ']') {
            ++pos_;
            --depth_;
            return true;
        }
        while (true) {
            skipSpace();
            Json item;
            if (!parseValue(item)) {
                --depth_;
                return false;
            }
            out.arr.push_back(std::move(item));
            skipSpace();
            if (eof()) {
                --depth_;
                return fail("数组未闭合");
            }
            char c = cur();
            if (c == ',') {
                ++pos_;
                continue;
            }
            if (c == ']') {
                ++pos_;
                --depth_;
                return true;
            }
            --depth_;
            return fail("数组中期望 , 或 ]");
        }
    }

    // 解析字符串，并还原转义字符。
    // 【关键】协议 §3 说 9x9 位置图 "JSON 文本中写作 \n，解析后为实际换行"，
    // 所以这里必须把 \n 转成真正的换行符 0x0A，不能留成反斜杠+n 两个字符。
    bool parseString(std::string& out) {
        ++pos_;  // 吃掉开头的 '"'
        out.clear();
        while (true) {
            if (eof()) {
                return fail("字符串未闭合");
            }
            char c = cur();
            if (c == '"') {
                ++pos_;
                return true;
            }
            if (c == '\\') {
                ++pos_;
                if (eof()) {
                    return fail("转义符后无内容");
                }
                char e = cur();
                ++pos_;
                switch (e) {
                    case '"':  out.push_back('"');  break;
                    case '\\': out.push_back('\\'); break;
                    case '/':  out.push_back('/');  break;
                    case 'b':  out.push_back('\b'); break;
                    case 'f':  out.push_back('\f'); break;
                    case 'n':  out.push_back('\n'); break;   // 真换行
                    case 'r':  out.push_back('\r'); break;
                    case 't':  out.push_back('\t'); break;
                    case 'u': {
                        // \uXXXX：本比赛的消息里基本不会出现，做一个安全兜底
                        if (pos_ + 4 > src_.size()) {
                            return fail("\\u 转义不完整");
                        }
                        std::string hex = src_.substr(pos_, 4);
                        pos_ += 4;
                        unsigned int cp = 0;
                        for (char h : hex) {
                            cp <<= 4;
                            if (h >= '0' && h <= '9')      cp |= static_cast<unsigned>(h - '0');
                            else if (h >= 'a' && h <= 'f') cp |= static_cast<unsigned>(h - 'a' + 10);
                            else if (h >= 'A' && h <= 'F') cp |= static_cast<unsigned>(h - 'A' + 10);
                            else return fail("\\u 转义含非法字符");
                        }
                        // 极简 UTF-8 编码（不处理代理对，够用于中文以外场景）
                        if (cp < 0x80) {
                            out.push_back(static_cast<char>(cp));
                        } else if (cp < 0x800) {
                            out.push_back(static_cast<char>(0xC0 | (cp >> 6)));
                            out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
                        } else {
                            out.push_back(static_cast<char>(0xE0 | (cp >> 12)));
                            out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
                            out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
                        }
                        break;
                    }
                    default:
                        return fail("非法转义字符");
                }
                continue;
            }
            out.push_back(c);
            ++pos_;
        }
    }

    bool parseNumber(Json& out) {
        size_t start = pos_;
        if (!eof() && (cur() == '-' || cur() == '+')) {
            ++pos_;
        }
        bool hasDigit = false;
        while (!eof() && std::isdigit(static_cast<unsigned char>(cur()))) {
            ++pos_;
            hasDigit = true;
        }
        if (!eof() && cur() == '.') {
            ++pos_;
            while (!eof() && std::isdigit(static_cast<unsigned char>(cur()))) {
                ++pos_;
                hasDigit = true;
            }
        }
        if (!hasDigit) {
            pos_ = start;
            return fail("非法数值");
        }
        // 指数部分（1e3 之类）
        if (!eof() && (cur() == 'e' || cur() == 'E')) {
            ++pos_;
            if (!eof() && (cur() == '-' || cur() == '+')) {
                ++pos_;
            }
            while (!eof() && std::isdigit(static_cast<unsigned char>(cur()))) {
                ++pos_;
            }
        }
        std::string numText = src_.substr(start, pos_ - start);
        out.type = JsonType::Number;
        out.numVal = std::strtod(numText.c_str(), nullptr);
        return true;
    }
};

}  // namespace

bool parseJson(const std::string& text, Json& out, std::string& err) {
    out = Json();
    if (text.empty()) {
        err = "输入为空字符串";
        return false;
    }
    Parser p(text);
    return p.parse(out, err);
}

std::string jsonEscape(const std::string& raw) {
    std::string out;
    out.reserve(raw.size() + 8);
    for (char c : raw) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:
                // 控制字符用 \u00XX 兜底，避免生成非法 JSON
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x",
                                  static_cast<unsigned int>(static_cast<unsigned char>(c)));
                    out += buf;
                } else {
                    out.push_back(c);
                }
        }
    }
    return out;
}

}  // namespace rmtc
