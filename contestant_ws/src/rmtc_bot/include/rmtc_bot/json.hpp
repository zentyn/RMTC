// ============================================================================
//  json.hpp —— 轻量 JSON 解析器（零外部依赖）
// ============================================================================
//
//  【为什么要自己写一个】
//  裁判发来的感知是一段 JSON 文本（协议 §3）。常见做法是用 nlohmann/json 库，
//  但那需要在虚拟机上额外 apt install libnlohmann-json3-dev，多一个可能失败的环节。
//  这个文件只用 C++ 标准库，g++ 直接就能编译，减少你踩环境坑的概率。
//
//  【设计原则：绝不抛异常】
//  协议 §3「感知解析要求」明确写：
//      "接收端须检查字段类型并处理解析异常，不得因字段缺失或类型错误导致进程退出"
//  所以这里所有取值函数都带"默认值"参数：拿不到就返回默认值，程序继续跑。
//
//  【用法示例】
//      rmtc::Json root;
//      std::string err;
//      if (!rmtc::parseJson(text, root, err)) { 记日志; return; }
//      int tick = root.find("tick") ? root.find("tick")->toInt(-1) : -1;
// ============================================================================

#pragma once

#include <map>
#include <string>
#include <vector>

namespace rmtc {

// JSON 值的类型标签
enum class JsonType {
    Null,
    Bool,
    Number,
    String,
    Array,
    Object
};

class Json {
public:
    JsonType type = JsonType::Null;

    bool boolVal = false;
    double numVal = 0.0;
    std::string strVal;

    // C++17 允许 vector/map 以自身为元素类型（不完整类型支持）
    std::vector<Json> arr;
    std::map<std::string, Json> obj;

    // ---------- 类型判断 ----------
    bool isNull() const { return type == JsonType::Null; }
    bool isBool() const { return type == JsonType::Bool; }
    bool isNumber() const { return type == JsonType::Number; }
    bool isString() const { return type == JsonType::String; }
    bool isArray() const { return type == JsonType::Array; }
    bool isObject() const { return type == JsonType::Object; }

    // ---------- 对象取键 ----------
    // 找不到键、或自己不是对象 → 返回 nullptr（调用方必须判空）
    const Json* find(const std::string& key) const;

    // 便捷版：找不到就返回一个"静态 Null"，永远不会返回空指针。
    // 适合链式取值：root["self"]["position"].arr
    const Json& operator[](const std::string& key) const;

    // ---------- 数组取下标 ----------
    // 越界或不是数组 → 返回静态 Null
    const Json& operator[](size_t index) const;

    size_t size() const;
    bool empty() const;
    bool hasKey(const std::string& key) const;

    // ---------- 带默认值的取值（绝不抛异常） ----------
    // 协议里的数值可能是 int、float，甚至被写成字符串 "12"，这里都兼容。
    int toInt(int defaultValue = 0) const;
    double toDouble(double defaultValue = 0.0) const;
    std::string toStr(const std::string& defaultValue = "") const;
    bool toBool(bool defaultValue = false) const;

    // 把 [x, y] 形式的数组解析成两个整数；失败返回 false
    bool toXY(int& x, int& y) const;
};

// 解析入口。成功返回 true；失败返回 false 并在 err 里写原因。
bool parseJson(const std::string& text, Json& out, std::string& err);

// 生成 JSON 文本（发动作、发队名注册时用）
std::string jsonEscape(const std::string& raw);

}  // namespace rmtc
