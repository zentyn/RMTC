// ============================================================================
//  test_core.cpp —— 核心逻辑单元测试（不依赖 ROS 2）
// ============================================================================
//
//  【为什么要有这个文件】
//  装 ROS 2 环境容易出问题，如果每次调试都得先起裁判、再起节点，
//  你根本分不清"是策略错了"还是"环境错了"。
//  这个测试只依赖标准库，用 g++ 一条命令就能编译运行：
//
//      g++ -std=c++17 -I include -o test_core
//          test/test_core.cpp src/json.cpp src/perception.cpp
//          src/world_model.cpp src/strategy.cpp
//      ./test_core
//  （把上面三行合成一条命令执行，末尾的续行反斜杠在这里省略以免被注释吞掉）
//
//  全部通过后会打印 "ALL TESTS PASSED"。
//  建议每次改完策略代码都先跑一遍，再去虚拟机里连裁判。
//
//  【测试用例来自协议和规则原文】
//  例如 9x9 相对位置图的例子直接取自协议 §3：
//      物体在自身右 1 格、上 2 格 → 相对位移 [1,-2]
// ============================================================================

#include <cmath>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "rmtc_bot/json.hpp"
#include "rmtc_bot/perception.hpp"
#include "rmtc_bot/strategy.hpp"
#include "rmtc_bot/world_model.hpp"

using namespace rmtc;

// ---------------------------------------------------------------- 极简测试框架

static int g_total = 0;
static int g_failed = 0;
static std::string g_currentCase;

#define CASE(name)                              \
    do {                                        \
        g_currentCase = name;                   \
        std::cout << "\n[TEST] " << name << "\n"; \
    } while (0)

#define CHECK(cond)                                                        \
    do {                                                                   \
        ++g_total;                                                         \
        if (!(cond)) {                                                     \
            ++g_failed;                                                    \
            std::cout << "  FAIL " << __LINE__ << ": " #cond << "\n";      \
        } else {                                                           \
            std::cout << "  ok   " << __LINE__ << ": " #cond << "\n";      \
        }                                                                  \
    } while (0)

#define CHECK_EQ_INT(actual, expected)                                          \
    do {                                                                        \
        ++g_total;                                                              \
        long long a_ = static_cast<long long>(actual);                          \
        long long e_ = static_cast<long long>(expected);                        \
        if (a_ != e_) {                                                         \
            ++g_failed;                                                         \
            std::cout << "  FAIL " << __LINE__ << ": " #actual " == " #expected \
                      << " (实际=" << a_ << ", 期望=" << e_ << ")\n";           \
        } else {                                                                \
            std::cout << "  ok   " << __LINE__ << ": " #actual " == "           \
                      << e_ << "\n";                                            \
        }                                                                       \
    } while (0)

#define CHECK_EQ_STR(actual, expected)                                          \
    do {                                                                        \
        ++g_total;                                                              \
        std::string a_ = (actual);                                              \
        std::string e_ = (expected);                                            \
        if (a_ != e_) {                                                         \
            ++g_failed;                                                         \
            std::cout << "  FAIL " << __LINE__ << ": " #actual " == " #expected \
                      << " (实际=\"" << a_ << "\", 期望=\"" << e_ << "\")\n";   \
        } else {                                                                \
            std::cout << "  ok   " << __LINE__ << ": " #actual " == "           \
                      << e_ << "\n";                                            \
        }                                                                       \
    } while (0)

#define CHECK_NEAR(actual, expected, eps)                                       \
    do {                                                                        \
        ++g_total;                                                              \
        double a_ = static_cast<double>(actual);                                \
        double e_ = static_cast<double>(expected);                              \
        if (std::fabs(a_ - e_) > (eps)) {                                       \
            ++g_failed;                                                         \
            std::cout << "  FAIL " << __LINE__ << ": " #actual " ~= " #expected \
                      << " (实际=" << a_ << ", 期望=" << e_ << ")\n";           \
        } else {                                                                \
            std::cout << "  ok   " << __LINE__ << ": " #actual " ~= "           \
                      << e_ << "\n";                                            \
        }                                                                       \
    } while (0)

// ---------------------------------------------------------------- 构造 9x9 位置图
//
//  协议 §3：'1' 固定在第 5 行第 5 列（行列从 1 开始数），'2' 是物体。
//  相对位移 = [2所在列 - 5, 2所在行 - 5]。
//  这个函数按"相对位移 (dx, dy)"反推出一张合法的位置图字符串，
//  用于造测试数据。dx 正=右，dy 正=下（y 向下增大）。

static std::string makeVisionMap(int dx, int dy) {
    // 0 基下标：row = dy + 4, col = dx + 4
    int row = dy + 4;
    int col = dx + 4;
    std::string out;
    for (int r = 0; r < 9; ++r) {
        for (int c = 0; c < 9; ++c) {
            if (r == 4 && c == 4) {
                out.push_back('1');          // 自己永远在中心
            } else if (r == row && c == col) {
                out.push_back('2');          // 物体
            } else {
                out.push_back('0');
            }
        }
        if (r != 8) {
            out.push_back('\n');             // 无末尾换行（协议明确）
        }
    }
    return out;
}

// 把一个 JSON 字符串里的换行写成 \n 转义（模拟真实 JSON 文本）
static std::string jsonEscapeForTest(const std::string& raw) {
    std::string out;
    for (char c : raw) {
        if (c == '\n') out += "\\n";
        else if (c == '"') out += "\\\"";
        else if (c == '\\') out += "\\\\";
        else out.push_back(c);
    }
    return out;
}

// ============================================================================
//  测试 1：JSON 解析器
// ============================================================================

static void testJson() {
    CASE("JSON 解析器：基本类型");
    {
        Json root;
        std::string err;
        bool ok = parseJson("{\"tick\":148,\"session_id\":\"example-match\"}", root, err);
        CHECK(ok);
        CHECK(root.isObject());
        CHECK_EQ_INT(root["tick"].toInt(-1), 148);
        CHECK_EQ_STR(root["session_id"].toStr(""), "example-match");
        // 不存在的键：返回静态 Null，不崩溃
        CHECK(root["不存在"].isNull());
        CHECK_EQ_INT(root["不存在"].toInt(99), 99);
    }

    CASE("JSON 解析器：数组与嵌套");
    {
        Json root;
        std::string err;
        bool ok = parseJson("{\"arr\":[1,2,3],\"o\":{\"a\":{\"b\":true}}}", root, err);
        CHECK(ok);
        CHECK(root["arr"].isArray());
        CHECK_EQ_INT(root["arr"].size(), 3);
        CHECK_EQ_INT(root["arr"][1].toInt(0), 2);
        // 越界访问不崩溃
        CHECK(root["arr"][99].isNull());
        CHECK(root["o"]["a"]["b"].toBool(false));
    }

    CASE("JSON 解析器：字符串转义 \\n 必须还原成真换行");
    {
        Json root;
        std::string err;
        // 这是协议里 9x9 位置图在 JSON 文本中的真实形态
        bool ok = parseJson("{\"p\":\"000000000\\n000000000\"}", root, err);
        CHECK(ok);
        std::string s = root["p"].toStr("");
        CHECK_EQ_INT(s.size(), 19);        // 9 + 1 个真换行 + 9
        CHECK(s.find('\n') != std::string::npos);   // 必须是真换行
        CHECK(s.find("\\n") == std::string::npos);   // 不能是反斜杠+n
    }

    CASE("JSON 解析器：中文 UTF-8");
    {
        Json root;
        std::string err;
        bool ok = parseJson("{\"mission_text\":\"获得至少两个普通资源\"}", root, err);
        CHECK(ok);
        CHECK_EQ_STR(root["mission_text"].toStr(""), "获得至少两个普通资源");
    }

    CASE("JSON 解析器：非法输入不崩溃");
    {
        Json root;
        std::string err;
        CHECK(!parseJson("", root, err));
        CHECK(!err.empty());
        CHECK(!parseJson("{", root, err));
        CHECK(!parseJson("{\"a\":}", root, err));
        CHECK(!parseJson("[1,2", root, err));
        CHECK(!parseJson("随便一段不是JSON的文字", root, err));
        // 深层嵌套不能导致栈溢出
        std::string deep(2000, '[');
        deep += std::string(2000, ']');
        CHECK(!parseJson(deep, root, err));
    }

    CASE("JSON 解析器：数值容错（字符串数字）");
    {
        Json root;
        std::string err;
        CHECK(parseJson("{\"n\":\"12\",\"f\":3.5,\"neg\":-7}", root, err));
        CHECK_EQ_INT(root["n"].toInt(0), 12);       // 字符串数字也能读
        CHECK_NEAR(root["f"].toDouble(0.0), 3.5, 1e-9);
        CHECK_EQ_INT(root["neg"].toInt(0), -7);
    }
}

// ============================================================================
//  测试 2：9x9 相对位置图
// ============================================================================

static void testRelativeMap() {
    CASE("9x9 图：协议 §3 原例（右 1 上 2 → [1,-2]）");
    {
        // 协议原文的例子：'2' 在第 3 行第 6 列
        std::string text =
            "000000000\n"
            "000000000\n"
            "000002000\n"
            "000000000\n"
            "000010000\n"
            "000000000\n"
            "000000000\n"
            "000000000\n"
            "000000000";
        int dx = 0, dy = 0;
        CHECK(parseRelativeMap(text, dx, dy));
        CHECK_EQ_INT(dx, 1);    // 右 1
        CHECK_EQ_INT(dy, -2);   // 上 2
    }

    CASE("9x9 图：字符串长度必须是 89（81 格 + 8 换行，无末尾换行）");
    {
        std::string text = makeVisionMap(0, 0);
        CHECK_EQ_INT(text.size(), 89);
    }

    CASE("9x9 图：四个方向");
    {
        int dx = 0, dy = 0;
        CHECK(parseRelativeMap(makeVisionMap(0, -1), dx, dy));   // 上
        CHECK_EQ_INT(dx, 0); CHECK_EQ_INT(dy, -1);
        CHECK(parseRelativeMap(makeVisionMap(0, 1), dx, dy));    // 下
        CHECK_EQ_INT(dx, 0); CHECK_EQ_INT(dy, 1);
        CHECK(parseRelativeMap(makeVisionMap(-1, 0), dx, dy));   // 左
        CHECK_EQ_INT(dx, -1); CHECK_EQ_INT(dy, 0);
        CHECK(parseRelativeMap(makeVisionMap(1, 0), dx, dy));    // 右
        CHECK_EQ_INT(dx, 1); CHECK_EQ_INT(dy, 0);
    }

    CASE("9x9 图：视野边缘（±4）");
    {
        int dx = 0, dy = 0;
        CHECK(parseRelativeMap(makeVisionMap(4, -4), dx, dy));   // 右上最远
        CHECK_EQ_INT(dx, 4); CHECK_EQ_INT(dy, -4);
        CHECK(parseRelativeMap(makeVisionMap(-4, 4), dx, dy));   // 左下最远
        CHECK_EQ_INT(dx, -4); CHECK_EQ_INT(dy, 4);
    }

    CASE("9x9 图：没有 '2' 表示与自身同格 [0,0]");
    {
        std::string text =
            "000000000\n000000000\n000000000\n000000000\n"
            "000010000\n"
            "000000000\n000000000\n000000000\n000000000";
        int dx = 99, dy = 99;
        CHECK(parseRelativeMap(text, dx, dy));
        CHECK_EQ_INT(dx, 0);
        CHECK_EQ_INT(dy, 0);
    }

    CASE("9x9 图：非法输入返回 false 而不崩溃");
    {
        int dx = 0, dy = 0;
        CHECK(!parseRelativeMap("", dx, dy));
        CHECK(!parseRelativeMap("abc", dx, dy));
        CHECK(!parseRelativeMap("000000000", dx, dy));   // 只有一行
    }

    CASE("坐标轴方向：y 向下增大，up 是 y-1");
    {
        int dx = 0, dy = 0;
        dirDelta(Dir::Up, dx, dy);
        CHECK_EQ_INT(dx, 0); CHECK_EQ_INT(dy, -1);
        dirDelta(Dir::Down, dx, dy);
        CHECK_EQ_INT(dx, 0); CHECK_EQ_INT(dy, 1);
        dirDelta(Dir::Left, dx, dy);
        CHECK_EQ_INT(dx, -1); CHECK_EQ_INT(dy, 0);
        dirDelta(Dir::Right, dx, dy);
        CHECK_EQ_INT(dx, 1); CHECK_EQ_INT(dy, 0);
    }

    CASE("directionBetween / directionTo");
    {
        Cell a(5, 5);
        CHECK(directionBetween(a, Cell(6, 5)) == Dir::Right);
        CHECK(directionBetween(a, Cell(5, 4)) == Dir::Up);
        CHECK(directionBetween(a, Cell(4, 5)) == Dir::Left);
        CHECK(directionBetween(a, Cell(5, 6)) == Dir::Down);
        CHECK(directionBetween(a, a) == Dir::None);
        CHECK(directionBetween(a, Cell(6, 6)) == Dir::None);   // 斜向不相邻

        int dist = 0;
        CHECK(directionTo(a, Cell(5, 3), dist) == Dir::Up);    // 直线 2 格
        CHECK_EQ_INT(dist, 2);
        CHECK(directionTo(a, Cell(7, 5), dist) == Dir::Right);
        CHECK_EQ_INT(dist, 2);
        CHECK(directionTo(a, Cell(6, 6), dist) == Dir::None);  // 斜向打不到
    }

    CASE("relativeToAbsolute");
    {
        // 自己在 [8,8]，物体在右 1 上 2 → 绝对坐标 [9,6]
        Cell abs = relativeToAbsolute(Cell(8, 8), 1, -2);
        CHECK_EQ_INT(abs.x, 9);
        CHECK_EQ_INT(abs.y, 6);
    }
}

// ============================================================================
//  测试 3：完整感知解析（用协议 §3 的示例数据）
// ============================================================================

// 协议 §3「感知示例」的 data 内容（/team_A/perception）
static const char* kProtocolExample =
    "{"
    "\"tick\": 148,"
    "\"session_id\": \"example-match\","
    "\"vision_radius\": 4,"
    "\"self\": {\"team\": \"A\", \"position\": [8, 8], \"hp\": 70, \"alive\": true,"
    " \"defending\": false, \"respawn_in\": 0, \"protected\": false, \"robot_id\": \"A\","
    " \"last_action\": {\"tick\": 147, \"action\": \"move\", \"moved\": true,"
    " \"effective_direction\": \"right\"}},"
    "\"visible_entities\": ["
    "{\"type\": \"robot\", \"id\": \"robot_B\", \"team\": \"B\", \"robot_id\": \"B\","
    " \"position\": \"000000000\\n000000000\\n000000000\\n000000000\\n000010000\\n"
    "000002000\\n000000000\\n000000000\\n000000000\","
    " \"hp\": 60, \"defending\": false, \"protected\": false},"
    "{\"type\": \"resource_normal\", \"id\": \"n09\","
    " \"position\": \"000000000\\n000000000\\n000000000\\n000000000\\n000010000\\n"
    "000020000\\n000000000\\n000000000\\n000000000\", \"value\": 10},"
    "{\"type\": \"resource_high\", \"id\": \"h01\","
    " \"position\": \"000000000\\n000000000\\n000000000\\n000000200\\n000010000\\n"
    "000000000\\n000000000\\n000000000\\n000000000\", \"value\": 20},"
    "{\"type\": \"obstacle\","
    " \"position\": \"000000000\\n000000000\\n000000000\\n000000000\\n000010000\\n"
    "000000020\\n000000000\\n000000000\\n000000000\"}"
    "],"
    "\"score\": {\"resource_value\": 30, \"effective_damage\": 0, \"mission_completed\": 1,"
    " \"mission_completed_ids\": [\"collect_high_value\"], \"invalid_actions\": 0},"
    "\"mission_text\": \"获得至少两个普通资源；获得至少一个高价值资源；比赛结束前进入坐标 [12, 12]。\","
    "\"missions\": ["
    "{\"id\": \"collect_two_normal\", \"type\": \"normal_resources\", \"target\": 2},"
    "{\"id\": \"collect_high_value\", \"type\": \"high_resources\", \"target\": 1},"
    "{\"id\": \"reach_target\", \"type\": \"reach\", \"target\": 1, \"position\": [12, 12]}"
    "],"
    "\"map\": {\"width\": 24, \"height\": 24},"
    "\"event_text\": \"\""
    "}";

static void testPerception() {
    CASE("感知解析：协议示例数据");
    {
        Perception p;
        bool ok = parsePerception(kProtocolExample, p);
        CHECK(ok);
        CHECK(p.ok);
        if (!p.ok) {
            for (const std::string& s : p.problems) std::cout << "    problem: " << s << "\n";
        }
        CHECK_EQ_INT(p.tick, 148);
        CHECK_EQ_STR(p.sessionId, "example-match");
        CHECK_EQ_INT(p.visionRadius, 4);

        CHECK_EQ_STR(p.self.team, "A");
        CHECK(p.self.hasPos);
        CHECK_EQ_INT(p.self.pos.x, 8);
        CHECK_EQ_INT(p.self.pos.y, 8);
        CHECK_EQ_INT(p.self.hp, 70);
        CHECK(p.self.alive);
        CHECK(!p.self.defending);
        CHECK(!p.self.protectedSpawn);
        CHECK_EQ_INT(p.self.respawnIn, 0);

        CHECK(p.self.hasLastAction);
        CHECK_EQ_INT(p.self.lastActionTick, 147);
        CHECK_EQ_STR(p.self.lastAction, "move");
        CHECK(p.self.lastMoved);
        CHECK_EQ_STR(p.self.lastEffectiveDir, "right");

        CHECK_EQ_INT(p.mapW, 24);
        CHECK_EQ_INT(p.mapH, 24);
        CHECK_NEAR(p.score.resourceValue, 30.0, 1e-9);
        CHECK_NEAR(p.score.effectiveDamage, 0.0, 1e-9);
        CHECK_EQ_INT(p.score.missionCompleted, 1);
        CHECK_EQ_INT(p.score.missionCompletedIds.size(), 1);
        CHECK_EQ_STR(p.score.missionCompletedIds[0], "collect_high_value");

        CHECK_EQ_INT(p.missions.size(), 3);
        CHECK_EQ_STR(p.missions[2].type, "reach");
        CHECK(p.missions[2].hasPos);
        CHECK_EQ_INT(p.missions[2].pos.x, 12);
        CHECK_EQ_INT(p.missions[2].pos.y, 12);
    }

    CASE("感知解析：实体类型与绝对坐标换算");
    {
        Perception p;
        CHECK(parsePerception(kProtocolExample, p));
        CHECK_EQ_INT(p.entities.size(), 4);

        std::vector<Entity> robots = p.robotList();
        CHECK_EQ_INT(robots.size(), 1);
        // robot_B 的 '2' 在第 6 行第 6 列 → 相对 (1, 1)，自己在 [8,8] → 绝对 [9,9]
        CHECK(robots[0].hasRel);
        CHECK_EQ_INT(robots[0].relDx, 1);
        CHECK_EQ_INT(robots[0].relDy, 1);
        CHECK(robots[0].hasPos);
        CHECK_EQ_INT(robots[0].pos.x, 9);
        CHECK_EQ_INT(robots[0].pos.y, 9);
        CHECK_EQ_INT(robots[0].hp, 60);
        CHECK_EQ_STR(robots[0].robotId, "B");

        std::vector<Entity> res = p.resourceList();
        CHECK_EQ_INT(res.size(), 2);
        // n09 的 '2' 在第 6 行第 5 列 → 相对 (0, 1) → 绝对 [8,9]
        // h01 的 '2' 在第 4 行第 7 列 → 相对 (2, -1) → 绝对 [10,7]
        Entity n09, h01;
        for (const Entity& e : res) {
            if (e.id == "n09") n09 = e;
            if (e.id == "h01") h01 = e;
        }
        CHECK_EQ_INT(n09.value, 10);
        CHECK(!n09.isHigh());
        CHECK_EQ_INT(n09.pos.x, 8);
        CHECK_EQ_INT(n09.pos.y, 9);
        CHECK_EQ_INT(h01.value, 20);
        CHECK(h01.isHigh());
        CHECK_EQ_INT(h01.pos.x, 10);
        CHECK_EQ_INT(h01.pos.y, 7);

        std::vector<Entity> obs = p.obstacleList();
        CHECK_EQ_INT(obs.size(), 1);
        // 障碍 '2' 在第 6 行第 8 列 → 相对 (3, 1) → 绝对 [11,9]
        CHECK_EQ_INT(obs[0].pos.x, 11);
        CHECK_EQ_INT(obs[0].pos.y, 9);
    }

    CASE("感知解析：value>=20 归为高价值（协议 §3）");
    {
        // type 写 normal 但 value 是 25，应按 value 校正成 high
        std::string text =
            "{\"tick\":5,\"session_id\":\"s\",\"self\":{\"team\":\"A\",\"position\":[3,3],"
            "\"hp\":100,\"alive\":true},"
            "\"visible_entities\":[{\"type\":\"resource_normal\",\"id\":\"x\","
            "\"position\":\"" + jsonEscapeForTest(makeVisionMap(1, 0)) + "\",\"value\":25}],"
            "\"map\":{\"width\":20,\"height\":20}}";
        Perception p;
        CHECK(parsePerception(text, p));
        CHECK_EQ_INT(p.entities.size(), 1);
        CHECK(p.entities[0].isHigh());
    }

    CASE("感知解析：字段缺失/类型错误不崩溃，标记为不完整");
    {
        Perception p;
        // 完全没有 self.position
        CHECK(parsePerception("{\"tick\":3,\"session_id\":\"s\"}", p));
        CHECK(!p.ok);
        CHECK(!p.problems.empty());

        // tick 是字符串
        CHECK(parsePerception("{\"tick\":\"abc\",\"session_id\":\"s\","
                              "\"self\":{\"position\":[1,1]}}", p));
        CHECK(!p.hasTick);

        // position 是字符串而不是数组
        CHECK(parsePerception("{\"tick\":3,\"session_id\":\"s\","
                              "\"self\":{\"position\":\"[1,1]\"}}", p));
        CHECK(!p.self.hasPos);
        CHECK(!p.ok);

        // visible_entities 不是数组
        CHECK(parsePerception("{\"tick\":3,\"session_id\":\"s\","
                              "\"self\":{\"position\":[1,1]},\"visible_entities\":{}}", p));
        CHECK(p.ok);   // 关键字段齐全，只是实体列表有问题
        CHECK(!p.problems.empty());

        // 完全不是 JSON
        CHECK(!parsePerception("这不是JSON", p));
        // 空字符串
        CHECK(!parsePerception("", p));
    }

    CASE("感知解析：vision_radius 越界要夹到 0-4");
    {
        Perception p;
        CHECK(parsePerception("{\"tick\":1,\"session_id\":\"s\",\"vision_radius\":99,"
                              "\"self\":{\"position\":[0,0]}}", p));
        CHECK_EQ_INT(p.visionRadius, 4);
        CHECK(parsePerception("{\"tick\":1,\"session_id\":\"s\",\"vision_radius\":-3,"
                              "\"self\":{\"position\":[0,0]}}", p));
        CHECK_EQ_INT(p.visionRadius, 0);
    }

    CASE("感知解析：effective_damage 可以是小数、可超过 500");
    {
        Perception p;
        CHECK(parsePerception("{\"tick\":1,\"session_id\":\"s\",\"self\":{\"position\":[0,0]},"
                              "\"score\":{\"effective_damage\":612.5,\"resource_value\":88.3}}", p));
        CHECK_NEAR(p.score.effectiveDamage, 612.5, 1e-9);
        CHECK_NEAR(p.score.resourceValue, 88.3, 1e-9);
    }

    CASE("感知解析：开局首帧 position_reason=initial 且 last_action 为 null");
    {
        Perception p;
        CHECK(parsePerception("{\"tick\":0,\"session_id\":\"s\",\"self\":{\"team\":\"A\","
                              "\"position\":[1,1],\"position_reason\":\"initial\","
                              "\"last_action\":null,\"hp\":100,\"alive\":true}}", p));
        CHECK(p.ok);
        CHECK_EQ_STR(p.self.positionReason, "initial");
        CHECK(!p.self.hasLastAction);   // null 不算有回执
    }

    CASE("感知解析：event_text 原样保存（不做任何关键词解析）");
    {
        Perception p;
        std::string text =
            "{\"tick\":50,\"session_id\":\"s\",\"self\":{\"position\":[2,2]},"
            "\"event_text\":\"第 50 Tick：随机事件预告：25 Tick 后（Tick 75），左上角 [4,4]、"
            "右下角 [9,9] 的矩形区域将发生爆炸\"}";
        CHECK(parsePerception(text, p));
        CHECK(p.eventText.find("爆炸") != std::string::npos);
    }
}

// ============================================================================
//  测试 4：世界记忆与寻路
// ============================================================================

static Perception makeSimplePerception(int tick, const std::string& session,
                                       int x, int y, int mapW = 24, int mapH = 24) {
    std::ostringstream os;
    os << "{\"tick\":" << tick << ",\"session_id\":\"" << session << "\","
       << "\"self\":{\"team\":\"A\",\"robot_id\":\"A\",\"position\":[" << x << "," << y << "],"
       << "\"hp\":100,\"alive\":true},\"visible_entities\":[],"
       << "\"score\":{\"resource_value\":0,\"effective_damage\":0},"
       << "\"map\":{\"width\":" << mapW << ",\"height\":" << mapH << "}}";
    Perception p;
    parsePerception(os.str(), p);
    return p;
}

static void testWorldModel() {
    CASE("世界记忆：换场必须清空记忆（协议 §4）");
    {
        WorldModel w;
        Perception p1 = makeSimplePerception(0, "match-1", 5, 5);
        w.syncSession(p1);
        w.observe(p1);
        CHECK(w.seenCells().size() > 0);

        Perception p2 = makeSimplePerception(0, "match-2", 1, 1);
        bool isNew = w.syncSession(p2);
        CHECK(isNew);
        CHECK_EQ_INT(w.seenCells().size(), 0);   // 记忆已清空
        CHECK_EQ_STR(w.sessionId(), "match-2");
    }

    CASE("世界记忆：同一 Tick 重复消息不重复推进");
    {
        WorldModel w;
        Perception p = makeSimplePerception(7, "s", 3, 3);
        w.syncSession(p);
        bool first = w.syncSession(p);
        CHECK(!first);
        w.observe(p);
        size_t n1 = w.seenCells().size();
        w.observe(p);   // 同 Tick 再来一次
        CHECK_EQ_INT(w.seenCells().size(), n1);
    }

    CASE("世界记忆：障碍记忆与 isBlocked");
    {
        WorldModel w;
        Perception p;
        std::string text =
            "{\"tick\":1,\"session_id\":\"s\",\"self\":{\"team\":\"A\",\"position\":[5,5],"
            "\"alive\":true,\"hp\":100},"
            "\"visible_entities\":[{\"type\":\"obstacle\",\"position\":\"" +
            jsonEscapeForTest(makeVisionMap(1, 0)) + "\"}],"
            "\"map\":{\"width\":24,\"height\":24}}";
        CHECK(parsePerception(text, p));
        w.syncSession(p);
        w.observe(p);
        CHECK(w.isObstacle(Cell(6, 5)));
        CHECK(w.isBlocked(Cell(6, 5)));
        CHECK(!w.isBlocked(Cell(5, 5)));
        // 越界也算 blocked
        CHECK(w.isBlocked(Cell(-1, 0)));
        CHECK(w.isBlocked(Cell(24, 0)));
        CHECK_EQ_INT(w.obstacleCount(), 1);
    }

    CASE("世界记忆：自己踩过的资源标记为已捡走（规则 §4.2 站上自动收集）");
    {
        WorldModel w;
        // 第 1 帧：看到右 1 格有个资源
        std::string t1 =
            "{\"tick\":1,\"session_id\":\"s\",\"self\":{\"team\":\"A\",\"position\":[5,5],"
            "\"alive\":true,\"hp\":100},"
            "\"visible_entities\":[{\"type\":\"resource_normal\",\"id\":\"n1\",\"value\":10,"
            "\"position\":\"" + jsonEscapeForTest(makeVisionMap(1, 0)) + "\"}],"
            "\"map\":{\"width\":24,\"height\":24}}";
        Perception p1;
        CHECK(parsePerception(t1, p1));
        w.syncSession(p1);
        w.observe(p1);
        CHECK_EQ_INT(w.pendingResources().size(), 1);

        // 第 2 帧：走到了 [6,5]，资源应该被标记为已捡
        Perception p2 = makeSimplePerception(2, "s", 6, 5);
        w.syncSession(p2);
        w.observe(p2);
        CHECK_EQ_INT(w.pendingResources().size(), 0);
    }

    CASE("世界记忆：敌人记忆按新鲜度过滤");
    {
        WorldModel w;
        std::string t1 =
            "{\"tick\":10,\"session_id\":\"s\",\"self\":{\"team\":\"A\",\"position\":[5,5],"
            "\"alive\":true,\"hp\":100},"
            "\"visible_entities\":[{\"type\":\"robot\",\"id\":\"robot_B\",\"team\":\"B\","
            "\"robot_id\":\"B\",\"hp\":80,\"position\":\"" +
            jsonEscapeForTest(makeVisionMap(2, 0)) + "\"}],"
            "\"map\":{\"width\":24,\"height\":24}}";
        Perception p1;
        CHECK(parsePerception(t1, p1));
        w.syncSession(p1);
        w.observe(p1);
        CHECK_EQ_INT(w.liveEnemies(10).size(), 1);
        CHECK_EQ_INT(w.liveEnemies(11).size(), 1);   // 1 Tick 后还记得
        CHECK_EQ_INT(w.liveEnemies(22).size(), 1);   // 12 Tick 内还记得
        CHECK_EQ_INT(w.liveEnemies(23).size(), 0);   // 超过 kEnemyMemoryTicks，认为不可靠
        CHECK_EQ_INT(w.liveEnemies(200).size(), 0);
    }

    CASE("世界记忆：reach 任务目标从感知读取（不写死 [12,12]）");
    {
        WorldModel w;
        std::string t1 =
            "{\"tick\":1,\"session_id\":\"s\",\"self\":{\"team\":\"A\",\"position\":[0,0],"
            "\"alive\":true,\"hp\":100},\"visible_entities\":[],"
            "\"missions\":[{\"id\":\"reach_x\",\"type\":\"reach\",\"target\":1,"
            "\"position\":[7,19]}],\"map\":{\"width\":24,\"height\":24}}";
        Perception p1;
        CHECK(parsePerception(t1, p1));
        w.syncSession(p1);
        w.observe(p1);
        CHECK_EQ_INT(w.reachTargets().size(), 1);
        CHECK_EQ_INT(w.reachTargets()[0].x, 7);
        CHECK_EQ_INT(w.reachTargets()[0].y, 19);
    }

    CASE("世界记忆：事件公告按行去重记录");
    {
        WorldModel w;
        std::string t1 =
            "{\"tick\":1,\"session_id\":\"s\",\"self\":{\"position\":[0,0],\"alive\":true},"
            "\"visible_entities\":[],\"event_text\":\"公告一\\n公告二\"}";
        Perception p1;
        CHECK(parsePerception(t1, p1));
        w.syncSession(p1);
        w.observe(p1);
        CHECK_EQ_INT(w.eventLines().size(), 2);
        // 重复收到同样公告不重复记录（协议：重复收到不表示重复触发）
        Perception p2;
        parsePerception("{\"tick\":2,\"session_id\":\"s\",\"self\":{\"position\":[0,0],"
                        "\"alive\":true},\"visible_entities\":[],\"event_text\":\"公告一\"}", p2);
        w.syncSession(p2);
        w.observe(p2);
        CHECK_EQ_INT(w.eventLines().size(), 2);
    }

    CASE("BFS 寻路：直线");
    {
        WorldModel w;
        Perception p = makeSimplePerception(1, "s", 0, 0);
        w.syncSession(p);
        w.observe(p);
        Cell next;
        Dir d = w.bfsNextStep(Cell(0, 0), Cell(3, 0), next);
        CHECK(d == Dir::Right);
        CHECK_EQ_INT(next.x, 1);
        CHECK_EQ_INT(next.y, 0);
        std::vector<Cell> path = w.bfsPath(Cell(0, 0), Cell(3, 0));
        CHECK_EQ_INT(path.size(), 4);   // 含起点和终点
    }

    CASE("BFS 寻路：绕开障碍");
    {
        WorldModel w;
        // 造一堵竖墙在 x=2，从 (0,2) 到 (4,2)，必须绕行
        std::string ents;
        for (int y = 0; y < 5; ++y) {
            // 障碍在绝对坐标 [2, y]；自己在 [0,2]，相对位移 (2, y-2)
            if (y != 2) {   // 视野只有 ±4，都在范围内
            }
            if (!ents.empty()) ents += ",";
            ents += "{\"type\":\"obstacle\",\"position\":\"" +
                    jsonEscapeForTest(makeVisionMap(2, y - 2)) + "\"}";
        }
        std::string text =
            "{\"tick\":1,\"session_id\":\"s\",\"self\":{\"team\":\"A\",\"position\":[0,2],"
            "\"alive\":true,\"hp\":100},\"visible_entities\":[" + ents + "],"
            "\"map\":{\"width\":10,\"height\":10}}";
        Perception p;
        CHECK(parsePerception(text, p));
        w.syncSession(p);
        w.observe(p);
        CHECK_EQ_INT(w.obstacleCount(), 5);
        CHECK(w.isBlocked(Cell(2, 2)));

        Cell next;
        Dir d = w.bfsNextStep(Cell(0, 2), Cell(4, 2), next);
        // 应该往上或往下绕，不会直直撞墙
        CHECK(d != Dir::None);
        CHECK(d != Dir::Right || !w.isBlocked(next));
        std::vector<Cell> path = w.bfsPath(Cell(0, 2), Cell(4, 2));
        CHECK(path.size() > 5);   // 绕行比直线长
        // 路径上不能有障碍
        for (const Cell& c : path) {
            CHECK(!w.isObstacle(c));
        }
    }

    CASE("BFS 寻路：已经在目标格 → Dir::None");
    {
        WorldModel w;
        Perception p = makeSimplePerception(1, "s", 4, 4);
        w.syncSession(p);
        w.observe(p);
        Cell next;
        CHECK(w.bfsNextStep(Cell(4, 4), Cell(4, 4), next) == Dir::None);
        CHECK_EQ_INT(next.x, 4);
    }

    CASE("BFS 寻路：完全被墙围住 → Dir::None（不崩溃）");
    {
        WorldModel w;
        std::string ents;
        // 上下左右四格都是障碍
        int dxs[4] = {0, 0, -1, 1};
        int dys[4] = {-1, 1, 0, 0};
        for (int i = 0; i < 4; ++i) {
            if (i) ents += ",";
            ents += "{\"type\":\"obstacle\",\"position\":\"" +
                    jsonEscapeForTest(makeVisionMap(dxs[i], dys[i])) + "\"}";
        }
        std::string text =
            "{\"tick\":1,\"session_id\":\"s\",\"self\":{\"team\":\"A\",\"position\":[5,5],"
            "\"alive\":true,\"hp\":100},\"visible_entities\":[" + ents + "],"
            "\"map\":{\"width\":20,\"height\":20}}";
        Perception p;
        CHECK(parsePerception(text, p));
        w.syncSession(p);
        w.observe(p);
        Cell next;
        CHECK(w.bfsNextStep(Cell(5, 5), Cell(1, 1), next) == Dir::None);
        CHECK(w.bfsPath(Cell(5, 5), Cell(1, 1)).empty());
    }

    CASE("snapshotText：包含关键字段（发给大模型用）");
    {
        WorldModel w;
        Perception p;
        CHECK(parsePerception(kProtocolExample, p));
        w.syncSession(p);
        w.observe(p);
        std::string s = w.snapshotText(p);
        CHECK(s.find("148") != std::string::npos);            // tick
        CHECK(s.find("example-match") != std::string::npos);   // session
        CHECK(s.find("任务描述") != std::string::npos);
        CHECK(s.find("资源价值") != std::string::npos);
        CHECK(s.size() > 100);
    }
}

// ============================================================================
//  测试 5：动作生成（最关键 —— 保证动作格式一定合法）
// ============================================================================

// 把 Decision 组装成协议要求的动作 JSON（和 bot_node 里的实现一致）
static std::string buildActionJson(const Perception& p, const Decision& d) {
    std::ostringstream os;
    os << "{\"tick\":" << p.tick
       << ",\"session_id\":\"" << jsonEscape(p.sessionId) << "\""
       << ",\"action\":\"" << actionName(d.action) << "\"";
    if (d.action == ActionType::Move || d.action == ActionType::Attack) {
        os << ",\"direction\":\"" << dirName(d.direction) << "\"";
    }
    os << "}";
    return os.str();
}

static void testActions() {
    CASE("动作 JSON：四种动作格式正确，字段小写");
    {
        Perception p;
        CHECK(parsePerception(kProtocolExample, p));

        Decision dMove;  dMove.action = ActionType::Move;   dMove.direction = Dir::Right;
        Decision dAtk;   dAtk.action = ActionType::Attack;  dAtk.direction = Dir::Up;
        Decision dDef;   dDef.action = ActionType::Defend;
        Decision dWait;  dWait.action = ActionType::Wait;

        std::string jm = buildActionJson(p, dMove);
        CHECK_EQ_STR(jm, "{\"tick\":148,\"session_id\":\"example-match\","
                         "\"action\":\"move\",\"direction\":\"right\"}");
        std::string ja = buildActionJson(p, dAtk);
        CHECK(ja.find("\"attack\"") != std::string::npos);
        CHECK(ja.find("\"up\"") != std::string::npos);
        // defend / wait 不带 direction（协议示例里就没有）
        std::string jd = buildActionJson(p, dDef);
        CHECK_EQ_STR(jd, "{\"tick\":148,\"session_id\":\"example-match\",\"action\":\"defend\"}");
        std::string jw = buildActionJson(p, dWait);
        CHECK_EQ_STR(jw, "{\"tick\":148,\"session_id\":\"example-match\",\"action\":\"wait\"}");
        CHECK(jw.find("direction") == std::string::npos);

        // 每条消息不超过 4096 字节（协议 E_RATE）
        CHECK(jm.size() < 4096);
        CHECK(ja.size() < 4096);
    }

    CASE("动作 JSON：tick 与 session_id 原样回填当前感知");
    {
        // 换一场：session_id 必须跟着变，不能沿用上一场
        Perception p1;
        parsePerception("{\"tick\":5,\"session_id\":\"match-1\",\"self\":{\"position\":[1,1],"
                        "\"alive\":true},\"visible_entities\":[],\"map\":{\"width\":20,\"height\":20}}", p1);
        Perception p2;
        parsePerception("{\"tick\":5,\"session_id\":\"match-2\",\"self\":{\"position\":[1,1],"
                        "\"alive\":true},\"visible_entities\":[],\"map\":{\"width\":20,\"height\":20}}", p2);
        Decision d; d.action = ActionType::Wait;
        CHECK(buildActionJson(p1, d).find("match-1") != std::string::npos);
        CHECK(buildActionJson(p2, d).find("match-2") != std::string::npos);
        CHECK(buildActionJson(p2, d).find("match-1") == std::string::npos);
    }

    CASE("第一阶段：只输出 move 或 wait（规则 §6，否则 E_STATE + 无效动作）");
    {
        Strategy s;
        s.setStage(1);
        WorldModel w;

        // 造一个带资源的第一阶段感知
        std::string text =
            "{\"tick\":3,\"session_id\":\"stage1\",\"self\":{\"team\":\"A\",\"robot_id\":\"A\","
            "\"position\":[5,5],\"hp\":100,\"alive\":true},\"visible_entities\":["
            "{\"type\":\"resource_normal\",\"id\":\"n1\",\"value\":10,\"position\":\"" +
            jsonEscapeForTest(makeVisionMap(2, 0)) + "\"}],"
            "\"score\":{\"resource_value\":0},\"map\":{\"width\":20,\"height\":20}}";
        Perception p;
        CHECK(parsePerception(text, p));
        w.syncSession(p);
        w.observe(p);

        Decision d = s.decide(p, w);
        CHECK(d.action == ActionType::Move || d.action == ActionType::Wait);
        CHECK(d.action != ActionType::Attack);
        CHECK(d.action != ActionType::Defend);
        if (d.action == ActionType::Move) {
            CHECK(d.direction != Dir::None);
        }
    }

    CASE("第一阶段：即使视野里有敌车也不许攻击");
    {
        Strategy s;
        s.setStage(1);
        WorldModel w;
        std::string text =
            "{\"tick\":3,\"session_id\":\"stage1\",\"self\":{\"team\":\"A\",\"position\":[5,5],"
            "\"hp\":100,\"alive\":true},\"visible_entities\":["
            "{\"type\":\"robot\",\"id\":\"robot_B\",\"team\":\"B\",\"robot_id\":\"B\",\"hp\":100,"
            "\"position\":\"" + jsonEscapeForTest(makeVisionMap(1, 0)) + "\"}],"
            "\"map\":{\"width\":20,\"height\":20}}";
        Perception p;
        CHECK(parsePerception(text, p));
        w.syncSession(p);
        w.observe(p);
        Decision d = s.decide(p, w);
        CHECK(d.action != ActionType::Attack);
        CHECK(d.action == ActionType::Move || d.action == ActionType::Wait);
    }

    CASE("阵亡期间：只能 wait（否则 E_STATE 且累加 invalid_actions）");
    {
        Strategy s;
        s.setStage(2);
        WorldModel w;
        std::string text =
            "{\"tick\":20,\"session_id\":\"s\",\"self\":{\"team\":\"A\",\"position\":[5,5],"
            "\"hp\":0,\"alive\":false,\"respawn_in\":2},\"visible_entities\":["
            "{\"type\":\"resource_normal\",\"id\":\"n1\",\"value\":10,\"position\":\"" +
            jsonEscapeForTest(makeVisionMap(1, 0)) + "\"}],"
            "\"map\":{\"width\":24,\"height\":24}}";
        Perception p;
        CHECK(parsePerception(text, p));
        w.syncSession(p);
        w.observe(p);
        Decision d = s.decide(p, w);
        CHECK(d.action == ActionType::Wait);
        CHECK(d.direction == Dir::None);
        CHECK(d.reason.find("阵亡") != std::string::npos);
    }

    CASE("出生保护期间：不能 ATTACK（规则 §6.2）");
    {
        Strategy s;
        s.setStage(2);
        s.setDecisionType(DecisionType::Aggressive);   // 激进档也不会破例
        WorldModel w;
        std::string text =
            "{\"tick\":30,\"session_id\":\"s\",\"self\":{\"team\":\"A\",\"position\":[5,5],"
            "\"hp\":100,\"alive\":true,\"protected\":true},\"visible_entities\":["
            "{\"type\":\"robot\",\"id\":\"robot_B\",\"team\":\"B\",\"robot_id\":\"B\",\"hp\":100,"
            "\"defending\":false,\"protected\":false,\"position\":\"" +
            jsonEscapeForTest(makeVisionMap(1, 0)) + "\"}],"
            "\"map\":{\"width\":24,\"height\":24}}";
        Perception p;
        CHECK(parsePerception(text, p));
        w.syncSession(p);
        w.observe(p);
        Decision d = s.decide(p, w);
        CHECK(d.action != ActionType::Attack);
    }

    CASE("第二阶段：敌人 adjacent 且能打死 → 攻击（拿 60 击杀奖励）");
    {
        Strategy s;
        s.setStage(2);
        s.setDecisionType(DecisionType::Balanced);
        WorldModel w;
        // 敌人在右 1 格，只剩 15 血 → 一刀 20 能打死
        std::string text =
            "{\"tick\":40,\"session_id\":\"s\",\"self\":{\"team\":\"A\",\"position\":[5,5],"
            "\"hp\":100,\"alive\":true},\"visible_entities\":["
            "{\"type\":\"robot\",\"id\":\"robot_B\",\"team\":\"B\",\"robot_id\":\"B\",\"hp\":15,"
            "\"defending\":false,\"protected\":false,\"position\":\"" +
            jsonEscapeForTest(makeVisionMap(1, 0)) + "\"}],"
            "\"score\":{\"resource_value\":100},\"map\":{\"width\":24,\"height\":24}}";
        Perception p;
        CHECK(parsePerception(text, p));
        w.syncSession(p);
        w.observe(p);
        Decision d = s.decide(p, w);
        CHECK(d.action == ActionType::Attack);
        CHECK(d.direction == Dir::Right);
    }

    CASE("第二阶段：敌人在 2 格外的斜向 → 不打（ATTACK 只打直线 1-2 格）");
    {
        Strategy s;
        s.setStage(2);
        WorldModel w;
        std::string text =
            "{\"tick\":40,\"session_id\":\"s\",\"self\":{\"team\":\"A\",\"position\":[5,5],"
            "\"hp\":100,\"alive\":true},\"visible_entities\":["
            "{\"type\":\"robot\",\"id\":\"robot_B\",\"team\":\"B\",\"robot_id\":\"B\",\"hp\":15,"
            "\"position\":\"" + jsonEscapeForTest(makeVisionMap(2, 2)) + "\"}],"
            "\"score\":{\"resource_value\":100},\"map\":{\"width\":24,\"height\":24}}";
        Perception p;
        CHECK(parsePerception(text, p));
        w.syncSession(p);
        w.observe(p);
        Decision d = s.decide(p, w);
        CHECK(d.action != ActionType::Attack);
    }

    CASE("第二阶段：敌人在 3 格外 → 超出攻击范围不打");
    {
        Strategy s;
        s.setStage(2);
        WorldModel w;
        std::string text =
            "{\"tick\":40,\"session_id\":\"s\",\"self\":{\"team\":\"A\",\"position\":[5,5],"
            "\"hp\":100,\"alive\":true},\"visible_entities\":["
            "{\"type\":\"robot\",\"id\":\"robot_B\",\"team\":\"B\",\"robot_id\":\"B\",\"hp\":15,"
            "\"position\":\"" + jsonEscapeForTest(makeVisionMap(3, 0)) + "\"}],"
            "\"score\":{\"resource_value\":100},\"map\":{\"width\":24,\"height\":24}}";
        Perception p;
        CHECK(parsePerception(text, p));
        w.syncSession(p);
        w.observe(p);
        Decision d = s.decide(p, w);
        CHECK(d.action != ActionType::Attack);
    }

    CASE("第二阶段：敌人在出生保护中 → 不打这个方向（受 0 伤害且挡住后方）");
    {
        Strategy s;
        s.setStage(2);
        s.setDecisionType(DecisionType::Aggressive);
        WorldModel w;
        std::string text =
            "{\"tick\":40,\"session_id\":\"s\",\"self\":{\"team\":\"A\",\"position\":[5,5],"
            "\"hp\":100,\"alive\":true},\"visible_entities\":["
            "{\"type\":\"robot\",\"id\":\"robot_B\",\"team\":\"B\",\"robot_id\":\"B\",\"hp\":15,"
            "\"protected\":true,\"position\":\"" + jsonEscapeForTest(makeVisionMap(1, 0)) + "\"}],"
            "\"score\":{\"resource_value\":100},\"map\":{\"width\":24,\"height\":24}}";
        Perception p;
        CHECK(parsePerception(text, p));
        w.syncSession(p);
        w.observe(p);
        Decision d = s.decide(p, w);
        CHECK(d.action != ActionType::Attack);
    }

    CASE("采集档（collect）：完全不打架");
    {
        Strategy s;
        s.setStage(2);
        s.setDecisionType(DecisionType::Collect);
        WorldModel w;
        std::string text =
            "{\"tick\":40,\"session_id\":\"s\",\"self\":{\"team\":\"A\",\"position\":[5,5],"
            "\"hp\":100,\"alive\":true},\"visible_entities\":["
            "{\"type\":\"robot\",\"id\":\"robot_B\",\"team\":\"B\",\"robot_id\":\"B\",\"hp\":15,"
            "\"position\":\"" + jsonEscapeForTest(makeVisionMap(1, 0)) + "\"}],"
            "\"score\":{\"resource_value\":100},\"map\":{\"width\":24,\"height\":24}}";
        Perception p;
        CHECK(parsePerception(text, p));
        w.syncSession(p);
        w.observe(p);
        Decision d = s.decide(p, w);
        CHECK(d.action != ActionType::Attack);
    }

    CASE("move/attack 必须带方向，否则降级为 wait（规则 §6）");
    {
        Strategy s;
        s.setStage(2);
        WorldModel w;
        // 感知关键字段缺失 → 不应产出带方向缺失的 move
        Perception p;
        parsePerception("{\"session_id\":\"s\",\"self\":{\"alive\":true}}", p);
        CHECK(!p.ok);
        Decision d = s.decide(p, w);
        CHECK(d.action == ActionType::Wait);
        CHECK(d.direction == Dir::None);
    }

    CASE("第二阶段：有资源时朝资源移动（不原地 wait）");
    {
        Strategy s;
        s.setStage(2);
        WorldModel w;
        std::string text =
            "{\"tick\":10,\"session_id\":\"s\",\"self\":{\"team\":\"A\",\"position\":[5,5],"
            "\"hp\":100,\"alive\":true},\"visible_entities\":["
            "{\"type\":\"resource_high\",\"id\":\"h1\",\"value\":20,\"position\":\"" +
            jsonEscapeForTest(makeVisionMap(0, -2)) + "\"}],"
            "\"score\":{\"resource_value\":0},\"map\":{\"width\":24,\"height\":24}}";
        Perception p;
        CHECK(parsePerception(text, p));
        w.syncSession(p);
        w.observe(p);
        Decision d = s.decide(p, w);
        CHECK(d.action == ActionType::Move);
        CHECK(d.direction == Dir::Up);   // 资源在正上方 2 格
    }
}

// ============================================================================
//  测试 6：事件提示解析（大模型输出）
// ============================================================================

static void testEventAdvice() {
    CASE("事件提示：解析爆炸矩形区域");
    {
        std::string modelOut =
            "{\"summary\":\"Tick 75 时 [4,4]-[9,9] 会爆炸，扣 80 HP，出生保护免疫\","
            "\"stance\":\"avoid_area\",\"from_tick\":75,"
            "\"danger_rects\":[[4,4,9,9]],\"danger_cells\":[],\"opportunity_cells\":[]}";
        EventAdvice a = parseEventAdvice(modelOut, 50, 24, 24);
        CHECK(a.valid);
        CHECK_EQ_STR(a.stance, "avoid_area");
        CHECK_EQ_INT(a.dangerRects.size(), 1);
        CHECK(a.isDangerous(Cell(4, 4)));     // 边界包含（规则 §8.2）
        CHECK(a.isDangerous(Cell(9, 9)));
        CHECK(a.isDangerous(Cell(6, 6)));     // 内部
        CHECK(!a.isDangerous(Cell(3, 4)));    // 外部
        CHECK(!a.isDangerous(Cell(10, 10)));
        // 矩形坐标写反了也能正确判断
        EventAdvice b = parseEventAdvice(
            "{\"danger_rects\":[[9,9,4,4]]}", 50, 24, 24);
        CHECK(b.valid);
        CHECK(b.isDangerous(Cell(5, 5)));
    }

    CASE("事件提示：模型输出被 markdown 包裹也能解析");
    {
        std::string modelOut =
            "```json\n{\"summary\":\"回血点在 [3,7]\",\"stance\":\"seek_point\","
            "\"opportunity_cells\":[[3,7]]}\n```";
        EventAdvice a = parseEventAdvice(modelOut, 10, 24, 24);
        CHECK(a.valid);
        CHECK_EQ_STR(a.stance, "seek_point");
        CHECK_EQ_INT(a.opportunityCells.size(), 1);
        CHECK_EQ_INT(a.opportunityCells[0].x, 3);
        CHECK_EQ_INT(a.opportunityCells[0].y, 7);
    }

    CASE("事件提示：模型输出前后有解释文字也能解析");
    {
        std::string modelOut =
            "好的，我的分析如下：{\"summary\":\"十字墙\",\"stance\":\"avoid_area\","
            "\"danger_cells\":[[5,0],[5,1]]} 希望有帮助。";
        EventAdvice a = parseEventAdvice(modelOut, 10, 24, 24);
        CHECK(a.valid);
        CHECK_EQ_INT(a.dangerCells.size(), 2);
    }

    CASE("事件提示：超出地图的坐标必须被丢弃");
    {
        std::string modelOut =
            "{\"summary\":\"x\",\"danger_cells\":[[999,999],[-5,-5],[3,3]]}";
        EventAdvice a = parseEventAdvice(modelOut, 10, 24, 24);
        CHECK(a.valid);
        CHECK_EQ_INT(a.dangerCells.size(), 1);   // 只有 [3,3] 合法
        CHECK_EQ_INT(a.dangerCells[0].x, 3);
    }

    CASE("事件提示：模型乱写 stance → 回落为空，不让策略走偏");
    {
        EventAdvice a = parseEventAdvice("{\"summary\":\"s\",\"stance\":\"毁灭全人类\"}",
                                         10, 24, 24);
        CHECK(a.valid);   // summary 有效所以 valid
        CHECK_EQ_STR(a.stance, "");   // 但 stance 被清空
    }

    CASE("事件提示：垃圾输出 → valid=false，上层退回保守打法");
    {
        CHECK(!parseEventAdvice("", 10, 24, 24).valid);
        CHECK(!parseEventAdvice("我不知道", 10, 24, 24).valid);
        CHECK(!parseEventAdvice("{坏掉的 json", 10, 24, 24).valid);
        CHECK(!parseEventAdvice("[]", 10, 24, 24).valid);
        EventAdvice a = parseEventAdvice("null", 10, 24, 24);
        CHECK(!a.valid);
    }

    CASE("策略消费事件提示：站在危险区 → 撤离");
    {
        Strategy s;
        s.setStage(2);
        WorldModel w;
        // 自己在 [6,6]，AI 说 [4,4]-[9,9] 危险
        std::string text =
            "{\"tick\":70,\"session_id\":\"s\",\"self\":{\"team\":\"A\",\"position\":[6,6],"
            "\"hp\":100,\"alive\":true},\"visible_entities\":[],"
            "\"score\":{\"resource_value\":100},\"map\":{\"width\":24,\"height\":24}}";
        Perception p;
        CHECK(parsePerception(text, p));
        w.syncSession(p);
        w.observe(p);

        EventAdvice adv = parseEventAdvice(
            "{\"summary\":\"爆炸\",\"stance\":\"avoid_area\","
            "\"danger_rects\":[[4,4,9,9]]}", 70, 24, 24);
        CHECK(adv.valid);
        s.setEventAdvice(adv);

        Decision d = s.decide(p, w);
        CHECK(d.usedAiAdvice);
        // 危险区覆盖 [4,4]-[9,9]，四个相邻格都在里面 → 无处可逃 → wait
        // （规则 §6.3：防御挡不住爆炸等环境伤害，所以 defend 没有意义）
        CHECK(d.action == ActionType::Wait);
    }

    CASE("策略消费事件提示：危险区可逃 → 移动离开");
    {
        Strategy s;
        s.setStage(2);
        WorldModel w;
        // 自己在 [9,9]，正好是危险区 [4,4]-[9,9] 的右下角。
        // 往上 [9,8]、往左 [8,9] 都还在区内；往下 [9,10]、往右 [10,9] 才出区。
        std::string text =
            "{\"tick\":70,\"session_id\":\"s\",\"self\":{\"team\":\"A\",\"position\":[9,9],"
            "\"hp\":100,\"alive\":true},\"visible_entities\":[],"
            "\"score\":{\"resource_value\":100},\"map\":{\"width\":24,\"height\":24}}";
        Perception p;
        CHECK(parsePerception(text, p));
        w.syncSession(p);
        w.observe(p);

        EventAdvice adv = parseEventAdvice(
            "{\"summary\":\"爆炸\",\"stance\":\"avoid_area\","
            "\"danger_rects\":[[4,4,9,9]]}", 70, 24, 24);
        CHECK(adv.valid);
        s.setEventAdvice(adv);

        Decision d = s.decide(p, w);
        CHECK(d.usedAiAdvice);
        CHECK(d.action == ActionType::Move);
        // 不指定具体方向（down 和 right 都能出区，代码按 上下左右 顺序取先找到的），
        // 但要验证"走完之后确实离开了危险区"—— 这才是真正该保证的行为。
        CHECK(d.direction == Dir::Down || d.direction == Dir::Right);
        int dx = 0, dy = 0;
        dirDelta(d.direction, dx, dy);
        Cell after(9 + dx, 9 + dy);
        CHECK(!adv.isDangerous(after));
    }

    CASE("策略消费事件提示：下一步会踩进危险区 → 绕开");
    {
        Strategy s;
        s.setStage(2);
        WorldModel w;
        // 资源在 [6,5]（右 1 格），但 AI 说 [6,5] 危险 → 不该往右走
        std::string text =
            "{\"tick\":70,\"session_id\":\"s\",\"self\":{\"team\":\"A\",\"position\":[5,5],"
            "\"hp\":100,\"alive\":true},\"visible_entities\":["
            "{\"type\":\"resource_high\",\"id\":\"h1\",\"value\":20,\"position\":\"" +
            jsonEscapeForTest(makeVisionMap(1, 0)) + "\"}],"
            "\"score\":{\"resource_value\":0},\"map\":{\"width\":24,\"height\":24}}";
        Perception p;
        CHECK(parsePerception(text, p));
        w.syncSession(p);
        w.observe(p);
        s.setEventAdvice(parseEventAdvice(
            "{\"summary\":\"火线\",\"stance\":\"avoid_area\",\"danger_cells\":[[6,5]]}",
            70, 24, 24));
        Decision d = s.decide(p, w);
        // 首选方向 right 危险 → 应该换成别的安全方向，而不是硬走
        if (d.action == ActionType::Move) {
            CHECK(d.direction != Dir::Right);
        }
    }
}

// ============================================================================
//  测试 7：稳定性 —— 程序绝不能崩
// ============================================================================

static void testRobustness() {
    CASE("稳定性：随机垃圾输入不崩溃");
    {
        Strategy s;
        WorldModel w;
        const char* garbage[] = {
            "", " ", "null", "[]", "{}", "{\"tick\":null}",
            "{\"tick\":\"x\",\"session_id\":123}",
            "{\"self\":\"不是对象\"}",
            "{\"visible_entities\":\"不是数组\"}",
            "{\"visible_entities\":[1,2,3,null,\"x\"]}",
            "{\"visible_entities\":[{\"type\":\"robot\"}]}",          // 没有 position
            "{\"visible_entities\":[{\"type\":\"未知类型\"}]}",
            "{\"score\":\"不是对象\"}",
            "{\"missions\":\"不是数组\"}",
            "{\"missions\":[1,2,3]}",
            "{\"map\":{\"width\":\"abc\"}}",
            "{\"self\":{\"position\":\"abc\"}}",
            "{\"self\":{\"position\":[1]}}",
            "{\"self\":{\"position\":[1,2,3,4,5]}}",
            "{\"event_text\":12345}",
            "{\"tick\":148,\"session_id\":\"s\",\"self\":{\"position\":[8,8],\"alive\":true}}",
        };
        for (const char* g : garbage) {
            Perception p;
            bool parsed = parsePerception(std::string(g), p);
            // 不管解析成不成，都不能崩溃
            w.syncSession(p);
            w.observe(p);
            s.setStage(1);
            Decision d1 = s.decide(p, w);
            s.setStage(2);
            Decision d2 = s.decide(p, w);
            // 动作字段永远合法
            if (d1.action == ActionType::Move || d1.action == ActionType::Attack) {
                CHECK(d1.direction != Dir::None);
            }
            if (d2.action == ActionType::Move || d2.action == ActionType::Attack) {
                CHECK(d2.direction != Dir::None);
            }
            (void)parsed;
            std::string j = buildActionJson(p, d2);
            CHECK(j.size() < 4096);   // 协议 E_RATE 上限
        }
        std::cout << "  （垃圾输入全部安全通过）\n";
    }

    CASE("稳定性：超长 position 字符串不崩溃");
    {
        std::string longStr(100000, '0');
        int dx = 0, dy = 0;
        bool r = parseRelativeMap(longStr, dx, dy);
        (void)r;
        CHECK(true);   // 走到这里就说明没崩
    }

    CASE("稳定性：连续跑 600 Tick 不崩溃（覆盖第二阶段 500 Tick）");
    {
        Strategy s;
        s.setStage(2);
        WorldModel w;
        for (int tick = 0; tick < 600; ++tick) {
            // 造一个逐渐移动的感知
            int x = (tick * 3) % 20;
            int y = (tick * 5) % 20;
            std::ostringstream os;
            os << "{\"tick\":" << tick << ",\"session_id\":\"long-run\","
               << "\"self\":{\"team\":\"A\",\"position\":[" << x << "," << y << "],"
               << "\"hp\":" << (100 - (tick % 100)) << ",\"alive\":true},"
               << "\"visible_entities\":[],"
               << "\"score\":{\"resource_value\":" << (tick % 130) << ","
               << "\"effective_damage\":" << (tick % 600) << "},"
               << "\"map\":{\"width\":24,\"height\":24}}";
            Perception p;
            if (!parsePerception(os.str(), p)) continue;
            w.syncSession(p);
            w.observe(p);
            Decision d = s.decide(p, w);
            if (d.action == ActionType::Move || d.action == ActionType::Attack) {
                CHECK(d.direction != Dir::None);
            }
            std::string j = buildActionJson(p, d);
            CHECK(j.size() < 4096);
        }
        std::cout << "  （600 Tick 循环安全通过）\n";
    }

    CASE("确定性：相同输入必得相同输出（规则 §1 要求）");
    {
        Strategy s1, s2;
        s1.setStage(2);
        s2.setStage(2);
        WorldModel w1, w2;
        Perception p;
        CHECK(parsePerception(kProtocolExample, p));
        w1.syncSession(p); w1.observe(p);
        w2.syncSession(p); w2.observe(p);
        Decision d1 = s1.decide(p, w1);
        Decision d2 = s2.decide(p, w2);
        CHECK(d1.action == d2.action);
        CHECK(d1.direction == d2.direction);
        CHECK_EQ_STR(buildActionJson(p, d1), buildActionJson(p, d2));
    }
}

// ============================================================================
//  main
// ============================================================================

int main() {
    std::cout << "==== RMTC 参赛程序核心逻辑测试 ====\n";
    std::cout << "（本测试不依赖 ROS 2，只验证解析、记忆、寻路与决策）\n";

    testJson();
    testRelativeMap();
    testPerception();
    testWorldModel();
    testActions();
    testEventAdvice();
    testRobustness();

    std::cout << "\n==== 结果 ====\n";
    std::cout << "断言总数: " << g_total << "\n";
    std::cout << "失败: " << g_failed << "\n";
    if (g_failed == 0) {
        std::cout << "\nALL TESTS PASSED\n";
        return 0;
    }
    std::cout << "\nTESTS FAILED\n";
    return 1;
}
