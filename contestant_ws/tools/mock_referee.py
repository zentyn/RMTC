#!/usr/bin/env python3
"""本地自测裁判（非官方）。

用途：没有 rmtc_practice 官方练习包时，先在本机验证参赛程序的
感知解析 / 动作提交 / 回填 tick 与 session_id / 两阶段策略是否工作。

它不是官方裁判，地图、事件、计分都只做近似，不能代表正式规则。

用法:
  ros2 run --prefix 'python3' ... 太麻烦，直接:
  python3 tools/mock_referee.py --stage 1
  python3 tools/mock_referee.py --stage 2 --tick-ms 500 --ticks 500
  python3 tools/mock_referee.py --stage 2 --event "Tick 200 后地图中心出现 3x3 堡垒"
  python3 tools/mock_referee.py --stage 2 --map maps/stage2-24x24.json
  python3 tools/mock_referee.py --stage 1 --map maps/stage1-20x20.json
  # 双人对战：默认 A 是真人，再加 --human B 表示 B 也由真人控制
  python3 tools/mock_referee.py --stage 2 --map maps/stage2-24x24.json --human B
"""

import argparse
import json
import random
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String

TEAM = 'A'
BOTS = ['B', 'C', 'D']
DIRS = {'up': (0, -1), 'down': (0, 1), 'left': (-1, 0), 'right': (1, 0)}


def qos(reliable, durable, depth):
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL if durable else DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def make_grid(offset):
    """把相对偏移编码成协议要求的 9x9 字符串。"""
    rows = []
    for r in range(9):
        row = []
        for c in range(9):
            if r == 4 and c == 4:
                row.append('1')
            elif r - 4 == offset[1] and c - 4 == offset[0]:
                row.append('2')
            else:
                row.append('0')
        rows.append(''.join(row))
    return '\n'.join(rows)


class Robot:
    def __init__(self, team, pos, spawn):
        self.team = team
        self.pos = list(pos)
        self.spawn = list(spawn)
        self.hp = 100
        self.alive = True
        self.defending = False
        self.respawn_in = 0
        self.protected = 0
        self.last_action = None
        self.resource_value = 0
        self.damage = 0.0
        self.deaths = 0
        self.mission_ids = set()
        self.collected_normal = 0
        self.collected_high = 0

    def alive_cell(self):
        return tuple(self.pos) if self.alive else None


class MockReferee(Node):
    def __init__(self, args):
        super().__init__('mock_referee')
        self.args = args
        self.session = 'mock-%d' % int(time.time())
        self.tick = 0
        self.rng = random.Random(args.seed)
        self.width = 20 if args.stage == 1 else 24
        self.height = self.width
        self.vision = 4
        self.obstacles = set()
        self.resources = {}
        self.humans = {TEAM}
        if args.human:
            self.humans.update(args.human)
        if self.args.stage == 1 and self.humans != {TEAM}:
            raise SystemExit('[裁判] 第一阶段只有 A 席位；双人对战请使用 --stage 2')
        self.pending = {t: None for t in self.humans}
        self.registrations = {}
        self.event_text = ''
        self.event_sent = False

        self.robots = {TEAM: Robot(TEAM, (2, 2), (2, 2))}
        if args.stage >= 2:
            spawns = {'B': (self.width - 3, 2), 'C': (2, self.height - 3),
                      'D': (self.width - 3, self.height - 3)}
            for team in BOTS:
                self.robots[team] = Robot(team, spawns[team], spawns[team])

        if args.map:
            self._load_map(args.map)
        else:
            self._build_map()

        self.perception_pubs = {}
        for t in sorted(self.humans):
            self.perception_pubs[t] = self.create_publisher(
                String, '/team_%s/perception' % t, qos(True, False, 1))
            self.create_subscription(String, '/team_%s/action' % t,
                                     self._action_cb(t), qos(True, False, 1))
            self.create_subscription(String, '/team_%s/registration' % t,
                                     self._registration_cb(t), qos(True, True, 1))
        self.mission_pub = self.create_publisher(String, '/referee/mission', qos(True, True, 1))
        self.event_pub = self.create_publisher(String, '/referee/event_text', qos(True, True, 128))
        self.registry_pub = self.create_publisher(String, '/referee/team_registry', qos(True, True, 1))

        self.missions = [
            {'id': 'collect_two_normal', 'type': 'normal_resources', 'target': 2},
            {'id': 'collect_high_value', 'type': 'high_resources', 'target': 1},
            {'id': 'reach_target', 'type': 'reach', 'target': 1,
             'position': [self.width // 2, self.height // 2]},
        ]
        self.mission_text = ('获得至少两个普通资源; 获得至少一个高价值资源; '
                             '比赛结束前进入坐标 [%d, %d]。' % (self.width // 2, self.height // 2))

    def _build_map(self):
        rng = self.rng
        count = 10 if self.args.stage == 1 else 16
        while len(self.obstacles) < count:
            x = rng.randint(1, self.width - 2)
            y = rng.randint(1, self.height - 2)
            if abs(x - 2) + abs(y - 2) < 4:
                continue
            self.obstacles.add((x, y))

        free = [(x, y) for x in range(self.width) for y in range(self.height)
                if (x, y) not in self.obstacles]
        rng.shuffle(free)
        for i in range(12):
            self.resources['n%02d' % i] = {'pos': free[i], 'value': 10}
        for i in range(4):
            self.resources['h%02d' % i] = {'pos': free[12 + i], 'value': 20}

    def _load_map(self, path):
        """加载地图编辑器导出的 JSON（geometry 或 practice 格式均可）。"""
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except (OSError, json.JSONDecodeError) as e:
            raise SystemExit('[裁判] 无法读取地图 %s：%s' % (path, e))
        if (not isinstance(data, dict) or not isinstance(data.get('map'), dict)
                or not isinstance(data.get('robots'), list)):
            raise SystemExit('[裁判] 地图 JSON 无效：需要 map 和 robots 字段（地图编辑器导出格式）')
        m = data['map']
        w, h = m.get('width'), m.get('height')
        if not isinstance(w, int) or not isinstance(h, int) or w < 13 or h < 13:
            raise SystemExit('[裁判] 地图宽高无效：%r x %r（到达任务在中心，宽高至少 13）' % (w, h))
        self.width, self.height = w, h
        for p in m.get('obstacles') or []:
            if not self.in_bounds(p):
                raise SystemExit('[裁判] 障碍越界：%r' % (p,))
            self.obstacles.add(tuple(p))
        seen = set()
        for r in m.get('resources') or []:
            rid, pos, value = r.get('id'), r.get('position'), r.get('value')
            if (not isinstance(rid, str) or not rid or rid in seen
                    or not self.in_bounds(pos)
                    or not isinstance(value, int) or value <= 0
                    or tuple(pos) in self.obstacles):
                raise SystemExit('[裁判] 资源条目无效：%r' % (r,))
            seen.add(rid)
            self.resources[rid] = {'pos': [pos[0], pos[1]], 'value': value}
        spawns = {}
        for r in data['robots']:
            team, sp = r.get('team'), r.get('spawn')
            if (team not in ('A', 'B', 'C', 'D') or team in spawns
                    or not self.in_bounds(sp) or tuple(sp) in self.obstacles):
                raise SystemExit('[裁判] 出生点条目无效：%r' % (r,))
            spawns[team] = sp
        if 'A' not in spawns:
            raise SystemExit('[裁判] 地图缺少 A 出生点')
        self.robots = {TEAM: Robot(TEAM, tuple(spawns['A']), tuple(spawns['A']))}
        if self.args.stage >= 2:
            for team in BOTS:
                if team not in spawns:
                    raise SystemExit('[裁判] 第二阶段地图缺少 %s 出生点' % team)
                self.robots[team] = Robot(team, tuple(spawns[team]), tuple(spawns[team]))
        center = (self.width // 2, self.height // 2)
        if center in self.obstacles:
            raise SystemExit('[裁判] 地图中心 %r 有障碍，到达任务无法完成' % (center,))
        print('[裁判] 使用地图 %s：%dx%d，障碍 %d，资源 %d，出生点 %s' % (
            path, self.width, self.height, len(self.obstacles), len(self.resources),
            {t: list(p) for t, p in spawns.items()}))

    # ---------- ROS 回调 ----------
    def _action_cb(self, team):
        def cb(msg):
            self.on_action(msg, team)
        return cb

    def _registration_cb(self, team):
        def cb(msg):
            try:
                data = json.loads(msg.data)
            except json.JSONDecodeError:
                return
            if data.get('team') == team:
                self.registrations[team] = data.get('name', team)
        return cb

    def on_action(self, msg, team):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            print('[裁判] %s E_PARSE: %s' % (team, msg.data[:120]))
            return
        tick = data.get('tick')
        session = data.get('session_id')
        if session != self.session:
            print('[裁判] %s E_SESSION: 收到 session_id=%r，本场是 %r' % (team, session, self.session))
            return
        if tick != self.tick:
            print('[裁判] %s E_TICK: 收到 tick=%r，当前 tick=%d' % (team, tick, self.tick))
            return
        action = data.get('action')
        direction = data.get('direction')
        if action not in ('move', 'attack', 'defend', 'wait'):
            print('[裁判] %s E_SCHEMA: 非法 action=%r' % (team, action))
            return
        if action in ('move', 'attack'):
            if direction not in DIRS:
                print('[裁判] %s E_SCHEMA: 非法 direction=%r' % (team, direction))
                return
            if self.args.stage == 1 and action == 'attack':
                print('[裁判] %s E_STATE: 第一阶段不允许 ATTACK' % team)
                return
        if self.robots[team].protected > 0 and action == 'attack':
            print('[裁判] %s E_STATE: 出生保护期间不能 ATTACK' % team)
            return
        self.pending[team] = {'action': action, 'direction': direction}

    # ---------- 感知 ----------
    def publish_perception(self):
        for team in sorted(self.humans):
            self._publish_perception_for(team)

    def _publish_perception_for(self, team):
        me = self.robots[team]
        entities = []
        for (x, y) in sorted(self.obstacles):
            if max(abs(x - me.pos[0]), abs(y - me.pos[1])) <= self.vision:
                entities.append({'type': 'obstacle', 'position': make_grid((x - me.pos[0], y - me.pos[1]))})
        for rid, res in sorted(self.resources.items()):
            x, y = res['pos']
            if max(abs(x - me.pos[0]), abs(y - me.pos[1])) <= self.vision:
                entities.append({
                    'type': 'resource_high' if res['value'] >= 20 else 'resource_normal',
                    'id': rid,
                    'position': make_grid((x - me.pos[0], y - me.pos[1])),
                    'value': res['value'],
                })
        for other_team, bot in self.robots.items():
            if other_team == team or not bot.alive:
                continue
            x, y = bot.pos
            if max(abs(x - me.pos[0]), abs(y - me.pos[1])) <= self.vision:
                entities.append({
                    'type': 'robot', 'id': 'robot_%s' % other_team, 'team': other_team,
                    'robot_id': other_team,
                    'position': make_grid((x - me.pos[0], y - me.pos[1])),
                    'hp': bot.hp, 'defending': bot.defending,
                    'protected': bot.protected > 0,
                })

        payload = {
            'tick': self.tick,
            'session_id': self.session,
            'vision_radius': self.vision,
            'self': {
                'team': team,
                'position': [me.pos[0], me.pos[1]],
                'hp': me.hp,
                'alive': me.alive,
                'defending': me.defending,
                'respawn_in': me.respawn_in,
                'protected': me.protected > 0,
                'robot_id': team,
                'last_action': me.last_action,
            },
            'visible_entities': entities,
            'score': {
                'resource_value': me.resource_value,
                'effective_damage': round(me.damage, 3),
                'mission_completed': len(me.mission_ids),
                'mission_completed_ids': sorted(me.mission_ids),
                'invalid_actions': 0,
            },
            'mission_text': self.mission_text,
            'missions': self.missions,
            'map': {'width': self.width, 'height': self.height},
            'event_text': self.event_text,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.perception_pubs[team].publish(msg)

    # ---------- 结算 ----------
    def in_bounds(self, cell):
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height

    def occupied(self, cell, ignore=None):
        for team, bot in self.robots.items():
            if team == ignore or not bot.alive:
                continue
            if tuple(bot.pos) == cell:
                return True
        return False

    def attack_damage(self, attacker, target):
        if target.protected > 0:
            return 0
        return 10 if target.defending else 20

    def apply_attack(self, bot, direction):
        dx, dy = DIRS[direction]
        best = None
        for dist in (1, 2):
            cell = (bot.pos[0] + dx * dist, bot.pos[1] + dy * dist)
            if not self.in_bounds(cell):
                continue
            for team, other in self.robots.items():
                if team == bot.team or not other.alive:
                    continue
                if tuple(other.pos) == cell:
                    best = other
                    break
            if best is not None:
                break
        if best is None:
            return
        dmg = self.attack_damage(bot, best)
        best.hp = max(0, best.hp - dmg)
        if dmg > 0:
            bot.damage += dmg
        if best.hp == 0:
            self.kill(best)

    def kill(self, bot):
        bot.alive = False
        bot.hp = 0
        bot.respawn_in = 3
        bot.deaths += 1
        bot.damage = max(0.0, bot.damage - 60)

    def try_move(self, bot, direction):
        dx, dy = DIRS[direction]
        target = (bot.pos[0] + dx, bot.pos[1] + dy)
        ok = (self.in_bounds(target) and target not in self.obstacles
              and not self.occupied(target, ignore=bot.team))
        if ok:
            bot.pos = [target[0], target[1]]
        return ok

    def collect(self, bot):
        for rid, res in list(self.resources.items()):
            if tuple(res['pos']) == tuple(bot.pos):
                bot.resource_value += res['value']
                if res['value'] >= 20:
                    bot.collected_high += 1
                else:
                    bot.collected_normal += 1
                del self.resources[rid]

    def update_missions(self, bot):
        if not bot.alive:
            return
        if bot.collected_normal >= 2:
            bot.mission_ids.add('collect_two_normal')
        if bot.collected_high >= 1:
            bot.mission_ids.add('collect_high_value')
        target = (self.missions[2]['position'][0], self.missions[2]['position'][1])
        if tuple(bot.pos) == target:
            bot.mission_ids.add('reach_target')

    def step(self):
        self.tick += 1
        for bot in self.robots.values():
            bot.defending = False

        for team in sorted(self.humans):
            me = self.robots[team]
            act_pending = self.pending[team]
            if me.alive and act_pending is not None:
                act = act_pending['action']
                moved = False
                error = None
                if act == 'move':
                    moved = self.try_move(me, act_pending['direction'])
                    if not moved:
                        error = 'E_BLOCKED'
                elif act == 'attack':
                    self.apply_attack(me, act_pending['direction'])
                elif act == 'defend':
                    me.defending = True
                me.last_action = {
                    'tick': self.tick - 1,
                    'action': act,
                    'moved': moved,
                    'effective_direction': act_pending.get('direction') if moved else None,
                }
                if error:
                    me.last_action['error'] = error

        for team, bot in self.robots.items():
            if team not in self.humans and bot.alive:
                self.bot_ai(bot)

        for bot in self.robots.values():
            if bot.alive:
                self.collect(bot)
                self.update_missions(bot)
            else:
                bot.respawn_in -= 1
                if bot.respawn_in <= 0:
                    bot.alive = True
                    bot.hp = 100
                    bot.pos = list(bot.spawn)
                    # 本 step 末尾会统一减 1，所以初值给 3，实际保护 2 个 Tick
                    bot.protected = 3
        for bot in self.robots.values():
            if bot.protected > 0:
                bot.protected -= 1

        for team in self.humans:
            self.pending[team] = None

    def _bfs(self, start):
        """从 start 出发的 BFS（只避开障碍），返回 (距离表, 来路表)。"""
        dist = {start: 0}
        prev = {start: None}
        q = deque([start])
        while q:
            cur = q.popleft()
            for name, (dx, dy) in DIRS.items():
                nxt = (cur[0] + dx, cur[1] + dy)
                if not self.in_bounds(nxt) or nxt in self.obstacles or nxt in prev:
                    continue
                prev[nxt] = (cur, name)
                dist[nxt] = dist[cur] + 1
                q.append(nxt)
        return dist, prev

    def _first_step(self, start, goal, prev):
        node = goal
        while prev.get(node) is not None and prev[node][0] != start:
            node = prev[node][0]
        return prev[node][1] if prev.get(node) else None

    def bot_ai(self, bot):
        """陪练：能攻击最近的人类队伍就攻击；离其曼哈顿距离<=10 就追击，否则 BFS 捡最近资源。"""
        me = None
        best_d = 1 << 30
        for t in self.humans:
            hb = self.robots[t]
            if not hb.alive:
                continue
            d = abs(hb.pos[0] - bot.pos[0]) + abs(hb.pos[1] - bot.pos[1])
            if d < best_d:
                best_d = d
                me = hb
        if me is not None and me.alive and bot.protected == 0:
            dx = me.pos[0] - bot.pos[0]
            dy = me.pos[1] - bot.pos[1]
            if dx == 0 and 0 < abs(dy) <= 2:
                self.apply_attack(bot, 'down' if dy > 0 else 'up')
                return
            if dy == 0 and 0 < abs(dx) <= 2:
                self.apply_attack(bot, 'right' if dx > 0 else 'left')
                return

        start = tuple(bot.pos)
        dist, prev = self._bfs(start)
        goal = None
        if me is not None and me.alive and abs(me.pos[0] - bot.pos[0]) + abs(me.pos[1] - bot.pos[1]) <= 10:
            goal = tuple(me.pos)
        else:
            for res in self.resources.values():
                p = tuple(res['pos'])
                if p in dist and (goal is None or dist[p] < dist[goal]):
                    goal = p
        if goal is not None and goal in dist:
            d = self._first_step(start, goal, prev)
            if d:
                self.try_move(bot, d)

    # ---------- 主循环 ----------
    def publish_registry(self):
        teams = [{'team': t, 'name': self.registrations.get(t, '队伍%s' % t)}
                 for t in sorted(self.robots)]
        self.registry_pub.publish(String(data=json.dumps({'teams': teams}, ensure_ascii=False)))
        print('[裁判] 队伍名录: %s' % '，'.join('%s=%s' % (x['team'], x['name']) for x in teams))

    def run(self):
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)
            self.mission_pub.publish(String(data=self.mission_text))
        self.publish_registry()

        period = self.args.tick_ms / 1000.0
        for tick in range(self.args.ticks):
            tick_start = time.time()
            if tick == 0:
                self.publish_registry()
            if self.args.event and tick == self.args.ticks // 3 and not self.event_sent:
                self.event_text = self.args.event
                self.event_pub.publish(String(data=self.event_text))
                self.event_sent = True
                print('[裁判] 发布事件(自然语言): %s' % self.event_text)

            self.publish_perception()
            deadline = tick_start + period * 0.7
            while time.time() < deadline and not all(self.pending.values()):
                rclpy.spin_once(self, timeout_sec=0.002)

            for team in self.humans:
                if self.pending[team] is None:
                    self.pending[team] = {'action': 'wait', 'direction': None}
                    print('[裁判] tick=%d %s 未收到动作，执行 WAIT' % (self.tick, team))

            self.step()

            if self.tick % 20 == 0:
                for team in sorted(self.humans):
                    me = self.robots[team]
                    print('[裁判] tick=%d %s 位置=%s HP=%d 资源=%d 伤害=%.0f 任务=%d 剩余资源=%d' % (
                        self.tick, team, me.pos, me.hp, me.resource_value, me.damage,
                        len(me.mission_ids), len(self.resources)))

            remain = period - (time.time() - tick_start)
            if remain > 0:
                time.sleep(remain)

        self.finish()

    def finish(self):
        if self.args.stage == 1:
            me = self.robots['A']
            score = 40.0 * min(me.resource_value, 200) / 200.0
            print('\n==== 第一阶段结束 ====')
            print('A队 资源总价值 R1=%d，得分 = 40 * R1/200 = %.1f' % (me.resource_value, score))
        else:
            print('\n==== 第二阶段结束 ====')
            for team in sorted(self.humans):
                me = self.robots[team]
                r2 = min(me.resource_value, 120)
                d = min(me.damage, 500)
                m = len(me.mission_ids)
                score = 30.0 * r2 / 120.0 + 15.0 * d / 500.0 + 5.0 * m
                print('[裁判] %s队 R2=%d D=%.0f M=%d -> 得分 %.1f' % (
                    team, me.resource_value, me.damage, m, score))
        print('（本地自测裁判，结果不代表正式比赛）')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--stage', type=int, default=1, choices=[1, 2])
    parser.add_argument('--tick-ms', type=int, default=0, help='0 表示使用阶段默认值')
    parser.add_argument('--ticks', type=int, default=0)
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--event', type=str, default='', help='测试用自然语言事件公告')
    parser.add_argument('--map', type=str, default='',
                        help='加载地图编辑器导出的 JSON 地图（含 map 与 robots 字段）')
    parser.add_argument('--human', action='append', choices=['A', 'B', 'C', 'D'], default=[],
                        help='真人操控的席位，可重复指定（默认 A；双人对战：--human B）')
    args = parser.parse_args()
    if args.tick_ms <= 0:
        args.tick_ms = 100 if args.stage == 1 else 500
    if args.ticks <= 0:
        args.ticks = 100 if args.stage == 1 else 500

    rclpy.init()
    node = MockReferee(args)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
