#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RMTC 比赛实时可视化（本地测试工具，不属于参赛程序）。

只订阅官方话题（/team_X/perception、/referee/mission、/referee/event_text、
/referee/team_registry），把各队视野的并集画出来，因此模拟裁判和官方练习
裁判（rmtc_practice）都能用。

用法（先 source ROS 环境并设好 ROS_DOMAIN_ID=27、ROS_LOCALHOST_ONLY=0）:

  python3 tools/visualizer.py                          # 自动选渲染方式
  python3 tools/visualizer.py --map maps/stage2-24x24.json   # 预载全图（配合本地模拟裁判）
  python3 tools/visualizer.py --ascii                  # 强制终端字符画（零依赖）
  python3 tools/visualizer.py --rate 10                # 刷新频率

有 matplotlib 就用图形窗口，没有就退回终端字符画。注意协议只下发各队
视野内的信息，没有 --map 时地图空白处是"未探明"，不代表真的没有东西。
"""

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String

TEAMS = 'ABCD'
TEAM_COLORS = {'A': '#00a2e8', 'B': '#e4002b', 'C': '#9c27b0', 'D': '#1e5eff'}
ANSI_COLORS = {'A': '36', 'B': '31', 'C': '35', 'D': '34'}


def qos(reliable, durable, depth):
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL if durable else DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def grid_offset(grid):
    """协议 9x9 相对坐标字符串 -> 实体相对自机的偏移 (dx, dy)。"""
    if not isinstance(grid, str):
        return None
    rows = grid.splitlines()
    for r, row in enumerate(rows[:9]):
        c = row.find('2')
        if c >= 0:
            return (c - 4, r - 4)
    return None


class World:
    def __init__(self):
        self.width = None
        self.height = None
        self.tick = 0
        self.obstacles = set()
        self.resources = {}   # id -> {pos, value, high}
        self.robots = {}      # team -> {pos, hp, alive, defending, protected, score}
        self.names = {}       # team -> 队伍名（来自 team_registry）
        self.mission = ''
        self.event = ''

    def in_bounds(self, x, y):
        return (self.width is not None and self.height is not None
                and 0 <= x < self.width and 0 <= y < self.height)

    def load_map_file(self, path):
        """预载地图编辑器导出的 JSON，让本地模拟对战时能看到全图。"""
        try:
            with open(path, encoding='utf-8') as f:
                data = json.load(f)
        except (OSError, json.JSONDecodeError) as e:
            print('[可视化] 无法读取地图 %s：%s' % (path, e))
            return
        m = data.get('map') or {}
        w, h = m.get('width'), m.get('height')
        if isinstance(w, int) and isinstance(h, int) and w > 0 and h > 0:
            self.width, self.height = w, h
        for p in m.get('obstacles') or []:
            if isinstance(p, (list, tuple)) and len(p) == 2:
                self.obstacles.add((int(p[0]), int(p[1])))
        for r in m.get('resources') or []:
            pos = r.get('position')
            if isinstance(pos, (list, tuple)) and len(pos) == 2:
                value = r.get('value', 0)
                self.resources[r.get('id') or 'map%02d' % len(self.resources)] = {
                    'pos': (int(pos[0]), int(pos[1])),
                    'value': value,
                    'high': value >= 20,
                }
        print('[可视化] 已预载地图 %s：%sx%s，障碍 %d，资源 %d' % (
            path, self.width, self.height, len(self.obstacles), len(self.resources)))


class Visualizer(Node):
    def __init__(self, args):
        super().__init__('visualizer')
        self.world = World()
        if args.map:
            self.world.load_map_file(args.map)
        for team in TEAMS:
            self.create_subscription(
                String, '/team_%s/perception' % team,
                self._perception_cb(team), qos(True, False, 1))
        self.create_subscription(String, '/referee/mission',
                                 self._mission_cb, qos(True, True, 1))
        self.create_subscription(String, '/referee/event_text',
                                 self._event_cb, qos(True, True, 128))
        self.create_subscription(String, '/referee/team_registry',
                                 self._registry_cb, qos(True, True, 1))
        self._printed_mission = ''
        self._printed_event = ''

    # ---------- 订阅回调 ----------
    def _perception_cb(self, team):
        def cb(msg):
            try:
                data = json.loads(msg.data)
            except json.JSONDecodeError:
                return
            if isinstance(data, dict):
                self._update(team, data)
        return cb

    def _mission_cb(self, msg):
        text = msg.data.strip()
        if text.startswith('{'):
            try:
                d = json.loads(text)
                if isinstance(d, dict) and d.get('mission_text'):
                    text = d['mission_text']
            except json.JSONDecodeError:
                pass
        if text:
            self.world.mission = text

    def _event_cb(self, msg):
        text = msg.data.strip()
        if text.startswith('{'):
            try:
                d = json.loads(text)
                if isinstance(d, dict) and (d.get('event') or d.get('text')):
                    text = d.get('event') or d.get('text')
            except json.JSONDecodeError:
                pass
        if text:
            self.world.event = text

    def _registry_cb(self, msg):
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(d, dict):
            return
        for t in d.get('teams') or []:
            team, name = t.get('team'), t.get('name')
            if team and name:
                self.world.names[team] = name

    # ---------- 状态合并 ----------
    def _update(self, team, data):
        w = self.world
        m = data.get('map') or {}
        if isinstance(m, dict):
            ww, hh = m.get('width'), m.get('height')
            if isinstance(ww, int) and isinstance(hh, int) and ww > 0 and hh > 0:
                w.width, w.height = ww, hh
        tick = data.get('tick')
        if isinstance(tick, int):
            w.tick = tick
        if data.get('mission_text'):
            w.mission = data['mission_text']
        if data.get('event_text'):
            w.event = data['event_text']

        self_pos = None
        s = data.get('self') or {}
        if isinstance(s, dict):
            pos = s.get('position')
            if isinstance(pos, (list, tuple)) and len(pos) == 2:
                self_pos = (int(pos[0]), int(pos[1]))
                w.robots.setdefault(team, {})
                w.robots[team].update({
                    'pos': self_pos,
                    'hp': s.get('hp'),
                    'alive': s.get('alive', True),
                    'defending': bool(s.get('defending')),
                    'protected': bool(s.get('protected')),
                    'score': data.get('score') or {},
                })
        for e in data.get('visible_entities') or []:
            if isinstance(e, dict):
                self._apply_entity(e, self_pos)

    def _apply_entity(self, e, self_pos):
        w = self.world
        off = grid_offset(e.get('position'))
        if off is None or self_pos is None:
            return
        x, y = self_pos[0] + off[0], self_pos[1] + off[1]
        etype = e.get('type')
        if etype == 'obstacle':
            w.obstacles.add((x, y))
        elif etype in ('resource_normal', 'resource_high'):
            rid = e.get('id')
            if rid:
                w.resources[rid] = {
                    'pos': (x, y),
                    'value': e.get('value', 0),
                    'high': etype == 'resource_high',
                }
        elif etype == 'robot':
            oteam = e.get('team') or e.get('robot_id')
            if oteam:
                w.robots.setdefault(oteam, {})
                w.robots[oteam].update({
                    'pos': (x, y),
                    'hp': e.get('hp'),
                    'alive': True,
                    'defending': bool(e.get('defending')),
                    'protected': bool(e.get('protected')),
                })

    # ---------- 文案 ----------
    def robot_lines(self):
        lines = []
        for team in TEAMS:
            r = self.world.robots.get(team)
            if not r:
                continue
            name = self.world.names.get(team)
            parts = ['%s%s' % (team, '(%s)' % name if name else '')]
            if not r.get('alive', True):
                parts.append('阵亡')
            else:
                if r.get('hp') is not None:
                    parts.append('HP=%s' % r['hp'])
                if r.get('defending'):
                    parts.append('防守')
                if r.get('protected'):
                    parts.append('保护')
            x, y = r.get('pos', ('?', '?'))
            parts.append('pos=[%s,%s]' % (x, y))
            s = r.get('score') or {}
            if 'resource_value' in s:
                parts.append('资源=%s' % s['resource_value'])
            if 'effective_damage' in s:
                parts.append('伤害=%s' % s['effective_damage'])
            if 'mission_completed' in s:
                parts.append('任务=%s' % s['mission_completed'])
            lines.append(' '.join(parts))
        return lines

    def announce(self):
        if self.world.mission and self.world.mission != self._printed_mission:
            self._printed_mission = self.world.mission
            print('[任务] %s' % self.world.mission.replace('\n', ' '))
        if self.world.event and self.world.event != self._printed_event:
            self._printed_event = self.world.event
            print('[事件] %s' % self.world.event.replace('\n', ' '))


# ---------- 渲染：终端字符画 ----------
class AsciiRenderer:
    def __init__(self, node):
        self.node = node

    def draw(self):
        w = self.node.world
        if w.width is None:
            return
        lines = ['\033[2J\033[H', '=== tick=%s ===' % w.tick]
        lines += self.node.robot_lines()
        grid = [['.'] * w.width for _ in range(w.height)]
        for (x, y) in w.obstacles:
            if w.in_bounds(x, y):
                grid[y][x] = '#'
        for res in w.resources.values():
            x, y = res['pos']
            if w.in_bounds(x, y):
                grid[y][x] = '$' if res.get('high') else '*'
        for team in TEAMS:
            r = w.robots.get(team)
            if not r:
                continue
            x, y = r['pos']
            if not w.in_bounds(x, y):
                continue
            grid[y][x] = team if r.get('alive', True) else team.lower()
        paint = {'#': '90', '*': '33', '$': '32'}
        for row in grid:
            out = []
            for ch in row:
                if ch in paint:
                    out.append('\033[%sm%s\033[0m' % (paint[ch], ch))
                elif ch in TEAMS:
                    out.append('\033[%sm%s\033[0m' % (ANSI_COLORS[ch], ch))
                elif ch in TEAMS.lower():
                    out.append('\033[90m%s\033[0m' % ch)
                else:
                    out.append(ch)
            lines.append(''.join(out))
        lines.append('图例: #障碍 *普通资源 $高级资源 大写=存活 小写=阵亡  行=y 列=x')
        print('\n'.join(lines))


# ---------- 渲染：matplotlib 图形窗口 ----------
class MatRenderer:
    def __init__(self, node):
        self.node = node
        self.failed = None
        self.closed = False
        try:
            import matplotlib
            matplotlib.use('TkAgg')
            import matplotlib.pyplot as plt
            import numpy as np
            from matplotlib.colors import ListedColormap
            from matplotlib.patches import Circle
            self.plt = plt
            self.np = np
            self.Circle = Circle
            self.cmap = ListedColormap(['#f2f2f2', '#8d8d8d', '#ffd54f', '#66bb6a'])
            self.fig, self.ax = plt.subplots(figsize=(7.5, 7.5))
            try:
                self.fig.canvas.manager.set_window_title('RMTC visualizer')
            except Exception:
                pass

            def on_close(ev):
                self.closed = True
            self.fig.canvas.mpl_connect('close_event', on_close)
        except Exception as e:
            self.failed = 'matplotlib 不可用：%s' % e

    def draw(self):
        w = self.node.world
        if self.closed or w.width is None:
            return
        try:
            self._draw()
            self.fig.canvas.draw_idle()
            self.plt.pause(0.001)
        except Exception:
            self.closed = True

    def _draw(self):
        w = self.node.world
        ax = self.ax
        ax.clear()
        grid = self.np.zeros((w.height, w.width))
        for (x, y) in w.obstacles:
            if w.in_bounds(x, y):
                grid[y][x] = 1
        for res in w.resources.values():
            x, y = res['pos']
            if w.in_bounds(x, y):
                grid[y][x] = 3 if res.get('high') else 2
        ax.imshow(grid, cmap=self.cmap, vmin=0, vmax=3, origin='upper', aspect='equal')
        ax.set_xticks(range(w.width))
        ax.set_yticks(range(w.height))
        ax.set_xticklabels(range(w.width), fontsize=6)
        ax.set_yticklabels(range(w.height), fontsize=6)
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.grid(True, color='#cccccc', linewidth=0.4)

        parts = []
        for team in TEAMS:
            r = w.robots.get(team)
            if not r:
                continue
            x, y = r['pos']
            if not w.in_bounds(x, y):
                continue
            alive = r.get('alive', True)
            ax.text(x, y, team if alive else team.lower(), ha='center', va='center',
                    fontsize=13, fontweight='bold',
                    color=TEAM_COLORS[team], alpha=1.0 if alive else 0.35)
            if alive:
                if r.get('hp') is not None:
                    ax.text(x, y + 0.3, '%s' % r['hp'], ha='center', va='center',
                            fontsize=6, color='#222222')
                    parts.append('%s:%sHP' % (team, r['hp']))
                if r.get('defending'):
                    ax.add_patch(self.Circle((x, y), 0.42, fill=False,
                                             edgecolor=TEAM_COLORS[team], linewidth=1.5))
        ax.set_title('tick=%s   %s' % (w.tick, '  '.join(parts)), fontsize=10)


def main():
    parser = argparse.ArgumentParser(description='RMTC 比赛实时可视化（测试工具）')
    parser.add_argument('--map', default='', help='地图编辑器导出的 JSON（预载全图障碍/资源）')
    parser.add_argument('--ascii', action='store_true', help='强制终端字符画，不依赖 matplotlib')
    parser.add_argument('--rate', type=float, default=10.0, help='刷新频率 Hz，默认 10')
    args = parser.parse_args()

    rclpy.init()
    node = Visualizer(args)

    renderer = None
    if not args.ascii:
        m = MatRenderer(node)
        if m.failed:
            print('[可视化] %s，改用终端字符画（想要图形窗口就 pip install matplotlib）。' % m.failed)
        else:
            renderer = m
    if renderer is None:
        renderer = AsciiRenderer(node)
        print('[可视化] 终端字符画模式，比赛开始后每个 Tick 刷新；Ctrl+C 退出。')

    period = 1.0 / max(0.5, args.rate)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if isinstance(renderer, MatRenderer) and renderer.closed:
                print('[可视化] 图形窗口已关闭，切换到终端字符画。')
                renderer = AsciiRenderer(node)
            node.announce()
            renderer.draw()
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
