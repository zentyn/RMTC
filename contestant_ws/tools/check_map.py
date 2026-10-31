#!/usr/bin/env python3
"""校验地图编辑器导出的 JSON 地图能否用于本地自测裁判。

检查项：格式、坐标越界、出生点/资源/障碍冲突、资源数量(12普通+4高级)、
中心到达点无障碍、从 A 出生点出发能否到达所有空格与所有出生点。

用法:
  python3 tools/check_map.py maps/stage2-24x24.json
  python3 tools/check_map.py maps/stage1-20x20.json
"""

import argparse
import json
import sys
from collections import deque


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('map', help='地图 JSON 路径')
    args = parser.parse_args()

    with open(args.map, 'r', encoding='utf-8') as f:
        try:
            data = json.load(f)
        except json.JSONDecodeError as e:
            print('FAIL: JSON 解析失败: %s' % e)
            return 1

    if (not isinstance(data, dict) or not isinstance(data.get('map'), dict)
            or not isinstance(data.get('robots'), list)):
        print('FAIL: 缺少 map 和 robots 字段（请用地图编辑器的导出格式）')
        return 1
    m = data['map']
    w, h = m.get('width'), m.get('height')
    if not isinstance(w, int) or not isinstance(h, int) or w < 13 or h < 13:
        print('FAIL: 宽高无效 %r x %r：本地裁判的到达任务在中心，宽高至少 13' % (w, h))
        return 1

    ok = True

    def fail(msg):
        nonlocal ok
        ok = False
        print('FAIL: ' + msg)

    def inside(p):
        return (isinstance(p, (list, tuple)) and len(p) == 2
                and all(isinstance(v, int) for v in p)
                and 0 <= p[0] < w and 0 <= p[1] < h)

    occupied = {}
    for p in m.get('obstacles') or []:
        if not inside(p):
            fail('障碍越界: %r' % (p,))
        elif tuple(p) in occupied:
            fail('障碍重复: %r' % (p,))
        else:
            occupied[tuple(p)] = 'obstacle'

    ids = set()
    normal = high = 0
    for r in m.get('resources') or []:
        rid, p, v = r.get('id'), r.get('position'), r.get('value')
        if (not isinstance(rid, str) or not rid or rid in ids
                or not isinstance(v, int) or v <= 0):
            fail('资源条目无效: %r' % (r,))
            continue
        ids.add(rid)
        if not inside(p):
            fail('资源 %s 越界: %r' % (rid, p))
        elif tuple(p) in occupied:
            fail('资源 %s 与 %s 重合: %r' % (rid, occupied[tuple(p)], p))
        else:
            occupied[tuple(p)] = 'resource:' + rid
            if v >= 20:
                high += 1
            else:
                normal += 1

    spawns = {}
    stage = 1 if len(data['robots']) == 1 else 2
    for r in data['robots']:
        team, p = r.get('team'), r.get('spawn')
        if team not in ('A', 'B', 'C', 'D') or not inside(p):
            fail('出生点条目无效: %r' % (r,))
            continue
        if team in spawns:
            fail('出生点重复: %s' % team)
        if tuple(p) in occupied:
            fail('%s 出生点与 %s 重合' % (team, occupied[tuple(p)]))
        spawns[team] = tuple(p)

    if 'A' not in spawns:
        fail('缺少 A 出生点')
    if stage == 2:
        for t in ('B', 'C', 'D'):
            if t not in spawns:
                fail('第二阶段地图缺少 %s 出生点' % t)

    if normal != 12 or high != 4:
        fail('资源数量应为 12 普通 + 4 高级，当前 %d + %d' % (normal, high))

    if not ok:
        print('校验失败，见上方 FAIL 条目')
        return 1

    walls = set(tuple(p) for p in m.get('obstacles') or [])
    start = spawns['A']
    seen = {start}
    q = deque([start])
    while q:
        x, y = q.popleft()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n = (x + dx, y + dy)
            if 0 <= n[0] < w and 0 <= n[1] < h and n not in walls and n not in seen:
                seen.add(n)
                q.append(n)
    free = w * h - len(walls)
    if len(seen) != free:
        print('FAIL: 地图不连通：可达 %d 格 / 共 %d 空格' % (len(seen), free))
        return 1
    for t, p in spawns.items():
        if p not in seen:
            print('FAIL: %s 出生点不可达: %r' % (t, p))
            return 1
    cx, cy = w // 2, h // 2
    if (cx, cy) in walls:
        print('FAIL: 中心到达点 (%d,%d) 有障碍' % (cx, cy))
        return 1

    print('OK: %dx%d 地图，障碍 %d，资源 %d（普通 %d + 高级 %d），阶段 %d，'
          '全部空格连通，出生点与中心点可用'
          % (w, h, len(walls), normal + high, normal, high, stage))
    return 0


if __name__ == '__main__':
    sys.exit(main())
