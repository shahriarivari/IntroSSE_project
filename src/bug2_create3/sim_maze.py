"""Maze simulator for the wall follower: many walls, narrow corridors, simulated lidar.

No ROS needed. Run from src/bug2_create3/:
    python3 sim_maze.py                     # random maze, summary
    python3 sim_maze.py --seed 3 -p         # other maze, save a picture (maze_seed3.png)
    python3 sim_maze.py --corridor 0.6 -p   # narrower corridors
    python3 sim_maze.py --all               # run 10 mazes and 2 corridor widths, summary table

What it measures:
  - gap:      smallest distance between the robot edge and any wall (< 0 = collision)
  - coverage: share of maze cells the robot visited (right-hand rule in a maze without
              islands should eventually visit every cell)
  - u-turns:  how often the robot turned > 150 deg in place at once (normal at dead ends,
              suspicious elsewhere)
"""
import argparse
import math
import random

import numpy as np

from bug2_create3 import geometry, lidar, wall_follow

# ---- lidar like the real one ----
ANGLE_MIN = -3.124139
INC = 0.00871450919657
N = 720
RANGE_MIN, RANGE_MAX = 0.15, 12.0
NOISE = 0.01            # [m]
DROPOUT = 0.03          # chance a single measurement is missing
LATENCY = 2             # scans are this many ticks old

# ---- same sectors as the node ----
WF_FRONT_HALF_WIDTH = math.radians(20)
SIDE_CENTER = math.radians(-75)
SIDE_HALF_WIDTH = math.radians(45)

DT = 0.1
ROBOT_RADIUS = 0.17
WALL_THICKNESS = 0.10


# ------------------------------------------------------------------ maze
def make_maze(cols, rows, seed):
    """Perfect maze (one path between any two cells) with depth-first search.
    Returns the set of open passages between neighbouring cells."""
    rng = random.Random(seed)
    open_between = set()
    seen = {(0, 0)}
    stack = [(0, 0)]
    while stack:
        c, r = stack[-1]
        options = [(c + dc, r + dr) for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1))
                   if 0 <= c + dc < cols and 0 <= r + dr < rows and (c + dc, r + dr) not in seen]
        if not options:
            stack.pop()
            continue
        nxt = rng.choice(options)
        open_between.add(frozenset(((c, r), nxt)))
        seen.add(nxt)
        stack.append(nxt)
    return open_between


def maze_walls(cols, rows, corridor, open_between):
    """Wall rectangles (x0, y0, x1, y1) for a maze with the given corridor width."""
    pitch = corridor + WALL_THICKNESS          # cell centre to cell centre
    t = WALL_THICKNESS
    rects = []
    for c in range(cols + 1):                  # vertical walls
        for r in range(rows):
            left, right = (c - 1, r), (c, r)
            if 0 < c < cols and frozenset((left, right)) in open_between:
                continue
            x = c * pitch
            rects.append((x, r * pitch, x + t, (r + 1) * pitch + t))
    for r in range(rows + 1):                  # horizontal walls
        for c in range(cols):
            below, above = (c, r - 1), (c, r)
            if 0 < r < rows and frozenset((below, above)) in open_between:
                continue
            y = r * pitch
            rects.append((c * pitch, y, (c + 1) * pitch + t, y + t))
    return rects, pitch


def rects_to_segments(rects):
    segs = []
    for x0, y0, x1, y1 in rects:
        p = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        segs += [(p[i], p[(i + 1) % 4]) for i in range(4)]
    a = np.array([s[0] for s in segs])
    b = np.array([s[1] for s in segs])
    return a, b


# ------------------------------------------------------------------ lidar + geometry
def fake_scan(x, y, yaw, seg_a, seg_b, rng):
    angles = yaw + ANGLE_MIN + np.arange(N) * INC
    dx, dy = np.cos(angles)[:, None], np.sin(angles)[:, None]      # rays (N, 1)
    ex, ey = (seg_b - seg_a)[:, 0][None, :], (seg_b - seg_a)[:, 1][None, :]
    ax, ay = seg_a[:, 0][None, :] - x, seg_a[:, 1][None, :] - y
    den = dx * ey - dy * ex
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (ax * ey - ay * ex) / den
        u = (ax * dy - ay * dx) / den
    hit = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
    t = np.where(hit, t, np.inf).min(axis=1)
    noise = rng.normal(0, NOISE, N)
    t = np.where(np.isfinite(t), t + noise, t)
    t = np.where(rng.random(N) < DROPOUT, np.inf, t)
    t = np.where(t > RANGE_MAX, np.inf, t)
    return t.tolist()


def gap_to_walls(x, y, rects):
    best = float('inf')
    for x0, y0, x1, y1 in rects:
        dx = max(x0 - x, 0, x - x1)
        dy = max(y0 - y, 0, y - y1)
        best = min(best, math.hypot(dx, dy))
    return best - ROBOT_RADIUS


# ------------------------------------------------------------------ simulation
def simulate(corridor=0.7, size=5.0, seed=0, minutes=6, verbose=False):
    cols = rows = int(size // (corridor + WALL_THICKNESS))
    rects, pitch = maze_walls(cols, rows, corridor, make_maze(cols, rows, seed))
    seg_a, seg_b = rects_to_segments(rects)
    rng = np.random.default_rng(seed)

    # start in the middle of cell (0, 0), facing +x
    x = y = WALL_THICKNESS + corridor / 2
    yaw = 0.0
    rule = 'follow'
    scans = []
    visited = set()
    min_gap = float('inf')
    path, rules = [], []
    turned, u_turns = 0.0, 0

    for step in range(int(minutes * 60 / DT)):
        scans.append(fake_scan(x, y, yaw, seg_a, seg_b, rng))
        ranges = scans[max(0, len(scans) - 1 - LATENCY)]
        front = lidar.sector_min(ranges, 0.0, WF_FRONT_HALF_WIDTH,
                                 ANGLE_MIN, INC, RANGE_MIN, RANGE_MAX)
        closest, closest_angle = lidar.sector_closest(ranges, SIDE_CENTER, SIDE_HALF_WIDTH,
                                                      ANGLE_MIN, INC, RANGE_MIN, RANGE_MAX)
        v, w, rule = wall_follow.compute_wall_follow(front, closest, closest_angle, rule)
        if rule == 'inner':
            turned += abs(w) * DT
        else:
            if turned > math.radians(150):
                u_turns += 1
            turned = 0.0

        x += v * math.cos(yaw) * DT
        y += v * math.sin(yaw) * DT
        yaw = geometry.wrap_angle(yaw + w * DT)

        gap = gap_to_walls(x, y, rects)
        min_gap = min(min_gap, gap)
        visited.add((int(x // pitch), int(y // pitch)))
        path.append((x, y))
        rules.append(rule)
        if verbose and step % 10 == 0:
            a = math.degrees(closest_angle) if closest_angle is not None else float('nan')
            print(f'{step:5d} {rule:6s} x={x:5.2f} y={y:5.2f} yaw={math.degrees(yaw):+4.0f} '
                  f'front={front:.2f} closest={closest:.2f}@{a:+4.0f} v={v:.2f} w={w:+.2f} '
                  f'gap={gap:+.2f}')
        if gap < 0:
            break

    coverage = len(visited & {(c, r) for c in range(cols) for r in range(rows)}) / (cols * rows)
    return dict(gap=min_gap, coverage=coverage, u_turns=u_turns, steps=step + 1,
                rects=rects, path=path, rules=rules, cols=cols, pitch=pitch)


def plot(result, filename):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7, 7))
    for x0, y0, x1, y1 in result['rects']:
        ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, color='black'))
    colours = {'follow': 'tab:blue', 'inner': 'tab:red', 'outer': 'tab:orange', 'round': 'tab:green'}
    xs, ys = zip(*result['path'])
    ax.plot(xs, ys, '-', color='lightgray', linewidth=1)
    for rule, colour in colours.items():
        pts = [p for p, r in zip(result['path'], result['rules']) if r == rule]
        if pts:
            px, py = zip(*pts)
            ax.plot(px, py, '.', markersize=2, color=colour, label=rule)
    ax.plot(xs[0], ys[0], 'go', label='start')
    ax.plot(xs[-1], ys[-1], 'kx', markersize=10, label='end')
    ax.set_aspect('equal')
    ax.legend(loc='upper right', fontsize=8)
    ax.set_title(f"gap {result['gap'] * 100:.1f} cm, coverage {result['coverage'] * 100:.0f} %, "
                 f"u-turns {result['u_turns']}")
    fig.savefig(filename, dpi=110)
    print('saved', filename)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--corridor', type=float, default=0.7)
    ap.add_argument('--minutes', type=float, default=6)
    ap.add_argument('-v', action='store_true', help='print a log line every second')
    ap.add_argument('-p', action='store_true', help='save a picture of the path')
    ap.add_argument('--all', action='store_true', help='run many mazes')
    args = ap.parse_args()

    if args.all:
        print(f"{'corridor':>8} {'seed':>4} {'gap cm':>7} {'coverage':>8} {'u-turns':>7}")
        for corridor in (0.6, 0.7):
            for seed in range(10):
                r = simulate(corridor, seed=seed, minutes=args.minutes)
                flag = '  COLLISION' if r['gap'] < 0 else ''
                print(f"{corridor:8.2f} {seed:4d} {r['gap'] * 100:7.1f} {r['coverage'] * 100:7.0f}% "
                      f"{r['u_turns']:7d}{flag}")
    else:
        r = simulate(args.corridor, seed=args.seed, minutes=args.minutes, verbose=args.v)
        print(f"gap {r['gap'] * 100:.1f} cm | coverage {r['coverage'] * 100:.0f} % | "
              f"u-turns {r['u_turns']} | {r['steps'] * DT:.0f} s")
        if args.p:
            plot(r, f'maze_seed{args.seed}_c{args.corridor}.png')