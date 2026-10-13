"""Bug2 in the real arena (obstacle table as in maps/make_arena_map.py), no ROS needed.

Runs the SAME decision logic as bug2_node.py, using the package modules
(bug2, geometry, go_to_goal, lidar, wall_follow) and a simulated lidar.

Run from src/bug2_create3/:
    python3 sim_bug2_arena.py                      # red -> green, picture + summary
    python3 sim_bug2_arena.py --seed 3             # other noise
    python3 sim_bug2_arena.py --all                # 10 noise seeds, summary table
    python3 sim_bug2_arena.py --start 0.4 1.82 0 --goal 3.25 1.20
"""
import argparse
import math

import numpy as np

from bug2_create3 import bug2, geometry, go_to_goal, lidar, wall_follow

# ---- lidar like the real one ----
ANGLE_MIN = -3.124139
INC = 0.00871450919657
N = 720
RANGE_MIN, RANGE_MAX = 0.15, 12.0
NOISE = 0.01
DROPOUT = 0.03
LATENCY = 1                     # ticks the scan is old (calibrated on the 2026-10-07 logs)
DT = 0.1
ROBOT_RADIUS = 0.17
ACCEL = 1.0                     # [m/s^2]   the real robot coasts ~0.3 s after v = 0
ANG_ACCEL = 6.0                 # [rad/s^2]
CMD_DELAY = 2                   # ticks between cmd_vel and the wheels reacting (seen in odom_w)
GOAL_CORRIDOR = 0.19            # [m] half width of the corridor toward the goal (= node)
SWITCH_HAND = True              # same as the node: try the other hand once
USE_CORRIDOR = True            # False = old ±35 deg cone (GOAL_HALF_WIDTH)

# ---- copied from bug2_node.py (keep in sync!) ----
FRONT_HALF_WIDTH = math.radians(40)
WF_FRONT_HALF_WIDTH = math.radians(20)
GOAL_HALF_WIDTH = math.radians(35)
SIDE_CENTER = math.radians(-75)
SIDE_HALF_WIDTH = math.radians(45)
HIT_LEFT_RADIUS = 0.50
HIT_RETURN_RADIUS = 0.20
MAX_FOLLOW_LENGTH = 30.0

# ---- arena (same table as maps/make_arena_map.py), in metres (x0, x1, y0, y1) ----
OBSTACLES = {      # version 3 (2026-10-07): open gaps >= 47 cm (measured ~50 cm)
    'O1': [(0.475, 0.595, 0.00, 1.01)],
    'O2': [(0.000, 1.020, 2.24, 2.36)],
    'O3': [(1.085, 1.205, 1.05, 1.77)],
    'O4': [(1.075, 1.785, 0.47, 0.58)],
    'O5': [(1.640, 1.760, 2.00, 2.34)],
    'O6': [(2.270, 2.400, 0.00, 0.73), (2.270, 2.800, 0.61, 0.73)],
    'O7': [(2.205, 2.335, 1.225, 1.625), (2.205, 2.735, 1.515, 1.625)],
    'O8': [(2.250, 2.370, 2.18, 2.91)],
    'O9': [(3.200, 3.330, 1.71, 2.41)],
    'Doos': [(3.930, 4.340, 1.51, 1.765)],
}
O10 = [(3.84, 0.10), (3.72, 0.10), (3.63, 0.79), (3.75, 0.79)]      # slanted
OUTER = [(-0.10, 0.00, -0.10, 2.94),       # back wall x = 0
         (4.34, 4.44, -0.10, 2.94),        # doos wall
         (-0.10, 2.34, -0.10, 0.00),       # wall y = 0 ...
         (2.34, 4.44, -0.10, 0.05),        # ... 5 cm further in after the kink
         (-0.10, 2.34, 2.837, 2.94),       # far wall ...
         (2.34, 4.44, 2.905, 3.00)]        # ... kink


def all_rects():
    rects = list(OUTER)
    for parts in OBSTACLES.values():
        rects += parts
    return rects


def segments():
    segs = []
    for x0, x1, y0, y1 in all_rects():
        p = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        segs += [(p[i], p[(i + 1) % 4]) for i in range(4)]
    segs += [(O10[i], O10[(i + 1) % 4]) for i in range(4)]
    return np.array([s[0] for s in segs]), np.array([s[1] for s in segs])


def fake_scan(x, y, yaw, a, b, rng):
    angles = yaw + ANGLE_MIN + np.arange(N) * INC
    dx, dy = np.cos(angles)[:, None], np.sin(angles)[:, None]
    ex, ey = (b - a)[:, 0][None, :], (b - a)[:, 1][None, :]
    ax, ay = a[:, 0][None, :] - x, a[:, 1][None, :] - y
    den = dx * ey - dy * ex
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (ax * ey - ay * ex) / den
        u = (ax * dy - ay * dx) / den
    hit = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
    t = np.where(hit, t, np.inf).min(axis=1)
    t = np.where(np.isfinite(t), t + rng.normal(0, NOISE, N), t)
    t = np.where(rng.random(N) < DROPOUT, np.inf, t)
    return np.where(t > RANGE_MAX, np.inf, t).tolist()


def point_segment(px, py, ax, ay, bx, by):
    vx, vy = bx - ax, by - ay
    s = max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / (vx * vx + vy * vy)))
    return math.hypot(px - ax - s * vx, py - ay - s * vy)


def gap(x, y, a, b):
    d = min(point_segment(x, y, *a[i], *b[i]) for i in range(len(a)))
    return d - ROBOT_RADIUS


def simulate(start, goal, seed=0, max_time=300.0, verbose=False, hand='right'):
    mirror = -1.0 if hand == 'left' else 1.0
    a, b = segments()
    rng = np.random.default_rng(seed)
    x, y, yaw = start
    gx, gy = goal
    sx, sy = x, y
    v_now = w_now = 0.0

    state, wf_rule = 'GO_TO_GOAL', 'follow'
    hit_point = hit_goal_distance = None
    left_hit_point = False
    switched = False
    follow_length = 0.0
    previous_side = previous_xy = None
    scans, path, events, cmds = [], [], [], []
    min_gap = float('inf')
    t = 0.0

    while t < max_time and state in ('GO_TO_GOAL', 'WALL_FOLLOW'):
        scans.append(fake_scan(x, y, yaw, a, b, rng))
        ranges = scans[max(0, len(scans) - 1 - LATENCY)]
        sector = (ANGLE_MIN, INC, RANGE_MIN, RANGE_MAX)

        # ---- same as bug2_node.publish_cmd_vel ----
        distance = geometry.distance_to_goal(x, y, gx, gy)
        side = geometry.signed_distance_to_line(sx, sy, gx, gy, x, y)
        line_distance = abs(side)
        on_line = bug2.on_m_line(side, previous_side)
        goal_dir = geometry.wrap_angle(geometry.angle_to_goal(x, y, gx, gy) - yaw)
        fhw = WF_FRONT_HALF_WIDTH if state == 'WALL_FOLLOW' else FRONT_HALF_WIDTH
        front = lidar.sector_min(ranges, 0.0, fhw, *sector)
        narrow_front = lidar.sector_min(ranges, 0.0, WF_FRONT_HALF_WIDTH, *sector)
        if USE_CORRIDOR:
            goal_clearance = lidar.corridor_clearance(ranges, goal_dir, GOAL_CORRIDOR, *sector)
        else:
            goal_clearance = lidar.sector_min(ranges, goal_dir, GOAL_HALF_WIDTH, *sector)
        closest, closest_angle = lidar.sector_closest(ranges, mirror * SIDE_CENTER,
                                                      SIDE_HALF_WIDTH, *sector)
        if closest_angle is not None:
            closest_angle *= mirror          # left-hand = mirrored right-hand follower
        v = w = 0.0
        rule = '-'
        if distance < go_to_goal.GOAL_TOLERANCE or bug2.goal_close_enough(
                distance, goal_clearance, go_to_goal.GOAL_TOLERANCE, GOAL_CORRIDOR):
            state = 'DONE'
            events.append((t, 'DONE', x, y))
        elif state == 'GO_TO_GOAL':
            v, w = go_to_goal.compute_command(x, y, yaw, gx, gy)
            # same as bug2_node.py: no hit if the robot can reach the goal before the obstacle
            path_to_goal_clear = bug2.goal_reachable(goal_clearance, distance,
                                                     go_to_goal.GOAL_TOLERANCE, GOAL_CORRIDOR)
            blocked_front = v > 0 and narrow_front < wall_follow.WALL_IN_FRONT
            if not path_to_goal_clear and (goal_clearance < wall_follow.WALL_IN_FRONT
                                           or blocked_front):
                v, w, rule = 0.0, 0.0, 'hit'
                hit_point, hit_goal_distance = (x, y), distance
                left_hit_point, follow_length, wf_rule = False, 0.0, 'follow'
                switched = False
                state = 'WALL_FOLLOW'
                events.append((t, f'hit (goal {distance:.2f} m)', x, y))
            else:
                rule = 'drive'
        elif state == 'WALL_FOLLOW':
            hit_distance = math.hypot(x - hit_point[0], y - hit_point[1])
            if hit_distance > HIT_LEFT_RADIUS:
                left_hit_point = True
            if previous_xy is not None:
                follow_length += math.hypot(x - previous_xy[0], y - previous_xy[1])
            leave_line_distance = 0.0 if on_line else line_distance
            if bug2.should_leave(leave_line_distance, distance, hit_goal_distance):
                rule, state = 'leave', 'GO_TO_GOAL'
                events.append((t, f'leave (line {line_distance:.3f}, goal {distance:.2f})', x, y))
            elif left_hit_point and hit_distance < HIT_RETURN_RADIUS and SWITCH_HAND and not switched:
                # Back at the hit point: maybe the follower drifted onto another obstacle.
                # Try once more, following with the other hand, before giving up.
                mirror, switched = -mirror, True
                left_hit_point, follow_length, wf_rule = False, 0.0, 'follow'
                events.append((t, 'switch hand', x, y))
            elif left_hit_point and hit_distance < HIT_RETURN_RADIUS:
                state = 'UNREACHABLE'
                events.append((t, 'UNREACHABLE (back at hit point)', x, y))
            elif follow_length > MAX_FOLLOW_LENGTH:
                state = 'UNREACHABLE'
                events.append((t, f'UNREACHABLE (followed {follow_length:.0f} m)', x, y))
            else:
                v, w, rule = wall_follow.compute_wall_follow(front, closest, closest_angle,
                                                             wf_rule)
                w *= mirror
                wf_rule = rule
        previous_side, previous_xy = side, (x, y)

        # ---- robot motion: command delay + acceleration limits ----
        cmds.append((v, w))
        v, w = cmds[max(0, len(cmds) - 1 - CMD_DELAY)]
        v_now += max(-ACCEL * DT, min(v - v_now, ACCEL * DT))
        w_now += max(-ANG_ACCEL * DT, min(w - w_now, ANG_ACCEL * DT))
        x += v_now * math.cos(yaw) * DT
        y += v_now * math.sin(yaw) * DT
        yaw = geometry.wrap_angle(yaw + w_now * DT)
        g = gap(x, y, a, b)
        min_gap = min(min_gap, g)
        path.append((x, y, state, rule))
        if verbose and int(round(t / DT)) % 10 == 0:
            print(f'{t:6.1f} {state:11s} {rule:6s} x={x:5.2f} y={y:5.2f} '
                  f'goal={distance:.2f} line={line_distance:.3f} front={front:.2f} '
                  f'goal_clr={goal_clearance:.2f} gap={g:+.2f}')
        if g < 0:
            events.append((t, 'COLLISION', x, y))
            break
        t += DT

    if state in ('GO_TO_GOAL', 'WALL_FOLLOW'):
        events.append((t, 'TIMEOUT', x, y))
    return dict(path=path, events=events, min_gap=min_gap, time=t, start=start, goal=goal,
                state=state)


def plot(r, filename):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(10, 7))
    for x0, x1, y0, y1 in all_rects():
        ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, color='black'))
    ax.add_patch(plt.Polygon(O10, color='black'))
    for name, parts in OBSTACLES.items():
        x0, x1, y0, y1 = parts[0]
        ax.text((x0 + x1) / 2, (y0 + y1) / 2, name, color='red', fontsize=8, ha='center')
    sx, sy, _ = r['start']
    gx, gy = r['goal']
    ax.plot([sx, gx], [sy, gy], '--', color='gray', label='m-line')
    colours = {'GO_TO_GOAL': 'tab:blue', 'WALL_FOLLOW': 'tab:orange'}
    for st, c in colours.items():
        pts = [(p[0], p[1]) for p in r['path'] if p[2] == st]
        if pts:
            xs, ys = zip(*pts)
            ax.plot(xs, ys, '.', markersize=2, color=c, label=st)
    for t, text, x, y in r['events']:
        ax.plot(x, y, 'kx')
        ax.annotate(f'{t:.0f}s {text.split(" ")[0]}', (x, y), fontsize=7,
                    xytext=(4, 4), textcoords='offset points')
    ax.plot(sx, sy, 'o', color='red', markersize=10, label='start')
    ax.plot(gx, gy, 'o', color='green', markersize=10, label='goal')
    ax.set_xlim(-0.2, 4.6)
    ax.set_ylim(-0.2, 3.1)
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.legend(loc='upper left', fontsize=7)
    ax.set_title(f"{r['state']} after {r['time']:.0f} s, min gap {r['min_gap'] * 100:.1f} cm")
    fig.savefig(filename, dpi=110, bbox_inches='tight')
    print('saved', filename)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--start', type=float, nargs=3, default=[0.40, 1.82, 0.0],
                    metavar=('X', 'Y', 'YAW_DEG'))
    ap.add_argument('--goal', type=float, nargs=2, default=[3.25, 1.20], metavar=('X', 'Y'))
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--all', action='store_true')
    ap.add_argument('-v', action='store_true')
    ap.add_argument('--hand', choices=['right', 'left'], default='right')
    args = ap.parse_args()
    start = (args.start[0], args.start[1], math.radians(args.start[2]))
    goal = tuple(args.goal)

    if args.all:
        for seed in range(10):
            r = simulate(start, goal, seed, hand=args.hand)
            print(f"seed {seed}: {r['state']:11s} {r['time']:6.1f} s  gap "
                  f"{r['min_gap'] * 100:5.1f} cm  | " + ' -> '.join(e[1] for e in r['events']))
    else:
        r = simulate(start, goal, args.seed, verbose=args.v, hand=args.hand)
        for e in r['events']:
            print(f'{e[0]:6.1f} s  {e[1]:45s} at ({e[2]:.2f}, {e[3]:.2f})')
        plot(r, f'bug2_arena_{args.hand}_seed{args.seed}.png')
