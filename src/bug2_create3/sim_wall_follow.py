"""Mini-simulator for go-to-goal + wall-follow, with a simulated lidar. No ROS needed.

Run from src/bug2_create3/:
    python3 sim_wall_follow.py          # prints a summary
    python3 sim_wall_follow.py -v       # prints a log line every second
    python3 sim_wall_follow.py -p       # also plots the path (needs matplotlib)

Change the constants in wall_follow.py and run again to see the effect.
"""
import math
import random
import sys

from bug2_create3 import geometry, go_to_goal, lidar, wall_follow

# ---- lidar like the real one (values from: ros2 topic echo --once --no-arr /robot_4/scan) ----
ANGLE_MIN = -3.124139
INC = 0.00871450919657
N = 720
RANGE_MIN, RANGE_MAX = 0.15, 12.0
NOISE = 0.01            # [m] standard deviation of the range noise
DROPOUT = 0.03          # chance that a single measurement is missing (inf)
LATENCY = 2             # scans are this many ticks old (the real lidar is a bit slow)

# ---- same sectors as the node ----
FRONT_HALF_WIDTH = math.radians(40)
WF_FRONT_HALF_WIDTH = math.radians(20)
SIDE_CENTER = math.radians(-75)          # right-hand side, from -120 ...
SIDE_HALF_WIDTH = math.radians(45)      # ... to -30 deg (not far behind!)

DT = 0.1                # [s] 10 Hz, like the timer
ROBOT_RADIUS = 0.17     # [m]
STEPS = 1200            # 120 s

GOAL = (2.0, 0.0)
# The obstacle: one wall, 10 cm thick and 40 cm long, 0.8 m in front of the start.
WALL = (0.80, -0.20, 0.90, 0.20)          # x_min, y_min, x_max, y_max


def rectangle(x0, y0, x1, y1):
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    return [(corners[i], corners[(i + 1) % 4]) for i in range(4)]


def ray_length(px, py, angle, segments):
    """Distance from (px, py) along 'angle' to the first segment hit (inf if none)."""
    dx, dy = math.cos(angle), math.sin(angle)
    best = float('inf')
    for (ax, ay), (bx, by) in segments:
        ex, ey = bx - ax, by - ay
        den = dx * ey - dy * ex
        if abs(den) < 1e-12:
            continue
        t = ((ax - px) * ey - (ay - py) * ex) / den      # along the ray
        u = ((ax - px) * dy - (ay - py) * dx) / den      # along the segment
        if t > 0 and 0 <= u <= 1:
            best = min(best, t)
    return best


def fake_scan(x, y, yaw, segments):
    ranges = []
    for i in range(N):
        r = ray_length(x, y, yaw + ANGLE_MIN + i * INC, segments)
        if math.isfinite(r):
            r = float('inf') if random.random() < DROPOUT else r + random.gauss(0, NOISE)
        ranges.append(r if r <= RANGE_MAX else float('inf'))
    return ranges


def distance_to_segment(px, py, segment):
    (ax, ay), (bx, by) = segment
    ex, ey = bx - ax, by - ay
    t = max(0.0, min(1.0, ((px - ax) * ex + (py - ay) * ey) / (ex * ex + ey * ey)))
    return math.hypot(px - (ax + t * ex), py - (ay + t * ey))


def simulate(segments, verbose=False):
    x = y = yaw = 0.0
    mode = 'go-to-goal'
    rule = 'follow'
    scans = []
    min_gap = float('inf')
    path = []
    for step in range(STEPS):
        scans.append(fake_scan(x, y, yaw, segments))
        ranges = scans[max(0, len(scans) - 1 - LATENCY)]

        half_width = WF_FRONT_HALF_WIDTH if mode == 'wall-follow' else FRONT_HALF_WIDTH
        front = lidar.sector_min(ranges, 0.0, half_width, ANGLE_MIN, INC, RANGE_MIN, RANGE_MAX)
        closest, closest_angle = lidar.sector_closest(ranges, SIDE_CENTER, SIDE_HALF_WIDTH,
                                                      ANGLE_MIN, INC, RANGE_MIN, RANGE_MAX)

        if mode == 'go-to-goal':
            if front < wall_follow.WALL_IN_FRONT:
                mode, v, w, rule = 'wall-follow', 0.0, 0.0, 'hit'
            else:
                v, w = go_to_goal.compute_command(x, y, yaw, *GOAL)
                rule = 'drive'
        else:
            v, w, rule = wall_follow.compute_wall_follow(front, closest, closest_angle, rule)

        x += v * math.cos(yaw) * DT
        y += v * math.sin(yaw) * DT
        yaw = geometry.wrap_angle(yaw + w * DT)

        gap = min(distance_to_segment(x, y, s) for s in segments) - ROBOT_RADIUS
        min_gap = min(min_gap, gap)
        path.append((x, y))

        if verbose and step % 10 == 0:
            angle = math.degrees(closest_angle) if closest_angle is not None else float('nan')
            print(f'{step:4d} {rule:6s} x={x:+.2f} y={y:+.2f} yaw={math.degrees(yaw):+4.0f} '
                  f'front={front:.2f} closest={closest:.2f}@{angle:+4.0f} '
                  f'v={v:.2f} w={w:+.2f} gap={gap:+.2f}')
        if gap < 0:
            print(f'COLLISION at step {step} ({x:.2f}, {y:.2f})')
            break
    return min_gap, path


def plot(path, segments):
    import matplotlib.pyplot as plt
    for (ax, ay), (bx, by) in segments:
        plt.plot([ax, bx], [ay, by], 'k-', linewidth=3)
    plt.plot([p[0] for p in path], [p[1] for p in path], '-', label='robot (centre)')
    plt.plot(0, 0, 'go', label='start')
    plt.plot(*GOAL, 'rx', markersize=12, label='goal')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.show()


if __name__ == '__main__':
    random.seed(0)
    segments = rectangle(*WALL)
    min_gap, path = simulate(segments, verbose='-v' in sys.argv)
    print(f'Smallest gap between robot edge and wall: {min_gap * 100:.1f} cm')
    if '-p' in sys.argv:
        plot(path, segments)