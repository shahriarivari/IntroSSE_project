"""Mini-simulator for go_to_goal: no ROS, no robot.

Run from src/bug2_create3/:
    python3 sim_go_to_goal.py
"""
import math

from bug2_create3 import geometry
from bug2_create3.go_to_goal import compute_command, GOAL_TOLERANCE

# ---- settings: change these to experiment ----
START_X, START_Y, START_YAW = 0.0, 0.0, 0.0   # [m], [m], [rad]
GOAL_X, GOAL_Y = 20.0, 18.0                     # [m]
DT = 0.1                                      # [s] time per step (10 Hz, like the timer)
MAX_STEPS = 10000                              # stop after 100 s, in case it never arrives


def simulate(x, y, yaw, goal_x, goal_y):
    """Drive a perfect robot with compute_command. Returns the path as a list of (x, y, yaw)."""
    path = [(x, y, yaw)]

    for step in range(MAX_STEPS):
        # 1. arrived?
        if geometry.distance_to_goal(x, y, goal_x, goal_y) < GOAL_TOLERANCE:
            print(f'Arrived after {step} steps ({step * DT:.1f} s)')
            break

        # 2. ask the controller what to do
        v, w = compute_command(x, y, yaw, goal_x, goal_y)

        # 3. move the robot for DT seconds (perfect motion: no slip, no noise)
        x += v * math.cos(yaw) * DT
        y += v * math.sin(yaw) * DT
        yaw = geometry.wrap_angle(yaw + w * DT)

        path.append((x, y, yaw))
    else:
        print(f'NOT arrived after {MAX_STEPS} steps!')

    print(f'End: x={x:.3f}  y={y:.3f}  yaw={math.degrees(yaw):.1f} deg')
    return path


def plot(path, goal_x, goal_y):
    """Plot the path. Needs matplotlib (sudo apt install python3-matplotlib)."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib not installed, skipping plot')
        return

    xs = [p[0] for p in path]
    ys = [p[1] for p in path]
    plt.plot(xs, ys, '-', label='path')
    plt.plot(xs[0], ys[0], 'go', label='start')
    plt.plot(goal_x, goal_y, 'rx', markersize=12, label='goal')
    plt.axis('equal')          # same scale on x and y, otherwise angles look wrong
    plt.grid(True)
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.legend()
    plt.show()


if __name__ == '__main__':
    path = simulate(START_X, START_Y, START_YAW, GOAL_X, GOAL_Y)
    plot(path, GOAL_X, GOAL_Y)