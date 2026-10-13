"""Where the round 2 goal comes from (ROS-free, unit-tested).

A goal is a dict: {'frame': 'map', 'x': .., 'y': .., 'yaw': .. [rad]}.
"""
import json
import math
import os
import time

GOAL_FILE = os.path.expanduser('~/bug2_logs/last_goal.json')   # written by bug2_node


def parse_goal(text, frame='map'):
    """'x,y' or 'x,y,yaw_deg' -> goal dict. Empty text -> None (no goal given)."""
    if not text.strip():
        return None
    parts = [float(part) for part in text.replace(' ', '').split(',')]
    if len(parts) not in (2, 3):
        raise ValueError(f"goal must be 'x,y' or 'x,y,yaw_deg', not '{text}'")
    yaw = math.radians(parts[2]) if len(parts) == 3 else 0.0
    return {'frame': frame, 'x': parts[0], 'y': parts[1], 'yaw': yaw}


def read_goal_file(path=GOAL_FILE, now=None):
    """Goal from Bug2's file -> (goal dict, age in s), or (None, None) if there is no file."""
    if not os.path.exists(path):
        return None, None
    with open(path) as goal_file:
        data = json.load(goal_file)
    goal = {'frame': data.get('frame', 'map'), 'x': float(data['x']),
            'y': float(data['y']), 'yaw': float(data.get('yaw', 0.0))}
    if 'source' in data:      # 'planned' (round 1 not finished) or 'reached' (where it ended)
        goal['source'] = data['source']
    age = (time.time() if now is None else now) - os.path.getmtime(path)
    return goal, age


def describe(goal):
    source = f", {goal['source']} by Bug2" if 'source' in goal else ''
    return (f"x={goal['x']:.2f} y={goal['y']:.2f} yaw={math.degrees(goal['yaw']):.0f} deg "
            f"(frame {goal['frame']}{source})")
