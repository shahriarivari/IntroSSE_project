import json
import math
import os

import pytest

from group4_navigation import goal_source


def test_no_text_means_no_goal():
    assert goal_source.parse_goal('') is None
    assert goal_source.parse_goal('  ') is None


def test_x_y():
    assert goal_source.parse_goal('3.0,0.5') == {'frame': 'map', 'x': 3.0, 'y': 0.5, 'yaw': 0.0}


def test_x_y_yaw_in_degrees_with_spaces():
    goal = goal_source.parse_goal(' 3, -1 , 90 ')
    assert (goal['x'], goal['y']) == (3.0, -1.0)
    assert goal['yaw'] == pytest.approx(math.pi / 2)


def test_bad_goal_text():
    with pytest.raises(ValueError):
        goal_source.parse_goal('3.0')
    with pytest.raises(ValueError):
        goal_source.parse_goal('a,b')


def test_missing_file(tmp_path):
    assert goal_source.read_goal_file(str(tmp_path / 'none.json')) == (None, None)


def test_file_written_by_bug2(tmp_path):
    path = tmp_path / 'last_goal.json'
    path.write_text(json.dumps({'frame': 'map', 'x': 2.95, 'y': 0.12, 'yaw': 0.05,
                                'written': '2026-10-13 10:00:00'}))
    goal, age = goal_source.read_goal_file(str(path), now=os.path.getmtime(path) + 60)
    assert goal == {'frame': 'map', 'x': 2.95, 'y': 0.12, 'yaw': 0.05}
    assert age == pytest.approx(60)


def test_file_says_whether_bug2_reached_the_goal(tmp_path):
    path = tmp_path / 'last_goal.json'
    path.write_text(json.dumps({'frame': 'map', 'x': -0.02, 'y': -3.54, 'yaw': -1.5,
                                'source': 'reached'}))
    goal, _ = goal_source.read_goal_file(str(path))
    assert goal['source'] == 'reached'
    assert 'reached by Bug2' in goal_source.describe(goal)
