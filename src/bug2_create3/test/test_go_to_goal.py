import math
import pytest
from bug2_create3 import go_to_goal

x = 0
y = 0
yaw = 0

def test_go_to_goal_2_0():
    v, w = go_to_goal.compute_command(x, y, yaw, 2, 0)
    assert v > 0
    assert w == pytest.approx(0)

def test_go_to_goal_0_2():
    v, w = go_to_goal.compute_command(x, y, yaw, 0,2)
    assert v == pytest.approx(0)
    assert w > 0

    