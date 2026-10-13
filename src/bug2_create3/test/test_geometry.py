import math
import pytest
from bug2_create3 import geometry

# def test_distance_3_4_5():
    # assert geometry.distance_to_goal(0, 0, 3, 4) == pytest.approx(5,0)

def test_angle_3_4_5():
    assert geometry.angle_to_goal(0, 0, 3, 3) == pytest.approx(math.radians(45))

def wrap_angle_360():
    assert geometry.wrap_angle(360) == pytest.approx(0)    