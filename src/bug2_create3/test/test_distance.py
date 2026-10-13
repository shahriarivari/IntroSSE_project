import pytest
from bug2_create3.geometry import distance_to_line
import math

expected = math.sqrt(2) / 2

def test_distance_to_line_1():
    assert distance_to_line(0, 0, 2, 0, 1, 0.3) == pytest.approx(0.3)

def test_distance_3():
    assert distance_to_line(0, 0, 2, 0, 1, -0.3) == pytest.approx(0.3)

def test_distance_4():
    assert distance_to_line(0, 0, 2, 0, 1, 0) == pytest.approx(0)

def test_distance_7():
    assert distance_to_line(0, 0, 1, 1, 1, 0) == pytest.approx(expected)

def test_distance_10():
    with pytest.raises(ValueError):
        distance_to_line(1, 1, 1, 1, 3, 0)

# ---- signed distance: same size as distance_to_line, sign = side ----
from bug2_create3.geometry import signed_distance_to_line


def test_signed_left_is_positive():
    assert signed_distance_to_line(0, 0, 2, 0, 1, 0.3) == pytest.approx(0.3)


def test_signed_right_is_negative():
    assert signed_distance_to_line(0, 0, 2, 0, 1, -0.3) == pytest.approx(-0.3)


def test_signed_diagonal():
    assert signed_distance_to_line(0, 0, 1, 1, 1, 0) == pytest.approx(-expected)
