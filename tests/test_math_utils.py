import pytest

from math_utils import mean


def test_mean_positive_numbers():
    assert mean([1, 2, 3, 4]) == 2.5


def test_mean_single_element():
    assert mean([42]) == 42


def test_mean_raises_on_empty_sequence():
    with pytest.raises(ValueError):
        mean([])
