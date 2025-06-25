import numpy as np
import pytest
from wanderline.reward import l2_distance, compute_reward, l2_distance_with_white_penalty, compute_reward_with_white_penalty, white_ratio

def test_l2_distance_zero():
    arr1 = np.zeros((3, 3))
    arr2 = np.zeros((3, 3))
    assert l2_distance(arr1, arr2) == 0.0

@pytest.mark.parametrize("a,b,expected", [
    (np.array([3.0, 4.0]), np.array([0.0, 0.0]), 5.0),
    (np.array([[1.0, 2.0], [2.0, 1.0]]), np.zeros((2, 2)), pytest.approx(np.linalg.norm([1,2,2,1]))),
])
def test_l2_distance_known(a, b, expected):
    assert l2_distance(a, b) == expected

def test_compute_reward_improvement():
    # motif is ones, prev_state zeros, next_state ones: reward = d(prev, motif) - d(next, motif)
    motif = np.ones((2, 2))
    prev = np.zeros((2, 2))
    next_s = np.ones((2, 2))
    # prev distance = sqrt(4) = 2, next distance = 0
    reward = compute_reward(prev, next_s, motif)
    assert reward == pytest.approx(2.0)

def test_compute_reward_worsen():
    # motif zeros, prev_state ones, next_state zeros: reward negative
    motif = np.zeros((2, 2))
    prev = np.ones((2, 2))
    next_s = np.zeros((2, 2))
    # prev distance = 2, next distance = 0, reward = 2 - 0 = 2
    reward = compute_reward(prev, next_s, motif)
    assert reward == pytest.approx(2.0)

def test_white_ratio():
    white = np.ones((2,2,3), dtype=np.uint8) * 255
    black = np.zeros((2,2,3), dtype=np.uint8)
    assert white_ratio(white) == 1.0
    assert white_ratio(black) == 0.0

def test_l2_distance_with_white_penalty_alpha():
    a = np.ones((2,2,3), dtype=np.uint8) * 255
    b = np.zeros((2,2,3), dtype=np.uint8)
    val = l2_distance_with_white_penalty(a, b, alpha=1.0)
    assert val > l2_distance(a, b)
    with pytest.raises(ValueError):
        l2_distance_with_white_penalty(a, b)

def test_compute_reward_with_white_penalty_alpha():
    a = np.zeros((2,2,3), dtype=np.uint8)
    b = np.ones((2,2,3), dtype=np.uint8) * 255
    motif = np.zeros((2,2,3), dtype=np.uint8)
    r = compute_reward_with_white_penalty(a, b, motif, alpha=1.0)
    assert isinstance(r, float)
    with pytest.raises(ValueError):
        compute_reward_with_white_penalty(a, b, motif)
