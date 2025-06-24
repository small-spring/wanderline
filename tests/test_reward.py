import numpy as np
import pytest
from wanderline.reward import l2_distance, compute_reward

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
