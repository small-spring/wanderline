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

def test_l2_distance_with_white_penalty():
    """Test l2_distance_with_white_penalty with white_penalty parameter"""
    a = np.ones((2,2,3), dtype=np.uint8) * 255
    b = np.zeros((2,2,3), dtype=np.uint8)
    val = l2_distance_with_white_penalty(a, b, white_penalty=0.1)
    assert val > l2_distance(a, b)
    with pytest.raises(ValueError):
        l2_distance_with_white_penalty(a, b)

def test_compute_reward_with_white_penalty():
    """Test compute_reward_with_white_penalty with white_penalty parameter"""
    a = np.zeros((2,2,3), dtype=np.uint8)
    b = np.ones((2,2,3), dtype=np.uint8) * 255
    motif = np.zeros((2,2,3), dtype=np.uint8)
    r = compute_reward_with_white_penalty(a, b, motif, white_penalty=0.1)
    assert isinstance(r, float)
    with pytest.raises(ValueError):
        compute_reward_with_white_penalty(a, b, motif)

def test_l2_distance_with_white_penalty_detailed():
    """Test l2_distance_with_white_penalty with detailed calculations"""
    # White canvas (all white pixels)
    white_canvas = np.ones((10, 10, 3), dtype=np.uint8) * 255
    # Black motif 
    black_motif = np.zeros((10, 10, 3), dtype=np.uint8)
    
    # Calculate base L2 distance
    base_l2 = l2_distance(white_canvas, black_motif)
    
    # Test with white_penalty = 0.1 (10% penalty)
    distance_with_penalty = l2_distance_with_white_penalty(white_canvas, black_motif, white_penalty=0.1)
    expected = base_l2 * (1.0 + 0.1 * 1.0)  # white_ratio = 1.0 for all white
    assert abs(distance_with_penalty - expected) < 1e-6
    
    # Test with white_penalty = 0.5 (50% penalty)  
    distance_with_penalty = l2_distance_with_white_penalty(white_canvas, black_motif, white_penalty=0.5)
    expected = base_l2 * (1.0 + 0.5 * 1.0)
    assert abs(distance_with_penalty - expected) < 1e-6


def test_compute_reward_with_white_penalty_detailed():
    """Test compute_reward_with_white_penalty with detailed calculations"""
    # Initial state: white canvas
    white_canvas = np.ones((10, 10, 3), dtype=np.uint8) * 255
    # After drawing: some pixels are black
    mixed_canvas = white_canvas.copy()
    mixed_canvas[5:7, 5:7] = 0  # 4 black pixels out of 100 total (10x10 spatial dimensions)
    # Motif: black
    black_motif = np.zeros((10, 10, 3), dtype=np.uint8)
    
    # Test white_penalty mode
    reward = compute_reward_with_white_penalty(white_canvas, mixed_canvas, black_motif, white_penalty=0.1)
    
    # Calculate expected values
    l2_before = l2_distance(white_canvas, black_motif) * (1.0 + 0.1 * 1.0)  # white_ratio = 1.0
    white_ratio_after = (100 - 4) / 100  # 96 white pixels out of 100 (spatial pixels)
    l2_after = l2_distance(mixed_canvas, black_motif) * (1.0 + 0.1 * white_ratio_after)
    expected_reward = l2_before - l2_after
    
    assert abs(reward - expected_reward) < 1e-6
    assert reward > 0  # Should be positive reward for improvement


def test_l2_distance_with_white_penalty_error_handling():
    """Test error handling for l2_distance_with_white_penalty"""
    canvas = np.ones((5, 5, 3), dtype=np.uint8) * 255
    motif = np.zeros((5, 5, 3), dtype=np.uint8)
    
    # Test: no parameters provided
    try:
        l2_distance_with_white_penalty(canvas, motif)
        assert False, "Should raise ValueError"
    except ValueError as e:
        assert "white_penalty must be specified" in str(e)
