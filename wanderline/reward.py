import numpy as np

def l2_distance(a: np.ndarray, b: np.ndarray) -> float:
    """
    Compute L2 (Euclidean) distance between two images (arrays).
    Both arrays must have the same shape.
    """
    a_f = a.astype(float)
    b_f = b.astype(float)
    return float(np.linalg.norm(a_f - b_f))


def compute_reward(prev_state: np.ndarray, next_state: np.ndarray, motif: np.ndarray) -> float:
    """
    Immediate reward: reduction in distance to motif.
        Δd = d(prev_state, motif) - d(next_state, motif)
    """
    return l2_distance(prev_state, motif) - l2_distance(next_state, motif)
