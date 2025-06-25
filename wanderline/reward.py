import numpy as np

def l2_distance(a: np.ndarray, b: np.ndarray) -> float:
    """
    Compute L2 (Euclidean) distance between two images (arrays).
    Both arrays must have the same shape.
    """
    a_f = a.astype(float)
    b_f = b.astype(float)
    return float(np.linalg.norm(a_f - b_f))

def white_ratio(canvas: np.ndarray) -> float:
    """
    Calculate the ratio of pure white pixels in the canvas.
    """
    if canvas.ndim == 3 and canvas.shape[2] == 3:
        white = np.all(canvas == 255, axis=2)
    else:
        white = (canvas == 255)
    return float(np.sum(white)) / white.size

def l2_distance_with_white_penalty(a: np.ndarray, b: np.ndarray, alpha: float = None) -> float:
    """
    Compute L2 distance plus a penalty proportional to the ratio of white pixels.
    loss = l2_distance(a, b) + alpha * white_ratio(a)
    """
    if alpha is None:
        raise ValueError("alpha must be specified for l2_distance_with_white_penalty")
    l2 = l2_distance(a, b)
    penalty = alpha * white_ratio(a)
    return l2 + penalty

def compute_reward(prev_state: np.ndarray, next_state: np.ndarray, motif: np.ndarray) -> float:
    """
    Immediate reward: reduction in distance to motif.
        Δd = d(prev_state, motif) - d(next_state, motif)
    """
    return l2_distance(prev_state, motif) - l2_distance(next_state, motif)

def compute_reward_with_white_penalty(prev_state: np.ndarray, next_state: np.ndarray, motif: np.ndarray, alpha: float = None) -> float:
    """
    Immediate reward with white penalty: reduction in penalized distance to motif.
    """
    if alpha is None:
        raise ValueError("alpha must be specified for compute_reward_with_white_penalty")
    return l2_distance_with_white_penalty(prev_state, motif, alpha) - l2_distance_with_white_penalty(next_state, motif, alpha)



