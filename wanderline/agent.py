import numpy as np
import math
from wanderline.canvas import apply_stroke
from wanderline.reward import compute_reward

def choose_next_angle(
        prev_canvas: np.ndarray,
        motif: np.ndarray,
        start_pt: tuple[int, int],
        length: int,
        n_samples: int = 36
        ) -> float:
    """
    Choose the angle that maximizes immediate reward.
    Samples n_samples angles uniformly from [0, 2*pi).
    """
    best_angle = 0.0
    best_reward = -math.inf
    for angle in np.linspace(0, 2 * math.pi, n_samples, endpoint=False):
        try:
            next_canvas, _ = apply_stroke(prev_canvas, angle, start=start_pt, length=length, return_end=True)
            r = compute_reward(prev_canvas, next_canvas, motif)
        except Exception:
            continue
        if r > best_reward:
            best_reward = r
            best_angle = float(angle)
    return best_angle
