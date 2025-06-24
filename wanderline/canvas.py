import numpy as np
import cv2
import math

def apply_stroke(state: np.ndarray, angle: float) -> np.ndarray:
    """
    Draw a stroke from the canvas center in the given angle.
    """
    h, w = state.shape[:2]
    cx, cy = w // 2, h // 2
    length = int(min(w, h) * 0.4)
    dx = int(math.cos(angle) * length)
    dy = int(math.sin(angle) * length)
    end_pt = (cx + dx, cy + dy)
    img = state.copy()
    cv2.line(img, (cx, cy), end_pt, (0, 0, 0), thickness=3)
    return img
