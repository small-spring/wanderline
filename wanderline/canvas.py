import numpy as np
import cv2
import math

def apply_stroke(state: np.ndarray, angle: float, start: tuple[int, int] = None, length: int = None) -> tuple[np.ndarray, tuple[int, int]]:
    """
    Draw a stroke from a start point (default center) in the given angle.
    Returns the new image and the end point of the stroke.
    """
    h, w = state.shape[:2]
    # determine start point and clamp within bounds
    if start is None:
        cx, cy = w // 2, h // 2
    else:
        sx, sy = start
        cx = min(max(sx, 0), w - 1)
        cy = min(max(sy, 0), h - 1)
    # determine stroke length
    if length is None:
        length = int(min(w, h) * 0.4)
    dx = int(math.cos(angle) * length)
    dy = int(math.sin(angle) * length)
    # compute and clamp end point
    ex = min(max(cx + dx, 0), w - 1)
    ey = min(max(cy + dy, 0), h - 1)
    end_pt = (ex, ey)
    img = state.copy()
    cv2.line(img, (cx, cy), end_pt, (0, 0, 0), thickness=3)
    return img, end_pt
