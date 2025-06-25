import numpy as np
import cv2
import math

def apply_stroke(state: np.ndarray, angle: float, start: tuple[int, int] = None, length: int = None, opacity: float = 1.0, return_end: bool = False, line_width: int = 3) -> np.ndarray:
    """
    Draw a stroke from a start point (default center) in the given angle.
    Supports semi-transparent strokes with opacity in [0,1].
    line_width: thickness of the stroke (default 3)
    Returns the new image, and optionally the end point if return_end is True.
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
    # draw with opacity: fully opaque or blended
    if opacity >= 1.0:
        cv2.line(img, (cx, cy), end_pt, (0, 0, 0), thickness=line_width)
    else:
        overlay = img.copy()
        cv2.line(overlay, (cx, cy), end_pt, (0, 0, 0), thickness=line_width)
        img = cv2.addWeighted(overlay, opacity, img, 1.0 - opacity, 0)
    if return_end:
        return img, end_pt
    return img
