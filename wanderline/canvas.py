import numpy as np
import cv2
import math
from .constants import DEFAULT_STROKE_LENGTH_RATIO, STROKE_COLOR, OPACITY_THRESHOLD

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
        length = int(min(w, h) * DEFAULT_STROKE_LENGTH_RATIO)
    dx = int(math.cos(angle) * length)
    dy = int(math.sin(angle) * length)
    # compute and clamp end point
    ex = min(max(cx + dx, 0), w - 1)
    ey = min(max(cy + dy, 0), h - 1)
    end_pt = (ex, ey)
    img = state.copy()
    # draw with opacity: fully opaque or blended
    if opacity >= OPACITY_THRESHOLD:
        cv2.line(img, (cx, cy), end_pt, STROKE_COLOR, thickness=line_width)
    else:
        overlay = img.copy()
        cv2.line(overlay, (cx, cy), end_pt, STROKE_COLOR, thickness=line_width)
        img = cv2.addWeighted(overlay, opacity, img, 1.0 - opacity, 0)
    if return_end:
        return img, end_pt
    return img

def apply_stroke_vectorized(state: np.ndarray, angles: np.ndarray, start: tuple[int, int] = None, 
                           length: int = None, opacity: float = 1.0, return_end: bool = False, 
                           line_width: int = 3) -> np.ndarray:
    """
    Vectorized stroke application for batch processing.
    
    Args:
        state: Canvas state (H, W, C)
        angles: Array of angles to process (n_samples,)
        start: Starting point for all strokes
        length: Length of strokes
        opacity: Opacity for all strokes
        return_end: Whether to return end points
        line_width: Thickness of strokes
        
    Returns:
        canvases: Array of resulting canvases (n_samples, H, W, C)
        end_points: Array of end points (n_samples, 2) if return_end=True
    """
    h, w = state.shape[:2]
    n_samples = len(angles)
    
    # Determine start point
    if start is None:
        cx, cy = w // 2, h // 2
    else:
        sx, sy = start
        cx = min(max(sx, 0), w - 1)
        cy = min(max(sy, 0), h - 1)
    
    # Determine stroke length
    if length is None:
        length = int(min(w, h) * DEFAULT_STROKE_LENGTH_RATIO)
    
    # Vectorized computation of end points
    dx = np.cos(angles) * length
    dy = np.sin(angles) * length
    
    # Compute and clamp end points
    ex = np.clip(cx + dx.astype(int), 0, w - 1)
    ey = np.clip(cy + dy.astype(int), 0, h - 1)
    end_points = np.column_stack((ex, ey))
    
    # Create batch of canvases
    canvases = np.tile(state[np.newaxis, :, :, :], (n_samples, 1, 1, 1))
    
    # Apply strokes to each canvas
    for i in range(n_samples):
        if opacity >= OPACITY_THRESHOLD:
            cv2.line(canvases[i], (cx, cy), (int(ex[i]), int(ey[i])), STROKE_COLOR, thickness=line_width)
        else:
            overlay = canvases[i].copy()
            cv2.line(overlay, (cx, cy), (int(ex[i]), int(ey[i])), STROKE_COLOR, thickness=line_width)
            canvases[i] = cv2.addWeighted(overlay, opacity, canvases[i], 1.0 - opacity, 0)
    
    if return_end:
        return canvases, end_points
    return canvases
