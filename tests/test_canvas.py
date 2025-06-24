import numpy as np
from wanderline.canvas import apply_stroke

def test_apply_stroke_horizontal():
    # Prepare white canvas
    size = 100
    canvas = np.ones((size, size, 3), dtype=np.uint8) * 255
    # Apply stroke at 0 radians (to the right)
    result = apply_stroke(canvas, angle=0.0)
    cx, cy = size // 2, size // 2
    length = int(min(size, size) * 0.4)
    # Check center pixel is black
    assert np.all(result[cy, cx] == [0, 0, 0])
    # Check endpoint pixel is black
    assert np.all(result[cy, cx + length] == [0, 0, 0])


def test_apply_stroke_vertical():
    # Prepare white canvas
    size = 100
    canvas = np.ones((size, size, 3), dtype=np.uint8) * 255
    # Apply stroke at pi/2 radians (downwards)
    result = apply_stroke(canvas, angle=np.pi / 2)
    cx, cy = size // 2, size // 2
    length = int(min(size, size) * 0.4)
    # Check center pixel is black
    assert np.all(result[cy, cx] == [0, 0, 0])
    # Check endpoint pixel is black
    assert np.all(result[cy + length, cx] == [0, 0, 0])
