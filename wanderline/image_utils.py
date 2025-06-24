import numpy as np
from PIL import Image

def load_image(path: str) -> np.ndarray:
    """
    Load an image from the given path and return as RGB numpy array.
    """
    img = Image.open(path).convert("RGB")
    return np.asarray(img)
