from wanderline.image_utils import load_image
from wanderline.canvas import apply_stroke
import numpy as np
import sys
import cv2
import os

def main():
    # prepare output directory
    out_dir = "outputs"
    os.makedirs(out_dir, exist_ok=True)

    # load motif image if provided and save it for inspection
    motif = load_image(sys.argv[1]) if len(sys.argv) > 1 else None
    if motif is not None:
        motif_path = os.path.join(out_dir, "motif.png")
        cv2.imwrite(motif_path, motif)
        print(f"Motif image saved as {motif_path}")

    # initialize canvas to white
    canvas = np.ones((256, 256, 3), dtype=np.uint8) * 255

    # テスト用ストロークを適用 on white canvas
    result = apply_stroke(canvas, angle=0.0)
    print(f"Canvas shape: {canvas.shape}, Output shape: {result.shape}")

    # save original canvas image
    in_path = os.path.join(out_dir, "canvas.png")
    cv2.imwrite(in_path, canvas)
    print(f"Input image saved as {in_path}")

    # save output image for visual inspection
    out_path = os.path.join(out_dir, "output.png")
    cv2.imwrite(out_path, result)
    print(f"Output image saved as {out_path}")

if __name__ == "__main__":
    main()
