from wanderline.image_utils import load_image
from wanderline.canvas import apply_stroke
import numpy as np
import sys

def main():
    # 引数で画像パスを指定しなければ、真っ白キャンバスを使用
    if len(sys.argv) > 1:
        img = load_image(sys.argv[1])
    else:
        img = np.ones((256, 256, 3), dtype=np.uint8) * 255

    # テスト用ストロークを適用
    result = apply_stroke(img, angle=0.0)
    print(f"Input shape: {img.shape}, Output shape: {result.shape}")

if __name__ == "__main__":
    main()
