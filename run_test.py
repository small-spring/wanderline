from wanderline.image_utils import load_image
from wanderline.canvas import apply_stroke
import sys
import json
import numpy as np
import argparse
import random
import math
from wanderline.reward import compute_reward
from wanderline.video_recorder import VideoRecorder
import cv2
import os

def main():
    # load config
    config = {}
    cfg_file = os.path.join(os.getcwd(), 'config.json')
    if os.path.isfile(cfg_file):
        with open(cfg_file, 'r') as f:
            config = json.load(f)
    ratio_def = config.get('ratio', 0.2)
    steps_def = config.get('steps', 5)
    fps_def = config.get('fps', 1)
    # parse arguments
    parser = argparse.ArgumentParser(description="Apply strokes and compute rewards")
    parser.add_argument('motif_path', nargs='?', default=None, help='Path to motif image')
    parser.add_argument('--ratio', type=float, default=ratio_def, help='Stroke length ratio of min(width,height)')
    parser.add_argument('--steps', type=int, default=steps_def, help='Number of strokes to apply')
    args = parser.parse_args()
    # validate arguments
    if not (0 < args.ratio <= 1):
        print("Error: --ratio must be > 0 and <= 1", file=sys.stderr)
        sys.exit(1)
    if args.steps < 0:
        print("Error: --steps must be non-negative", file=sys.stderr)
        sys.exit(1)
    # prepare output directory
    out_dir = "outputs"
    os.makedirs(out_dir, exist_ok=True)

    # load motif image with error handling
    motif = None
    if args.motif_path:
        if not os.path.isfile(args.motif_path):
            print(f"Error: motif file '{args.motif_path}' does not exist", file=sys.stderr)
            sys.exit(1)
        try:
            motif = load_image(args.motif_path)
        except Exception as e:
            print(f"Error loading motif image: {e}", file=sys.stderr)
            sys.exit(1)
        motif_path_out = os.path.join(out_dir, "motif.png")
        cv2.imwrite(motif_path_out, motif)
        print(f"Motif image saved as {motif_path_out}")
    # initialize canvas to white (same size as motif if provided)
    if motif is not None:
        canvas = np.ones_like(motif, dtype=np.uint8) * 255
    else:
        canvas = np.ones((256, 256, 3), dtype=np.uint8) * 255
    # store initial blank canvas
    initial_canvas = canvas.copy()
    # compute stroke length based on ratio
    h, w = canvas.shape[:2]
    stroke_length = int(min(w, h) * args.ratio)
    # starting point for strokes: center (一筆書き開始位置)
    current_start = (w // 2, h // 2)

    # initialize video recorder for strokes
    frame_size = (canvas.shape[1], canvas.shape[0])
    video_path = os.path.join(out_dir, "drawing.mp4")
    recorder = VideoRecorder(video_path, frame_size, fps= fps_def)
    recorder.record(initial_canvas)

    # apply strokes and compute rewards
    total_reward = 0.0
    prev_canvas = canvas
    for i in range(args.steps):
        try:
            angle = random.uniform(0, 2 * math.pi)
            next_canvas, end_pt = apply_stroke(prev_canvas, angle, start=current_start, length=stroke_length)
        except Exception as e:
            print(f"Error during stroke application at step {i+1}: {e}", file=sys.stderr)
            sys.exit(1)
        recorder.record(next_canvas)
        if motif is not None:
            r = compute_reward(prev_canvas, next_canvas, motif)
            total_reward += r
            print(f"Step {i + 1}/{args.steps}: angle={angle:.2f}, reward={r:.2f}")
        prev_canvas = next_canvas
        current_start = end_pt

    canvas = prev_canvas

    # save initial blank canvas image
    in_path = os.path.join(out_dir, "initial_canvas.png")
    cv2.imwrite(in_path, initial_canvas)
    print(f"Initial canvas image saved as {in_path}")

    # save final canvas image
    final_path = os.path.join(out_dir, "final_canvas.png")
    cv2.imwrite(final_path, canvas)
    print(f"Final canvas image saved as {final_path}")
    # finalize and save video
    recorder.release()
    print(f"Drawing video saved as {video_path}")
    if motif is not None:
        print(f"Total reward after {args.steps} steps: {total_reward:.2f}")

if __name__ == "__main__":
    main()
