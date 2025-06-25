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
from wanderline.agent import choose_next_angle
import cv2
from wanderline.reward import l2_distance
from wanderline.plot_utils import plot_distances
import os
import datetime

def main():
    # load config
    config = {}
    cfg_file = os.path.join(os.getcwd(), 'config.json')
    if os.path.isfile(cfg_file):
        with open(cfg_file, 'r') as f:
            config = json.load(f)
    ratio_def = config.get('ratio', 0.2)
    steps_def = config.get('steps', 100)
    duration_def = config.get('duration', 15.0)
    # parse arguments
    parser = argparse.ArgumentParser(
        description="Wanderline: A system for generating single-stroke drawings that mimic a motif image.",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument('motif_path', nargs='?', default=None,
                        help='Path to the motif image file. This is the target image that the agent will try to replicate.\nIf not provided, a blank canvas will be used.')
    parser.add_argument('--ratio', type=float, default=ratio_def,
                        help='Length of each stroke as a ratio of the smaller dimension (width or height) of the canvas.\nDefault is based on config.json or 0.2.')
    parser.add_argument('--steps', type=int, default=steps_def,
                        help='The total number of strokes to draw on the canvas.\nDefault is based on config.json or 100.')
    parser.add_argument('--duration', type=float, default=duration_def,
                        help='The desired duration of the output video in seconds. The frame rate will be adjusted to match this duration.\nDefault is based on config.json or 15.0.')
    parser.add_argument('--greedy', action='store_true',
                        help='If set, the agent will use a greedy algorithm to choose the next stroke angle that minimizes the immediate distance to the motif.\nIf not set, angles are chosen randomly.')
    parser.add_argument('--opacity', type=float, default=1.0,
                        help='The opacity of each stroke, where 0.0 is fully transparent and 1.0 is fully opaque.\nDefault is 1.0.')
    parser.add_argument('--patience', type=int, default=10,
                        help='For early stopping. The number of consecutive steps with no significant improvement (less than `min_delta`) before stopping the run.\nSet to 0 to disable. Default is 10.')
    parser.add_argument('--min-delta', type=float, default=1e-4,
                        help='For early stopping. The minimum change in distance to be considered an improvement.\nDefault is 1e-4.')
    parser.add_argument('--resume_from', type=str, default=None,
                        help="Path to a previous output directory (e.g., 'outputs/20231027_123456').\nThe run will resume from the state saved in that directory, including the canvas, step count, and configuration.")
    parser.add_argument('--reward-type', type=str, default='l2', choices=['l2', 'l2_white_penalty'],
                        help='Reward/loss function type to use. Options: l2, l2_white_penalty. Default: l2.')
    parser.add_argument('--white-penalty-alpha', type=float, default=None,
                        help='Alpha value for white penalty (only used if reward-type is l2_white_penalty).')
    args = parser.parse_args()

    # --- Resume Logic ---
    initial_distances = []
    resumed_from_path = None
    previous_total_steps = 0
    canvas = None
    current_start = None

    if args.resume_from:
        resumed_from_path = args.resume_from
        print(f"Resuming from: {resumed_from_path}")

        # 1. Load previous summary
        resume_summary_path = os.path.join(resumed_from_path, 'run_summary.json')
        if not os.path.isfile(resume_summary_path):
            print(f"Error: Cannot find 'run_summary.json' in {resumed_from_path}", file=sys.stderr)
            sys.exit(1)
        with open(resume_summary_path, 'r') as f:
            resume_summary = json.load(f)

        # 2. Override args with previous config for consistency
        original_config = resume_summary['config']
        args.motif_path = original_config.get('motif_path')
        args.ratio = float(original_config.get('ratio'))
        args.opacity = float(original_config.get('opacity'))
        args.greedy = str(original_config.get('greedy')).lower() == 'true'

        # 3. Load previous canvas
        resume_canvas_path = os.path.join(resumed_from_path, 'final_canvas.png')
        if not os.path.isfile(resume_canvas_path):
            print(f"Error: Cannot find 'final_canvas.png' in {resumed_from_path}", file=sys.stderr)
            sys.exit(1)
        canvas = cv2.imread(resume_canvas_path)
        if canvas is None:
            print(f"Error: Failed to load canvas from {resume_canvas_path}", file=sys.stderr)
            sys.exit(1)
        # canvas is now loaded in BGR format, which is what OpenCV uses.

        # 4. Load last position
        if 'results' in resume_summary and 'last_position' in resume_summary['results']:
            current_start = tuple(resume_summary['results']['last_position'])
        else:
            print("Error: 'last_position' not found in summary. Cannot resume.", file=sys.stderr)
            sys.exit(1)

        # 5. Load previous distances and step counts
        resume_distances_path = os.path.join(resumed_from_path, 'distances.json')
        if os.path.isfile(resume_distances_path):
            with open(resume_distances_path, 'r') as f:
                initial_distances = json.load(f)
            previous_total_steps = len(initial_distances) - 1 if initial_distances else 0
        else:
            print("Warning: 'distances.json' not found. Loss curve will be based on this run only.")
            if 'results' in resume_summary:
                if 'final_distance' in resume_summary['results']:
                    initial_distances = [resume_summary['results']['final_distance']]
                if 'total_steps_from_start' in resume_summary['results']:
                    previous_total_steps = resume_summary['results']['total_steps_from_start']
                elif 'total_steps_executed' in resume_summary['results']:
                    previous_total_steps = resume_summary['results']['total_steps_executed']

        # Load the motif image for the resumed run
        if args.motif_path:
            try:
                motif_rgb = load_image(args.motif_path) # Returns RGB
                motif = cv2.cvtColor(motif_rgb, cv2.COLOR_RGB2BGR) # Convert to BGR for CV2 ops
            except Exception as e:
                print(f"Error loading motif image for resumed run: {e}", file=sys.stderr)
                sys.exit(1)

    # --- New Run Setup ---
    else:
        # Not resuming, set up a new run from scratch
        # 1. Load motif (as RGB first)
        motif_rgb = None
        if args.motif_path:
            if not os.path.isfile(args.motif_path):
                print(f"Error: motif file '{args.motif_path}' does not exist", file=sys.stderr)
                sys.exit(1)
            try:
                # load_image returns RGB, used for initial analysis (e.g., finding darkest point)
                motif_rgb = load_image(args.motif_path)
            except Exception as e:
                print(f"Error loading motif image: {e}", file=sys.stderr)
                sys.exit(1)

        # 2. Initialize canvas (as RGB) and determine start point
        if motif_rgb is not None:
            # Canvas is white, same size as motif. Stays in RGB for now.
            canvas_rgb = np.ones_like(motif_rgb, dtype=np.uint8) * 255
            # Find darkest point from a grayscale version of the RGB motif
            gray_motif = cv2.cvtColor(motif_rgb, cv2.COLOR_RGB2GRAY)
            min_loc = np.unravel_index(np.argmin(gray_motif), gray_motif.shape)
            # np.unravel_index returns (row, col) which is (y, x).
            current_start = (int(min_loc[1]), int(min_loc[0])) # (x, y)
            print(f"Starting from the darkest point in motif: {current_start}")
        else:
            # Default canvas if no motif
            canvas_rgb = np.ones((256, 256, 3), dtype=np.uint8) * 255
            h, w = canvas_rgb.shape[:2]
            current_start = (w // 2, h // 2)

        # 3. Store a copy of the initial RGB canvas. This is the "master" initial image.
        initial_canvas = canvas_rgb.copy() # This is a pristine white RGB canvas

        # 4. Convert images to BGR for all OpenCV operations that follow
        # The main canvas for drawing will be in BGR
        canvas = cv2.cvtColor(canvas_rgb, cv2.COLOR_RGB2BGR)
        # The motif for comparison will also be in BGR
        motif = cv2.cvtColor(motif_rgb, cv2.COLOR_RGB2BGR) if motif_rgb is not None else None

    # validate arguments
    if not (0 < args.ratio <= 1):
        print("Error: --ratio must be > 0 and <= 1", file=sys.stderr)
        sys.exit(1)
    if args.steps < 0:
        print("Error: --steps must be non-negative", file=sys.stderr)
        sys.exit(1)
    # prepare timestamped output directory
    base_out = "outputs"
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join(base_out, timestamp)
    os.makedirs(out_dir, exist_ok=True)

    # If we have a motif, save the BGR version used in the run
    if motif is not None:
        motif_path_out = os.path.join(out_dir, "motif.png")
        cv2.imwrite(motif_path_out, motif)
        print(f"Motif image saved as {motif_path_out}")

    # If resuming, we don't have initial_canvas set. We can create it from the loaded canvas.
    if args.resume_from:
        # Create a blank version of the canvas for the "initial_canvas.png" output
        initial_canvas = np.ones_like(canvas, dtype=np.uint8) * 255

    # compute stroke length based on ratio
    h, w = canvas.shape[:2]
    stroke_length = int(min(w, h) * args.ratio)

    # initialize video recorder for strokes
    frame_size = (canvas.shape[1], canvas.shape[0])
    video_path = os.path.join(out_dir, "drawing.mp4")
    # compute fps to fit desired duration, with frame dropping for high step counts
    max_fps = 60
    total_source_frames = args.steps + 1
    desired_fps = total_source_frames / args.duration
    frames_to_record = set(range(args.steps))
    if desired_fps > max_fps:
        fps = max_fps
        num_target_frames = max(2, int(args.duration * fps))
        indices = np.linspace(0, args.steps - 1, num=num_target_frames - 1, dtype=int)
        frames_to_record = set(indices)
        print(f"High step count. Capping FPS at {max_fps} and selecting {len(frames_to_record)} frames to record.")
    else:
        fps = max(1, int(round(desired_fps)))
        print(f"Computed fps: {desired_fps:.2f}, using integer fps: {fps}")
    recorder = VideoRecorder(video_path, frame_size, fps=fps)
    # The canvas is now in BGR format, which is what VideoWriter expects.
    # For a new run, it's a blank white BGR canvas. For a resumed run, it's the loaded final_canvas.png.
    recorder.record(canvas)

    # track loss (distance) over frames
    distances = initial_distances.copy()
    best_distance = min(distances) if distances else math.inf
    patience_counter = 0
    converged = False
    if args.patience <= 0:
        patience_counter = -1  # a sentinel to disable early stopping
    if motif is not None:
        if not args.resume_from:
            init_dist = l2_distance(initial_canvas, motif)
            print(f"Initial distance: {init_dist:.2f}")
            distances.append(init_dist)
            best_distance = init_dist
        elif distances:
            print(f"Resumed with distance: {distances[-1]:.2f}")

    # apply strokes and compute rewards
    total_reward = 0.0
    prev_canvas = canvas
    for i in range(args.steps):
        try:
            if args.greedy and motif is not None:
                angle = choose_next_angle(prev_canvas, motif, current_start, stroke_length)
            else:
                angle = random.uniform(0, 2 * math.pi)
            next_canvas, end_pt = apply_stroke(prev_canvas, angle, start=current_start, length=stroke_length, opacity=args.opacity, return_end=True)
        except Exception as e:
            print(f"Error during stroke application at step {i+1}: {e}", file=sys.stderr)
            sys.exit(1)
        if i in frames_to_record:
            recorder.record(next_canvas)
        if motif is not None:
            r = compute_reward(prev_canvas, next_canvas, motif)
            total_reward += r
            # log reward and new distance
            dist_next = l2_distance(next_canvas, motif)
            print(f"Step {i + 1}/{args.steps}: angle={angle:.2f}, reward={r:.2f}, distance={dist_next:.2f}")
            distances.append(dist_next)
            # Early stopping check
            if patience_counter != -1:
                if best_distance - dist_next > args.min_delta:
                    best_distance = dist_next
                    patience_counter = 0
                else:
                    patience_counter += 1
                if patience_counter >= args.patience:
                    print(f"Stopping early at step {i+1} due to no improvement for {args.patience} steps.")
                    converged = True
                    break
        prev_canvas = next_canvas
        current_start = end_pt

    canvas = prev_canvas

    # save initial blank canvas image
    in_path = os.path.join(out_dir, "initial_canvas.png")
    # initial_canvas was stored as RGB for new runs, but created as BGR for resumed runs.
    # To be safe, we assume it could be RGB and convert to BGR for cv2.imwrite.
    # If it's already BGR, this specific conversion (RGB->BGR) might not be ideal but often works visually.
    # A cleaner way would be to ensure it's always RGB until this point.
    # For now, we handle the new-run case correctly.
    if not args.resume_from:
        cv2.imwrite(in_path, cv2.cvtColor(initial_canvas, cv2.COLOR_RGB2BGR))
    else:
        cv2.imwrite(in_path, initial_canvas) # Already BGR
    print(f"Initial canvas image saved as {in_path}")

    # save final canvas image
    final_path = os.path.join(out_dir, "final_canvas.png")
    cv2.imwrite(final_path, canvas)
    print(f"Final canvas image saved as {final_path}")
    # finalize and save video
    recorder.release()
    print(f"Drawing video saved as {video_path}")

    # Create and save summary
    config_to_save = {k: str(v) if v is not None else None for k, v in vars(args).items()}
    summary = {'config': config_to_save}
    if resumed_from_path:
        summary['resumed_from'] = resumed_from_path
    executed_steps = i + 1
    total_steps_so_far = previous_total_steps + executed_steps

    if motif is not None:
        stats = np.array(distances)
        summary['results'] = {
            'converged': converged,
            'steps_in_this_run': executed_steps,
            'total_steps_from_start': total_steps_so_far,
            'last_position': current_start,
            'total_reward_this_run': total_reward,
            'initial_distance': distances[0] if distances else None,
            'final_distance': distances[-1],
            'distance_stats_full_run': {
                'min': float(stats.min()),
                'max': float(stats.max()),
                'mean': float(stats.mean()),
                'var': float(stats.var())
            }
        }
        # Print info to console
        print(f"Total reward for this run after {executed_steps} steps: {total_reward:.2f}")
        print(f"Distance stats (full) - min: {stats.min():.2f}, max: {stats.max():.2f}, mean: {stats.mean():.2f}, var: {stats.var():.2f}")
        # Plot
        plot_distances(distances, out_dir)
        # Save distances list
        distances_path = os.path.join(out_dir, "distances.json")
        with open(distances_path, 'w') as f:
            json.dump(distances, f)

    summary_path = os.path.join(out_dir, "run_summary.json")
    with open(summary_path, 'w') as f:
        json.dump(summary, f, indent=4)
    print(f"Run summary saved to {summary_path}")


if __name__ == "__main__":
    main()
