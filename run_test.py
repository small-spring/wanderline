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
    parser = argparse.ArgumentParser(description="Apply strokes and compute rewards")
    parser.add_argument('motif_path', nargs='?', default=None, help='Path to motif image')
    parser.add_argument('--ratio', type=float, default=ratio_def, help='Stroke length ratio of min(width,height)')
    parser.add_argument('--steps', type=int, default=steps_def, help='Number of strokes to apply (default 100)')
    parser.add_argument('--duration', type=float, default=duration_def, help='Desired total video duration in seconds')
    parser.add_argument('--greedy', action='store_true', help='Use greedy agent to choose stroke angles')
    parser.add_argument('--opacity', type=float, default=1.0, help='Stroke opacity (0.0 to 1.0)')
    parser.add_argument('--patience', type=int, default=10, help='Early stopping patience in steps. Set to 0 to disable.')
    parser.add_argument('--min-delta', type=float, default=1e-4, help='Minimum improvement for early stopping.')
    parser.add_argument('--resume_from', type=str, default=None, help='Path to a previous run directory to resume from.')
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

    # load motif image with error handling
    motif = None
    if args.motif_path:
        if not os.path.isfile(args.motif_path):
            print(f"Error: motif file '{args.motif_path}' does not exist", file=sys.stderr)
            sys.exit(1)
        try:
            motif = load_image(args.motif_path)
            # OpenCV uses BGR channel order, but load_image provides RGB.
            # Convert to BGR for all subsequent cv2 operations.
            motif = cv2.cvtColor(motif, cv2.COLOR_RGB2BGR)
        except Exception as e:
            print(f"Error loading motif image: {e}", file=sys.stderr)
            sys.exit(1)
        motif_path_out = os.path.join(out_dir, "motif.png")
        cv2.imwrite(motif_path_out, motif)
        print(f"Motif image saved as {motif_path_out}")
    # initialize canvas to white (same size as motif if provided)
    if not args.resume_from:
        if motif is not None:
            canvas = np.ones_like(motif, dtype=np.uint8) * 255
        else:
            canvas = np.ones((256, 256, 3), dtype=np.uint8) * 255
        # starting point for strokes: center (一筆書き開始位置)
        h, w = canvas.shape[:2]
        current_start = (w // 2, h // 2)

    # store initial blank canvas
    initial_canvas = canvas.copy()
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
    recorder.record(initial_canvas)
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
    cv2.imwrite(in_path, initial_canvas)
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
