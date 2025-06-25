import argparse
import json
import os
import sys


def load_config_and_parse_args():
    """
    Load configuration from JSON file and parse command line arguments.
    Returns parsed arguments with config defaults applied.
    """
    # Load config
    config = {}
    parser = argparse.ArgumentParser(
        description="Wanderline: A system for generating single-stroke drawings that mimic a motif image.",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument('--config', type=str, default=None,
                        help='Path to config JSON file. If not set, uses ./config.json if present.')
    
    # Parse only --config first
    args_pre, _ = parser.parse_known_args()
    cfg_file = args_pre.config if args_pre.config else os.path.join(os.getcwd(), 'config.json')
    if os.path.isfile(cfg_file):
        with open(cfg_file, 'r') as f:
            config = json.load(f)
    
    # Get defaults from config
    ratio_def = config.get('ratio', 0.2)
    steps_def = config.get('steps', 100)
    duration_def = config.get('duration', 15.0)
    
    # Parse all arguments with config defaults
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
    parser.add_argument('--line-width', type=int, default=int(config.get('line_width', 3)),
                        help='Stroke thickness (line width) for drawing. Default is from config.json or 3.')
    
    args = parser.parse_args()
    
    return args


def validate_args(args):
    """
    Validate command line arguments and exit with error if invalid.
    """
    if not (0 < args.ratio <= 1):
        print("Error: --ratio must be > 0 and <= 1", file=sys.stderr)
        sys.exit(1)
    if args.steps < 0:
        print("Error: --steps must be non-negative", file=sys.stderr)
        sys.exit(1)
    if args.reward_type == 'l2_white_penalty' and args.white_penalty_alpha is None:
        print("Error: --white-penalty-alpha must be set when using l2_white_penalty.", file=sys.stderr)
        sys.exit(1)
