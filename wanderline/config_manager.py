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
    
    # Create a temporary parser just for --config option
    temp_parser = argparse.ArgumentParser(add_help=False)
    temp_parser.add_argument('--config', type=str, default=None)
    
    # Parse only --config first
    args_pre, _ = temp_parser.parse_known_args()
    cfg_file = args_pre.config if args_pre.config else os.path.join(os.getcwd(), 'configs', 'default.json')
    if os.path.isfile(cfg_file):
        with open(cfg_file, 'r') as f:
            config = json.load(f)
    
    # Get defaults from config
    motif_path_def = config.get('motif_path', None)
    ratio_def = config.get('ratio', 0.2)
    steps_def = config.get('steps', 100)
    duration_def = config.get('duration', 15.0)
    opacity_def = config.get('opacity', 1.0)
    patience_def = config.get('patience', 10)
    min_delta_def = config.get('min_delta', 1e-4)
    reward_type_def = config.get('reward_type', 'l2')
    white_penalty_def = config.get('white_penalty', None)
    greedy_def = config.get('greedy', False)
    agent_type_def = config.get('agent_type', 'greedy')
    lookahead_depth_def = config.get('lookahead_depth', 1)
    n_samples_def = config.get('n_samples', None)  # Auto-select based on lookahead_depth if None
    verbose_def = config.get('verbose', False)
    memory_efficient_def = config.get('memory_efficient', False)
    fast_agent_def = config.get('fast_agent', False)
    ultra_fast_def = config.get('ultra_fast', False)
    beam_width_def = config.get('beam_width', 5)
    n_rollouts_def = config.get('n_rollouts', 10)
    visualization_def = config.get('visualization', {})
    
    # Create the main parser with all arguments
    parser = argparse.ArgumentParser(
        description="Wanderline: A system for generating single-stroke drawings that mimic a motif image.",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument('--config', type=str, default=None,
                        help='Path to config JSON file. If not set, uses ./configs/default.json if present.')
    parser.add_argument('motif_path', nargs='?', default=motif_path_def,
                        help='Path to the motif image file. This is the target image that the agent will try to replicate.\nIf not provided, a blank canvas will be used.')
    parser.add_argument('--ratio', type=float, default=ratio_def,
                        help='Length of each stroke as a ratio of the smaller dimension (width or height) of the canvas.\nDefault is based on config.json or 0.2.')
    parser.add_argument('--steps', type=int, default=steps_def,
                        help='The total number of strokes to draw on the canvas.\nDefault is based on config.json or 100.')
    parser.add_argument('--duration', type=float, default=duration_def,
                        help='The desired duration of the output video in seconds. The frame rate will be adjusted to match this duration.\nDefault is based on config.json or 15.0.')
    parser.add_argument('--greedy', action='store_true', default=greedy_def,
                        help='If set, the agent will use a greedy algorithm to choose the next stroke angle that minimizes the immediate distance to the motif.\nIf not set, angles are chosen randomly.')
    parser.add_argument('--agent-type', type=str, default=agent_type_def, choices=['greedy', 'transformer'],
                        help='Type of agent to use for drawing.\nOptions:\n  greedy: Greedy search agent (existing implementation)\n  transformer: Transformer-based agent for multi-step planning\nDefault: greedy')
    parser.add_argument('--lookahead-depth', type=int, default=lookahead_depth_def,
                        help='Lookahead depth for greedy agent. 1 = standard greedy (immediate reward), >1 = multi-step lookahead.\nHigher values provide better planning but increase computation time exponentially.\nDefault: 1')
    parser.add_argument('--n-samples', type=int, default=n_samples_def,
                        help='Number of angle candidates to consider per step.\nRecommended: 36 for 1-step, 12-16 for 2-step lookahead.\nIf not set, auto-selected based on lookahead-depth.')
    parser.add_argument('--opacity', type=float, default=opacity_def,
                        help='The opacity of each stroke, where 0.0 is fully transparent and 1.0 is fully opaque.\nDefault is 1.0.')
    parser.add_argument('--patience', type=int, default=patience_def,
                        help='For early stopping. The number of consecutive steps with no significant improvement (less than `min_delta`) before stopping the run.\nSet to 0 to disable. Default is 10.')
    parser.add_argument('--min-delta', type=float, default=min_delta_def,
                        help='For early stopping. The minimum change in distance to be considered an improvement.\nDefault is 1e-4.')
    parser.add_argument('--resume_from', type=str, default=None,
                        help="Path to a previous output directory (e.g., 'outputs/20231027_123456').\nThe run will resume from the state saved in that directory, including the canvas, step count, and configuration.")
    parser.add_argument('--reward-type', type=str, default=reward_type_def, choices=['l2', 'l2_white_penalty', 'l2_coverage_bonus', 'l2_exploration_bonus'],
                        help='Reward/loss function type to use.\nOptions:\n  l2: Standard L2 distance (mean squared error)\n  l2_white_penalty: L2 distance with penalty for white pixels\nDefault: l2')
    parser.add_argument('--white-penalty', type=float, default=white_penalty_def,
                        help='White penalty strength (scale-invariant mode). Required when --reward-type is l2_white_penalty.\nControls penalty strength as ratio to L2 distance.\nRecommended range: 0.05-0.5\nExample: 0.1')
    parser.add_argument('--line-width', type=int, default=int(config.get('line_width', 3)),
                        help='Stroke thickness (line width) for drawing. Default is from config.json or 3.')
    parser.add_argument('--verbose', action='store_true', default=verbose_def,
                        help='Enable verbose output showing timing information for agent computations. Useful for performance analysis.')
    parser.add_argument('--memory-efficient', action='store_true', default=memory_efficient_def,
                        help='Enable memory-efficient mode for long runs. Saves stroke coordinates instead of video frames during computation, then reconstructs video afterward. Recommended for runs with >1000 steps to prevent crashes.')
    parser.add_argument('--fast-agent', action='store_true', default=fast_agent_def,
                        help='Enable optimized fast greedy agent for 3-5x speedup. Uses adaptive sampling, early termination, and memory-efficient operations. Only works with 1-step lookahead (lookahead_depth=1).')
    parser.add_argument('--ultra-fast', action='store_true', default=ultra_fast_def,
                        help='Enable ultra-fast mode for maximum speed (5-10x speedup). Uses memory-efficient operations and progressive refinement. Sample count is configurable via --n-samples (default 16). Automatically disables real-time visualization for maximum performance.')
    parser.add_argument('--headless', action='store_true', default=False,
                        help='Disable real-time visualization entirely. Useful for batch processing or remote execution.')
    
    args = parser.parse_args()
    
    # Configure visualization based on flags
    # Ultra-fast mode automatically disables visualization for maximum speed
    viz_enabled = not args.headless and not args.ultra_fast
    if hasattr(args, 'ultra_fast') and args.ultra_fast and not args.headless:
        print("Ultra-fast mode: Real-time visualization automatically disabled for maximum performance")
    
    # Add visualization config to args
    args.visualization = {
        'enabled': viz_enabled,
        'mode': visualization_def.get('mode', 'opencv'),
        'update_frequency': visualization_def.get('update_frequency', 10),
        'window_size': visualization_def.get('window_size', [800, 600]),
        'save_snapshots': visualization_def.get('save_snapshots', False),
        'snapshot_interval': visualization_def.get('snapshot_interval', 100)
    }
    
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
    if args.reward_type == 'l2_white_penalty':
        if args.white_penalty is None:
            print("Error: --white-penalty must be set when using l2_white_penalty.", file=sys.stderr)
            sys.exit(1)
    if args.lookahead_depth < 1:
        print("Error: --lookahead-depth must be >= 1", file=sys.stderr)
        sys.exit(1)
