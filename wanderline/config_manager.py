import argparse
import json
import os
import sys
from .constants import (
    DEFAULT_N_SAMPLES, PERFORMANCE_SAMPLES, DEFAULT_PATIENCE, DEFAULT_MIN_DELTA,
    MEMORY_EFFICIENT_RECOMMENDED_STEPS
)


def load_config_and_parse_args():
    """
    Load configuration and parse command line arguments.
    Streamlined to 10 essential parameters for conservation.
    """
    # Essential defaults (simplified from 45+ to 10 parameters)
    defaults = {
        'motif_path': None,
        'steps': MEMORY_EFFICIENT_RECOMMENDED_STEPS,
        'ratio': 0.1,
        'opacity': 0.3,
        'line_width': 2,
        'duration': 15.0,
        'reward_type': 'l2',
        'performance_mode': 'standard',
        'memory_efficient': False,
        'verbose': False
    }
    
    # Optional: Load from config file if provided
    temp_parser = argparse.ArgumentParser(add_help=False)
    temp_parser.add_argument('--config', type=str, default=None)
    args_pre, _ = temp_parser.parse_known_args()
    
    if args_pre.config and os.path.isfile(args_pre.config):
        with open(args_pre.config, 'r') as f:
            config_overrides = json.load(f)
            defaults.update(config_overrides)
    
    # Create streamlined parser with 10 essential parameters
    parser = argparse.ArgumentParser(
        description="Wanderline: Single-stroke drawing agent with streamlined configuration",
        formatter_class=argparse.RawTextHelpFormatter
    )
    
    # Essential parameters only (10 total)
    parser.add_argument('--config', type=str, default=None,
                        help='Optional JSON config file path')
    parser.add_argument('motif_path', nargs='?', default=defaults['motif_path'],
                        help='Target motif image path')
    parser.add_argument('--steps', type=int, default=defaults['steps'],
                        help=f'Number of strokes to draw (default: {MEMORY_EFFICIENT_RECOMMENDED_STEPS})')
    parser.add_argument('--ratio', type=float, default=defaults['ratio'],
                        help='Stroke length ratio (default: 0.1)')
    parser.add_argument('--opacity', type=float, default=defaults['opacity'],
                        help='Stroke transparency 0.0-1.0 (default: 0.3)')
    parser.add_argument('--line-width', type=int, default=defaults['line_width'],
                        help='Stroke thickness (default: 2)')
    parser.add_argument('--duration', type=float, default=defaults['duration'],
                        help='Video duration in seconds (default: 15.0)')
    parser.add_argument('--reward-type', type=str, default=defaults['reward_type'], 
                        choices=['l2', 'l2_white_penalty'],
                        help='Loss function: l2 or l2_white_penalty (default: l2)')
    parser.add_argument('--performance-mode', type=str, default=defaults['performance_mode'],
                        choices=['standard', 'fast', 'ultra_fast', 'headless'],
                        help='Performance mode (default: standard)')
    parser.add_argument('--memory-efficient', action='store_true', default=defaults['memory_efficient'],
                        help=f'Enable for runs >{MEMORY_EFFICIENT_RECOMMENDED_STEPS} steps (default: False)')
    parser.add_argument('--verbose', action='store_true', default=defaults['verbose'],
                        help='Show timing information (default: False)')
    
    # Advanced parameters (respect user configuration)
    parser.add_argument('--n-samples', type=int, default=None,
                        help='Number of angle candidates to evaluate (default: auto-selected based on performance mode)')
    parser.add_argument('--lookahead-depth', type=int, default=1,
                        help='Multi-step lookahead depth (default: 1)')
    
    # Backward compatibility (deprecated, will be removed)
    parser.add_argument('--resume_from', type=str, default=None, help='Resume from previous run')
    parser.add_argument('--white-penalty', type=float, default=None, help='White penalty for l2_white_penalty mode')
    
    # Legacy flags (mapped to performance_mode)
    parser.add_argument('--greedy', action='store_true', help='Deprecated: use --performance-mode standard')
    parser.add_argument('--fast-agent', action='store_true', help='Deprecated: use --performance-mode fast')
    parser.add_argument('--ultra-fast', action='store_true', help='Deprecated: use --performance-mode ultra_fast')
    parser.add_argument('--headless', action='store_true', help='Deprecated: use --performance-mode headless')
    
    args = parser.parse_args()
    
    # Map legacy flags to performance_mode for backward compatibility
    if args.ultra_fast:
        args.performance_mode = 'ultra_fast'
        print("⚠️  --ultra-fast is deprecated, use --performance-mode ultra_fast")
    elif args.fast_agent:
        args.performance_mode = 'fast'
        print("⚠️  --fast-agent is deprecated, use --performance-mode fast")
    elif args.headless:
        args.performance_mode = 'headless'
        print("⚠️  --headless is deprecated, use --performance-mode headless")
    elif args.greedy:
        args.performance_mode = 'standard'
        print("⚠️  --greedy is deprecated, use --performance-mode standard")
    
    # Auto-configure based on performance mode (only if user didn't specify)
    if args.performance_mode == 'ultra_fast':
        args.fast_agent = True
        args.ultra_fast = True
        args.memory_efficient = True
        # Only set n_samples if user didn't specify
        if args.n_samples is None:
            args.n_samples = PERFORMANCE_SAMPLES['ultra_fast']
    elif args.performance_mode == 'fast':
        args.fast_agent = True
        args.ultra_fast = False
        args.memory_efficient = True
        # Only set n_samples if user didn't specify  
        if args.n_samples is None:
            args.n_samples = PERFORMANCE_SAMPLES['fast']
    elif args.performance_mode == 'headless':
        args.fast_agent = False
        args.ultra_fast = False
        # Only set n_samples if user didn't specify
        if args.n_samples is None:
            args.n_samples = PERFORMANCE_SAMPLES['headless']
    else:  # standard
        args.fast_agent = False
        args.ultra_fast = False
        # Only set n_samples if user didn't specify
        if args.n_samples is None:
            args.n_samples = PERFORMANCE_SAMPLES['headless']
    
    # Add missing attributes for backward compatibility
    args.patience = DEFAULT_PATIENCE  # Early stopping patience (simplified)
    args.min_delta = DEFAULT_MIN_DELTA  # Early stopping threshold
    args.agent_type = 'greedy'  # Only greedy is production-ready
    args.greedy = True  # Always true now
    
    # Configure visualization (simplified)
    args.visualization = {
        'enabled': args.performance_mode not in ['ultra_fast', 'headless'],
        'mode': 'opencv',
        'update_frequency': 10,
        'window_size': [800, 600],
        'save_snapshots': False,
        'snapshot_interval': 100
    }
    
    # Ensure headless mode disables visualization completely
    if args.performance_mode == 'headless':
        args.headless = True
    else:
        args.headless = False
    
    return args


def validate_args(args):
    """
    Validate streamlined configuration arguments.
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
    if args.performance_mode not in ['standard', 'fast', 'ultra_fast', 'headless']:
        print("Error: --performance-mode must be one of: standard, fast, ultra_fast, headless", file=sys.stderr)
        sys.exit(1)
