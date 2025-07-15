import numpy as np
import math
import time
from wanderline.canvas import apply_stroke
from wanderline.reward import compute_reward
from wanderline.memory_efficient_canvas import choose_angle_memory_efficient
from wanderline.constants import DEFAULT_N_SAMPLES, LOOKAHEAD_PERFORMANCE_WARNING_THRESHOLD

def choose_next_angle(
        prev_canvas: np.ndarray,
        motif: np.ndarray,
        start_pt: tuple[int, int],
        length: int,
        n_samples: int = DEFAULT_N_SAMPLES,
        lookahead_depth: int = 1,
        reward_type: str = 'l2',
        white_penalty: float = None,
        opacity: float = 1.0,
        line_width: int = 3,
        use_vectorized: bool = True,
        use_full_vectorization: bool = True,
        verbose: bool = False
        ) -> float:
    """
    Choose the angle that maximizes reward using multi-step lookahead.
    
    This is the main entry point for angle selection. It routes to different
    implementations based on the parameters provided.
    
    Args:
        prev_canvas: Current canvas state
        motif: Target motif image
        start_pt: Starting point for the next stroke
        length: Length of stroke
        n_samples: Number of angle candidates to consider (default: DEFAULT_N_SAMPLES for 1-step)
        lookahead_depth: How many steps ahead to search (1 = greedy, >1 = multi-step)
        reward_type: Type of reward function ('l2' or 'l2_white_penalty')
        white_penalty: White penalty strength (required for l2_white_penalty)
        opacity: Stroke opacity
        line_width: Stroke thickness
        use_vectorized: Whether to use vectorized computation (faster)
        use_full_vectorization: Whether to use full vectorization for 2-step (simplified)
        verbose: Whether to print timing information
        
    Returns:
        Best angle (in radians)
    """
    # Always use memory-efficient implementation for best performance
    # (Previous vectorized implementations had 36x memory overhead)
    if lookahead_depth == 1:
        result, timing_info = choose_angle_memory_efficient(
            prev_canvas, motif, start_pt, length, n_samples,
            reward_type, white_penalty, opacity, line_width, 
            progressive_refinement=True, verbose=verbose
        )
        if verbose and timing_info:
            print(f"Memory-efficient: {timing_info.get('n_samples', n_samples)} samples in {timing_info.get('total_time', 0):.3f}s")
        return result
    else:
        # Multi-step lookahead: use recursive approach with memory-efficient single steps
        best_angle = 0.0
        best_total_reward = -math.inf
        
        # Use user-specified n_samples - warn if this might cause performance issues
        if lookahead_depth > 1 and n_samples > LOOKAHEAD_PERFORMANCE_WARNING_THRESHOLD:
            print(f"⚠️  Warning: {n_samples} samples with {lookahead_depth}-step lookahead may be slow")
            print(f"   Consider reducing --n-samples or using --lookahead-depth 1 for better performance")
        
        for angle in np.linspace(0, 2 * math.pi, n_samples, endpoint=False):
            try:
                next_canvas, end_pt = apply_stroke(prev_canvas, angle, start=start_pt, length=length, return_end=True)
                immediate_reward = compute_reward(prev_canvas, next_canvas, motif)
                
                if lookahead_depth <= 1:
                    total_reward = immediate_reward
                else:
                    # Recursively evaluate future steps using memory-efficient approach
                    future_reward = _evaluate_future_reward_memory_efficient(
                        next_canvas, motif, end_pt, length, n_samples, lookahead_depth - 1,
                        reward_type, white_penalty, opacity, line_width
                    )
                    total_reward = immediate_reward + future_reward
                
                if total_reward > best_total_reward:
                    best_total_reward = total_reward
                    best_angle = float(angle)
            except Exception:
                continue
        
        return best_angle

def _evaluate_future_reward_memory_efficient(
        canvas: np.ndarray,
        motif: np.ndarray,
        start_pt: tuple[int, int],
        length: int,
        n_samples: int,
        lookahead_depth: int,
        reward_type: str = 'l2',
        white_penalty: float = None,
        opacity: float = 1.0,
        line_width: int = 3
        ) -> float:
    """
    Evaluate future reward using memory-efficient approach.
    """
    if lookahead_depth <= 0:
        return 0.0
    
    best_future_reward = -math.inf
    
    for angle in np.linspace(0, 2 * math.pi, n_samples, endpoint=False):
        try:
            next_canvas, end_pt = apply_stroke(canvas, angle, start=start_pt, length=length, return_end=True)
            immediate_reward = compute_reward(canvas, next_canvas, motif)
            
            if lookahead_depth <= 1:
                future_reward = immediate_reward
            else:
                future_reward = immediate_reward + _evaluate_future_reward_memory_efficient(
                    next_canvas, motif, end_pt, length, n_samples, lookahead_depth - 1,
                    reward_type, white_penalty, opacity, line_width
                )
            
            if future_reward > best_future_reward:
                best_future_reward = future_reward
        except Exception:
            continue
    
    return best_future_reward

# REMOVED: _choose_next_angle_greedy() - redundant implementation 
# Used memory-efficient approach instead (see conservation_analysis.md)

# REMOVED: _choose_next_angle_lookahead() - redundant implementation
# Multi-step lookahead now handled in main choose_next_angle() function

# REMOVED: _evaluate_future_reward() - redundant implementation
# Multi-step lookahead now uses memory-efficient approach in main function

# REMOVED: choose_next_angle_vectorized() - redundant implementation with 36x memory overhead
# Main choose_next_angle() now routes to memory-efficient implementation instead


# REMOVED: choose_next_angle_vectorized_lookahead() - redundant implementation
# Multi-step lookahead now handled directly in main choose_next_angle() function


