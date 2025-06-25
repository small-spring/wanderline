import numpy as np
import math
import time
from wanderline.canvas import apply_stroke, apply_stroke_vectorized
from wanderline.reward import compute_reward, compute_reward_vectorized, compute_reward_with_white_penalty_vectorized

def choose_next_angle(
        prev_canvas: np.ndarray,
        motif: np.ndarray,
        start_pt: tuple[int, int],
        length: int,
        n_samples: int = 36,
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
        n_samples: Number of angle candidates to consider (default: 36 for 1-step)
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
    if use_vectorized:
        if lookahead_depth == 1:
            result, timing_info = choose_next_angle_vectorized(
                prev_canvas, motif, start_pt, length, n_samples,
                reward_type, white_penalty, opacity, line_width, verbose
            )
            if verbose and timing_info:
                print(f"1-step vectorized: {timing_info['n_samples']} samples in {timing_info['total_time']:.3f}s")
            return result
        elif lookahead_depth == 2 and use_full_vectorization:
            # Use vectorized lookahead for 2-step (full vectorization removed for simplicity)
            result = choose_next_angle_vectorized_lookahead(
                prev_canvas, motif, start_pt, length, n_samples, 2,
                reward_type, white_penalty, opacity, line_width
            )
            return result
        else:
            return choose_next_angle_vectorized_lookahead(
                prev_canvas, motif, start_pt, length, n_samples, lookahead_depth,
                reward_type, white_penalty, opacity, line_width
            )
    else:
        # Use original sequential implementation
        if lookahead_depth == 1:
            return _choose_next_angle_greedy(prev_canvas, motif, start_pt, length, n_samples)
        else:
            return _choose_next_angle_lookahead(prev_canvas, motif, start_pt, length, n_samples, lookahead_depth)

def _choose_next_angle_greedy(
        prev_canvas: np.ndarray,
        motif: np.ndarray,
        start_pt: tuple[int, int],
        length: int,
        n_samples: int = 36
        ) -> float:
    """
    Choose the angle that maximizes immediate reward (1-step lookahead).
    
    Sequential implementation that samples n_samples angles uniformly from [0, 2*pi).
    This is the reference implementation for testing vectorized versions.
    """
    best_angle = 0.0
    best_reward = -math.inf
    for angle in np.linspace(0, 2 * math.pi, n_samples, endpoint=False):
        try:
            next_canvas, _ = apply_stroke(prev_canvas, angle, start=start_pt, length=length, return_end=True)
            r = compute_reward(prev_canvas, next_canvas, motif)
        except Exception:
            continue
        if r > best_reward:
            best_reward = r
            best_angle = float(angle)
    return best_angle

def _choose_next_angle_lookahead(
        prev_canvas: np.ndarray,
        motif: np.ndarray,
        start_pt: tuple[int, int],
        length: int,
        n_samples: int,
        lookahead_depth: int
        ) -> float:
    """
    Choose the angle that maximizes cumulative reward over multiple steps.
    Uses recursive lookahead to evaluate action sequences.
    """
    best_angle = 0.0
    best_total_reward = -math.inf
    
    for angle in np.linspace(0, 2 * math.pi, n_samples, endpoint=False):
        try:
            next_canvas, end_pt = apply_stroke(prev_canvas, angle, start=start_pt, length=length, return_end=True)
            immediate_reward = compute_reward(prev_canvas, next_canvas, motif)
            
            if lookahead_depth <= 1:
                total_reward = immediate_reward
            else:
                # Recursively evaluate future steps
                future_reward = _evaluate_future_reward(
                    next_canvas, motif, end_pt, length, n_samples, lookahead_depth - 1
                )
                total_reward = immediate_reward + future_reward
            
        except Exception:
            continue
            
        if total_reward > best_total_reward:
            best_total_reward = total_reward
            best_angle = float(angle)
    
    return best_angle

def _evaluate_future_reward(
        canvas: np.ndarray,
        motif: np.ndarray,
        start_pt: tuple[int, int],
        length: int,
        n_samples: int,
        remaining_depth: int
        ) -> float:
    """
    Evaluate the maximum expected reward for future steps.
    """
    if remaining_depth <= 0:
        return 0.0
    
    best_future_reward = -math.inf
    
    for angle in np.linspace(0, 2 * math.pi, n_samples, endpoint=False):
        try:
            next_canvas, end_pt = apply_stroke(canvas, angle, start=start_pt, length=length, return_end=True)
            immediate_reward = compute_reward(canvas, next_canvas, motif)
            
            if remaining_depth <= 1:
                future_reward = immediate_reward
            else:
                # Recursively evaluate further future steps
                further_reward = _evaluate_future_reward(
                    next_canvas, motif, end_pt, length, n_samples, remaining_depth - 1
                )
                future_reward = immediate_reward + further_reward
            
        except Exception:
            continue
            
        if future_reward > best_future_reward:
            best_future_reward = future_reward
    
    return best_future_reward if best_future_reward != -math.inf else 0.0

def choose_next_angle_vectorized(
        prev_canvas: np.ndarray,
        motif: np.ndarray,
        start_pt: tuple[int, int],
        length: int,
        n_samples: int = 36,
        reward_type: str = 'l2',
        white_penalty: float = None,
        opacity: float = 1.0,
        line_width: int = 3,
        verbose: bool = False
        ) -> tuple[float, dict]:
    """
    Vectorized version of greedy angle selection for fast 1-step lookahead.
    
    This function applies all angle candidates in batch operations, significantly
    faster than sequential processing. Recommended for 1-step lookahead with
    n_samples up to 36.
    
    Args:
        prev_canvas: Current canvas state
        motif: Target motif image
        start_pt: Starting point for the next stroke
        length: Length of stroke
        n_samples: Number of angle candidates to consider (recommended: up to 36)
        reward_type: Type of reward function ('l2' or 'l2_white_penalty')
        white_penalty: White penalty strength (required for l2_white_penalty)
        opacity: Stroke opacity
        line_width: Stroke thickness
        verbose: Whether to return timing information
        
    Returns:
        Tuple of (best_angle, timing_info_dict)
        - best_angle: Best angle in radians
        - timing_info: Dict with timing details (empty if verbose=False)
    """
    timing_info = {}
    start_time = time.time()
    
    # Generate all angle candidates to test
    angles = np.linspace(0, 2 * np.pi, n_samples, endpoint=False)
    
    try:
        # Apply all strokes in batch
        stroke_start = time.time()
        canvas_batch = apply_stroke_vectorized(
            prev_canvas, angles, start=start_pt, length=length, 
            opacity=opacity, return_end=False, line_width=line_width
        )
        timing_info['stroke_time'] = time.time() - stroke_start
        
        # Compute rewards in batch
        reward_start = time.time()
        if reward_type == 'l2_white_penalty' and white_penalty is not None:
            rewards = compute_reward_with_white_penalty_vectorized(
                prev_canvas, canvas_batch, motif, white_penalty
            )
        else:
            rewards = compute_reward_vectorized(prev_canvas, canvas_batch, motif)
        timing_info['reward_time'] = time.time() - reward_start
        
        # Find best angle
        best_idx = np.argmax(rewards)
        best_angle = float(angles[best_idx])
        
        timing_info['total_time'] = time.time() - start_time
        timing_info['n_samples'] = n_samples
        timing_info['method'] = 'vectorized'
        
        if verbose:
            return best_angle, timing_info
        else:
            return best_angle, {}
        
    except Exception as e:
        # Fallback to non-vectorized version
        if verbose:
            print(f"Vectorized computation failed, falling back to sequential: {e}")
        fallback_start = time.time()
        result = _choose_next_angle_greedy(prev_canvas, motif, start_pt, length, n_samples)
        timing_info['total_time'] = time.time() - start_time
        timing_info['fallback_time'] = time.time() - fallback_start
        timing_info['method'] = 'sequential_fallback'
        return result, timing_info if verbose else {}


def choose_next_angle_vectorized_lookahead(
        prev_canvas: np.ndarray,
        motif: np.ndarray,
        start_pt: tuple[int, int],
        length: int,
        n_samples: int = 12,  # Reduced default for multi-step to avoid memory issues
        lookahead_depth: int = 2,
        reward_type: str = 'l2',
        white_penalty: float = None,
        opacity: float = 1.0,
        line_width: int = 3
        ) -> float:
    """
    Vectorized multi-step lookahead angle selection.
    
    Uses smaller n_samples by default due to exponential growth in computation.
    Computational complexity: O(n_samples^lookahead_depth)
    
    Recommended settings:
    - For depth=2: n_samples up to 16 (16^2 = 256 combinations)
    - For depth=3: n_samples up to 8 (8^3 = 512 combinations)
    
    Args:
        prev_canvas: Current canvas state
        motif: Target motif image
        start_pt: Starting point for the next stroke
        length: Length of stroke
        n_samples: Number of angle candidates per step (reduced for multi-step)
        lookahead_depth: How many steps ahead to search
        reward_type: Type of reward function
        white_penalty: White penalty strength
        opacity: Stroke opacity
        line_width: Stroke thickness
        
    Returns:
        Best angle (in radians)
    """
    if lookahead_depth == 1:
        return choose_next_angle_vectorized(
            prev_canvas, motif, start_pt, length, n_samples, 
            reward_type, white_penalty, opacity, line_width
        )
    
    # Generate first level angle candidates
    angles1 = np.linspace(0, 2 * math.pi, n_samples, endpoint=False)
    
    try:
        # Apply first level strokes
        canvas_batch1, end_points1 = apply_stroke_vectorized(
            prev_canvas, angles1, start=start_pt, length=length,
            opacity=opacity, return_end=True, line_width=line_width
        )
        
        # For 2-step lookahead, expand each first-level result
        if lookahead_depth == 2:
            best_total_reward = -np.inf
            best_angle = 0.0
            
            for i in range(n_samples):
                # For each first-level result, try all second-level angles
                angles2 = np.linspace(0, 2 * math.pi, n_samples, endpoint=False)
                
                # Apply second level strokes
                canvas_batch2 = apply_stroke_vectorized(
                    canvas_batch1[i], angles2, start=tuple(end_points1[i]), 
                    length=length, opacity=opacity, return_end=False, line_width=line_width
                )
                
                # Compute immediate reward for first step
                if reward_type == 'l2_white_penalty' and white_penalty is not None:
                    immediate_reward = compute_reward_with_white_penalty_vectorized(
                        prev_canvas, canvas_batch1[i:i+1], motif, white_penalty
                    )[0]
                else:
                    immediate_reward = compute_reward_vectorized(
                        prev_canvas, canvas_batch1[i:i+1], motif
                    )[0]
                
                # Compute future rewards for second step
                if reward_type == 'l2_white_penalty' and white_penalty is not None:
                    future_rewards = compute_reward_with_white_penalty_vectorized(
                        canvas_batch1[i], canvas_batch2, motif, white_penalty
                    )
                else:
                    future_rewards = compute_reward_vectorized(
                        canvas_batch1[i], canvas_batch2, motif
                    )
                
                # Total reward is immediate + best future
                total_reward = immediate_reward + np.max(future_rewards)
                
                if total_reward > best_total_reward:
                    best_total_reward = total_reward
                    best_angle = float(angles1[i])
            
            return best_angle
        
        else:
            # For deeper lookahead, fall back to recursive approach
            # (Full vectorization becomes memory-intensive)
            return _choose_next_angle_lookahead(
                prev_canvas, motif, start_pt, length, n_samples, lookahead_depth
            )
            
    except Exception as e:
        print(f"Vectorized lookahead failed, falling back to sequential: {e}")
        return _choose_next_angle_lookahead(
            prev_canvas, motif, start_pt, length, n_samples, lookahead_depth
        )


