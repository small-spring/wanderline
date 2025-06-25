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
    
    Args:
        prev_canvas: Current canvas state
        motif: Target motif image
        start_pt: Starting point for the next stroke
        length: Length of stroke
        n_samples: Number of angle samples to consider
        lookahead_depth: How many steps ahead to search (1 = greedy, >1 = multi-step)
        reward_type: Type of reward function ('l2' or 'l2_white_penalty')
        white_penalty: White penalty strength (required for l2_white_penalty)
        opacity: Stroke opacity
        line_width: Stroke thickness
        use_vectorized: Whether to use vectorized computation (faster)
        use_full_vectorization: Whether to use full vectorization for 2-step (fastest)
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
            # Use fully vectorized version for 2-step
            result, timing_info = choose_next_angle_fully_vectorized_2step(
                prev_canvas, motif, start_pt, length, n_samples,
                reward_type, white_penalty, opacity, line_width, verbose
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
    Samples n_samples angles uniformly from [0, 2*pi).
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
    Vectorized version of greedy angle selection for faster computation.
    
    Args:
        prev_canvas: Current canvas state
        motif: Target motif image
        start_pt: Starting point for the next stroke
        length: Length of stroke
        n_samples: Number of angle samples to consider
        reward_type: Type of reward function ('l2' or 'l2_white_penalty')
        white_penalty: White penalty strength (required for l2_white_penalty)
        opacity: Stroke opacity
        line_width: Stroke thickness
        verbose: Whether to return timing information
        
    Returns:
        Best angle (in radians), timing_info dict if verbose=True else empty dict
    """
    timing_info = {}
    start_time = time.time()
    
    # Generate all angles to test
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
        n_samples: int = 12,  # Reduced default for multi-step
        lookahead_depth: int = 2,
        reward_type: str = 'l2',
        white_penalty: float = None,
        opacity: float = 1.0,
        line_width: int = 3
        ) -> float:
    """
    Vectorized multi-step lookahead angle selection.
    
    Uses smaller n_samples by default due to exponential growth in computation.
    For depth=2 with n_samples=12: 12^2 = 144 combinations
    For depth=3 with n_samples=8: 8^3 = 512 combinations
    
    Args:
        prev_canvas: Current canvas state
        motif: Target motif image
        start_pt: Starting point for the next stroke
        length: Length of stroke
        n_samples: Number of angle samples per step (reduced for multi-step)
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
    
    # Generate first level angles
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


def choose_next_angle_fully_vectorized_2step(
        prev_canvas: np.ndarray,
        motif: np.ndarray,
        start_pt: tuple[int, int],
        length: int,
        n_samples: int = 16,  # 16^2 = 256 combinations
        reward_type: str = 'l2',
        white_penalty: float = None,
        opacity: float = 1.0,
        line_width: int = 3,
        verbose: bool = False
        ) -> tuple[float, dict]:
    """
    Fully vectorized 2-step lookahead using complete tensor operations.
    
    Computes all n_samples^2 combinations in a single batch operation, 
    taking advantage of available memory for maximum speed.
    
    Args:
        prev_canvas: Current canvas state (H, W, C)
        motif: Target motif image (H, W, C)  
        start_pt: Starting point for the next stroke
        length: Length of stroke
        n_samples: Number of angle samples per step (total: n_samples^2)
        reward_type: Type of reward function
        white_penalty: White penalty strength
        opacity: Stroke opacity
        line_width: Stroke thickness
        verbose: Whether to return timing information
        
    Returns:
        Best first angle, timing_info dict
    """
    timing_info = {}
    start_time = time.time()
    
    # Generate angle combinations
    angles = np.linspace(0, 2 * np.pi, n_samples, endpoint=False)
    
    # Create all combinations: (n_samples, n_samples)
    angles1_grid, angles2_grid = np.meshgrid(angles, angles, indexing='ij')
    angles1_flat = angles1_grid.flatten()  # (n_samples^2,)
    angles2_flat = angles2_grid.flatten()  # (n_samples^2,)
    
    total_combinations = n_samples * n_samples
    timing_info['total_combinations'] = total_combinations
    
    try:
        # Step 1: Apply all first strokes in batch
        step1_start = time.time()
        canvas_batch1, end_points1 = apply_stroke_vectorized(
            prev_canvas, angles1_flat, start=start_pt, length=length,
            opacity=opacity, return_end=True, line_width=line_width
        )
        timing_info['step1_stroke_time'] = time.time() - step1_start
        
        # Step 2: Apply all second strokes in batch
        step2_start = time.time()
        canvas_batch2 = np.zeros_like(canvas_batch1)
        
        # Apply second strokes (this is the tricky part - vectorizing over different start points)
        for i in range(total_combinations):
            canvas_batch2[i] = apply_stroke(
                canvas_batch1[i], angles2_flat[i], 
                start=tuple(end_points1[i].astype(int)), 
                length=length, opacity=opacity, return_end=False, line_width=line_width
            )
        timing_info['step2_stroke_time'] = time.time() - step2_start
        
        # Compute rewards for all combinations
        reward_start = time.time()
        
        # Immediate rewards (step 1)
        if reward_type == 'l2_white_penalty' and white_penalty is not None:
            immediate_rewards = compute_reward_with_white_penalty_vectorized(
                prev_canvas, canvas_batch1, motif, white_penalty
            )
            future_rewards = compute_reward_with_white_penalty_vectorized(
                canvas_batch1, canvas_batch2, motif, white_penalty
            )
        else:
            immediate_rewards = compute_reward_vectorized(prev_canvas, canvas_batch1, motif)
            future_rewards = compute_reward_vectorized(canvas_batch1, canvas_batch2, motif)
        
        # Total rewards
        total_rewards = immediate_rewards + future_rewards
        timing_info['reward_time'] = time.time() - reward_start
        
        # Reshape back to (n_samples, n_samples) and find best first action
        total_rewards_grid = total_rewards.reshape(n_samples, n_samples)
        
        # For each first action, get the maximum total reward across all second actions
        best_total_rewards_per_first_action = np.max(total_rewards_grid, axis=1)
        best_first_action_idx = np.argmax(best_total_rewards_per_first_action)
        best_angle = float(angles[best_first_action_idx])
        
        timing_info['total_time'] = time.time() - start_time
        timing_info['method'] = 'fully_vectorized_2step'
        timing_info['memory_usage_mb'] = (canvas_batch1.nbytes + canvas_batch2.nbytes) / (1024 * 1024)
        
        if verbose:
            print(f"Fully vectorized 2-step: {total_combinations} combinations in {timing_info['total_time']:.3f}s")
            print(f"Memory usage: {timing_info['memory_usage_mb']:.1f}MB")
        
        return best_angle, timing_info
        
    except Exception as e:
        if verbose:
            print(f"Fully vectorized 2-step failed, falling back: {e}")
        
        # Fallback to partial vectorization
        fallback_start = time.time()
        result, fallback_timing = choose_next_angle_vectorized_lookahead(
            prev_canvas, motif, start_pt, length, n_samples, 2,
            reward_type, white_penalty, opacity, line_width
        )
        
        timing_info['total_time'] = time.time() - start_time
        timing_info['fallback_time'] = time.time() - fallback_start
        timing_info['method'] = 'partial_vectorized_fallback'
        
        return result, timing_info


def choose_next_angle_memory_efficient_2step(
        prev_canvas: np.ndarray,
        motif: np.ndarray,
        start_pt: tuple[int, int],
        length: int,
        n_samples: int = 12,
        batch_size: int = 64,  # Process in smaller batches to control memory
        reward_type: str = 'l2',
        white_penalty: float = None,
        opacity: float = 1.0,
        line_width: int = 3,
        verbose: bool = False
        ) -> tuple[float, dict]:
    """
    Memory-efficient 2-step lookahead that processes combinations in batches.
    
    Balances speed and memory usage by processing n_samples^2 combinations 
    in chunks of batch_size.
    
    Args:
        prev_canvas: Current canvas state (H, W, C)
        motif: Target motif image (H, W, C)
        start_pt: Starting point for the next stroke
        length: Length of stroke
        n_samples: Number of angle samples per step
        batch_size: Maximum combinations to process at once
        reward_type: Type of reward function
        white_penalty: White penalty strength
        opacity: Stroke opacity
        line_width: Stroke thickness
        verbose: Whether to return timing information
        
    Returns:
        Best first angle, timing_info dict
    """
    timing_info = {}
    start_time = time.time()
    
    angles = np.linspace(0, 2 * np.pi, n_samples, endpoint=False)
    total_combinations = n_samples * n_samples
    
    # Estimate memory usage
    canvas_size_mb = (prev_canvas.nbytes * batch_size) / (1024 * 1024)
    timing_info['estimated_memory_mb'] = canvas_size_mb * 2  # Two canvas batches
    
    if verbose:
        print(f"Processing {total_combinations} combinations in batches of {batch_size}")
        print(f"Estimated memory per batch: {timing_info['estimated_memory_mb']:.1f}MB")
    
    try:
        # Apply first level strokes (vectorized)
        step1_start = time.time()
        canvas_batch1, end_points1 = apply_stroke_vectorized(
            prev_canvas, angles, start=start_pt, length=length,
            opacity=opacity, return_end=True, line_width=line_width
        )
        timing_info['step1_time'] = time.time() - step1_start
        
        # Process second level in batches
        best_angle = 0.0
        best_total_reward = -np.inf
        total_processed = 0
        
        batch_start = time.time()
        
        for i in range(n_samples):
            # For each first-level result, try all second-level angles in batches
            current_canvas = canvas_batch1[i]
            current_position = end_points1[i]
            
            # Compute immediate reward for this first step
            if reward_type == 'l2_white_penalty' and white_penalty is not None:
                immediate_reward = compute_reward_with_white_penalty_vectorized(
                    prev_canvas, current_canvas[np.newaxis, :, :, :], motif, white_penalty
                )[0]
            else:
                immediate_reward = compute_reward_vectorized(
                    prev_canvas, current_canvas[np.newaxis, :, :, :], motif
                )[0]
            
            # Process second-level angles in smaller batches
            for batch_start_idx in range(0, n_samples, min(batch_size, n_samples)):
                batch_end_idx = min(batch_start_idx + batch_size, n_samples)
                batch_angles = angles[batch_start_idx:batch_end_idx]
                
                # Apply second level strokes for this batch
                canvas_batch2 = apply_stroke_vectorized(
                    current_canvas, batch_angles, start=tuple(current_position.astype(int)),
                    length=length, opacity=opacity, return_end=False, line_width=line_width
                )
                
                # Compute future rewards
                if reward_type == 'l2_white_penalty' and white_penalty is not None:
                    future_rewards = compute_reward_with_white_penalty_vectorized(
                        current_canvas, canvas_batch2, motif, white_penalty
                    )
                else:
                    future_rewards = compute_reward_vectorized(
                        current_canvas, canvas_batch2, motif
                    )
                
                # Check if any combination in this batch is the best so far
                total_rewards = immediate_reward + future_rewards
                batch_best_idx = np.argmax(total_rewards)
                batch_best_reward = total_rewards[batch_best_idx]
                
                if batch_best_reward > best_total_reward:
                    best_total_reward = batch_best_reward
                    best_angle = float(angles[i])  # The first action
                
                total_processed += len(batch_angles)
        
        timing_info['batch_processing_time'] = time.time() - batch_start
        timing_info['total_time'] = time.time() - start_time
        timing_info['total_processed'] = total_processed
        timing_info['method'] = 'memory_efficient_2step'
        
        if verbose:
            print(f"Processed {total_processed} combinations in {timing_info['total_time']:.3f}s")
        
        return best_angle, timing_info
        
    except Exception as e:
        if verbose:
            print(f"Memory-efficient 2-step failed, falling back: {e}")
        
        return choose_next_angle_vectorized_lookahead(
            prev_canvas, motif, start_pt, length, n_samples, 2,
            reward_type, white_penalty, opacity, line_width
        ), {'method': 'fallback', 'error': str(e)}