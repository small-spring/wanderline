import numpy as np

def l2_distance(a: np.ndarray, b: np.ndarray) -> float:
    """
    Compute L2 (Euclidean) distance between two images (arrays).
    Both arrays must have the same shape.
    """
    a_f = a.astype(float)
    b_f = b.astype(float)
    return float(np.linalg.norm(a_f - b_f))

def white_ratio(canvas: np.ndarray) -> float:
    """
    Calculate the ratio of pure white pixels in the canvas.
    """
    if canvas.ndim == 3 and canvas.shape[2] == 3:
        white = np.all(canvas == 255, axis=2)
    else:
        white = (canvas == 255)
    return float(np.sum(white)) / white.size

def l2_distance_with_white_penalty(a: np.ndarray, b: np.ndarray, white_penalty: float = None) -> float:
    """
    Compute L2 distance with white pixel penalty.
    
    The white penalty encourages agents to draw on white pixels rather than leaving them untouched.
    Higher white pixel ratios result in higher penalties, motivating the agent to cover white areas.
    
    Uses scale-invariant penalty mode:
    loss = l2_distance(a, b) * (1 + white_penalty * white_ratio(a))
    
    Args:
        a: First image array (typically the canvas)
        b: Second image array (typically the motif)
        white_penalty: Penalty strength as ratio to L2 distance (scale-invariant)
        
    Returns:
        Penalized L2 distance
        
    Example:
        # Scale-invariant white penalty (recommended range: 0.05-0.5)
        loss = l2_distance_with_white_penalty(canvas, motif, white_penalty=0.1)
    """
    if white_penalty is None:
        raise ValueError("white_penalty must be specified for l2_distance_with_white_penalty")
    
    l2 = l2_distance(a, b)
    white_ratio_value = white_ratio(a)
    
    # Scale-invariant relative penalty
    penalty_factor = white_penalty * white_ratio_value
    return l2 * (1.0 + penalty_factor)

def compute_reward(prev_state: np.ndarray, next_state: np.ndarray, motif: np.ndarray) -> float:
    """
    Immediate reward: reduction in distance to motif.
        Δd = d(prev_state, motif) - d(next_state, motif)
    """
    return l2_distance(prev_state, motif) - l2_distance(next_state, motif)

def compute_reward_with_white_penalty(prev_state: np.ndarray, next_state: np.ndarray, motif: np.ndarray, white_penalty: float = None) -> float:
    """
    Immediate reward with white penalty: reduction in penalized distance to motif.
    
    Args:
        prev_state: Canvas state before action
        next_state: Canvas state after action
        motif: Target motif image
        white_penalty: Penalty strength as ratio to L2 distance (scale-invariant)
        
    Returns:
        Reward value (higher is better)
    """
    if white_penalty is None:
        raise ValueError("white_penalty must be specified for compute_reward_with_white_penalty")
    
    return l2_distance_with_white_penalty(prev_state, motif, white_penalty=white_penalty) - l2_distance_with_white_penalty(next_state, motif, white_penalty=white_penalty)

def l2_distance_vectorized(a: np.ndarray, b_batch: np.ndarray) -> np.ndarray:
    """
    Vectorized L2 distance computation for batch processing.
    
    Args:
        a: Reference image (H, W, C)
        b_batch: Batch of images to compare (n_samples, H, W, C)
        
    Returns:
        distances: Array of L2 distances (n_samples,)
    """
    a_f = a.astype(float)
    b_batch_f = b_batch.astype(float)
    
    # Broadcast a to match batch dimensions: (1, H, W, C)
    a_expanded = a_f[np.newaxis, :, :, :]
    
    # Compute differences and L2 norms for each sample
    diff = b_batch_f - a_expanded
    # Sum over spatial and channel dimensions (H, W, C), keeping batch dimension
    distances = np.linalg.norm(diff.reshape(b_batch.shape[0], -1), axis=1)
    
    return distances

def compute_reward_vectorized(prev_canvas: np.ndarray, canvas_batch: np.ndarray, motif: np.ndarray) -> np.ndarray:
    """
    Vectorized reward computation for batch processing.
    
    Args:
        prev_canvas: Previous canvas state (H, W, C)
        canvas_batch: Batch of new canvas states (n_samples, H, W, C)
        motif: Target motif image (H, W, C)
        
    Returns:
        rewards: Array of rewards (n_samples,)
    """
    # Compute distances before and after for all samples
    prev_distances = l2_distance_vectorized(motif, np.tile(prev_canvas[np.newaxis, :, :, :], (canvas_batch.shape[0], 1, 1, 1)))
    new_distances = l2_distance_vectorized(motif, canvas_batch)
    
    # Reward is improvement (reduction in distance)
    rewards = prev_distances - new_distances
    
    return rewards

def white_ratio_vectorized(canvas_batch: np.ndarray) -> np.ndarray:
    """
    Calculate the ratio of pure white pixels for a batch of canvases.
    
    Args:
        canvas_batch: Batch of canvases (n_samples, H, W, C)
        
    Returns:
        ratios: Array of white pixel ratios (n_samples,)
    """
    if canvas_batch.ndim == 4 and canvas_batch.shape[3] == 3:
        # Check if all channels are 255 for each pixel
        white = np.all(canvas_batch == 255, axis=3)  # (n_samples, H, W)
    else:
        white = (canvas_batch == 255)
    
    # Sum white pixels for each sample and divide by total pixels
    ratios = np.sum(white, axis=(1, 2)) / (white.shape[1] * white.shape[2])
    
    return ratios

def compute_reward_with_white_penalty_vectorized(prev_canvas: np.ndarray, canvas_batch: np.ndarray, 
                                               motif: np.ndarray, white_penalty: float) -> np.ndarray:
    """
    Vectorized reward computation with white penalty for batch processing.
    
    Args:
        prev_canvas: Previous canvas state (H, W, C)
        canvas_batch: Batch of new canvas states (n_samples, H, W, C)
        motif: Target motif image (H, W, C)
        white_penalty: White penalty strength
        
    Returns:
        rewards: Array of rewards (n_samples,)
    """
    # Compute L2 distances
    prev_distances = l2_distance_vectorized(motif, np.tile(prev_canvas[np.newaxis, :, :, :], (canvas_batch.shape[0], 1, 1, 1)))
    new_distances = l2_distance_vectorized(motif, canvas_batch)
    
    # Apply white penalty to new distances
    white_ratios = white_ratio_vectorized(canvas_batch)
    penalty_multiplier = 1.0 + white_penalty * white_ratios
    penalized_new_distances = new_distances * penalty_multiplier
    
    # Reward is improvement
    rewards = prev_distances - penalized_new_distances
    
    return rewards



