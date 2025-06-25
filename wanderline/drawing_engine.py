import sys
import random
import math
import numpy as np
from wanderline.canvas import apply_stroke
from wanderline.agent import choose_next_angle
from wanderline.transformer_agent import TransformerAgent
from wanderline.reward import compute_reward, compute_reward_with_white_penalty, l2_distance, l2_distance_with_white_penalty
from wanderline.video_recorder import VideoRecorder
from wanderline.plot_utils import plot_distances


class DrawingEngine:
    """
    Handles the main drawing loop and reward computation.
    """
    
    def __init__(self, args):
        self.args = args
        
        # Determine white penalty parameters
        white_penalty = getattr(args, 'white_penalty', None)
        
        self.reward_fns = {
            'l2': compute_reward,
            'l2_white_penalty': lambda prev, next_, motif: compute_reward_with_white_penalty(
                prev, next_, motif, white_penalty=white_penalty)
        }
        self.distance_fns = {
            'l2': l2_distance,
            'l2_white_penalty': lambda a, b: l2_distance_with_white_penalty(
                a, b, white_penalty=white_penalty)
        }
        self.reward_fn = self.reward_fns[args.reward_type]
        self.distance_fn = self.distance_fns[args.reward_type]
        
        # Initialize agent based on type
        if hasattr(args, 'agent_type') and args.agent_type == 'transformer':
            # Canvas shape will be determined later when we have the actual canvas
            self.agent = None
            self.agent_type = 'transformer'
        else:
            # Use existing greedy behavior for backward compatibility
            self.agent_type = 'greedy'
    
    def setup_video_recording(self, canvas, out_dir):
        """
        Set up video recording with appropriate frame rate.
        Returns: (recorder, frames_to_record)
        """
        frame_size = (canvas.shape[1], canvas.shape[0])
        video_path = f"{out_dir}/drawing.mp4"
        
        # Compute fps to fit desired duration, with frame dropping for high step counts
        max_fps = 60
        total_source_frames = self.args.steps + 1
        desired_fps = total_source_frames / self.args.duration
        frames_to_record = set(range(self.args.steps))
        
        if desired_fps > max_fps:
            fps = max_fps
            num_target_frames = max(2, int(self.args.duration * fps))
            indices = np.linspace(0, self.args.steps - 1, num=num_target_frames - 1, dtype=int)
            frames_to_record = set(indices)
            print(f"High step count. Capping FPS at {max_fps} and selecting {len(frames_to_record)} frames to record.")
        else:
            fps = max(1, int(round(desired_fps)))
            print(f"Computed fps: {desired_fps:.2f}, using integer fps: {fps}")
        
        recorder = VideoRecorder(video_path, frame_size, fps=fps)
        recorder.record(canvas)
        
        return recorder, frames_to_record, video_path
    
    def run_drawing_loop(self, canvas, motif, current_start, initial_distances, recorder, frames_to_record, initial_canvas):
        """
        Execute the main drawing loop.
        Returns: (final_canvas, distances, total_reward, current_start, converged, executed_steps)
        """
        # Initialize transformer agent if needed
        if self.agent_type == 'transformer' and self.agent is None:
            canvas_shape = (canvas.shape[0], canvas.shape[1])
            self.agent = TransformerAgent(canvas_shape, sequence_length=10, feature_dim=128)
            print("Initialized TransformerAgent")
        
        # Compute stroke length based on ratio
        h, w = canvas.shape[:2]
        stroke_length = int(min(w, h) * self.args.ratio)
        
        # Track loss (distance) over frames
        distances = initial_distances.copy()
        best_distance = min(distances) if distances else math.inf
        patience_counter = 0
        converged = False
        if self.args.patience <= 0:
            patience_counter = -1  # a sentinel to disable early stopping
        
        # Initialize distance tracking
        if motif is not None:
            if not self.args.resume_from:
                init_dist = self.distance_fn(initial_canvas, motif)
                print(f"Initial distance: {init_dist:.2f}")
                distances.append(init_dist)
                best_distance = init_dist
            elif distances:
                print(f"Resumed with distance: {distances[-1]:.2f}")
        
        # Apply strokes and compute rewards
        total_reward = 0.0
        prev_canvas = canvas
        executed_steps = 0
        
        for i in range(self.args.steps):
            try:
                if self.agent_type == 'transformer' and motif is not None:
                    angle = self.agent.choose_next_angle(prev_canvas, motif, current_start, stroke_length)
                elif self.args.greedy and motif is not None:
                    # Auto-select n_samples based on lookahead_depth if not specified
                    n_samples = getattr(self.args, 'n_samples', None)
                    if n_samples is None:
                        if self.args.lookahead_depth == 1:
                            n_samples = 36  # Fast 1-step lookahead
                        elif self.args.lookahead_depth == 2:
                            n_samples = 12  # Practical 2-step lookahead
                        else:
                            n_samples = 8   # Conservative for 3+ steps
                    
                    angle = choose_next_angle(
                        prev_canvas, motif, current_start, stroke_length, 
                        n_samples=n_samples,
                        lookahead_depth=self.args.lookahead_depth,
                        reward_type=self.args.reward_type,
                        white_penalty=getattr(self.args, 'white_penalty', None),
                        opacity=self.args.opacity,
                        line_width=self.args.line_width,
                        use_vectorized=True,
                        use_full_vectorization=True,
                        verbose=getattr(self.args, 'verbose', False)
                    )
                else:
                    angle = random.uniform(0, 2 * math.pi)
                next_canvas, end_pt = apply_stroke(prev_canvas, angle, start=current_start, length=stroke_length, 
                                                 opacity=self.args.opacity, return_end=True, line_width=self.args.line_width)
            except Exception as e:
                print(f"Error during stroke application at step {i+1}: {e}", file=sys.stderr)
                sys.exit(1)
            
            if i in frames_to_record:
                recorder.record(next_canvas)
            
            if motif is not None:
                r = self.reward_fn(prev_canvas, next_canvas, motif)
                total_reward += r
                # Log reward and new distance
                dist_next = self.distance_fn(next_canvas, motif)
                print(f"Step {i + 1}/{self.args.steps}: angle={angle:.2f}, reward={r:.2f}, distance={dist_next:.2f}")
                distances.append(dist_next)
                
                # Early stopping check
                if patience_counter != -1:
                    if best_distance - dist_next > self.args.min_delta:
                        best_distance = dist_next
                        patience_counter = 0
                    else:
                        patience_counter += 1
                    if patience_counter >= self.args.patience:
                        print(f"Stopping early at step {i+1} due to no improvement for {self.args.patience} steps.")
                        converged = True
                        break
            
            prev_canvas = next_canvas
            current_start = end_pt
            executed_steps = i + 1
        
        return prev_canvas, distances, total_reward, current_start, converged, executed_steps
    
    def finalize_outputs(self, distances, out_dir, video_path, recorder):
        """
        Finalize video recording and create plots.
        """
        # Finalize and save video
        recorder.release()
        print(f"Drawing video saved as {video_path}")
        
        # Plot distances if available
        if distances:
            plot_distances(distances, out_dir)
