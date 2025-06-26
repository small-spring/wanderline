import sys
import random
import math
import signal
import numpy as np
from wanderline.canvas import apply_stroke
from wanderline.agent import choose_next_angle
from wanderline.fast_agent import choose_angle_fast
from wanderline.memory_efficient_canvas import choose_angle_memory_efficient, choose_angle_ultra_fast
from wanderline.transformer_agent import TransformerAgent
from wanderline.reward import compute_reward, compute_reward_with_white_penalty, l2_distance, l2_distance_with_white_penalty
from wanderline.video_recorder import VideoRecorder
from wanderline.stroke_logger import StrokeLogger
from wanderline.plot_utils import plot_distances
from wanderline.realtime_visualizer import RealtimeVisualizer


class DrawingEngine:
    """
    Handles the main drawing loop and reward computation.
    """
    
    def __init__(self, args):
        self.args = args
        self.shutdown_requested = False
        
        # Setup signal handlers for graceful interruption
        self.setup_signal_handlers()
        
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
        
        # Initialize real-time visualizer
        viz_config = getattr(args, 'visualization', {})
        if isinstance(viz_config, dict):
            self.visualizer = RealtimeVisualizer(viz_config)
        else:
            # Default visualization config if not provided
            default_viz_config = {
                'enabled': getattr(args, 'realtime_viz', False),
                'mode': 'opencv',
                'update_frequency': 10,
                'save_snapshots': True,
                'snapshot_interval': 100
            }
            self.visualizer = RealtimeVisualizer(default_viz_config)
    
    def setup_signal_handlers(self):
        """Setup signal handlers for graceful interruption."""
        signal.signal(signal.SIGINT, self._handle_interrupt)
        signal.signal(signal.SIGTERM, self._handle_interrupt)
    
    def _handle_interrupt(self, signum, frame):
        """Handle interrupt signals (Ctrl+C, SIGTERM) gracefully."""
        print("\n🛑 Interrupt received - requesting graceful shutdown...")
        print("   Current progress will be saved. Please wait...")
        self.shutdown_requested = True
    
    def setup_video_recording(self, canvas, out_dir):
        """
        Set up video recording with appropriate frame rate.
        Returns: (recorder_or_logger, frames_to_record, video_path)
        """
        # Check if memory-efficient mode is enabled
        memory_efficient = getattr(self.args, 'memory_efficient', False)
        
        if memory_efficient:
            # Use stroke logger for memory-efficient recording
            stroke_logger = StrokeLogger(canvas, out_dir)
            print(f"Memory-efficient mode enabled. Logging strokes for {self.args.steps} steps.")
            
            # Still calculate frames_to_record for compatibility
            max_fps = 60
            total_source_frames = self.args.steps + 1
            desired_fps = total_source_frames / self.args.duration
            frames_to_record = set(range(self.args.steps))
            
            if desired_fps > max_fps:
                fps = max_fps
                num_target_frames = max(2, int(self.args.duration * fps))
                indices = np.linspace(0, self.args.steps - 1, num=num_target_frames - 1, dtype=int)
                frames_to_record = set(indices)
                print(f"High step count. Will reconstruct {len(frames_to_record)} frames for video.")
            else:
                fps = max(1, int(round(desired_fps)))
                print(f"Will reconstruct video at {fps} FPS after calculation.")
            
            return stroke_logger, frames_to_record, f"{out_dir}/drawing.mp4"
        
        else:
            # Use traditional video recording
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
    
    def run_drawing_loop(self, canvas, motif, current_start, initial_distances, recorder, frames_to_record, initial_canvas, out_dir=None, previous_total_steps=0):
        """
        Execute the main drawing loop.
        Returns: (final_canvas, distances, total_reward, current_start, converged, executed_steps)
        """
        # Initialize transformer agent if needed
        if self.agent_type == 'transformer' and self.agent is None:
            canvas_shape = (canvas.shape[0], canvas.shape[1])
            self.agent = TransformerAgent(canvas_shape, sequence_length=10, feature_dim=128)
            print("Initialized TransformerAgent")
        
        # Setup visualizer
        if out_dir:
            self.visualizer.setup_snapshot_dir(out_dir)
        self.visualizer.set_previous_total_steps(previous_total_steps)
        self.visualizer.set_total_steps(self.args.steps)
        
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
            # Check for graceful shutdown request
            if self.shutdown_requested:
                print(f"💾 Graceful shutdown requested at step {i+1}/{self.args.steps}")
                break
            
            try:
                if self.agent_type == 'transformer' and motif is not None:
                    angle = self.agent.choose_next_angle(prev_canvas, motif, current_start, stroke_length)
                elif self.args.greedy and motif is not None:
                    # Check which optimization mode to use (for 1-step lookahead only)
                    use_ultra_fast = (getattr(self.args, 'ultra_fast', False) and 
                                    getattr(self.args, 'lookahead_depth', 1) == 1)
                    use_fast_agent = (getattr(self.args, 'fast_agent', False) and 
                                    getattr(self.args, 'lookahead_depth', 1) == 1)
                    
                    if use_ultra_fast:
                        # Use ultra-fast memory-efficient agent (5-10x speedup)
                        angle, timing_info = choose_angle_ultra_fast(
                            prev_canvas, motif, current_start, stroke_length,
                            reward_type=self.args.reward_type,
                            white_penalty=getattr(self.args, 'white_penalty', None),
                            opacity=self.args.opacity,
                            line_width=self.args.line_width,
                            max_samples=getattr(self.args, 'n_samples', 16),
                            verbose=getattr(self.args, 'verbose', False)
                        )
                    elif use_fast_agent:
                        # Use optimized fast agent
                        angle, timing_info = choose_angle_fast(
                            prev_canvas, motif, current_start, stroke_length,
                            reward_type=self.args.reward_type,
                            white_penalty=getattr(self.args, 'white_penalty', None),
                            opacity=self.args.opacity,
                            line_width=self.args.line_width,
                            adaptive=True,
                            early_termination=True,
                            verbose=getattr(self.args, 'verbose', False)
                        )
                    else:
                        # Use standard greedy agent
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
            
            # Handle recording based on mode
            if isinstance(recorder, StrokeLogger):
                # Memory-efficient mode: log stroke data
                dist_current = self.distance_fn(next_canvas, motif) if motif is not None else None
                reward_current = self.reward_fn(prev_canvas, next_canvas, motif) if motif is not None else None
                recorder.log_stroke(
                    step=i,
                    angle=angle,
                    start_pt=current_start,
                    end_pt=end_pt,
                    opacity=self.args.opacity,
                    line_width=self.args.line_width,
                    distance=dist_current,
                    reward=reward_current
                )
            elif i in frames_to_record:
                # Traditional mode: record video frame
                recorder.record(next_canvas)
            
            if motif is not None:
                r = self.reward_fn(prev_canvas, next_canvas, motif)
                total_reward += r
                # Log reward and new distance
                dist_next = self.distance_fn(next_canvas, motif)
                if previous_total_steps > 0:
                    print(f"Step ({previous_total_steps} +) {i + 1}/{self.args.steps}: angle={angle:.2f}, reward={r:.2f}, distance={dist_next:.2f}")
                else:
                    print(f"Step {i + 1}/{self.args.steps}: angle={angle:.2f}, reward={r:.2f}, distance={dist_next:.2f}")
                distances.append(dist_next)
                
                # Update real-time visualizer
                self.visualizer.update(next_canvas, i + 1, dist_next, r, angle)
                
                # Early stopping check
                if patience_counter != -1:
                    if best_distance - dist_next > self.args.min_delta:
                        best_distance = dist_next
                        patience_counter = 0
                    else:
                        patience_counter += 1
                    if patience_counter >= self.args.patience:
                        if previous_total_steps > 0:
                            print(f"Stopping early at step ({previous_total_steps} +) {i+1} due to no improvement for {self.args.patience} steps.")
                        else:
                            print(f"Stopping early at step {i+1} due to no improvement for {self.args.patience} steps.")
                        converged = True
                        break
                
                # Check if visualizer window was closed
                if self.visualizer.is_window_closed():
                    print(f"Real-time window closed by user at step {i+1}")
                    break
            
            prev_canvas = next_canvas
            current_start = end_pt
            executed_steps = i + 1
        
        return prev_canvas, distances, total_reward, current_start, converged, executed_steps
    
    def finalize_outputs(self, distances, out_dir, video_path, recorder):
        """
        Finalize video recording and create plots.
        """
        if isinstance(recorder, StrokeLogger):
            # Memory-efficient mode: save stroke data and reconstruct video
            stroke_file = recorder.save_stroke_data()
            print(f"Stroke data saved to {stroke_file}")
            
            # Show memory usage comparison
            memory_stats = recorder.get_memory_usage_estimate()
            if memory_stats:
                print(f"Memory savings: {memory_stats['memory_savings_ratio']:.1f}x")
                print(f"Stroke data: {memory_stats['stroke_data_mb']:.2f} MB vs Video frames: {memory_stats['equivalent_video_mb']:.1f} MB")
            
            # Reconstruct video from stroke data
            print("Reconstructing video from stroke data...")
            
            # Calculate fps and frames to record for video reconstruction
            max_fps = 60
            total_source_frames = len(recorder.strokes) + 1
            desired_fps = total_source_frames / self.args.duration
            frames_to_record = None  # Use all frames by default
            
            if desired_fps > max_fps:
                fps = max_fps
                num_target_frames = max(2, int(self.args.duration * fps))
                indices = np.linspace(0, len(recorder.strokes) - 1, num=num_target_frames - 1, dtype=int)
                frames_to_record = set(indices)
            else:
                fps = max(1, int(round(desired_fps)))
            
            final_video_path = recorder.reconstruct_video(fps=fps, frames_to_record=frames_to_record)
            print(f"Drawing video saved as {final_video_path}")
            
        else:
            # Traditional mode: finalize video recorder
            recorder.release()
            print(f"Drawing video saved as {video_path}")
        
        # Plot distances if available
        if distances:
            plot_distances(distances, out_dir)
        
        # Close visualizer and show final stats
        self.visualizer.close()
