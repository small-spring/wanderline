"""
Real-time canvas visualization module for Wanderline.

Provides live display of drawing progress using various methods:
- OpenCV window display
- Periodic snapshot saving
- ASCII terminal display
- Progress monitoring with metrics
"""

import cv2
import numpy as np
import os
import time


class RealtimeVisualizer:
    """
    Real-time visualization manager for drawing progress.
    
    Features:
    - OpenCV live window display
    - Periodic snapshot saving
    - Progress metrics tracking
    - Non-blocking updates
    """
    
    def __init__(self, config: dict):
        """
        Initialize visualizer with configuration.
        
        Args:
            config: Visualization configuration dict with keys:
                - enabled: bool, enable visualization
                - mode: str, visualization mode ("opencv", "snapshots", "both")
                - update_frequency: int, update every N steps
                - window_size: tuple, OpenCV window size
                - save_snapshots: bool, save intermediate snapshots
                - snapshot_interval: int, save snapshot every N steps
        """
        self.config = config
        self.enabled = config.get('enabled', False)
        self.mode = config.get('mode', 'opencv')
        self.update_frequency = config.get('update_frequency', 10)
        self.default_window_size = tuple(config.get('window_size', [800, 600]))
        self.max_window_width = config.get('max_window_width', 1000)
        self.max_window_height = config.get('max_window_height', 800)
        self.window_size = self.default_window_size  # Will be updated dynamically
        self.save_snapshots = config.get('save_snapshots', False)
        self.snapshot_interval = config.get('snapshot_interval', 100)
        
        # State tracking
        self.step_count = 0
        self.previous_total_steps = 0
        self.total_steps = 0
        self.start_time = time.time()
        self.last_update_time = time.time()
        self.window_name = "Wanderline - Live Drawing"
        self.snapshot_dir = None
        self.output_dir = None
        
        # Performance metrics
        self.step_times = []
        self.distances = []
        self.rewards = []
        
        # OpenCV window state
        self.window_created = False
        self.window_closed = False
        self.show_overlay = True  # Text overlay display state (default ON)
        
        if not self.enabled:
            return
            
        # Initialize based on mode
        if self.mode in ['opencv', 'both']:
            self._init_opencv_window()
            
    def _init_opencv_window(self):
        """Initialize OpenCV window for live display."""
        try:
            # Create window with specific flags for better compatibility
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
            cv2.resizeWindow(self.window_name, *self.default_window_size)
            
            # Create and display initial placeholder image
            placeholder = np.ones((self.default_window_size[1], self.default_window_size[0], 3), dtype=np.uint8) * 50
            cv2.putText(placeholder, "Wanderline - Starting...", (50, self.default_window_size[1]//2), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(placeholder, "Press 'q' to quit", (50, self.default_window_size[1]//2 + 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
            cv2.putText(placeholder, "Window should be visible now", (50, self.default_window_size[1]//2 + 100), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 2)
            
            cv2.imshow(self.window_name, placeholder)
            
            # Force window to front and refresh
            cv2.setWindowProperty(self.window_name, cv2.WND_PROP_TOPMOST, 1)
            cv2.waitKey(500)  # Longer wait for initialization
            cv2.setWindowProperty(self.window_name, cv2.WND_PROP_TOPMOST, 0)
            
            self.window_created = True
            print(f"OpenCV live window initialized: {self.default_window_size}")
            print("If you cannot see the window, check if it's hidden behind other windows")
        except Exception as e:
            print(f"Failed to initialize OpenCV window: {e}")
            self.window_created = False
    
    def setup_snapshot_dir(self, output_dir: str):
        """Setup directory for saving snapshots."""
        self.output_dir = output_dir  # Store the main output directory
        if self.save_snapshots:
            self.snapshot_dir = os.path.join(output_dir, "live_snapshots")
            os.makedirs(self.snapshot_dir, exist_ok=True)
            print(f"Snapshot directory: {self.snapshot_dir}")
    
    def set_previous_total_steps(self, previous_steps: int):
        """Set the number of steps from previous runs for display purposes."""
        self.previous_total_steps = previous_steps
    
    def set_total_steps(self, total_steps: int):
        """Set the total number of steps for this run."""
        self.total_steps = total_steps
    
    def update(self, canvas: np.ndarray, step: int, distance: float = None, 
               reward: float = None, angle: float = None, force_update: bool = False):
        """
        Update visualization with current canvas state.
        
        Args:
            canvas: Current canvas state (H, W, 3) uint8 array
            step: Current step number
            distance: Current distance to target (optional)
            reward: Step reward (optional)
            angle: Selected angle (optional)
            force_update: Force update regardless of frequency
        """
        if not self.enabled:
            return
            
        self.step_count = step
        current_time = time.time()
        
        # Track performance metrics
        if self.last_update_time is not None:
            time_for_steps = current_time - self.last_update_time
            steps_in_interval = step - self.last_step if hasattr(self, 'last_step') else self.update_frequency
            if steps_in_interval > 0:
                avg_step_time = time_for_steps / steps_in_interval
                self.step_times.append(avg_step_time)
                # Keep only recent measurements for moving average
                if len(self.step_times) > 100:
                    self.step_times = self.step_times[-100:]
        
        self.last_step = step
        
        if distance is not None:
            self.distances.append(distance)
        if reward is not None:
            self.rewards.append(reward)
            
        # Update based on frequency or force
        should_update = force_update or (step % self.update_frequency == 0)
        
        if should_update:
            if self.mode in ['opencv', 'both'] and self.window_created and not self.window_closed:
                self._update_opencv_display(canvas, step, distance, reward, angle)
                
            if self.save_snapshots and (step % self.snapshot_interval == 0 or force_update):
                self._save_snapshot(canvas, step)
        
        self.last_update_time = current_time
    
    def _calculate_optimal_window_size(self, canvas_shape):
        """Calculate optimal window size that preserves aspect ratio within constraints."""
        h, w = canvas_shape[:2]
        aspect_ratio = w / h
        
        # Try fitting to max width first
        new_w = min(self.max_window_width, w)
        new_h = int(new_w / aspect_ratio)
        
        # If height exceeds limit, fit to max height instead
        if new_h > self.max_window_height:
            new_h = self.max_window_height
            new_w = int(new_h * aspect_ratio)
        
        return (new_w, new_h)
    
    def _update_opencv_display(self, canvas: np.ndarray, step: int, 
                              distance: float = None, reward: float = None, angle: float = None):
        """Update OpenCV window with current canvas and metrics."""
        try:
            # Create display canvas with info overlay
            display_canvas = self._create_info_overlay(canvas, step, distance, reward, angle)
            
            # Calculate and set optimal window size for this canvas
            optimal_size = self._calculate_optimal_window_size(display_canvas.shape)
            if optimal_size != self.window_size:
                self.window_size = optimal_size
                cv2.resizeWindow(self.window_name, *self.window_size)
                print(f"📐 Window resized to {self.window_size} (aspect ratio preserved)")
            
            # Resize canvas to fit window while preserving aspect ratio
            h, w = display_canvas.shape[:2]
            if (w, h) != self.window_size:
                display_canvas = cv2.resize(display_canvas, self.window_size)
            
            cv2.imshow(self.window_name, display_canvas)
            
            # Non-blocking key check (longer wait for better display)
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q') or key == 27:  # 'q' or ESC to quit
                self.window_closed = True
                cv2.destroyWindow(self.window_name)
            elif key == ord('s'):  # 's' to save current frame
                self._save_snapshot(canvas, step, manual=True)
            elif key == ord('h'):  # 'h' to toggle text overlay
                self.show_overlay = not self.show_overlay
                status = "shown" if self.show_overlay else "hidden"
                print(f"\n👁️  Text overlay {status}\n")
                
        except Exception as e:
            print(f"Error updating OpenCV display: {e}")
            self.window_closed = True
    
    def _create_info_overlay(self, canvas: np.ndarray, step: int,
                           distance: float = None, reward: float = None, angle: float = None) -> np.ndarray:
        """Create canvas with information overlay."""
        # Copy canvas to avoid modifying original
        display_canvas = canvas.copy()
        
        # Calculate metrics
        elapsed_time = time.time() - self.start_time
        avg_step_time = np.mean(self.step_times) if self.step_times else 0
        steps_per_sec = 1.0 / avg_step_time if avg_step_time > 0 else 0
        
        # Prepare info text
        info_lines = []
        
        # Add path display if overlay enabled and available
        if self.show_overlay and self.output_dir:
            info_lines.append(f"Path: {self.output_dir}")
        
        # Add step info if overlay enabled
        if self.show_overlay:
            info_lines.extend([
                f"Step: {step}/{self.total_steps}" if self.total_steps > 0 else f"Step: {step}",
                f"Time: {elapsed_time:.1f}s",
                f"Speed: {steps_per_sec:.1f} steps/s"
            ])
        
        # Add metrics if overlay enabled
        if self.show_overlay:
            if distance is not None:
                info_lines.append(f"Distance: {distance:.2f}")
            if reward is not None:
                info_lines.append(f"Reward: {reward:.2f}")
            if angle is not None:
                info_lines.append(f"Angle: {angle:.2f}")
                
            # Add improvement metrics
            if len(self.distances) > 1:
                improvement = self.distances[0] - self.distances[-1]
                improvement_pct = (improvement / self.distances[0]) * 100 if self.distances[0] > 0 else 0
                info_lines.append(f"Improved: {improvement:.2f} ({improvement_pct:.1f}%)")
        
        # Draw info overlay (top-left corner)
        y_offset = 30
        for i, line in enumerate(info_lines):
            y_pos = y_offset + i * 25
            # Black outline for better visibility
            cv2.putText(display_canvas, line, (11, y_pos + 1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
            # White text
            cv2.putText(display_canvas, line, (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # Add controls hint at bottom if overlay enabled
        if self.show_overlay:
            h, w = display_canvas.shape[:2]
            controls_text = "Controls: 'q' = quit, 's' = save, 'h' = hide/show overlay"
            cv2.putText(display_canvas, controls_text, (11, h - 19), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            cv2.putText(display_canvas, controls_text, (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return display_canvas
    
    def _save_snapshot(self, canvas: np.ndarray, step: int, manual: bool = False):
        """Save current canvas as snapshot."""
        if not self.snapshot_dir:
            return
            
        try:
            suffix = "_manual" if manual else ""
            filename = f"step_{step:06d}{suffix}.png"
            filepath = os.path.join(self.snapshot_dir, filename)
            cv2.imwrite(filepath, canvas)
            if manual:
                print(f"Manual snapshot saved: {filename}")
        except Exception as e:
            print(f"Error saving snapshot: {e}")
    
    def is_window_closed(self) -> bool:
        """Check if OpenCV window was closed by user."""
        return self.window_closed
    
    def get_performance_stats(self) -> dict:
        """Get current performance statistics."""
        elapsed_time = time.time() - self.start_time
        avg_step_time = np.mean(self.step_times) if self.step_times else 0
        steps_per_sec = 1.0 / avg_step_time if avg_step_time > 0 else 0
        
        stats = {
            'elapsed_time': elapsed_time,
            'step_count': self.step_count,
            'avg_step_time': avg_step_time,
            'steps_per_sec': steps_per_sec,
            'total_distances': len(self.distances),
            'total_rewards': len(self.rewards)
        }
        
        if self.distances:
            stats['initial_distance'] = self.distances[0]
            stats['current_distance'] = self.distances[-1]
            stats['best_distance'] = min(self.distances)
            stats['improvement'] = self.distances[0] - self.distances[-1]
            stats['improvement_pct'] = (stats['improvement'] / self.distances[0]) * 100 if self.distances[0] > 0 else 0
            
        return stats
    
    def close(self):
        """Clean up resources."""
        if self.window_created and not self.window_closed:
            cv2.destroyWindow(self.window_name)
            self.window_closed = True
        
        # Print final statistics
        if self.enabled:
            stats = self.get_performance_stats()
            print("\n=== Drawing Session Statistics ===")
            print(f"Total time: {stats['elapsed_time']:.1f}s")
            print(f"Total steps: {stats['step_count']}")
            print(f"Average speed: {stats['steps_per_sec']:.1f} steps/s")
            
            if 'improvement_pct' in stats:
                print(f"Distance improvement: {stats['improvement']:.2f} ({stats['improvement_pct']:.1f}%)")
                print(f"Initial distance: {stats['initial_distance']:.2f}")
                print(f"Final distance: {stats['current_distance']:.2f}")
                print(f"Best distance: {stats['best_distance']:.2f}")


class ASCIIVisualizer:
    """
    ASCII art visualization for terminal-only environments.
    """
    
    def __init__(self, width: int = 60, height: int = 30):
        """
        Initialize ASCII visualizer.
        
        Args:
            width: ASCII art width in characters
            height: ASCII art height in characters
        """
        self.width = width
        self.height = height
        self.chars = " .:-=+*#%@"  # Grayscale to character mapping
        
    def canvas_to_ascii(self, canvas: np.ndarray) -> str:
        """
        Convert canvas to ASCII art string.
        
        Args:
            canvas: Canvas array (H, W) or (H, W, 3)
            
        Returns:
            ASCII art string
        """
        # Convert to grayscale if needed
        if len(canvas.shape) == 3:
            gray = cv2.cvtColor(canvas, cv2.COLOR_BGR2GRAY)
        else:
            gray = canvas.copy()
            
        # Resize to ASCII dimensions
        resized = cv2.resize(gray, (self.width, self.height))
        
        # Normalize to 0-255 range
        normalized = cv2.normalize(resized, None, 0, 255, cv2.NORM_MINMAX)
        
        # Convert to ASCII
        ascii_lines = []
        for row in normalized:
            line = ""
            for pixel in row:
                char_idx = int((pixel / 255.0) * (len(self.chars) - 1))
                line += self.chars[char_idx]
            ascii_lines.append(line)
            
        return "\n".join(ascii_lines)
    
    def print_canvas(self, canvas: np.ndarray, step: int, distance: float = None):
        """Print canvas as ASCII art with step information."""
        ascii_art = self.canvas_to_ascii(canvas)
        
        # Clear terminal (works on most systems)
        print("\033[2J\033[H", end="")
        
        print(f"Step: {step}")
        if distance is not None:
            print(f"Distance: {distance:.2f}")
        print("-" * self.width)
        print(ascii_art)
        print("-" * self.width)
