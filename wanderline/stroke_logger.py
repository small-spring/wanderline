"""
Lightweight stroke logging system for memory-efficient drawing.
Saves only stroke coordinates and reconstructs video after calculation.
"""

import json
import numpy as np
from typing import List, Dict, Any, Tuple
from dataclasses import dataclass, asdict
from wanderline.canvas import apply_stroke
from wanderline.video_recorder import VideoRecorder


@dataclass
class StrokeData:
    """Minimal data needed to reconstruct a stroke."""
    step: int
    angle: float
    start_pt: Tuple[int, int]
    end_pt: Tuple[int, int]
    opacity: float
    line_width: int
    distance: float = None  # Optional: current distance to motif
    reward: float = None    # Optional: reward for this stroke


class StrokeLogger:
    """
    Logs stroke data instead of video frames for memory efficiency.
    Enables post-calculation video reconstruction for long runs.
    """
    
    def __init__(self, initial_canvas: np.ndarray, output_dir: str):
        self.initial_canvas = initial_canvas.copy()
        self.output_dir = output_dir
        self.strokes: List[StrokeData] = []
        self.canvas_shape = initial_canvas.shape
        
    def log_stroke(self, step: int, angle: float, start_pt: Tuple[int, int], 
                   end_pt: Tuple[int, int], opacity: float, line_width: int,
                   distance: float = None, reward: float = None):
        """Log a single stroke's data."""
        stroke = StrokeData(
            step=step,
            angle=angle,
            start_pt=start_pt,
            end_pt=end_pt,
            opacity=opacity,
            line_width=line_width,
            distance=distance,
            reward=reward
        )
        self.strokes.append(stroke)
    
    def save_stroke_data(self) -> str:
        """Save stroke data to JSON file."""
        stroke_file = f"{self.output_dir}/stroke_data.json"
        
        # Convert to serializable format
        stroke_data = {
            'canvas_shape': list(self.canvas_shape),
            'total_strokes': len(self.strokes),
            'strokes': [asdict(stroke) for stroke in self.strokes]
        }
        
        with open(stroke_file, 'w') as f:
            json.dump(stroke_data, f, indent=2)
            
        return stroke_file
    
    def reconstruct_video(self, fps: int = 30, frames_to_record: set = None) -> str:
        """
        Reconstruct video from logged stroke data.
        
        Args:
            fps: Target frames per second
            frames_to_record: Set of step indices to include (None = all steps)
            
        Returns:
            Path to generated video file
        """
        if frames_to_record is None:
            frames_to_record = set(range(len(self.strokes)))
        
        # Setup video recorder
        video_path = f"{self.output_dir}/drawing.mp4"
        frame_size = (self.canvas_shape[1], self.canvas_shape[0])
        recorder = VideoRecorder(video_path, frame_size, fps=fps)
        
        # Start with initial canvas
        current_canvas = self.initial_canvas.copy()
        recorder.record(current_canvas)
        
        print(f"Reconstructing video from {len(self.strokes)} strokes...")
        
        # Replay each stroke
        for i, stroke in enumerate(self.strokes):
            if stroke.step in frames_to_record:
                # Calculate stroke length from start and end points
                start_x, start_y = stroke.start_pt
                end_x, end_y = stroke.end_pt
                stroke_length = int(np.sqrt((end_x - start_x)**2 + (end_y - start_y)**2))
                
                # Apply stroke to get next canvas state
                current_canvas = apply_stroke(
                    current_canvas,
                    stroke.angle,
                    start=stroke.start_pt,
                    length=stroke_length,
                    opacity=stroke.opacity,
                    line_width=stroke.line_width
                )
                recorder.record(current_canvas)
                
                if (i + 1) % 1000 == 0:
                    print(f"Reconstructed {i + 1}/{len(self.strokes)} frames")
        
        recorder.release()
        print(f"Video reconstruction complete: {video_path}")
        return video_path
    
    def get_memory_usage_estimate(self) -> Dict[str, float]:
        """Estimate memory usage comparison."""
        if not self.strokes:
            return {}
            
        # Estimate stroke data size (in MB)
        stroke_size_bytes = len(json.dumps(asdict(self.strokes[0])))
        total_stroke_mb = (stroke_size_bytes * len(self.strokes)) / (1024 * 1024)
        
        # Estimate equivalent video frame size (in MB)
        frame_size_mb = (np.prod(self.canvas_shape) * 3) / (1024 * 1024)  # RGB
        total_frame_mb = frame_size_mb * len(self.strokes)
        
        return {
            'stroke_data_mb': total_stroke_mb,
            'equivalent_video_mb': total_frame_mb,
            'memory_savings_ratio': total_frame_mb / total_stroke_mb if total_stroke_mb > 0 else 0
        }


def load_stroke_data(stroke_file: str) -> Tuple[List[StrokeData], Tuple[int, int, int]]:
    """Load stroke data from JSON file."""
    with open(stroke_file, 'r') as f:
        data = json.load(f)
    
    canvas_shape = tuple(data['canvas_shape'])
    strokes = [StrokeData(**stroke) for stroke in data['strokes']]
    
    return strokes, canvas_shape