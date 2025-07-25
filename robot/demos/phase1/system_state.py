#!/usr/bin/env python3
"""
System State Manager for Phase 1 Robot Drawing

Provides centralized state management for all system components.
Single source of truth for robot state, canvas state, and drawing progress.
"""

import time
import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional


@dataclass
class ContactPoint:
    """Represents a point where pen contacted the canvas."""
    position_3d: Tuple[float, float, float]  # Robot coordinates
    position_2d: Tuple[int, int]             # Canvas pixel coordinates
    timestamp: float
    stroke_id: int
    contact_force: float = 0.0               # Future: force sensor integration


@dataclass
class CompletedStroke:
    """Represents a completed drawing stroke."""
    stroke_id: int
    start_position: Tuple[float, float, float]
    end_position: Tuple[float, float, float]
    contact_points: List[ContactPoint]
    timestamp: float


@dataclass
class CanvasConfig:
    """Canvas configuration parameters."""
    physical_size: float = 0.4               # 40cm canvas size
    pixel_width: int = 800                   # Pixel width
    pixel_height: int = 600                  # Pixel height
    position: Tuple[float, float, float] = (0.5, 0.0, 0.1)  # Robot base coordinates
    contact_height: float = 0.02             # 2cm realistic pen length


@dataclass
class TargetCircle:
    """Target circle drawing parameters."""
    center: Tuple[float, float]              # Pixel coordinates (400, 300)
    radius: float                            # Pixel radius (80)
    total_segments: int = 24                 # Number of line segments
    stroke_length: float = 0.02              # Max length per stroke (2cm)


@dataclass
class SystemState:
    """
    Central system state management class.
    Single source of truth for all system components.
    """
    # Robot state
    robot_joint_positions: Dict[str, float] = field(default_factory=dict)
    calculated_pen_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    pen_contact_state: bool = False
    
    # Canvas state  
    canvas_config: CanvasConfig = field(default_factory=CanvasConfig)
    drawn_strokes: List[CompletedStroke] = field(default_factory=list)
    current_canvas_image: Optional[np.ndarray] = None
    contact_history: List[ContactPoint] = field(default_factory=list)
    
    # Drawing session
    target_shape: Optional[TargetCircle] = None
    completion_percentage: float = 0.0
    current_stroke_id: int = 0
    
    # Timestamps and errors
    last_update_timestamp: float = field(default_factory=time.time)
    system_errors: List[str] = field(default_factory=list)
    
    def __post_init__(self):
        """Initialize canvas image if not provided."""
        if self.current_canvas_image is None:
            self.current_canvas_image = np.ones(
                (self.canvas_config.pixel_height, self.canvas_config.pixel_width, 3), 
                dtype=np.uint8
            ) * 255  # White canvas
    
    def update_robot_position(self, joint_positions: Dict[str, float], 
                            pen_position: Tuple[float, float, float]):
        """Update robot state with new positions."""
        self.robot_joint_positions = joint_positions.copy()
        self.calculated_pen_position = pen_position
        self.last_update_timestamp = time.time()
    
    def add_contact_point(self, contact_point: ContactPoint):
        """Add new contact point to history."""
        self.contact_history.append(contact_point)
        self.last_update_timestamp = time.time()
    
    def add_completed_stroke(self, stroke: CompletedStroke):
        """Add completed stroke to drawing history."""
        self.drawn_strokes.append(stroke)
        self.last_update_timestamp = time.time()
    
    def update_canvas_image(self, new_image: np.ndarray):
        """Update canvas image with new drawing state."""
        if new_image.shape == self.current_canvas_image.shape:
            self.current_canvas_image = new_image.copy()
            self.last_update_timestamp = time.time()
        else:
            error_msg = f"Canvas image shape mismatch: expected {self.current_canvas_image.shape}, got {new_image.shape}"
            self.system_errors.append(error_msg)
    
    def set_target_circle(self, center: Tuple[float, float], radius: float):
        """Set target circle for drawing."""
        self.target_shape = TargetCircle(center=center, radius=radius)
        self.last_update_timestamp = time.time()
    
    def update_completion_percentage(self, percentage: float):
        """Update drawing completion percentage."""
        self.completion_percentage = max(0.0, min(1.0, percentage))
        self.last_update_timestamp = time.time()
    
    def get_current_pen_position(self) -> Tuple[float, float, float]:
        """Get current pen position."""
        return self.calculated_pen_position
    
    def get_contact_count(self) -> int:
        """Get total number of contact points."""
        return len(self.contact_history)
    
    def get_stroke_count(self) -> int:
        """Get total number of completed strokes."""
        return len(self.drawn_strokes)
    
    def is_drawing_complete(self) -> bool:
        """Check if drawing is complete."""
        return self.completion_percentage >= 1.0
    
    def get_system_status(self) -> Dict:
        """Get comprehensive system status."""
        return {
            'pen_position': self.calculated_pen_position,
            'pen_contact': self.pen_contact_state,
            'contact_count': self.get_contact_count(),
            'stroke_count': self.get_stroke_count(),
            'completion': self.completion_percentage,
            'errors': len(self.system_errors),
            'last_update': self.last_update_timestamp
        }
    
    def clear_errors(self):
        """Clear system error log."""
        self.system_errors.clear()
        self.last_update_timestamp = time.time()


def create_default_system_state() -> SystemState:
    """Create system state with default configuration."""
    state = SystemState()
    
    # Set default target circle (center of canvas)
    state.set_target_circle(center=(400, 300), radius=80)
    
    # Initialize robot joint positions
    state.robot_joint_positions = {
        'shoulder_pan_joint': 0.0,
        'shoulder_lift_joint': -1.2,
        'elbow_joint': -1.0,
        'wrist_1_joint': -1.5,
        'wrist_2_joint': 1.57,
        'wrist_3_joint': 0.0
    }
    
    return state


if __name__ == "__main__":
    # Test basic functionality
    print("Testing SystemState class...")
    
    # Create default state
    state = create_default_system_state()
    print(f"Initial state: {state.get_system_status()}")
    
    # Test contact point addition
    contact = ContactPoint(
        position_3d=(0.5, 0.0, 0.105),
        position_2d=(400, 300),
        timestamp=time.time(),
        stroke_id=0
    )
    state.add_contact_point(contact)
    print(f"After adding contact: {state.get_system_status()}")
    
    # Test completion update
    state.update_completion_percentage(0.25)
    print(f"After 25% completion: {state.get_system_status()}")
    
    print("✅ SystemState test completed successfully!")