#!/usr/bin/env python3
"""
Coordinate Calculator for Phase 1 Robot Drawing

Module 1: Calculate where to draw next based on current canvas state.
Implements circle drawing algorithm with contact-based progress tracking.
"""

import math
import time
from typing import Tuple, Optional, List
from system_state import SystemState, TargetCircle, ContactPoint


class CircleCoordinateCalculator:
    """
    Calculate next coordinate for circle drawing.
    Uses contact-based progress tracking for accurate drawing.
    """
    
    def __init__(self, config: dict):
        """Initialize calculator with configuration."""
        self.segments = config.get('segments', 24)  # 24 segments = smooth circle
        self.max_stroke_length = config.get('max_stroke_length', 0.005)  # 5mm max
        self.closure_threshold = config.get('closure_threshold', 0.95)  # 95% completion
        self.perimeter_tolerance = config.get('perimeter_tolerance', 5.0)  # 5 pixels
        
    def calculate_next_coordinate(self, current_position: Tuple[float, float], 
                                 canvas_state: SystemState) -> Tuple[Optional[Tuple[float, float]], bool]:
        """
        Calculate next coordinate for circle drawing.
        
        Args:
            current_position: Current pen position (pixel coordinates)
            canvas_state: Current system state
            
        Returns:
            Tuple of (next_coordinate, drawing_complete)
            next_coordinate: Next pixel coordinate to draw, or None if complete
            drawing_complete: True if drawing is finished
        """
        if not canvas_state.target_shape:
            return None, True
            
        # Calculate current progress based on contact points
        progress = self._calculate_circle_progress(canvas_state)
        
        # Check if drawing is complete
        if progress >= self.closure_threshold:
            # Return to start point to close circle
            start_point = self._get_circle_start_point(canvas_state.target_shape)
            return start_point, True
            
        # Calculate next point on circle
        next_point = self._get_next_circle_point(progress, canvas_state.target_shape, current_position)
        
        return next_point, False
    
    def _calculate_circle_progress(self, canvas_state: SystemState) -> float:
        """
        Calculate drawing progress based on actual contact points.
        
        Args:
            canvas_state: Current system state
            
        Returns:
            Progress percentage (0.0 to 1.0)
        """
        if not canvas_state.contact_history or not canvas_state.target_shape:
            return 0.0
            
        # Calculate progress based on actual angle coverage of the circle
        if len(canvas_state.contact_history) == 0:
            return 0.0
            
        # Get all angles of contact points relative to circle center
        center_x, center_y = canvas_state.target_shape.center
        angles = []
        
        for contact in canvas_state.contact_history:
            pixel_x, pixel_y = contact.position_2d
            # Calculate angle from center
            angle = math.atan2(pixel_y - center_y, pixel_x - center_x)
            # Normalize to 0-2π range
            if angle < 0:
                angle += 2 * math.pi
            angles.append(angle)
        
        if len(angles) == 0:
            return 0.0
            
        # Calculate angular coverage
        angles.sort()
        
        # Debug: Log angle distribution every 100 contacts
        if len(angles) % 100 == 0:
            print(f"🔍 ANGLE DEBUG: {len(angles)} contacts, angles: min={min(angles):.3f}, max={max(angles):.3f}")
            print(f"   First 5 angles: {[f'{a:.3f}' for a in angles[:5]]}")
        
        # Simple approach: check if we have points in different quadrants
        angle_ranges = [False, False, False, False]  # 4 quadrants
        for angle in angles:
            quadrant = int(angle / (math.pi / 2))
            if quadrant < 4:
                angle_ranges[quadrant] = True
                
        # Progress based on quadrant coverage
        quadrants_covered = sum(angle_ranges)
        angle_progress = quadrants_covered / 4.0
        
        # Also consider contact density (minimum contacts needed)
        contact_progress = min(len(canvas_state.contact_history) / 24, 1.0)  # 24 points for smooth circle
        
        # Debug: Log quadrant coverage
        if len(angles) % 100 == 0:
            print(f"   Quadrants covered: {angle_ranges} = {quadrants_covered}/4 ({angle_progress:.3f})")
            print(f"   Contact progress: {contact_progress:.3f}, Final: {min(angle_progress, contact_progress):.3f}")
        
        # Use the minimum of both (ensures both angle coverage AND sufficient density)
        final_progress = min(angle_progress, contact_progress)
        
        # Debug: Log progress calculation details
        if len(canvas_state.contact_history) % 10 == 0:  # Log every 10 contacts
            print(f"🔍 PROGRESS DEBUG: Contacts={len(canvas_state.contact_history)} | "
                  f"QuadrantsCovered={quadrants_covered}/4 ({angle_progress:.3f}) | "
                  f"ContactProgress={contact_progress:.3f} | Final={final_progress:.3f}")
        
        return final_progress
    
    def _get_next_circle_point(self, current_progress: float, target_circle: TargetCircle, 
                              current_position: Tuple[float, float]) -> Tuple[float, float]:
        """
        Calculate next point on circle circumference.
        
        Args:
            current_progress: Current drawing progress (0.0 to 1.0)
            target_circle: Target circle parameters
            current_position: Current pen position
            
        Returns:
            Next point coordinates (pixel coordinates)
        """
        # Simple incremental approach: advance by a small step in angle
        # This ensures continuous progression around the circle
        segment_angle = 2 * math.pi / self.segments
        
        # Calculate current angle based on current position
        current_x, current_y = current_position
        center_x, center_y = target_circle.center
        current_angle = math.atan2(current_y - center_y, current_x - center_x)
        
        # Advance to next angle increment - proper step size for smooth circle
        next_angle = current_angle + segment_angle / 2  # Normal increment for smooth circle
        
        # Calculate next point on circumference
        next_x = target_circle.center[0] + target_circle.radius * math.cos(next_angle)
        next_y = target_circle.center[1] + target_circle.radius * math.sin(next_angle)
        
        # Debug: Log circle movement (every 50 calls)
        if not hasattr(self, '_movement_debug_counter'):
            self._movement_debug_counter = 0
        self._movement_debug_counter += 1
        
        if self._movement_debug_counter % 50 == 0:
            print(f"🎯 CIRCLE MOVE: Current({current_x:.1f},{current_y:.1f}) → "
                  f"Next({next_x:.1f},{next_y:.1f}) | Angle: {math.degrees(current_angle):.1f}° → {math.degrees(next_angle):.1f}°")
        
        # Ensure stroke length doesn't exceed maximum (5 pixels max step)
        distance = math.sqrt((next_x - current_x)**2 + (next_y - current_y)**2)
        max_pixel_step = 5.0  # 5 pixels maximum step
        
        if distance > max_pixel_step:
            # Interpolate to maximum stroke length
            ratio = max_pixel_step / distance
            next_x = current_x + (next_x - current_x) * ratio
            next_y = current_y + (next_y - current_y) * ratio
            
        return next_x, next_y
    
    def _get_circle_start_point(self, target_circle: TargetCircle) -> Tuple[float, float]:
        """Get starting point of circle (rightmost point)."""
        start_x = target_circle.center[0] + target_circle.radius
        start_y = target_circle.center[1]
        return start_x, start_y
    
    def _is_point_on_circle(self, contact_point: ContactPoint, target_circle: TargetCircle) -> bool:
        """Check if contact point is on target circle perimeter."""
        pixel_x, pixel_y = contact_point.position_2d
        center_x, center_y = target_circle.center
        
        # Calculate distance from center
        distance = math.sqrt((pixel_x - center_x)**2 + (pixel_y - center_y)**2)
        
        # Check if within tolerance
        return abs(distance - target_circle.radius) <= self.perimeter_tolerance
    
    def _calculate_angle_from_center(self, x: float, y: float, target_circle: TargetCircle) -> float:
        """Calculate angle of point relative to circle center."""
        center_x, center_y = target_circle.center
        return math.atan2(y - center_y, x - center_x)
    
    def _calculate_angular_coverage(self, angles: List[float]) -> float:
        """Calculate total angular coverage from sorted angle list."""
        if len(angles) < 2:
            return 0.0
            
        total_coverage = 0.0
        for i in range(len(angles) - 1):
            gap = angles[i + 1] - angles[i]
            if gap > math.pi:  # Handle wrap-around
                gap = 2 * math.pi - gap
            total_coverage += gap
            
        return total_coverage


def create_coordinate_calculator(config: dict) -> CircleCoordinateCalculator:
    """Create coordinate calculator with configuration."""
    return CircleCoordinateCalculator(config)


if __name__ == "__main__":
    # Test basic functionality
    print("Testing CircleCoordinateCalculator...")
    
    # Import system state for testing
    from system_state import create_default_system_state, ContactPoint
    
    # Create calculator with default config
    config = {
        'segments': 24,
        'max_stroke_length': 0.005,
        'closure_threshold': 0.95,
        'perimeter_tolerance': 5.0
    }
    calculator = create_coordinate_calculator(config)
    
    # Create test state
    state = create_default_system_state()
    current_pos = (400, 300)  # Start at center
    
    # Test initial calculation
    next_coord, complete = calculator.calculate_next_coordinate(current_pos, state)
    print(f"Initial calculation: next_coord={next_coord}, complete={complete}")
    
    # Add some contact points to simulate drawing
    for i in range(5):
        angle = i * (2 * math.pi / 24)  # 5 points on circle
        x = 400 + 80 * math.cos(angle)
        y = 300 + 80 * math.sin(angle)
        
        contact = ContactPoint(
            position_3d=(0.5, 0.0, 0.105),
            position_2d=(int(x), int(y)),
            timestamp=time.time(),
            stroke_id=i
        )
        state.add_contact_point(contact)
    
    # Test progress calculation
    progress = calculator._calculate_circle_progress(state)
    print(f"Progress after 5 contact points: {progress:.2f}")
    
    # Test next coordinate with progress
    next_coord, complete = calculator.calculate_next_coordinate(current_pos, state)
    print(f"Next coordinate with progress: next_coord={next_coord}, complete={complete}")
    
    print("✅ CircleCoordinateCalculator test completed successfully!")