#!/usr/bin/env python3
"""
Canvas Coordinate System for Robot Drawing

This module provides coordinate transformation between 2D pixel coordinates
and 3D robot coordinates for canvas drawing operations.

Key Features:
- Converts pixel coordinates (0-800, 0-600) to robot coordinates
- Handles pen up/down Z-axis positioning
- Generates stroke data for geometric shapes
- Validates coordinate boundaries
- Supports circle drawing with customizable resolution

Physical Specifications:
- Canvas size: 40cm × 40cm (matches drawing_canvas.sdf)
- Canvas position: [0.5, 0.0, 0.1] in robot base coordinates
- Pixel resolution: 800×600 (matches Wanderline)
- Scale: ~0.5mm per pixel

Usage:
    canvas = CanvasCoordinateSystem()
    robot_coords = canvas.pixel_to_robot_coords(400, 300, pen_down=True)
    circle_data = canvas.generate_circle_stroke_data(400, 300, 50)
"""

import math
from typing import Tuple, List, Dict

# Constants for better maintainability
CANVAS_PHYSICAL_SIZE = 0.4  # 40cm in meters
CANVAS_PIXEL_WIDTH = 800
CANVAS_PIXEL_HEIGHT = 600
CANVAS_POSITION = [0.5, 0.0, 0.1]  # Robot base coordinates
PEN_CONTACT_HEIGHT = 0.005  # 5mm contact depth
PEN_SAFE_HEIGHT = 0.05      # 5cm safety clearance


class CanvasCoordinateSystem:
    """
    Canvas coordinate system management class.
    
    Handles conversion between 2D pixel coordinates and 3D robot coordinates
    for drawing operations on a physical canvas.
    """
    
    def __init__(self):
        """Initialize canvas coordinate system with default parameters."""
        # Canvas physical specifications (matches drawing_canvas.sdf)
        self.canvas_width = CANVAS_PHYSICAL_SIZE
        self.canvas_height = CANVAS_PHYSICAL_SIZE
        self.canvas_position = CANVAS_POSITION.copy()
        
        # Digital resolution (matches Wanderline)
        self.pixel_width = CANVAS_PIXEL_WIDTH
        self.pixel_height = CANVAS_PIXEL_HEIGHT
        
        # Coordinate transformation scale
        self.scale_x = self.canvas_width / self.pixel_width
        self.scale_y = self.canvas_height / self.pixel_height
        
        # Drawing state management
        self.drawn_strokes = []
        self.current_position = [0.0, 0.0, 0.15]  # Initial position (pen up)
    
    def pixel_to_robot_coords(self, pixel_x: float, pixel_y: float, pen_down: bool = True) -> Tuple[float, float, float]:
        """
        Convert pixel coordinates to robot coordinate system.
        
        Args:
            pixel_x: X coordinate on canvas (0-800)
            pixel_y: Y coordinate on canvas (0-600)
            pen_down: Whether to lower pen (True for canvas contact)
        
        Returns:
            Tuple[float, float, float]: (robot_x, robot_y, robot_z) in meters
            
        Example:
            >>> canvas = CanvasCoordinateSystem()
            >>> robot_coords = canvas.pixel_to_robot_coords(400, 300, pen_down=True)
            >>> print(robot_coords)
            (0.5, 0.0, 0.105)
        """
        # Convert pixel coordinates to canvas-relative coordinates
        # Canvas center (400, 300) maps to (0, 0) in canvas space
        canvas_x = (pixel_x - self.pixel_width / 2) * self.scale_x
        canvas_y = (pixel_y - self.pixel_height / 2) * self.scale_y
        
        # Transform to robot coordinate system
        robot_x = self.canvas_position[0] + canvas_x
        robot_y = self.canvas_position[1] + canvas_y
        robot_z = self.canvas_position[2] + (PEN_CONTACT_HEIGHT if pen_down else PEN_SAFE_HEIGHT)
        
        return robot_x, robot_y, robot_z
    
    def robot_to_pixel_coords(self, robot_x: float, robot_y: float) -> Tuple[float, float]:
        """
        Convert robot coordinate system to pixel coordinates (inverse transformation).
        
        Args:
            robot_x: Robot X coordinate in meters
            robot_y: Robot Y coordinate in meters
            
        Returns:
            Tuple[float, float]: (pixel_x, pixel_y) on canvas
            
        Example:
            >>> canvas = CanvasCoordinateSystem()
            >>> pixel_coords = canvas.robot_to_pixel_coords(0.5, 0.0)
            >>> print(pixel_coords)
            (400.0, 300.0)
        """
        # Transform from robot to canvas coordinate system
        canvas_x = robot_x - self.canvas_position[0]
        canvas_y = robot_y - self.canvas_position[1]
        
        # Convert canvas coordinates to pixel coordinates
        pixel_x = canvas_x / self.scale_x + self.pixel_width / 2
        pixel_y = canvas_y / self.scale_y + self.pixel_height / 2
        
        return pixel_x, pixel_y
    
    def generate_circle_stroke_data(self, center_x: float, center_y: float, radius: float, num_points: int = 24) -> List[Dict]:
        """
        Generate stroke data for circle drawing.
        
        Args:
            center_x: Circle center X coordinate (pixels)
            center_y: Circle center Y coordinate (pixels)
            radius: Circle radius (pixels)
            num_points: Number of points to generate (higher = smoother)
            
        Returns:
            List[Dict]: List of stroke data dictionaries containing:
                - pixel_coords: [x, y] pixel coordinates
                - robot_coords: [x, y, z] robot coordinates
                - pen_down: Boolean indicating pen state
                - stroke_id: Stroke identifier
                - point_id: Point identifier within stroke
                
        Example:
            >>> canvas = CanvasCoordinateSystem()
            >>> circle_data = canvas.generate_circle_stroke_data(400, 300, 50, 8)
            >>> len(circle_data)
            9
        """
        stroke_data = []
        
        for i in range(num_points + 1):
            angle = 2 * math.pi * i / num_points
            
            # Calculate point on circle circumference
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            
            # Convert to robot coordinates
            robot_x, robot_y, robot_z = self.pixel_to_robot_coords(x, y, pen_down=True)
            
            stroke_data.append({
                'pixel_coords': [x, y],
                'robot_coords': [robot_x, robot_y, robot_z],
                'pen_down': True,
                'stroke_id': 0,
                'point_id': i
            })
        
        return stroke_data
    
    def add_stroke(self, stroke_data: List[Dict]) -> None:
        """
        Add drawing stroke to history.
        
        Args:
            stroke_data: List of stroke point dictionaries
        """
        self.drawn_strokes.append(stroke_data)
        if stroke_data:
            self.current_position = stroke_data[-1]['robot_coords']
    
    def get_canvas_bounds(self) -> Dict:
        """
        Get canvas area boundary information.
        
        Returns:
            Dict: Dictionary containing pixel_bounds and robot_bounds
                pixel_bounds: min/max pixel coordinates
                robot_bounds: min/max robot coordinates and Z levels
        """
        return {
            'pixel_bounds': {
                'min_x': 0, 'max_x': self.pixel_width,
                'min_y': 0, 'max_y': self.pixel_height
            },
            'robot_bounds': {
                'min_x': self.canvas_position[0] - self.canvas_width/2,
                'max_x': self.canvas_position[0] + self.canvas_width/2,
                'min_y': self.canvas_position[1] - self.canvas_height/2,
                'max_y': self.canvas_position[1] + self.canvas_height/2,
                'z_contact': self.canvas_position[2] + PEN_CONTACT_HEIGHT,
                'z_safe': self.canvas_position[2] + PEN_SAFE_HEIGHT
            }
        }
    
    def validate_coordinates(self, pixel_x: float, pixel_y: float) -> bool:
        """
        Check if coordinates are within canvas area.
        
        Args:
            pixel_x: X coordinate in pixels
            pixel_y: Y coordinate in pixels
            
        Returns:
            bool: True if coordinates are valid, False otherwise
        """
        bounds = self.get_canvas_bounds()
        pixel_bounds = bounds['pixel_bounds']
        
        return (pixel_bounds['min_x'] <= pixel_x <= pixel_bounds['max_x'] and
                pixel_bounds['min_y'] <= pixel_y <= pixel_bounds['max_y'])
    
    def get_scale_info(self) -> Dict:
        """
        Get coordinate scaling information.
        
        Returns:
            Dict: Scaling factors and resolution information
        """
        return {
            'scale_x_mm_per_pixel': self.scale_x * 1000,
            'scale_y_mm_per_pixel': self.scale_y * 1000,
            'pixel_resolution': f"{self.pixel_width}×{self.pixel_height}",
            'physical_size_cm': f"{self.canvas_width*100}×{self.canvas_height*100}"
        }


def main():
    """
    Test execution example demonstrating coordinate system functionality.
    """
    # Initialize canvas coordinate system
    canvas = CanvasCoordinateSystem()
    
    print("Canvas Coordinate System Test")
    print("=" * 40)
    
    # Display system information
    scale_info = canvas.get_scale_info()
    print(f"Physical size: {scale_info['physical_size_cm']} cm")
    print(f"Pixel resolution: {scale_info['pixel_resolution']}")
    print(f"Scale: {scale_info['scale_x_mm_per_pixel']:.2f} mm/pixel")
    print()
    
    # Test coordinate conversion
    center_x, center_y = 400, 300
    robot_coords = canvas.pixel_to_robot_coords(center_x, center_y)
    print(f"Canvas center ({center_x}, {center_y}) px -> {robot_coords} m")
    
    # Test inverse conversion
    pixel_coords = canvas.robot_to_pixel_coords(robot_coords[0], robot_coords[1])
    print(f"Inverse conversion: {robot_coords[:2]} m -> {pixel_coords} px")
    
    # Generate circle stroke data
    circle_strokes = canvas.generate_circle_stroke_data(
        center_x=400, center_y=300, radius=50, num_points=8
    )
    print(f"Generated {len(circle_strokes)} stroke points for circle")
    
    # Display canvas bounds
    bounds = canvas.get_canvas_bounds()
    print(f"Canvas bounds (pixels): {bounds['pixel_bounds']}")
    print(f"Robot bounds (meters): {bounds['robot_bounds']}")


if __name__ == "__main__":
    main()
