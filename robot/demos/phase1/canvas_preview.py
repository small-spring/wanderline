#!/usr/bin/env python3
"""
Canvas Preview Window for Phase 1 Robot Drawing

Dual Display System - Window 2: Canvas Preview (2D Drawing View)
Based on Wanderline's RealtimeVisualizer with robot-specific adaptations.
"""

import cv2
import numpy as np
import time
import threading
from typing import Tuple, List, Optional
from system_state import SystemState, ContactPoint


class CanvasPreviewWindow:
    """
    Real-time canvas preview window for Phase 1 robot drawing.
    
    Features:
    - Contact-based stroke updates (not coordinate-based)
    - Real-time drawing progress visualization
    - Drawing completion percentage display
    - Thread-safe updates for ROS2 integration
    """
    
    def __init__(self, config: dict):
        """
        Initialize canvas preview window.
        
        Args:
            config: Configuration dict with keys:
                - canvas_size: tuple (width, height) in pixels
                - window_size: tuple (width, height) for display
                - stroke_radius: int, pen width in pixels
                - update_frequency: int, update every N contacts
        """
        # Canvas configuration
        self.canvas_width = config.get('canvas_width', 800)
        self.canvas_height = config.get('canvas_height', 600)
        self.window_width = config.get('window_width', 800)
        self.window_height = config.get('window_height', 600)
        self.stroke_radius = config.get('stroke_radius', 2)
        self.update_frequency = config.get('update_frequency', 1)  # Update every contact
        
        # Canvas state
        self.canvas_image = self._create_blank_canvas()
        self.contact_count = 0
        self.total_contacts_expected = config.get('expected_total_contacts', 100)
        
        # Pen trail tracking for continuous line drawing
        self.pen_trail_points = []  # Store all contact points for trail visualization
        
        # Window state
        self.window_name = "Phase 1 - Canvas Preview"
        self.window_created = False
        self.window_closed = False
        self.display_available = False  # Will be set to True if GUI is available
        self.running = True
        
        # Thread-safe update mechanism
        self.update_lock = threading.Lock()
        self.pending_contacts = []
        
        # Performance tracking
        self.start_time = time.time()
        self.last_update_time = time.time()
        
    def _create_blank_canvas(self) -> np.ndarray:
        """Create a blank white canvas."""
        canvas = np.ones((self.canvas_height, self.canvas_width, 3), dtype=np.uint8) * 255
        return canvas
    
    def _init_window(self):
        """Initialize OpenCV window (Wanderline style) with headless detection."""
        if not self.window_created:
            try:
                # Create window with Wanderline flags for better compatibility
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
                cv2.resizeWindow(self.window_name, self.window_width, self.window_height)
                self.display_available = True
                print("✅ Canvas Preview Window (Window 2) initialized successfully")
                
                # Create and display initial placeholder (Wanderline style)
                placeholder = np.ones((self.canvas_height, self.canvas_width, 3), dtype=np.uint8) * 50
                cv2.putText(
                    placeholder, "Phase 1 - Canvas Preview", (50, self.canvas_height//2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2
                )
                cv2.putText(
                    placeholder, "Waiting for robot drawing...", (50, self.canvas_height//2 + 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2
                )
                cv2.putText(
                    placeholder, "Press 'q' to quit", (50, self.canvas_height//2 + 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 2
                )
                
                cv2.imshow(self.window_name, placeholder)
                
                # Force window to front and refresh (Wanderline technique)
                cv2.setWindowProperty(self.window_name, cv2.WND_PROP_TOPMOST, 1)
                cv2.waitKey(500)  # Longer wait for initialization
                cv2.setWindowProperty(self.window_name, cv2.WND_PROP_TOPMOST, 0)
                
                self.window_created = True
                print(f"✅ Canvas preview window initialized ({self.window_width}x{self.window_height})")
                print("If you cannot see the window, check if it's hidden behind other windows")
            except Exception as e:
                print(f"ℹ️  Canvas Preview Window (Window 2) disabled - headless environment detected")
                print(f"   Technical details: {e}")
                print(f"   💡 This is normal for Docker/server environments without GUI")
                self.display_available = False
                self.window_created = False
    
    def update_from_contact(self, contact_point: ContactPoint):
        """
        Update canvas from robot contact point (thread-safe).
        
        Args:
            contact_point: Contact point with 3D robot position and 2D pixel coordinates
        """
        with self.update_lock:
            self.pending_contacts.append(contact_point)
            
            # Process updates at specified frequency
            if len(self.pending_contacts) >= self.update_frequency:
                self._process_pending_contacts()
    
    def _process_pending_contacts(self):
        """Process all pending contact points and update canvas."""
        for contact in self.pending_contacts:
            # Add to trail for continuous line drawing
            self.pen_trail_points.append(contact.position_2d)
            
            # Draw contact stroke
            self._draw_contact_stroke(contact)
            self.contact_count += 1
        
        # Draw continuous trail between contact points
        self._draw_pen_trail()
        
        # Clear processed contacts
        self.pending_contacts.clear()
        
        # Update display
        self._update_display()
    
    def _draw_contact_stroke(self, contact_point: ContactPoint):
        """
        Draw a stroke at contact point (simulates pen contact).
        
        Args:
            contact_point: Contact point with 2D pixel coordinates
        """
        pixel_x, pixel_y = contact_point.position_2d
        
        # Ensure coordinates are within canvas bounds
        if not (0 <= pixel_x < self.canvas_width and 0 <= pixel_y < self.canvas_height):
            return
        
        # Draw circular stroke at contact point (simulates pen tip)
        for dx in range(-self.stroke_radius, self.stroke_radius + 1):
            for dy in range(-self.stroke_radius, self.stroke_radius + 1):
                if dx*dx + dy*dy <= self.stroke_radius*self.stroke_radius:
                    px, py = int(pixel_x + dx), int(pixel_y + dy)
                    if 0 <= px < self.canvas_width and 0 <= py < self.canvas_height:
                        self.canvas_image[py, px] = [0, 0, 0]  # Black stroke
    
    def _draw_pen_trail(self):
        """Draw continuous trail between contact points."""
        if len(self.pen_trail_points) < 2:
            return
            
        # Draw lines between consecutive trail points
        for i in range(1, len(self.pen_trail_points)):
            pt1 = self.pen_trail_points[i-1]
            pt2 = self.pen_trail_points[i]
            
            # Ensure points are valid
            if (0 <= pt1[0] < self.canvas_width and 0 <= pt1[1] < self.canvas_height and
                0 <= pt2[0] < self.canvas_width and 0 <= pt2[1] < self.canvas_height):
                
                # Draw line between points (red color for trail)
                cv2.line(self.canvas_image, 
                        (int(pt1[0]), int(pt1[1])), 
                        (int(pt2[0]), int(pt2[1])), 
                        (0, 0, 255),  # Red color (BGR format)
                        thickness=2)
    
    def _update_display(self):
        """Update the OpenCV window display."""
        if not self.running:
            return
            
        # Initialize window if needed
        if not self.window_created:
            self._init_window()
        
        # Create display image with overlay information
        display_image = self._create_display_image()
        
        # Update window
        # Only show window if display is available
        if self.display_available:
            cv2.imshow(self.window_name, display_image)
        else:
            # Headless mode - just track progress internally
            pass
        
        # Handle window events (non-blocking) - only if display available
        if self.display_available:
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or cv2.getWindowProperty(self.window_name, cv2.WND_PROP_VISIBLE) < 1:
                self.close()
    
    def _create_display_image(self) -> np.ndarray:
        """Create display image with progress overlay."""
        # Resize canvas to window size if needed
        if (self.canvas_width, self.canvas_height) != (self.window_width, self.window_height):
            display_image = cv2.resize(self.canvas_image, (self.window_width, self.window_height))
        else:
            display_image = self.canvas_image.copy()
        
        # Add progress overlay
        self._draw_progress_overlay(display_image)
        
        return display_image
    
    def _draw_progress_overlay(self, image: np.ndarray):
        """Draw progress information overlay on image."""
        # Calculate progress - more realistic calculation
        if self.contact_count < 50:
            # Initial phase: 0-50%
            progress_percent = (self.contact_count / 100) * 50
        else:
            # Use actual circle completion logic
            # Estimate based on contact density (realistic for circle drawing)
            estimated_total = max(200, self.contact_count * 1.2)  # Dynamic estimation
            progress_percent = min((self.contact_count / estimated_total) * 100, 95)
        
        elapsed_time = time.time() - self.start_time
        
        # Overlay text configuration
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        color = (0, 0, 255)  # Red text
        thickness = 2
        
        # Progress text - more informative
        progress_text = f"Drawing: {progress_percent:.1f}% ({self.contact_count} points)"
        time_text = f"Time: {elapsed_time:.1f}s"
        
        # Draw text with background for visibility
        texts = [progress_text, time_text]
        y_offset = 30
        
        for i, text in enumerate(texts):
            y_pos = y_offset + i * 30
            
            # Get text size for background rectangle
            (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)
            
            # Draw background rectangle
            cv2.rectangle(image, (10, y_pos - text_height - 5), 
                         (15 + text_width, y_pos + baseline + 5), (255, 255, 255), -1)
            
            # Draw text
            cv2.putText(image, text, (12, y_pos), font, font_scale, color, thickness)
    
    def update_from_system_state(self, system_state: SystemState):
        """
        Update canvas from complete system state.
        
        Args:
            system_state: Current robot system state
        """
        # Process any new contact points
        new_contacts = len(system_state.contact_history) - self.contact_count
        if new_contacts > 0:
            # Get only the new contacts
            latest_contacts = system_state.contact_history[-new_contacts:]
            for contact in latest_contacts:
                self.update_from_contact(contact)
    
    def reset_canvas(self):
        """Reset canvas to blank state."""
        with self.update_lock:
            self.canvas_image = self._create_blank_canvas()
            self.contact_count = 0
            self.pending_contacts.clear()
            self.start_time = time.time()
            print("🔄 Canvas preview reset")
    
    def save_canvas(self, filepath: str):
        """Save current canvas state to file."""
        with self.update_lock:
            cv2.imwrite(filepath, self.canvas_image)
            print(f"💾 Canvas saved to {filepath}")
    
    def close(self):
        """Close the preview window and cleanup."""
        self.running = False
        self.window_closed = True
        
        if self.window_created:
            cv2.destroyWindow(self.window_name)
            print("🔄 Canvas preview window closed")
    
    def is_closed(self) -> bool:
        """Check if window has been closed."""
        return self.window_closed
    
    def get_progress(self) -> float:
        """Get current drawing progress (0.0 to 1.0)."""
        return min(self.contact_count / self.total_contacts_expected, 1.0)


# Helper functions for integration with Phase 1 main.py

def create_preview_window(config: dict) -> CanvasPreviewWindow:
    """
    Create and initialize canvas preview window.
    
    Args:
        config: Configuration dictionary
        
    Returns:
        CanvasPreviewWindow instance
    """
    canvas_config = {
        'canvas_width': config.get('canvas_pixel_width', 800),
        'canvas_height': config.get('canvas_pixel_height', 600),
        'window_width': config.get('preview_window_width', 800),
        'window_height': config.get('preview_window_height', 600),
        'stroke_radius': config.get('pen_radius_pixels', 2),
        'update_frequency': config.get('preview_update_frequency', 1),
        'expected_total_contacts': config.get('expected_circle_contacts', 100)
    }
    
    return CanvasPreviewWindow(canvas_config)


def robot_to_canvas_pixel(robot_x: float, robot_y: float, robot_z: float, 
                         canvas_config: dict) -> Tuple[int, int]:
    """
    Convert 3D robot coordinates to 2D canvas pixel coordinates.
    
    Args:
        robot_x, robot_y, robot_z: Robot coordinates in meters
        canvas_config: Canvas configuration parameters
        
    Returns:
        Tuple of (pixel_x, pixel_y)
    """
    # Canvas parameters (from existing canvas_coordinate_system.py)
    canvas_center_x, canvas_center_y = 0.5, 0.0  # Robot coordinates
    canvas_size_m = 0.4  # 40cm canvas
    canvas_pixel_width = canvas_config.get('canvas_pixel_width', 800)
    canvas_pixel_height = canvas_config.get('canvas_pixel_height', 600)
    
    # Convert robot coordinates to canvas-relative coordinates
    canvas_rel_x = robot_x - canvas_center_x
    canvas_rel_y = robot_y - canvas_center_y
    
    # Convert to pixel coordinates
    pixel_x = (canvas_rel_x / (canvas_size_m / 2)) * (canvas_pixel_width / 2) + (canvas_pixel_width / 2)
    pixel_y = (canvas_rel_y / (canvas_size_m / 2)) * (canvas_pixel_height / 2) + (canvas_pixel_height / 2)
    
    return int(pixel_x), int(pixel_y)


if __name__ == "__main__":
    """Test canvas preview window independently."""
    import random
    
    # Test configuration
    test_config = {
        'canvas_width': 800,
        'canvas_height': 600,
        'window_width': 800,
        'window_height': 600,
        'stroke_radius': 2,
        'update_frequency': 1,
        'expected_total_contacts': 100
    }
    
    # Create preview window
    preview = CanvasPreviewWindow(test_config)
    
    print("🎨 Testing Canvas Preview Window")
    print("Press 'q' to quit, or close window")
    
    # Simulate robot drawing a circle
    center_x, center_y = 400, 300
    radius = 80
    
    try:
        for i in range(100):
            # Simulate circle drawing
            angle = (i / 100) * 2 * 3.14159
            pixel_x = center_x + radius * np.cos(angle)
            pixel_y = center_y + radius * np.sin(angle)
            
            # Create mock contact point
            mock_contact = ContactPoint(
                position_3d=(0.5, 0.0, 0.1),  # Mock robot coordinates
                position_2d=(int(pixel_x), int(pixel_y)),
                timestamp=time.time(),
                stroke_id=i
            )
            
            # Update preview
            preview.update_from_contact(mock_contact)
            
            # Small delay to simulate real drawing
            time.sleep(0.05)
            
            # Check if window was closed
            if preview.is_closed():
                break
                
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted")
    
    finally:
        preview.close()
        print("✅ Canvas preview test completed")