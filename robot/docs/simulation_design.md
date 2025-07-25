# Robot Drawing Simulation - Design Principles

**Extracted from**: tmp.md, tmp_simulation.md
**Status**: ✅ **VERIFIED** - Core design validated

## 🎯 Simulation Design Philosophy

### Key Insight: End-Effector as Pen Tip

The simulation treats the **robot end-effector itself as the pen tip**, not a separate tool:

- **No physical pen**: End-effector directly contacts canvas
- **Contact height**: 2cm above canvas surface (realistic pen-to-canvas distance)
- **Visual pen**: Added for realism but not part of core simulation

```python
# Core simulation logic
robot_z = canvas_position[2] + (0.02 if pen_down else 0.05)
# 0.02m = contact height (pen tip simulation)
# 0.05m = safe movement height
```

## 🔄 Circle Drawing Algorithm Details

### Coordinate Calculation Flow
```
pixel座標 → robot座標 → joint角度 → 実際のrobot位置
```

### `calculate_next_coordinate` Implementation
```python
def calculate_next_coordinate(current_position, canvas_state):
    # 1. Calculate progress (quadrant-based + contact density)
    progress = _calculate_circle_progress(canvas_state)
    
    # 2. Check completion (95% threshold)
    if progress >= 0.95:
        return _get_circle_start_point(), True
    
    # 3. Calculate next point on circumference
    return _get_next_circle_point(progress, target_circle, current_pos), False
```

### Circle Point Generation
```python
def _get_next_circle_point(current_progress, target_circle, current_position):
    # Angle increment for smooth circle (24 segments)
    segment_angle = 2 * math.pi / 24  # 15° increments
    
    # Current angle from circle center
    current_angle = math.atan2(current_y - center_y, current_x - center_x)
    
    # Next angle (counter-clockwise progression)
    next_angle = current_angle + segment_angle / 2  # 7.5° steps
    
    # Next point on circumference
    next_x = center_x + radius * math.cos(next_angle)
    next_y = center_y + radius * math.sin(next_angle)
    
    # Apply max step constraint (5 pixels)
    return limit_stroke_length(next_x, next_y, max_pixels=5)
```

## 🎯 Progress Tracking System

### Two-Phase Validation
1. **Angular Coverage**: 4 quadrants must have contact points
2. **Contact Density**: Minimum 24 contact points required
3. **Final Progress**: `min(angle_progress, contact_progress)`

```python
def _calculate_circle_progress(canvas_state):
    # Quadrant coverage (0-2π divided into 4 sections)
    angle_ranges = [False, False, False, False]
    for contact in canvas_state.contact_history:
        angle = math.atan2(pixel_y - center_y, pixel_x - center_x)
        quadrant = int(angle / (math.pi / 2))
        angle_ranges[quadrant] = True
    
    angle_progress = sum(angle_ranges) / 4.0
    contact_progress = min(len(contacts) / 24, 1.0)
    
    return min(angle_progress, contact_progress)
```

## 🎨 Visualization Architecture

### RViz Display Components
- **Canvas**: White cube (40cm × 40cm × 5mm)
- **Pen Body**: Blue cylinder from Tool flange (8cm × 1.2cm)
- **Pen Tip**: Red sphere at end-effector position
- **Trail**: Red line strip (up to 5000 points, persistent)
- **Current Position**: Green sphere (trail endpoint)

### Canvas Preview Window
- **Technology**: OpenCV with thread-safe updates
- **Update Rate**: Every contact point (real-time)
- **Features**: Progress overlay, contact visualization
- **Headless Support**: Graceful fallback for server environments

## 🔧 Tool Flange Integration

### Coordinate Frame Hierarchy
```
base_link → ... → wrist_3_link → tool0 (Tool flange)
```

### Pen Attachment Strategy
```python
# Pen body attached to tool0 frame
pen_body.header.frame_id = "tool0"
pen_body.pose.position.z = 0.04  # 4cm toward canvas
```

### Wrist Orientation Correction
```python
# Original: Tool flange pointed away from canvas
joints[4] = 1.57  # Wrist 2 perpendicular

# Corrected: Tool flange points toward canvas  
joints[4] = 1.57 + 3.14159  # + 180° rotation = 4.713
joints[5] = 0.0  # Wrist 3 no rotation
```

## 🎛️ Configuration Parameters

### Robot Control
- **Update Rate**: 50Hz joint state publishing
- **Interpolation**: 25 steps for smooth movement
- **Joint Limits**: ±360° range, ±180°/s max speed

### Drawing Parameters
- **Circle Center**: (400, 300) pixels = canvas center
- **Circle Radius**: 80 pixels
- **Segments**: 24 (smooth circle)
- **Max Step**: 5 pixels (stroke length limit)
- **Closure Threshold**: 95% completion

### Memory Optimization
- **Trail Limit**: 5000 points (complete circle + margin)
- **Update Frequency**: Every 10 points for RViz
- **Coordinate Storage**: Position-only (no full video frames)

## 🔍 Debugging and Validation

### Coordinate Verification
```python
# Debug logging every 10 steps
if self._debug_counter % 10 == 0:
    self.get_logger().info(
        f"Joints {[f'{j:.3f}' for j in joints[:3]]} → "
        f"Robot({robot_x:.3f},{robot_y:.3f},{robot_z:.3f}) → "
        f"Pixel({pixel_x:.1f},{pixel_y:.1f})"
    )
```

### Performance Monitoring
- **Contact Rate**: Contacts per second
- **Coordinate Accuracy**: Pixel error measurements
- **Memory Usage**: Trail point counts
- **Rendering Performance**: Frame rate monitoring

This design achieves **mathematical precision** with **visual realism** while maintaining **computational efficiency** for real-time robot control.