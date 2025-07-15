# Phase 1: Robot Circle Drawing with Stroke-Based System

## 🎯 Goal: Canvas-Aware Single-Line Drawing

**What You Will See**:
- Robot arm drawing smooth, continuous circle on canvas
- Pen maintains contact with canvas surface throughout drawing
- Real-time canvas state updates as drawing progresses
- Configurable canvas size and stroke parameters

## 🎬 Expected Behavior (Animation in Mind)

```
1. Robot starts at home position, pen up
2. Robot moves to circle start point, lowers pen to canvas
3. Robot draws first stroke: short line segment along canvas surface
4. Robot continues to next stroke without lifting pen
5. Each stroke connects smoothly to previous stroke
6. Canvas state updates show accumulated drawing
7. Process repeats until complete circle is drawn
8. Robot lifts pen and returns to home position
```

## 🏗️ System Architecture (4 Core Components)

```
                          ┌─────────────────────┐
                          │  System State       │
                          │  Manager            │
                          │  (Single Truth)     │
                          └─────────┬───────────┘
                                    │ state_updates
                    ┌───────────────┼───────────────┐
                    ▼               ▼               ▼
┌─────────────────────┐    target_coords    ┌─────────────────────┐
│  Module 1:          │ ─────────────────→  │  Module 2:          │
│  Coordinate         │                     │  Stroke             │
│  Calculator         │                     │  Generator          │
│  (Where to go)      │ ←─────────────────  │  (How to move)      │
└─────────────────────┘  canvas_state +    └─────────────────────┘
                        current_position             │
                                                     │ stroke_plan
                                                     ▼
                                              ┌─────────────────────┐
                                              │  Module 3:          │
                                              │  Robot Executor +   │
                                              │  Contact Detector   │
                                              │  (Physical motion)  │
                                              └─────────────────────┘
                                                     │
                                                     ▼
                                              ┌─────────────────────┐
                                              │  Dual Display       │
                                              │  System             │
                                              │  (RViz + Canvas)    │
                                              └─────────────────────┘
```

## 🔧 Core Components Detail

### System State Manager (NEW)
**File**: `scripts/system_state_manager.py`

**Purpose**: Single source of truth for all system state

**Critical Insight**: Multiple components need access to current state:
- Robot joint positions vs. calculated pen position vs. actual pen position
- Canvas state (what's been drawn) vs. target state
- Contact detection results vs. drawing progress

**What it maintains**:
```python
@dataclass
class SystemState:
    # Robot state
    robot_joint_positions: Dict[str, float]
    calculated_pen_position: Tuple[float, float, float]
    pen_contact_state: bool
    
    # Canvas state  
    canvas_config: CanvasConfig
    drawn_strokes: List[CompletedStroke]
    current_canvas_image: np.ndarray  # For preview window
    contact_history: List[ContactPoint]
    
    # Drawing session
    target_shape: TargetCircle
    completion_percentage: float
    current_stroke_id: int
    
    # Timestamps and errors
    last_update_timestamp: float
    system_errors: List[str]
```

### Module 1: Coordinate Calculator
**File**: `demos/circle_coordinate_generator.py`

**Purpose**: Decide where to draw next based on current canvas state

**Input**:
- Current pen position
- Canvas state (what's already drawn)
- Target shape (circle parameters)

**Output**:
- Next target coordinate
- Drawing complete flag

**What it does**:
```python
def calculate_next_coordinate(current_pos, canvas_state, target_circle):
    # Analyze current progress toward circle
    progress = analyze_circle_progress(canvas_state, target_circle)
    
    # Calculate next point on circle circumference
    if progress < 1.0:
        next_angle = calculate_next_angle(progress)
        next_coord = circle_point(target_circle.center, target_circle.radius, next_angle)
        return next_coord, False
    else:
        return None, True  # Drawing complete
```

### Module 2: Stroke Generator
**File**: `scripts/stroke_generator.py`

**Purpose**: Convert coordinate pair into executable robot motion plan

**Input**:
- Start coordinate (current pen position)
- End coordinate (target from Module 1)
- Canvas parameters

**Output**:
- Stroke definition with waypoints
- Pen control commands

**What it does**:
```python
def generate_stroke(start_pos, end_pos, canvas_config):
    # Generate waypoints along canvas surface
    waypoints = []
    
    # Check if stroke is valid
    if not validate_stroke_on_canvas(start_pos, end_pos, canvas_config):
        return None
    
    # Create intermediate points (e.g., every 5mm)
    num_points = calculate_waypoint_count(start_pos, end_pos)
    
    for i in range(num_points):
        progress = i / (num_points - 1)
        waypoint = interpolate_on_canvas(start_pos, end_pos, progress, canvas_config)
        waypoints.append(waypoint)
    
    return StrokeDefinition(waypoints=waypoints, pen_down=True)
```

### Module 3: Robot Executor & Contact Detector
**File**: `demos/robot_stroke_executor.py`

**Purpose**: Execute robot motion with real-time contact detection

**Key Insight**: Canvas updates should be based on actual pen-canvas contact, not planned coordinates

**Input**:
- Stroke definition (waypoints + pen commands)
- Current system state

**Output**:
- Updated pen position
- Contact detection results
- Canvas state updates (via System State Manager)

**What it does**:
```python
def execute_stroke_with_contact_detection(stroke_def, system_state):
    contact_points = []
    
    # Execute robot movement with real-time contact detection
    for waypoint in stroke_def.waypoints:
        # Move robot to waypoint
        success = move_robot_to_position(waypoint)
        
        if success:
            # Get actual robot position (may differ from planned)
            actual_position = get_current_robot_position()
            
            # Check for canvas contact
            contact = check_pen_canvas_contact(actual_position, system_state.canvas_config)
            
            if contact:
                # Record contact point
                contact_point = ContactPoint(
                    position_3d=actual_position,
                    timestamp=time.time(),
                    stroke_id=stroke_def.stroke_id
                )
                contact_points.append(contact_point)
                
                # Update canvas state via contact
                update_canvas_from_contact(contact_point, system_state)
    
    return contact_points, actual_position
```

### Dual Display System (NEW)
**Files**: `demos/canvas_preview_window.py` + RViz integration

**Purpose**: Two-window demonstration like professional drawing software

**Window 1: RViz (3D Robot View)**:
- Robot arm movement
- Canvas surface (white plane)
- Pen tool with contact state colors
- 3D stroke trajectories

**Window 2: Canvas Preview (2D Drawing View)**:
- Real-time drawing progress (based on Wanderline preview)
- Contact-based stroke updates (not coordinate-based)
- Drawing completion percentage
- Canvas state visualization

**Contact-Based Canvas Updates**:
```python
def update_canvas_from_contact(contact_point, system_state):
    # Convert 3D robot position to 2D canvas pixel
    pixel_x, pixel_y = robot_to_canvas_pixel(contact_point.position_3d, system_state.canvas_config)
    
    # Simulate pen stroke (not just line drawing)
    stroke_radius = 2  # 2-pixel pen width
    
    # Draw circular stroke at contact point
    for dx in range(-stroke_radius, stroke_radius + 1):
        for dy in range(-stroke_radius, stroke_radius + 1):
            if dx*dx + dy*dy <= stroke_radius*stroke_radius:
                px, py = pixel_x + dx, pixel_y + dy
                if 0 <= px < 800 and 0 <= py < 600:
                    system_state.current_canvas_image[py, px] = [0, 0, 0]  # Black stroke
    
    # Update preview window
    refresh_canvas_preview_window(system_state.current_canvas_image)
```

## 💡 Key Insights and Design Decisions

### Critical Problems Identified:

1. **State Management Chaos**
   - **Problem**: Multiple sources of "truth" (robot joints, calculated pen position, canvas state)
   - **Solution**: Single `SystemState` class as source of truth
   - **Why**: Prevents inconsistencies and race conditions

2. **Canvas Updates Based on Wrong Data**
   - **Problem**: Previous design updated canvas from planned coordinates, not actual contact
   - **Solution**: Contact-based canvas updates only
   - **Why**: Reflects reality - only pen-canvas contact should draw

3. **Missing Demonstration System**
   - **Problem**: No way to see drawing progress in real-time
   - **Solution**: Dual-window system (RViz + Canvas Preview)
   - **Why**: Users need both 3D robot view and 2D drawing progress

4. **No Contact Detection**
   - **Problem**: System assumes pen is always where it should be
   - **Solution**: Real-time contact detection with tolerance
   - **Why**: Handles robot inaccuracies and provides feedback

### Design Philosophy Changes:

- **FROM**: Coordinate-based drawing (teleportation)
- **TO**: Contact-based drawing (physical simulation)

- **FROM**: Single RViz view
- **TO**: Dual display system

- **FROM**: Distributed state management
- **TO**: Centralized state management

- **FROM**: Planned position = actual position
- **TO**: Verify actual position and contact

## 📊 Data Structures

### Enhanced Canvas State
```python
@dataclass
class CanvasState:
    size: Tuple[float, float]           # Physical size (0.4, 0.4)
    position: Tuple[float, float, float] # Position (0.5, 0.0, 0.1)
    drawn_strokes: List[Stroke]         # All completed strokes
    coverage_map: np.ndarray            # 2D map of drawn areas
    current_drawing_progress: float     # 0.0 to 1.0
    current_canvas_image: np.ndarray    # For preview window (800x600x3)
    contact_history: List[ContactPoint] # All contact points
```

### Contact Point (NEW)
```python
@dataclass
class ContactPoint:
    position_3d: Tuple[float, float, float]  # Robot coordinates
    position_2d: Tuple[int, int]             # Canvas pixel coordinates
    timestamp: float
    stroke_id: int
    contact_force: float = 0.0               # Future: force sensor integration
```

### Stroke Definition
```python
@dataclass
class StrokeDefinition:
    waypoints: List[Point]              # Path to follow
    pen_down: bool                      # Pen contact state
    canvas_height: float                # Z-coordinate for drawing
    stroke_id: int                      # Unique identifier
    estimated_duration: float          # Expected execution time
```

### Target Circle
```python
@dataclass
class TargetCircle:
    center: Tuple[float, float]         # Pixel coordinates (400, 300)
    radius: float                       # Pixel radius (80)
    total_segments: int                 # Number of line segments (24)
    stroke_length: float                # Max length per stroke (0.02m)
```

## 🔄 Communication Flow

### Main Drawing Loop
```python
def drawing_loop():
    canvas_state = initialize_canvas()
    current_position = get_robot_home_position()
    target_circle = load_circle_config()
    
    while True:
        # Module 1: Calculate next coordinate
        next_coord, complete = calculate_next_coordinate(
            current_position, canvas_state, target_circle
        )
        
        if complete:
            break
        
        # Module 2: Generate stroke
        stroke_def = generate_stroke(
            current_position, next_coord, canvas_state.config
        )
        
        # Module 3: Execute and update
        current_position, canvas_state, status = execute_stroke_and_update_canvas(
            stroke_def, canvas_state
        )
        
        if status != "success":
            handle_error(status)
```

### ROS2 Communication
```python
# Service for coordinate calculation
srv/GetNextCoordinate.srv:
    # Request
    geometry_msgs/Point current_position
    robot_msgs/CanvasState canvas_state
    ---
    # Response
    geometry_msgs/Point next_coordinate
    bool drawing_complete

# Service for stroke generation
srv/GenerateStroke.srv:
    # Request
    geometry_msgs/Point start_position
    geometry_msgs/Point end_position
    ---
    # Response
    robot_msgs/StrokeDefinition stroke_definition
    bool stroke_valid

# Topic for canvas state updates
msg/CanvasState.msg:
    geometry_msgs/Point[] drawn_points
    float32 coverage_percentage
    int32 total_strokes_completed
```

## 🚀 Implementation Strategy

### Core Insight: Organized Directory Structure
**Problem**: Single file becomes too complex, multiple scattered files are hard to manage
**Solution**: Create dedicated `phase1/` directory with logically organized files

### Phase 1A: Minimal Working System (Week 1)
**Directory**: `demos/phase1/` (ORGANIZED STRUCTURE)
```
demos/phase1/
├── __init__.py
├── main.py                    # Main orchestrator
├── system_state.py            # SystemState class
├── coordinate_calculator.py   # Module 1
├── stroke_generator.py        # Module 2  
├── robot_executor.py          # Module 3
├── canvas_preview.py          # Canvas preview window
└── config.yaml               # Configuration
```

### Phase 1B: Polish and Integration (Week 2)
- Add ROS2 services if needed
- Improve error handling
- Add more drawing patterns
- Performance optimization

## 📁 Files to Implement (Organized Set)

### Core Implementation (Phase 1A)
1. **`demos/phase1/main.py`** - Main orchestrator and drawing loop
2. **`demos/phase1/system_state.py`** - SystemState class and state management
3. **`demos/phase1/coordinate_calculator.py`** - Module 1: Where to go next
4. **`demos/phase1/stroke_generator.py`** - Module 2: How to move
5. **`demos/phase1/robot_executor.py`** - Module 3: Execute + Contact detection
6. **`demos/phase1/canvas_preview.py`** - Canvas preview window
7. **`demos/phase1/config.yaml`** - Configuration parameters
8. **`launch/phase1_demo.launch.py`** - Launch file

### Research Task (Before Implementation)
9. **Study Wanderline preview**: Find canvas rendering in main Wanderline codebase
   - Look for: `wanderline/canvas.py`, `wanderline/video_recorder.py`
   - Understand: How Wanderline updates canvas in real-time
   - Adapt: Use same mechanisms for robot canvas preview

## ✅ Complete Implementation Specifications

### 1. **Robot Control Interface - DEFINED**
**Based on existing `robot_draw_circle.py` analysis**:

**Approach**: Use simplified 2-DOF control (like existing system)
- Only use 2 joints: `shoulder_pan_joint` (base rotation) and `shoulder_lift_joint` (shoulder up/down)
- Fix other 4 joints: `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint`
- Direct joint space control (no complex inverse kinematics needed)

**Control Interface**:
```python
class RobotController:
    def __init__(self):
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.joint_names = [
            'shoulder_pan_joint',      # Base rotation (X motion)
            'shoulder_lift_joint',     # Shoulder up/down (Y motion)
            'elbow_joint',            # Fixed at -1.0
            'wrist_1_joint',          # Fixed at -1.5
            'wrist_2_joint',          # Fixed at 1.57
            'wrist_3_joint'           # Fixed at 0.0
        ]
        # Base configuration (from existing system)
        self.base_joints = [0.0, -1.2, -1.0, -1.5, 1.57, 0.0]
    
    def get_current_joint_positions(self) -> Dict[str, float]:
        """Get current joint positions (stored in system state)"""
        return dict(zip(self.joint_names, self.current_joints))
    
    def move_to_joint_position(self, joint_positions: List[float]) -> bool:
        """Send joint positions to robot"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = joint_positions
        self.joint_pub.publish(msg)
        return True
    
    def pixel_to_joint_position(self, pixel_x: float, pixel_y: float) -> List[float]:
        """Convert pixel coordinates to joint positions (2-DOF mapping)"""
        # Convert pixel to robot coords using existing system
        robot_x, robot_y, robot_z = canvas_system.pixel_to_robot_coords(pixel_x, pixel_y)
        
        # Map to joint space (from existing robot_draw_circle.py)
        canvas_center_x, canvas_center_y = 0.5, 0.0  # Canvas center in robot coords
        
        # Calculate joint offsets
        pan_amplitude = 0.6   # ±34 degrees base rotation range
        lift_amplitude = 0.3  # ±17 degrees shoulder range
        
        # Map robot coordinates to joint space
        pan_offset = (robot_x - canvas_center_x) / 0.2 * pan_amplitude
        lift_offset = (robot_y - canvas_center_y) / 0.2 * lift_amplitude
        
        # Apply to base configuration
        joints = self.base_joints.copy()
        joints[0] = self.base_joints[0] + pan_offset   # shoulder_pan_joint
        joints[1] = self.base_joints[1] + lift_offset  # shoulder_lift_joint
        
        return joints
```

**Performance**: 50Hz update rate (from existing system)

### 2. **Contact Detection Algorithm - DEFINED**
**Based on existing `canvas_coordinate_system.py` constants and user feedback**:

**Tolerance Values** (configurable):
- **Z-axis tolerance**: 5mm (realistic for physical contact, not 1.5cm)
- **Canvas bounds**: Within 40cm × 40cm canvas area
- **No pressure consideration** (as requested)

**Implementation**:
```python
class ContactDetector:
    def __init__(self, config):
        # Contact tolerances (configurable)
        self.z_tolerance = config.get('z_tolerance', 0.005)  # 5mm default
        self.canvas_position = config['canvas_position']     # [0.5, 0.0, 0.1]
        self.canvas_size = config['canvas_size']             # [0.4, 0.4]
        
        # Contact height reference (from existing system)
        self.canvas_surface_z = self.canvas_position[2] + 0.005  # Canvas surface + pen contact
    
    def check_pen_canvas_contact(self, pen_pos: Tuple[float, float, float]) -> bool:
        """
        Contact detection with realistic tolerances:
        - Z-axis tolerance: 5mm (configurable)
        - X-Y boundary check: Must be within canvas bounds
        - No pressure consideration (as requested)
        """
        pen_x, pen_y, pen_z = pen_pos
        
        # 1. Check Z-axis proximity (vertical distance to canvas surface)
        z_distance = abs(pen_z - self.canvas_surface_z)
        z_contact = z_distance <= self.z_tolerance
        
        # 2. Check X-Y canvas bounds
        canvas_min_x = self.canvas_position[0] - self.canvas_size[0] / 2
        canvas_max_x = self.canvas_position[0] + self.canvas_size[0] / 2
        canvas_min_y = self.canvas_position[1] - self.canvas_size[1] / 2
        canvas_max_y = self.canvas_position[1] + self.canvas_size[1] / 2
        
        x_in_bounds = canvas_min_x <= pen_x <= canvas_max_x
        y_in_bounds = canvas_min_y <= pen_y <= canvas_max_y
        
        # 3. Contact detected if both conditions met
        return z_contact and x_in_bounds and y_in_bounds
    
    def get_contact_quality(self, pen_pos: Tuple[float, float, float]) -> float:
        """Return contact quality (0.0 to 1.0) for gradual feedback"""
        pen_x, pen_y, pen_z = pen_pos
        z_distance = abs(pen_z - self.canvas_surface_z)
        
        # Quality decreases with distance from surface
        if z_distance <= self.z_tolerance:
            return 1.0 - (z_distance / self.z_tolerance)
        else:
            return 0.0
```

**Configurable Parameters**:
```yaml
contact_detection:
  z_tolerance: 0.005        # 5mm (adjustable based on testing)
  canvas_position: [0.5, 0.0, 0.1]
  canvas_size: [0.4, 0.4]   # 40cm × 40cm
  
  # Additional tolerance for edge cases
  boundary_tolerance: 0.01   # 1cm tolerance for canvas edges
```

**Edge Case Handling**:
- Pen slightly outside canvas bounds: No contact
- Pen at canvas edge: Contact if within boundary_tolerance
- Pen too high/low: No contact, but return quality value for feedback

### 3. **Canvas Coordinate Conversion - SOLVED**
**Solution**: Use existing `canvas_coordinate_system.py` (already implemented)**

**Available Functions** (from existing system):
```python
from scripts.canvas_coordinate_system import CanvasCoordinateSystem

canvas_system = CanvasCoordinateSystem()

# Convert pixel to robot coordinates
robot_x, robot_y, robot_z = canvas_system.pixel_to_robot_coords(pixel_x, pixel_y, pen_down=True)

# Convert robot to pixel coordinates  
pixel_x, pixel_y = canvas_system.robot_to_pixel_coords(robot_x, robot_y)

# Validate coordinates
is_valid = canvas_system.validate_coordinates(pixel_x, pixel_y)

# Get canvas bounds
bounds = canvas_system.get_canvas_bounds()
```

**Conversion Formulas** (from existing implementation):
```python
# Pixel to Robot (existing implementation)
def pixel_to_robot_coords(pixel_x, pixel_y, pen_down=True):
    # Convert pixel coordinates to canvas-relative coordinates
    canvas_x = (pixel_x - 400) * 0.0005  # 400 = canvas center, 0.0005 = scale
    canvas_y = (pixel_y - 300) * 0.0005  # 300 = canvas center
    
    # Transform to robot coordinate system
    robot_x = 0.5 + canvas_x  # Canvas center at [0.5, 0.0, 0.1]
    robot_y = 0.0 + canvas_y
    robot_z = 0.1 + (0.005 if pen_down else 0.05)  # Contact vs safe height
    
    return robot_x, robot_y, robot_z

# Robot to Pixel (existing implementation)
def robot_to_pixel_coords(robot_x, robot_y):
    # Transform from robot to canvas coordinate system
    canvas_x = robot_x - 0.5
    canvas_y = robot_y - 0.0
    
    # Convert to pixel coordinates
    pixel_x = canvas_x / 0.0005 + 400
    pixel_y = canvas_y / 0.0005 + 300
    
    return pixel_x, pixel_y
```

**Specifications**:
- **Canvas size**: 40cm × 40cm (0.4m × 0.4m)
- **Canvas position**: [0.5, 0.0, 0.1] in robot base coordinates
- **Pixel resolution**: 800×600 pixels
- **Scale**: ~0.5mm per pixel
- **Pen contact height**: 0.105m (canvas surface + 5mm)
- **Pen safe height**: 0.15m (canvas surface + 5cm)

**Edge Case Handling** (existing implementation):
- **Out of bounds**: `validate_coordinates()` returns False
- **Boundary check**: Canvas bounds checking available
- **Coordinate clamping**: Can be added if needed

**Integration**:
```python
# In Phase 1 implementation
from scripts.canvas_coordinate_system import CanvasCoordinateSystem

class Phase1System:
    def __init__(self):
        self.canvas_system = CanvasCoordinateSystem()
    
    def convert_target_to_robot(self, target_pixel_x, target_pixel_y):
        """Convert target pixel coordinates to robot coordinates"""
        return self.canvas_system.pixel_to_robot_coords(target_pixel_x, target_pixel_y, pen_down=True)
    
    def convert_robot_to_pixel(self, robot_x, robot_y):
        """Convert robot coordinates to pixel coordinates for canvas updates"""
        return self.canvas_system.robot_to_pixel_coords(robot_x, robot_y)
```

### 4. **Circle Drawing Algorithm - DEFINED**
**Based on existing `robot_draw_circle.py` patterns and `canvas_coordinate_system.py`**:

**Algorithm Strategy**: Progressive circle drawing with contact-based progress tracking
- **Circle division**: 24 segments (like existing system) for smooth curves
- **Progress tracking**: Based on actual contact points, not planned coordinates
- **Closure handling**: Automatic return to start point when 95% complete
- **Stroke length**: 5mm maximum segments for smooth drawing

**Implementation**:
```python
class CircleDrawingAlgorithm:
    def __init__(self, config):
        self.segments = config.get('circle_segments', 24)  # 24 points = smooth circle
        self.max_stroke_length = config.get('max_stroke_length', 0.005)  # 5mm max
        self.closure_threshold = config.get('closure_threshold', 0.95)  # 95% completion
        
    def calculate_circle_progress(self, canvas_state: CanvasState, target_circle: TargetCircle) -> float:
        """
        Calculate drawing progress based on actual contact points (not planned coordinates)
        
        Progress calculation:
        1. Count unique contact points within circle perimeter
        2. Calculate angular coverage of contact points
        3. Return percentage of circle circumference covered
        """
        if not canvas_state.contact_history:
            return 0.0
        
        # Get contact points for current circle
        circle_contacts = [
            point for point in canvas_state.contact_history 
            if self._is_point_on_circle(point, target_circle)
        ]
        
        if not circle_contacts:
            return 0.0
        
        # Calculate angular coverage
        angles = []
        for contact in circle_contacts:
            pixel_x, pixel_y = contact.position_2d
            angle = self._calculate_angle_from_center(pixel_x, pixel_y, target_circle)
            angles.append(angle)
        
        # Calculate coverage percentage
        angles.sort()
        total_coverage = self._calculate_angular_coverage(angles)
        return min(total_coverage / (2 * math.pi), 1.0)
    
    def get_next_circle_point(self, current_progress: float, target_circle: TargetCircle, 
                             current_position: Tuple[float, float]) -> Tuple[float, float]:
        """
        Calculate next target point on circle circumference
        
        Strategy:
        1. Calculate target angle based on progress
        2. Find closest undrawn segment
        3. Ensure stroke length doesn't exceed 5mm
        4. Handle circle closure when 95% complete
        """
        # Calculate target angle (progressive drawing)
        target_angle = current_progress * 2 * math.pi
        
        # Find next segment to draw
        segment_angle = 2 * math.pi / self.segments
        next_segment = int(current_progress * self.segments)
        
        # Calculate next point on circumference
        next_angle = next_segment * segment_angle
        next_x = target_circle.center[0] + target_circle.radius * math.cos(next_angle)
        next_y = target_circle.center[1] + target_circle.radius * math.sin(next_angle)
        
        # Check if we need to close the circle
        if current_progress >= self.closure_threshold:
            # Return to start point to close circle
            start_x = target_circle.center[0] + target_circle.radius
            start_y = target_circle.center[1]
            return start_x, start_y
        
        # Ensure stroke length doesn't exceed maximum
        current_x, current_y = current_position
        distance = math.sqrt((next_x - current_x)**2 + (next_y - current_y)**2)
        
        if distance > self.max_stroke_length:
            # Interpolate to maximum stroke length
            ratio = self.max_stroke_length / distance
            next_x = current_x + (next_x - current_x) * ratio
            next_y = current_y + (next_y - current_y) * ratio
        
        return next_x, next_y
    
    def _is_point_on_circle(self, contact_point: ContactPoint, target_circle: TargetCircle) -> bool:
        """Check if contact point is on target circle perimeter (within tolerance)"""
        pixel_x, pixel_y = contact_point.position_2d
        center_x, center_y = target_circle.center
        
        # Calculate distance from center
        distance = math.sqrt((pixel_x - center_x)**2 + (pixel_y - center_y)**2)
        
        # Check if within circle tolerance (±5 pixels)
        tolerance = 5.0
        return abs(distance - target_circle.radius) <= tolerance
    
    def _calculate_angle_from_center(self, x: float, y: float, target_circle: TargetCircle) -> float:
        """Calculate angle of point relative to circle center"""
        center_x, center_y = target_circle.center
        return math.atan2(y - center_y, x - center_x)
    
    def _calculate_angular_coverage(self, angles: List[float]) -> float:
        """Calculate total angular coverage from sorted angle list"""
        if len(angles) < 2:
            return 0.0
        
        # Handle angle wrapping and calculate total coverage
        total_coverage = 0.0
        for i in range(len(angles) - 1):
            gap = angles[i + 1] - angles[i]
            if gap > math.pi:  # Handle wrap-around
                gap = 2 * math.pi - gap
            total_coverage += gap
        
        return total_coverage
```

**Circle Configuration**:
```yaml
circle_drawing:
  segments: 24              # Number of circle segments
  max_stroke_length: 0.005  # 5mm maximum stroke length
  closure_threshold: 0.95   # 95% completion before closing
  perimeter_tolerance: 5.0  # ±5 pixels tolerance for circle detection
  
  # Default circle parameters
  default_center: [400, 300]  # Canvas center
  default_radius: 80         # 80 pixels radius
```

**Progress Tracking**:
- **Real-time**: Progress updates based on actual contact points
- **Accuracy**: Contact-based progress, not coordinate-based
- **Closure**: Automatic return to start point when 95% complete
- **Smoothness**: 5mm maximum stroke length ensures smooth curves

### 5. **Error Handling and Recovery - DEFINED**
**Comprehensive error handling strategy for robust operation**:

**Error Categories and Response Strategies**:

**1. Robot Movement Errors**:
```python
class RobotMovementError(Exception):
    """Robot failed to reach target position"""
    pass

class RobotErrorHandler:
    def __init__(self, config):
        self.max_retries = config.get('max_retries', 3)
        self.retry_delay = config.get('retry_delay', 0.1)  # 100ms
        self.position_tolerance = config.get('position_tolerance', 0.002)  # 2mm
        
    def handle_movement_failure(self, target_position: Tuple[float, float, float], 
                                actual_position: Tuple[float, float, float]) -> bool:
        """
        Handle robot movement failures with retry mechanism
        
        Strategy:
        1. Check if position is within tolerance
        2. Retry movement up to max_retries times
        3. If still failing, try safe position recovery
        4. Log failure for debugging
        """
        distance = self._calculate_distance(target_position, actual_position)
        
        if distance <= self.position_tolerance:
            return True  # Close enough
        
        # Retry movement
        for attempt in range(self.max_retries):
            time.sleep(self.retry_delay)
            success = self._retry_movement(target_position)
            if success:
                return True
        
        # Failed after retries - try safe recovery
        self._safe_position_recovery()
        return False
    
    def _safe_position_recovery(self):
        """Move robot to safe position and lift pen"""
        safe_position = [0.5, 0.0, 0.15]  # Canvas center, pen up
        self._move_to_safe_position(safe_position)
```

**2. Contact Detection Errors**:
```python
class ContactDetectionError(Exception):
    """Contact detection system failure"""
    pass

class ContactErrorHandler:
    def __init__(self, config):
        self.detection_timeout = config.get('detection_timeout', 1.0)  # 1 second
        self.fallback_contact_height = config.get('fallback_height', 0.105)  # 10.5cm
        
    def handle_detection_failure(self, pen_position: Tuple[float, float, float]) -> bool:
        """
        Handle contact detection failures
        
        Strategy:
        1. Use fallback contact detection (Z-height only)
        2. Estimate contact based on planned position
        3. Mark as uncertain contact for user feedback
        4. Continue with degraded accuracy
        """
        # Fallback: assume contact if pen is at expected height
        pen_z = pen_position[2]
        if abs(pen_z - self.fallback_contact_height) <= 0.01:  # 1cm tolerance
            return True  # Assume contact
        
        return False  # No contact
    
    def log_detection_uncertainty(self, position: Tuple[float, float, float]):
        """Log uncertain contact detection for debugging"""
        self.get_logger().warning(
            f"Contact detection uncertain at position {position}. "
            f"Using fallback detection."
        )
```

**3. Canvas Update Errors**:
```python
class CanvasUpdateError(Exception):
    """Canvas state update failure"""
    pass

class CanvasErrorHandler:
    def __init__(self, config):
        self.backup_interval = config.get('backup_interval', 10)  # Backup every 10 updates
        self.canvas_backup = None
        self.update_counter = 0
        
    def handle_canvas_update_failure(self, failed_update: ContactPoint, 
                                    canvas_state: CanvasState) -> bool:
        """
        Handle canvas update failures
        
        Strategy:
        1. Retry update once
        2. If still failing, skip this update
        3. Log missing update for post-processing
        4. Continue with drawing
        """
        try:
            # Retry update
            self._retry_canvas_update(failed_update, canvas_state)
            return True
        except Exception as e:
            # Log and skip this update
            self._log_missed_update(failed_update, e)
            return False
    
    def create_canvas_backup(self, canvas_state: CanvasState):
        """Create backup of canvas state for recovery"""
        self.canvas_backup = {
            'drawn_strokes': canvas_state.drawn_strokes.copy(),
            'contact_history': canvas_state.contact_history.copy(),
            'current_canvas_image': canvas_state.current_canvas_image.copy(),
            'timestamp': time.time()
        }
```

**4. System-Level Error Recovery**:
```python
class SystemErrorRecovery:
    def __init__(self, config):
        self.emergency_stop_enabled = config.get('emergency_stop', True)
        self.auto_recovery_enabled = config.get('auto_recovery', True)
        self.recovery_timeout = config.get('recovery_timeout', 30.0)  # 30 seconds
        
    def handle_system_failure(self, error_type: str, error_details: str) -> bool:
        """
        Handle system-level failures
        
        Strategy:
        1. Emergency stop if critical failure
        2. Attempt automatic recovery
        3. Fall back to safe mode
        4. Notify user of system state
        """
        if self._is_critical_failure(error_type):
            self._emergency_stop()
            return False
        
        if self.auto_recovery_enabled:
            recovery_success = self._attempt_recovery(error_type)
            if recovery_success:
                return True
        
        # Fall back to safe mode
        self._enter_safe_mode()
        return False
    
    def _emergency_stop(self):
        """Emergency stop - halt all robot movement"""
        # Stop robot movement
        # Lift pen to safe position
        # Save current state
        # Notify user
        pass
    
    def _enter_safe_mode(self):
        """Enter safe mode - limited functionality"""
        # Disable complex operations
        # Enable basic movement only
        # Increase error tolerances
        # Notify user of degraded performance
        pass
```

**Error Handling Configuration**:
```yaml
error_handling:
  # Movement errors
  max_movement_retries: 3
  movement_retry_delay: 0.1    # 100ms
  position_tolerance: 0.002    # 2mm
  
  # Contact detection errors
  detection_timeout: 1.0       # 1 second
  fallback_contact_height: 0.105  # 10.5cm
  
  # Canvas update errors
  canvas_backup_interval: 10   # Backup every 10 updates
  
  # System-level errors
  emergency_stop: true
  auto_recovery: true
  recovery_timeout: 30.0       # 30 seconds
  
  # Logging
  error_log_level: "WARNING"
  detailed_error_logging: true
```

**Recovery Strategies**:
- **Movement failures**: Retry with exponential backoff, then safe position recovery
- **Contact detection**: Fallback to Z-height estimation, continue with warnings
- **Canvas updates**: Skip failed updates, maintain backup for recovery
- **System failures**: Emergency stop for critical errors, safe mode for others

### 6. **Integration with Existing Robot System - DEFINED**
**Seamless integration with existing robot infrastructure**:

**Integration Strategy**: Build on existing components rather than replacing them

**1. Robot Control Integration**:
```python
# Use existing RobotCircleDrawer patterns from robot_draw_circle.py
from demos.robot_draw_circle import RobotCircleDrawer

class Phase1RobotController(RobotCircleDrawer):
    """Extend existing robot controller for Phase 1 functionality"""
    
    def __init__(self):
        super().__init__()
        # Inherit existing joint publishing and interpolation
        # Add Phase 1 specific functionality
        self.contact_detector = ContactDetector(config)
        self.canvas_system = CanvasCoordinateSystem()
        
    def move_to_pixel_position(self, pixel_x: float, pixel_y: float) -> bool:
        """Move robot to pixel position using existing joint control"""
        # Convert pixel to joint positions (using existing method)
        target_joints = self.pixel_to_joint_position(pixel_x, pixel_y)
        
        # Use existing smooth interpolation
        self.current_joints = self.get_current_joint_positions()
        self.target_joints = target_joints
        self.current_step = 0
        
        return True
    
    def pixel_to_joint_position(self, pixel_x: float, pixel_y: float) -> List[float]:
        """Convert pixel coordinates to joint positions (reuse existing logic)"""
        # Use existing base_joints configuration
        base_joints = [0.0, -1.2, -1.0, -1.5, 1.57, 0.0]
        
        # Convert pixel to robot coordinates
        robot_x, robot_y, robot_z = self.canvas_system.pixel_to_robot_coords(
            pixel_x, pixel_y, pen_down=True
        )
        
        # Map to joint space using existing parameters
        canvas_center_x, canvas_center_y = 0.5, 0.0
        pan_amplitude = 0.6   # From existing system
        lift_amplitude = 0.3  # From existing system
        
        # Calculate joint offsets
        pan_offset = (robot_x - canvas_center_x) / 0.2 * pan_amplitude
        lift_offset = (robot_y - canvas_center_y) / 0.2 * lift_amplitude
        
        # Apply to base configuration
        joints = base_joints.copy()
        joints[0] = base_joints[0] + pan_offset   # shoulder_pan_joint
        joints[1] = base_joints[1] + lift_offset  # shoulder_lift_joint
        
        return joints
```

**2. Coordinate System Integration**:
```python
# Direct usage of existing canvas_coordinate_system.py
from scripts.canvas_coordinate_system import CanvasCoordinateSystem

class Phase1CoordinateManager:
    def __init__(self):
        # Use existing coordinate system directly
        self.canvas_system = CanvasCoordinateSystem()
        
    def convert_coordinates(self, pixel_x: float, pixel_y: float) -> Tuple[float, float, float]:
        """Convert pixel to robot coordinates using existing system"""
        return self.canvas_system.pixel_to_robot_coords(pixel_x, pixel_y, pen_down=True)
    
    def validate_position(self, pixel_x: float, pixel_y: float) -> bool:
        """Validate position using existing system"""
        return self.canvas_system.validate_coordinates(pixel_x, pixel_y)
    
    def get_canvas_bounds(self) -> Dict:
        """Get canvas bounds using existing system"""
        return self.canvas_system.get_canvas_bounds()
```

**3. Launch System Integration**:
```python
# Create Phase 1 launch file that extends existing launch system
# File: launch/phase1_demo.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Include existing UR5e launch
    ur5e_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot'),
                'launch',
                'ur5e_standard.launch.py'
            )
        )
    )
    
    # Add Phase 1 nodes
    phase1_main = Node(
        package='robot',
        executable='phase1_main',
        name='phase1_main',
        output='screen',
        parameters=[{
            'config_file': os.path.join(
                get_package_share_directory('robot'),
                'demos', 'phase1', 'config.yaml'
            )
        }]
    )
    
    # Add canvas preview window
    canvas_preview = Node(
        package='robot',
        executable='canvas_preview',
        name='canvas_preview',
        output='screen'
    )
    
    return LaunchDescription([
        ur5e_launch,
        phase1_main,
        canvas_preview
    ])
```

**4. Configuration Integration**:
```yaml
# demos/phase1/config.yaml - extends existing configurations
phase1:
  # Use existing robot parameters
  robot:
    joint_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    base_joints: [0.0, -1.2, -1.0, -1.5, 1.57, 0.0]
    pan_amplitude: 0.6
    lift_amplitude: 0.3
    interpolation_steps: 25
    update_rate: 50  # 50Hz like existing system
  
  # Use existing canvas parameters
  canvas:
    physical_size: 0.4        # 40cm (from existing system)
    pixel_width: 800          # From existing system
    pixel_height: 600         # From existing system
    position: [0.5, 0.0, 0.1] # From existing system
    contact_height: 0.005     # 5mm (from existing system)
    safe_height: 0.05         # 5cm (from existing system)
  
  # Phase 1 specific parameters
  drawing:
    circle_segments: 24
    max_stroke_length: 0.005
    closure_threshold: 0.95
```

**5. ROS2 Communication Integration**:
```python
# Reuse existing ROS2 patterns from robot_draw_circle.py
class Phase1ROSInterface:
    def __init__(self):
        # Use existing joint state publisher
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Add Phase 1 specific topics
        self.canvas_state_pub = self.create_publisher(CanvasState, '/canvas_state', 10)
        self.drawing_progress_pub = self.create_publisher(DrawingProgress, '/drawing_progress', 10)
        
        # Use existing timer pattern
        self.timer = self.create_timer(0.02, self.update_callback)  # 50Hz like existing
    
    def publish_joint_state(self, joint_positions: List[float]):
        """Publish joint state using existing message format"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names  # From existing system
        msg.position = joint_positions
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        
        self.joint_pub.publish(msg)
```

**6. Development Workflow Integration**:
```bash
# Use existing development commands
# Start existing robot simulation
ros2 launch robot ur5e_standard.launch.py

# Start Phase 1 demo (extends existing system)
ros2 launch robot phase1_demo.launch.py

# Test with existing coordinate system
python3 scripts/canvas_coordinate_system.py

# Test with existing robot control
python3 demos/robot_draw_circle.py
```

**Integration Benefits**:
- **Reuse existing code**: 70% of functionality already implemented
- **Maintain compatibility**: Existing demos still work
- **Extend gracefully**: Phase 1 adds features without breaking existing system
- **Leverage testing**: Existing robot control already tested and working
- **Consistent parameters**: Same coordinate system, joint configurations, and update rates

### 7. **Performance Requirements - DEFINED**
**Realistic performance targets based on existing system capabilities**:

**Real-Time Performance Targets**:

**1. Update Frequencies**:
```yaml
performance:
  # Robot control (based on existing robot_draw_circle.py)
  robot_update_rate: 50        # 50Hz joint state updates (proven in existing system)
  interpolation_steps: 25      # 25 steps between waypoints (smooth motion)
  
  # Canvas updates (contact-based)
  canvas_update_rate: 30       # 30Hz canvas state updates (sufficient for visual feedback)
  contact_detection_rate: 100  # 100Hz contact detection (responsive to pen contact)
  
  # UI updates (preview window)
  preview_window_fps: 24       # 24 FPS canvas preview (smooth visual experience)
  rviz_update_rate: 30         # 30Hz RViz updates (matches robot visualization)
```

**2. Latency Requirements**:
```yaml
latency:
  # Robot response
  robot_command_latency: 20    # 20ms max delay from command to movement
  joint_interpolation_latency: 2  # 2ms between interpolation steps
  
  # Contact detection
  contact_detection_latency: 10   # 10ms max delay for contact detection
  canvas_update_latency: 33      # 33ms max delay for canvas updates (30Hz)
  
  # User interface
  preview_update_latency: 42     # 42ms max delay for preview updates (24 FPS)
  user_feedback_latency: 100     # 100ms max delay for user feedback
```

**3. Memory Usage Limits**:
```yaml
memory:
  # Canvas state
  canvas_image_size: 1.44      # 1.44MB (800x600x3 bytes)
  contact_history_limit: 1000  # Max 1000 contact points (~50KB)
  stroke_history_limit: 100    # Max 100 strokes (~10KB)
  
  # System state
  system_state_size: 0.1       # 100KB max system state
  error_log_size: 1.0          # 1MB max error log
  
  # Total memory target
  total_memory_usage: 10       # 10MB max total memory usage
```

**4. Processing Strategy**:
```python
class PerformanceManager:
    def __init__(self, config):
        self.update_frequencies = config['performance']
        self.latency_limits = config['latency']
        self.memory_limits = config['memory']
        
        # Real-time processing decisions
        self.real_time_modules = [
            'robot_control',      # Must be real-time (50Hz)
            'contact_detection',  # Must be real-time (100Hz)
        ]
        
        self.batch_processing_modules = [
            'canvas_rendering',   # Can be batched (30Hz)
            'progress_calculation', # Can be batched (10Hz)
            'error_logging',      # Can be batched (1Hz)
        ]
    
    def check_performance_targets(self) -> Dict[str, bool]:
        """Monitor performance against targets"""
        results = {}
        
        # Check update rates
        results['robot_update_rate'] = self._check_update_rate('robot_control', 50)
        results['canvas_update_rate'] = self._check_update_rate('canvas_updates', 30)
        
        # Check latency
        results['robot_latency'] = self._check_latency('robot_commands', 20)
        results['contact_latency'] = self._check_latency('contact_detection', 10)
        
        # Check memory usage
        results['memory_usage'] = self._check_memory_usage()
        
        return results
    
    def optimize_performance(self, performance_results: Dict[str, bool]):
        """Adjust performance based on current system load"""
        if not performance_results['robot_update_rate']:
            # Reduce non-critical update rates
            self._reduce_canvas_update_rate()
            self._reduce_preview_fps()
        
        if not performance_results['memory_usage']:
            # Clean up memory
            self._cleanup_contact_history()
            self._cleanup_stroke_history()
```

**5. Real-Time vs. Batch Processing**:

**Real-Time Processing (Critical)**:
```python
# Robot control - MUST be real-time
class RealTimeRobotControl:
    def __init__(self):
        self.update_rate = 50  # 50Hz - critical for smooth motion
        self.max_latency = 20  # 20ms max latency
        
    def process_robot_command(self, command):
        """Process robot commands in real-time"""
        # Must complete within 20ms
        # Direct hardware control
        # No blocking operations
        pass

# Contact detection - MUST be real-time
class RealTimeContactDetection:
    def __init__(self):
        self.update_rate = 100  # 100Hz - responsive contact detection
        self.max_latency = 10   # 10ms max latency
        
    def detect_contact(self, robot_position):
        """Detect contact in real-time"""
        # Fast contact detection algorithm
        # Minimal computational overhead
        # No file I/O or network operations
        pass
```

**Batch Processing (Non-Critical)**:
```python
# Canvas rendering - Can be batched
class BatchCanvasRenderer:
    def __init__(self):
        self.update_rate = 30     # 30Hz - sufficient for visual feedback
        self.batch_size = 10      # Process 10 contact points at once
        
    def render_canvas_batch(self, contact_points):
        """Render canvas updates in batches"""
        # Process multiple contact points together
        # More efficient than individual updates
        # Can tolerate slight delays
        pass

# Progress calculation - Can be batched
class BatchProgressCalculator:
    def __init__(self):
        self.update_rate = 10     # 10Hz - sufficient for progress tracking
        
    def calculate_progress_batch(self, canvas_state):
        """Calculate drawing progress in batches"""
        # Complex calculations can be batched
        # Not time-critical
        # Can use more computational resources
        pass
```

**6. Performance Monitoring**:
```python
class PerformanceMonitor:
    def __init__(self):
        self.performance_log = []
        self.alert_thresholds = {
            'robot_latency': 20,     # 20ms
            'contact_latency': 10,   # 10ms
            'memory_usage': 10,      # 10MB
            'cpu_usage': 70,         # 70%
        }
    
    def log_performance_metrics(self):
        """Log performance metrics for monitoring"""
        metrics = {
            'timestamp': time.time(),
            'robot_update_rate': self._measure_update_rate('robot'),
            'canvas_update_rate': self._measure_update_rate('canvas'),
            'memory_usage': self._measure_memory_usage(),
            'cpu_usage': self._measure_cpu_usage(),
        }
        
        self.performance_log.append(metrics)
        
        # Check for performance issues
        self._check_performance_alerts(metrics)
    
    def generate_performance_report(self) -> Dict:
        """Generate performance report for debugging"""
        if not self.performance_log:
            return {}
        
        recent_metrics = self.performance_log[-100:]  # Last 100 measurements
        
        return {
            'average_robot_rate': np.mean([m['robot_update_rate'] for m in recent_metrics]),
            'average_canvas_rate': np.mean([m['canvas_update_rate'] for m in recent_metrics]),
            'max_memory_usage': max([m['memory_usage'] for m in recent_metrics]),
            'average_cpu_usage': np.mean([m['cpu_usage'] for m in recent_metrics]),
            'performance_issues': self._identify_performance_issues(recent_metrics)
        }
```

**7. Performance Optimization Strategies**:

**Computational Optimization**:
- **Vectorized operations**: Use NumPy for canvas operations
- **Efficient data structures**: Use appropriate containers for contact history
- **Memory pooling**: Reuse objects to reduce garbage collection
- **Lazy evaluation**: Only compute when needed

**I/O Optimization**:
- **Asynchronous updates**: Non-blocking canvas updates
- **Buffered logging**: Batch log writes
- **Minimal file operations**: Avoid frequent file I/O

**Threading Strategy**:
- **Real-time thread**: Robot control and contact detection
- **Rendering thread**: Canvas updates and preview window
- **Logging thread**: Error logging and performance monitoring

**Performance Validation**:
```bash
# Performance testing commands
# Monitor robot update rate
ros2 topic hz /joint_states

# Monitor canvas update rate
ros2 topic hz /canvas_state

# Monitor system resources
top -p $(pgrep -f phase1_main)

# Performance profiling
python3 -m cProfile demos/phase1/main.py
```

**Success Criteria**:
- **Robot control**: Consistent 50Hz updates with <20ms latency
- **Contact detection**: 100Hz detection with <10ms latency
- **Canvas updates**: 30Hz rendering with <33ms latency
- **Memory usage**: <10MB total system memory
- **Smooth operation**: No dropped frames or stuttering motion

## 🎯 Success Criteria

### Visual Success
- [ ] Robot draws recognizable circle shape
- [ ] Pen maintains contact with canvas throughout drawing
- [ ] Smooth, continuous line (no gaps or jumps)
- [ ] Circle closes properly (start connects to end)

### Technical Success
- [ ] All 3 modules communicate correctly
- [ ] Canvas state updates accurately reflect drawing progress
- [ ] Stroke generation produces valid waypoints
- [ ] Robot executes strokes smoothly
- [ ] Error handling and recovery functional

### Configuration Success
- [ ] Canvas size easily configurable
- [ ] Circle parameters adjustable
- [ ] Stroke length and density configurable
- [ ] System works with different canvas positions

## 📋 Implementation Priority

### Week 1: Individual Modules
1. Implement Module 1 (coordinate calculator)
2. Implement Module 2 (stroke generator)
3. Test modules independently

### Week 2: Integration
1. Implement Module 3 (robot executor)
2. Create ROS2 service interfaces
3. Integrate all modules

### Week 3: Testing & Polish
1. End-to-end testing
2. Parameter tuning
3. Error handling improvement

## ✅ All Specifications Complete - Ready for Implementation

### ✅ Completed Specifications
1. **✅ Robot Control Interface** - 2-DOF control using existing `robot_draw_circle.py` patterns
2. **✅ Contact Detection Algorithm** - 5mm tolerance, configurable parameters, realistic contact detection
3. **✅ Canvas Coordinate Conversion** - Direct integration with existing `canvas_coordinate_system.py`
4. **✅ Circle Drawing Algorithm** - 24-segment progressive drawing with contact-based progress tracking
5. **✅ Error Handling and Recovery** - Comprehensive error handling with retry mechanisms and safe recovery
6. **✅ Integration with Existing Robot System** - Seamless extension of existing components without breaking compatibility
7. **✅ Performance Requirements** - 50Hz robot control, 30Hz canvas updates, <10MB memory usage

### Implementation Progress (2025-07-15)
1. **✅ Create `demos/phase1/` directory structure** - Complete
2. **✅ Create `demos/phase1/config.yaml`** - Complete 
3. **✅ Implement `demos/phase1/system_state.py`** - Complete
4. **✅ Implement `demos/phase1/coordinate_calculator.py`** - Complete (修正済み)
5. **🚧 Implement `demos/phase1/stroke_generator.py`** - Partial (integrated in main.py)
6. **🚧 Implement `demos/phase1/robot_executor.py`** - Partial (integrated in main.py)
7. **✅ Implement `demos/phase1/canvas_preview.py`** - Canvas Preview Window完了
8. **✅ Implement `demos/phase1/main.py`** - Complete (ROS2統合済み)
9. **✅ Create `launch/phase1_demo.launch.py`** - Complete

### VNC Testing Results (2025-07-15)
**✅ Phase 1A Core Functionality**:
- ROS2統合とロボット制御：動作確認済み
- 円描画アルゴリズム：95%完了で正常動作
- RViz統合：3Dロボット動作の視覚確認完了
- 制御信号競合：`jsp_gui:=false`で解決済み

**🔄 Next Priority: Canvas Preview Window**
- **目的**: Phase 1設計の「Dual Display System」完成
- **機能**: リアルタイム描画進捗の2D可視化
- **統合**: Wanderlineキャンバスシステムとの連携

### Research Task (完了)
- **✅ Wanderline canvas system調査完了**
   - ✅ `wanderline/canvas.py`: apply_stroke()とvectorized処理確認
   - ✅ `wanderline/video_recorder.py`: VideoRecorderクラス確認
   - ✅ `wanderline/realtime_visualizer.py`: RealtimeVisualizerの詳細分析
   - ✅ `wanderline/drawing_engine.py`: visualizer.update()の使用方法確認
   - ✅ Robot用canvas preview実装: Wanderlineスタイルで実装完了

### Canvas Preview Window Implementation (完了)
**✅ Wanderlineスタイル実装**:
- **RealtimeVisualizer分析**: OpenCVウィンドウ初期化、更新ループ、キー処理
- **技術的改善**: WINDOW_NORMAL | WINDOW_KEEPRATIO、topmost設定、500ms初期化待機
- **エラーハンドリング**: 堅牢な例外処理とフォールバック
- **統合**: main.pyとの条件付き統合（OpenCV未インストール環境対応）

### Development Approach
- **✅ Core functionality**: ROS2統合、ロボット制御完了
- **✅ Visual enhancement**: Canvas preview window実装完了（Wanderlineベース）
- **💡 Proven approach**: VNC環境での動作確認済み
- **🎯 Achievement**: Dual Display System実装完了

---

**Status**: ✅ Phase 1A Implementation Complete - Dual Display System Ready

**Next Step**: VNC環境での完全動作確認とCanvas Preview Window表示テスト

### VNC環境テスト手順 (Canvas Preview Window)
```bash
# 1. OpenCV確認
docker exec docker-wanderline-robot-1 python3 -c "import cv2; print('OpenCV version:', cv2.__version__)"

# 2. 必要に応じてOpenCVインストール
docker exec docker-wanderline-robot-1 pip install opencv-python

# 3. Dual Display System実行
# Terminal 1: Robot + RViz
ros2 launch /workspace/robot/launch/ur5e_standard.launch.py jsp_gui:=false use_rviz:=true

# Terminal 2: Phase 1 + Canvas Preview
cd /workspace/robot/demos/phase1
export DISPLAY=:1.0
python3 main.py
```

**期待結果**: Window 1 (RViz) + Window 2 (Canvas Preview) 同時表示