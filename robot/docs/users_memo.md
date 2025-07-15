# Users Memo - Robot Drawing Project

## <¯ Project Goal

Create a system where **Wanderline** algorithms compute drawing decisions in real-time, and the **robot** executes the drawing movements physically.

**Vision**: Real-time algorithmic drawing ’ Physical robot execution

## =Ë Current Status

###  What We Have Working
- Robot arm visualization in RViz (UR5e)
- Basic robot movement demos (circle drawing)
- Docker environment setup
- Clean file structure

### <¯ Next Implementation Target

**Phase 1: Robot Circle Drawing with Wanderline Integration**

Create a circle drawing system where:
1. Wanderline algorithm calculates optimal stroke angles
2. Robot receives these angles in real-time
3. Robot physically draws the circle with computed angles

## =' Implementation Plan

### Step 1: Circle Drawing Foundation
**Goal**: Get robot to draw a circle using Wanderline's approach

**Technical Requirements**:
- Canvas coordinate system (pixel ’ robot coordinates)
- Pen position control (up/down)
- Real-time angle computation
- Stroke-by-stroke execution

**Expected Behavior**:
```
Wanderline Algorithm ’ Angle Decisions ’ Robot Movement ’ Physical Drawing
```

### Step 2: Integration Architecture

**Components Needed**:

1. **Wanderline Interface**
   - Use existing Wanderline core functions
   - Real-time angle optimization
   - Canvas state management

2. **Robot Controller**
   - Convert angles to robot coordinates
   - Control pen position (up/down)
   - Execute smooth movements

3. **Communication Bridge**
   - ROS2 topics for real-time data
   - Coordinate conversion utilities
   - State synchronization

### Step 3: Circle Drawing Flow

**Process Flow**:
```
1. Initialize canvas (blank state)
2. Set target: circle shape
3. For each drawing step:
   a. Wanderline computes next best angle
   b. Convert angle to robot coordinates
   c. Move robot to position
   d. Apply stroke (pen down ’ move ’ pen up)
   e. Update canvas state
   f. Repeat until circle complete
```

## =Ð Technical Specifications

### Canvas Setup
- Physical size: 40cm × 40cm
- Digital resolution: 800×600 pixels
- Robot coordinate system: base_link origin
- Canvas position: [0.5, 0.0, 0.1] meters

### Circle Target
- Center: Canvas center (400, 300) pixels = (0.5, 0.0) robot coords
- Radius: 80 pixels = ~4cm physical
- Target completion: Smooth circular shape

### Stroke Definition
- Stroke length: 2cm (configurable)
- Pen contact: Z < 0.11 meters
- Pen lift: Z > 0.14 meters
- Movement speed: Smooth interpolation

## <¨ Circle Drawing Strategy

### Approach 1: Wanderline Angle Optimization
Use Wanderline's core algorithm to compute optimal angles for circle approximation:

1. **Start**: Pen at canvas edge
2. **For each stroke**: 
   - Current position + current canvas state ’ Wanderline algorithm
   - Algorithm returns optimal angle for next stroke
   - Convert angle to end position
   - Execute robot movement
3. **Target**: Minimize distance to perfect circle

### Approach 2: Stroke-by-Stroke Computation
Each stroke is independently optimized:
- Input: Current pen position, current canvas state, target circle
- Wanderline computation: Best angle to approach target
- Output: Next stroke angle and length
- Robot execution: Physical movement

## = Integration Points

### Wanderline ’ Robot Bridge

**Data Flow**:
```python
# Wanderline side
current_position = get_pen_position()
canvas_state = get_current_canvas()
target_circle = define_circle_target()

optimal_angle = wanderline_choose_angle(
    canvas_state, 
    current_position, 
    target_circle
)

# Robot side
end_position = calculate_stroke_end(current_position, optimal_angle, stroke_length)
robot_coords = pixel_to_robot_coords(end_position)
execute_robot_movement(robot_coords)
```

### Files to Create/Modify

1. **`demos/wanderline_circle_demo.py`**
   - Main integration demo
   - Wanderline + Robot coordination
   - Real-time circle drawing

2. **`scripts/wanderline_robot_bridge.py`**
   - Communication interface
   - Coordinate conversion
   - State synchronization

3. **Update `scripts/canvas_coordinate_system.py`**
   - Add stroke calculations
   - Angle to coordinate conversion
   - Canvas state management

## <¯ Success Criteria

### Minimum Viable Product (MVP)
- [ ] Robot draws recognizable circle shape
- [ ] Uses Wanderline algorithm for angle decisions
- [ ] Smooth pen movements (no jerky motions)
- [ ] Real-time computation (< 1 second per stroke)

### Enhanced Goals
- [ ] Circle quality comparable to Wanderline simulation
- [ ] Adaptive stroke length based on curvature
- [ ] Real-time visualization of algorithm decisions
- [ ] Error correction and recovery

## =§ Known Challenges

### Technical Challenges
1. **Coordinate Conversion**: Pixel space ” Robot space precision
2. **Real-time Performance**: Wanderline computation speed
3. **Robot Smoothness**: Avoiding jerky movements
4. **State Synchronization**: Canvas state vs. physical reality

### Solutions to Investigate
1. **Pre-computation**: Cache common angle calculations
2. **Interpolation**: Smooth robot movements between waypoints
3. **Feedback Loop**: Monitor actual vs. intended positions
4. **Error Handling**: Recovery from failed strokes

## =Ý Development Notes

### Priority Order
1. **First**: Get basic circle drawing working with simple angle sequence
2. **Second**: Integrate actual Wanderline algorithm
3. **Third**: Optimize for real-time performance
4. **Fourth**: Add visualization and monitoring

### Testing Strategy
- Start with simulation (existing robot_draw_circle.py)
- Add Wanderline integration step by step
- Test each component independently
- Integrate incrementally

---

**Next Action**: Implement `demos/wanderline_circle_demo.py` as the foundation for Wanderline-Robot integration.