# TODO / Task List for Wanderline

## Purpose & Guidelines

This document tracks development tasks for the Wanderline project, organized by implementation stage to clarify what needs discussion versus what's ready for implementation.

### Task Categories

- **Discussion Needed**: Tasks requiring specification discussion or design decisions
- **Ready for Implementation**: Tasks with confirmed specifications, ready to be implemented
- **In Progress**: Tasks currently being worked on
- **Completed**: Finished and tested tasks

### Update Rules

1. **When adding new tasks**: Place them in "Discussion Needed" unless specifications are already clear
2. **When specifications are confirmed**: Move tasks from "Discussion Needed" to "Ready for Implementation"
3. **When starting work**: Move tasks to "In Progress" and assign a brief status note
4. **When completing work**: Move to "Completed" with completion date and brief summary
5. **Regular cleanup**: Archive old completed tasks to maintain document readability

### Task Format

Use clear, concise descriptions with technical details when needed. Include:
- **Context**: Why this task is needed
- **Scope**: What needs to be implemented
- **Dependencies**: What other tasks must be completed first
- **Success criteria**: How to know when the task is complete

---

## Discussion Needed

### Advanced Optimization Strategies
**Context**: Current greedy search is naive gradient descent. Need to decide which optimization methods to prioritize.

- **Adam-style Angle Optimizer**: Momentum-based angle selection with past gradient information
  - Need to determine: learning rate, momentum parameters, exploration noise strategy
  - Question: How to handle angle wrapping in momentum calculations?

- **Q-Learning for Angle Selection**: Learn long-term value of angle choices
  - Need to determine: state representation, action discretization, reward shaping
  - Question: How many angle bins? How to handle continuous state space?

- **Variable Stroke Length Optimization**: Joint optimization of angle and stroke length
  - Need to determine: parameter bounds, constraint handling, loss function impact
  - Question: How does this affect loss function convexity?

### Transformer Agent Development
**Context**: Current implementation uses MLP instead of actual transformer. Need to decide architecture and training approach.

- **Actual Transformer Architecture**: Replace current MLP with transformer model
  - Need to determine: attention mechanism, sequence length, embedding dimensions
  - Question: How to handle variable-length sequences? Multi-head attention configuration?

- **Expert Trajectory Generation**: Generate training data from existing greedy agent
  - Need to determine: trajectory format, sampling strategy, data augmentation
  - Question: How many trajectories needed? How to ensure diversity?

- **Training Pipeline**: Supervised pre-training and RL fine-tuning
  - Need to determine: loss functions, training schedule, evaluation metrics
  - Question: Pre-training first or end-to-end training? Which RL algorithm?

### Memory Optimization for Long Runs
**Context**: 10000+ step runs cause system crashes. Need to decide on memory management strategy.

- **Coordinate-only Recording**: Record xy coordinates, reconstruct video later
  - Need to determine: storage format, reconstruction algorithm, canvas state handling
  - Question: How to balance memory savings vs reconstruction complexity?

- **Streaming Video Generation**: Generate video in chunks to avoid memory buildup
  - Need to determine: chunk size, temporary file management, quality settings
  - Question: How to handle video continuity across chunks?

## Ready for Implementation

### Real-time Canvas Visualization
**Context**: Need visual feedback during drawing process for debugging and user experience.

- **OpenCV Real-time Window**: Display live canvas updates using `cv2.imshow()`
  - Requirements: Non-blocking updates, keyboard shortcuts (pause/resume/save)
  - Implementation: Add to `realtime_visualizer.py`, integrate with drawing loop
  - Success criteria: Smooth updates without performance impact

- **Periodic Snapshot Saving**: Save canvas states at configurable intervals
  - Requirements: Organized directory structure, configurable frequency
  - Implementation: Extend `run_manager.py` with snapshot functionality
  - Success criteria: Snapshots saved without affecting drawing performance

- **ASCII Terminal Display**: Convert canvas to ASCII for terminal-only environments
  - Requirements: Configurable resolution, grayscale-to-character mapping
  - Implementation: Add ASCII converter utility, integrate with visualizer
  - Success criteria: Readable ASCII representation updating in real-time

### Configuration System Enhancement
**Context**: Need better organization of visualization and optimizer settings.

- **Visualization Configuration**: Add structured config for visualization options
  - Requirements: Mode selection, update frequency, window size settings
  - Implementation: Extend `config_manager.py` with visualization section
  - Success criteria: All visualization options configurable via JSON/CLI

- **Optimizer Configuration**: Add structured config for optimization strategies
  - Requirements: Optimizer type, learning parameters, temperature settings
  - Implementation: Create optimizer config section, update CLI parsing
  - Success criteria: Easy switching between optimization strategies

### Progress Monitoring & Visualization
**Context**: Need detailed feedback during drawing process for debugging and optimization analysis.

- **Verbose Angle Evaluation Mode**: Show angle candidates and rewards for each step
  - Requirements: Display all angle candidates, highlight selected angle with reasoning
  - Implementation: Add debug mode to agent, integrate with visualizer
  - Success criteria: Clear display of angle selection process without performance impact

- **Interactive Controls**: Keyboard controls for drawing process
  - Requirements: Pause/resume with keyboard, save current state, manual intervention mode
  - Implementation: Add keyboard handler to drawing loop, state save/load functionality
  - Success criteria: Responsive controls that don't interfere with drawing performance

- **Live Performance Metrics**: Real-time performance monitoring
  - Requirements: Memory usage tracking, steps/second calculation, convergence analysis
  - Implementation: Add metrics collector, integrate with visualizer display
  - Success criteria: Accurate metrics updated in real-time

- **Multi-panel Dashboard**: Split view display for comprehensive monitoring
  - Requirements: Current canvas, target motif, difference map, performance graphs
  - Implementation: Extend visualizer with multi-panel layout, real-time graph updates
  - Success criteria: Clear, organized display that updates smoothly

## In Progress

### Documentation Updates
**Status**: Updating config guide and README for recent changes
- **Config Guide**: Adding new white penalty examples and multi-step settings
- **README**: Reorganizing project structure documentation

## Completed

### White Penalty System (2025-06-24)
- **Implementation**: L2_white_penalty reward function with configurable penalty
- **Configuration**: Added white penalty settings to config files and CLI
- **Testing**: All tests updated and passing with white penalty functionality
- **Documentation**: Config guide updated with white penalty usage examples

### Multi-step Lookahead Analysis (2025-06-25)
- **Memory Analysis**: Implemented `debug/memory_analysis.py` for memory usage profiling
- **Practical Limits**: Created `debug/practical_limits.py` for performance recommendations
- **Established Settings**: Determined optimal n_samples (36 for 1-step, 12-16 for 2-step)
- **Documentation**: Added performance recommendations to configuration guide

### Vectorized Multi-Step Agent (2025-06-25)
- **Implementation**: Efficient 2-step lookahead with practical memory limits
- **Performance Analysis**: Determined optimal n_samples (36 for 1-step, 12-16 for 2-step)
- **Integration**: Fully integrated with drawing engine, all tests passing
- **Documentation**: Updated with usage examples and performance recommendations

### Parameter Naming Improvements (2025-06-24)
- **Change**: Renamed confusing parameters (ratio/alpha) to clearer names
- **Implementation**: Updated to absolute_penalty/relative_penalty system
- **Testing**: All tests updated and passing
- **Documentation**: Config guide updated with new parameter names

### Code Cleanup (2025-06-25)
- **Scope**: Removed unused functions, improved English documentation
- **Result**: Reduced agent.py from 611 to 372 lines
- **Quality**: Improved code readability and maintainability
- **Testing**: All existing functionality preserved

---

*This document should be updated whenever task status changes. Use the categories above to maintain clear development priorities.*
