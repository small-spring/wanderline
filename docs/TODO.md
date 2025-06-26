# TODO / Task List for Wanderline

## 🚨 CODE CONSERVATION EMERGENCY

**CRITICAL PRIORITY**: Address severe resource waste before any new features.

### Phase 1: Immediate Waste Elimination (URGENT)
**Start here - Low risk, high impact**

1. **Remove 5 Redundant Implementations** ⚡ START HERE
   - Delete: `_choose_next_angle_greedy()`, `_choose_next_angle_lookahead()` in agent.py  
   - Delete: `choose_next_angle_vectorized()`, `choose_next_angle_vectorized_lookahead()` 
   - Keep only: Memory-efficient variants from memory_efficient_canvas.py
   - **Expected**: 65% code reduction (~900 lines removed)

2. **Configuration Explosion Cleanup**
   - Remove: `use_vectorized`, `use_full_vectorization`, `adaptive_sampling`, etc.
   - Keep only: `memory_efficient`, `steps`, `opacity`, `duration` (essential 10)
   - **Expected**: 35+ parameters → 10 parameters

3. **Delete Workaround Files**
   - Delete: `fast_agent.py` (324 lines of workarounds)
   - Integrate memory-efficient logic directly into agent.py
   - **Expected**: 324 lines eliminated

### Phase 2: Architecture Reform (STRATEGIC)
**After Phase 1 stabilizes**

4. **Root Cause Fix: Canvas→Stroke Architecture**
   - Replace: Full canvas copying with incremental pixel evaluation
   - Redesign: `apply_stroke()` to return affected pixel coordinates only
   - **Expected**: 15,000x memory reduction, 2-3x speed improvement

5. **Single Algorithm Path**
   - Unify all angle selection into one well-designed implementation
   - Remove complex routing logic in `choose_next_angle()`
   - **Expected**: Eliminate function call overhead, cleaner codebase

### Success Metrics
- [ ] Memory usage: <1MB per stroke (from 208MB)
- [ ] Code size: <1000 lines in agent system (from 2776)
- [ ] Configuration: <10 parameters (from 45+)
- [ ] Performance: 2x faster than current "optimized" mode

### Risk Assessment
- **Phase 1**: LOW RISK - Just removing redundant code
- **Phase 2**: MEDIUM RISK - Architectural changes, needs testing

**RECOMMENDATION**: Start with #1 (Remove redundant implementations) TODAY.

---

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

### OpenCV Window Display Issues
**Context**: Current real-time visualization has UI/display problems affecting user experience.

**Specific Issues**:
1. ~~**Aspect Ratio Distortion**: OpenCV window displays with incorrect aspect ratio~~ ✅ **COMPLETED**
   - **Solution implemented**: Smart dynamic window sizing with aspect ratio preservation
   - **Features**: 
     - Automatically calculates optimal window size based on canvas aspect ratio
     - Respects maximum constraints (1000×800) to prevent oversized windows
     - Preserves perfect aspect ratio for all motif types (4:3, 16:9, 3:2, etc.)
     - Provides user feedback when window is resized
   - **Tested with**: Default sample (4:3), sample3.png (16:9), sample2.png (3:2)

2. ~~**UI Text Cleanup**: Remove unwanted "p option (print path)" text from OpenCV window~~ ✅ **COMPLETED**
   - Removed 'p' key handler and updated controls text in realtime_visualizer.py
   - Enhanced step display to show "current/total" format matching terminal output

**Status**: ✅ **ALL ISSUES RESOLVED** - OpenCV window now displays with perfect aspect ratio and clean interface

### Advanced Optimization Strategies
**Context**: Core performance bottlenecks have been addressed. Consider advanced methods for further optimization.

- **Adam-style Angle Optimizer**: Momentum-based angle selection with past gradient information
  - Need to determine: learning rate, momentum parameters, exploration noise strategy
  - Question: How to handle angle wrapping in momentum calculations?
  - **Note**: May provide diminishing returns after 3.5x speedup achieved with progressive refinement

- **Q-Learning for Angle Selection**: Learn long-term value of angle choices
  - Need to determine: state representation, action discretization, reward shaping
  - Question: How many angle bins? How to handle continuous state space?
  - **Note**: Consider if complexity is justified vs current ultra-fast mode performance

- **Variable Stroke Length Optimization**: Joint optimization of angle and stroke length
  - Need to determine: parameter bounds, constraint handling, loss function impact
  - Question: How does this affect loss function convexity?
  - **Note**: Lower priority after memory and speed optimizations

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

### User Interface Improvements
**Context**: User feedback on current interface and functionality.

- **Option Consolidation**: Reduce and organize CLI options for better usability
  - Requirements: Review current options, identify redundancies, group related options
  - Implementation: Refactor argument parser, update documentation
  - Success criteria: Cleaner CLI interface with logical option grouping

- **Resume Mode Step Display**: Show cumulative step count during resume operations
  - Requirements: Display "Step X/Y (total: Z)" format where Z includes previous runs
  - Implementation: Update step display logic in drawing engine and visualizer
  - Success criteria: Clear indication of progress across resumed runs

- **OpenCV Window Cleanup**: Remove obsolete 'p' option from OpenCV visualization
  - Requirements: Remove 'p' key handler and update help text
  - Implementation: Update realtime_visualizer.py keyboard handling
  - Success criteria: Clean keyboard interface without deprecated options

- **Pause Functionality**: Add ability to pause/resume drawing process
  - Requirements: Pause key (spacebar), visual indication of pause state, resume capability
  - Implementation: Add pause state management to drawing loop and visualizer
  - Success criteria: Smooth pause/resume with clear user feedback

### Performance Optimization Enhancement
**Context**: Completed - Major performance optimizations for greedy calculation implemented.

- ✅ **Memory-Efficient Canvas Operations**: Eliminate memory explosion in vectorized operations
  - Requirements: Single canvas processing, progressive refinement, early termination
  - Implementation: `wanderline/memory_efficient_canvas.py` + `wanderline/fast_agent.py`
  - Success criteria: 3.5x speedup with 97%+ quality retention and 36x memory reduction
  - **COMPLETED 2025-06-26**: Full implementation with ultra-fast and fast-agent modes

- ✅ **Video Memory Optimization**: Coordinate-only logging for long runs
  - Requirements: Stroke data logging, post-calculation video reconstruction
  - Implementation: `wanderline/stroke_logger.py` + memory-efficient mode integration
  - Success criteria: Support for 10,000+ steps without crashes, 15,000x+ memory reduction
  - **COMPLETED 2025-06-26**: Full implementation with automatic video reconstruction

### Configuration Management Enhancement  
**Context**: Completed - config updater function implemented as alternative to inheritance system.

- ✅ **Config Update Function**: Batch update multiple config files simultaneously
  - Requirements: CLI tool, pattern matching, backup/restore, dry-run preview
  - Implementation: `wanderline/config_updater.py` + `scripts/update_configs.py`
  - Success criteria: Safe batch updates with rollback capability
  - **COMPLETED 2025-06-26**: Full implementation with CLI interface and safety features

### Test-Driven Development Implementation
**Context**: Current tests exist but need assessment and TDD approach design for project.

- ✅ **Test-Driven Development Implementation**: Complete TDD methodology for project
  - Requirements: Test effectiveness analysis, TDD examples, testing patterns, workflow integration
  - Implementation: Comprehensive development guidelines with TDD workflow and examples
  - Success criteria: Clear TDD methodology with working examples and complete documentation
  - **COMPLETED 2025-06-26**: Full TDD implementation merged into `docs/coding_rules.md` with:
    - RED-GREEN-REFACTOR cycle examples
    - 12/12 passing tests for config updater
    - Complete test organization and best practices
    - Development workflow integration

### Configuration System Enhancement
**Context**: Need better organization of optimizer settings.

- **Optimizer Configuration**: Add structured config for optimization strategies
  - Requirements: Optimizer type, learning parameters, temperature settings
  - Implementation: Create optimizer config section, update CLI parsing
  - Success criteria: Easy switching between optimization strategies

### Progress Monitoring & Visualization
**Context**: Need additional detailed feedback during drawing process for debugging and optimization analysis.

- **Verbose Angle Evaluation Mode**: Show angle candidates and rewards for each step
  - Requirements: Display all angle candidates, highlight selected angle with reasoning
  - Implementation: Add debug mode to agent, integrate with visualizer
  - Success criteria: Clear display of angle selection process without performance impact

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

### Interrupt Handling System (2025-06-26)
- **Signal Handlers**: Added SIGINT/SIGTERM handling to DrawingEngine for graceful shutdown on Ctrl+C
- **State Preservation**: Interruptions trigger immediate state saving using existing resume infrastructure  
- **User Feedback**: Clear messages indicating progress is being saved and how to resume
- **Pytest Integration**: Comprehensive test suite covering signal handling, graceful shutdown, and resume workflow
- **PYTHONPATH Resolution**: Added conftest.py and pytest.ini to automatically resolve import paths for all tests
- **Zero Performance Impact**: Signal handlers only activate during interruption, no overhead during normal operation
- **Resume Compatibility**: Interrupted runs generate identical state files to normal runs, full resume functionality

### Tensor Performance Analysis (2025-06-26)
- **Hypothesis Testing**: Confirmed that tensor calculations are NOT effective in current implementation
- **Performance Results**: Vectorized approach is 0.67x slower than sequential with 36x memory overhead
- **Root Cause**: OpenCV line drawing bottleneck - cv2.line() requires loops, eliminating vectorization benefits  
- **Memory Analysis**: 36 canvas copies (31.6 MB) vs sequential (0.9 MB) causes memory pressure
- **Validation**: Current fast/ultra-fast optimizations (3.5x speedup) outperform "vectorized" approach
- **Documentation**: Findings archived in git history and documented in CLAUDE.md

### Real-time Canvas Visualization (2025-06-26)
- **OpenCV Real-time Window**: Full implementation with live canvas updates and keyboard controls
- **Periodic Snapshot Saving**: Automatic and manual snapshot functionality with organized directory structure
- **ASCII Terminal Display**: Complete ASCII converter utility for terminal-only environments
- **Visualization Configuration**: Comprehensive JSON configuration support for all visualization options
- **Interactive Controls**: Keyboard shortcuts (q/ESC=quit, s=save, h=toggle overlay, p=print path)
- **Live Performance Metrics**: Real-time display of step count, timing, distance, reward, and speed
- **Integration**: Full integration with drawing loop and memory-efficient modes

### Speed Display Fix (2025-06-26)
- **Issue**: OpenCV window showed 0.0 steps/s due to incorrect calculation logic
- **Fix**: Updated realtime_visualizer.py to properly calculate steps/second by dividing elapsed time by actual number of steps processed
- **Result**: Accurate real-time speed display in OpenCV visualization window

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
