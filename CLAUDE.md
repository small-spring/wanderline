# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Wanderline is a single-stroke drawing agent that generates drawings based on motif images. The system uses various optimization strategies (greedy, multi-step lookahead, transformer-based) to select stroke angles that minimize the distance between the generated drawing and the target motif.

## Development Commands

### Testing and Building
- `uv run pytest` - Run all tests
- `uv run pytest tests/test_<module>.py` - Run specific test file
- `scripts/test_and_commit.sh "commit message"` - Run tests and commit if they pass

### Running the Application
- `uv run python run_test.py` - Basic run with default settings
- `uv run python run_test.py assets/sample.png` - Run with motif image
- `uv run python run_test.py --config configs/quick_test_2step.json` - Multi-step lookahead test
- `uv run python run_test.py --steps 500 --greedy --opacity 0.5` - CLI overrides
- `uv run python run_test.py --steps 10000 --memory-efficient` - Memory-efficient mode for long runs

### Quick Test Commands (replacing removed config files)
- `uv run python run_test.py --steps 100 --duration 5.0 --opacity 0.3 --reward-type l2` - L2 reward test
- `uv run python run_test.py --steps 100 --duration 5.0 --opacity 0.3 --reward-type l2_white_penalty --white-penalty 0.1` - White penalty test

### Performance Optimization Modes
- `uv run python run_test.py --steps 1000 --fast-agent --memory-efficient --opacity 0.3` - Fast mode (1.5x speedup)
- `uv run python run_test.py --steps 500 --ultra-fast --memory-efficient --opacity 0.3` - Ultra-fast mode (3.5x speedup, auto-disables visualization)
- `uv run python run_test.py --steps 500 --headless --memory-efficient` - Headless mode (no visualization for any configuration)

### Configuration Management
- `uv run python scripts/update_configs.py --set steps=1000` - Update all config files
- `uv run python scripts/update_configs.py --set opacity=0.5 --files default.json long_run.json` - Update specific files
- `uv run python scripts/update_configs.py --set duration=10.0 --pattern "quick_*.json"` - Update with pattern matching
- `uv run python scripts/update_configs.py --set steps=500 --dry-run` - Preview changes without applying
- `uv run python scripts/update_configs.py --show-diff` - Show differences from backups
- `uv run python scripts/update_configs.py --restore` - Restore all configs from backups

### Development Setup
- `uv init` - Initialize project
- `uv env create 3.12` - Create Python 3.12 environment
- `uv add <package>` - Add dependencies

## Architecture Overview

### Core Components

1. **Agent System (wanderline/agent.py)**
   - `choose_next_angle()` - Main entry point for angle selection
   - Supports 1-step greedy and multi-step lookahead (2+ steps)
   - Vectorized implementations for performance
   - Memory-optimized for long runs (10,000+ steps)

2. **Drawing Engine (wanderline/drawing_engine.py)**
   - Orchestrates the main drawing loop
   - Manages different agent types (greedy, transformer)
   - Handles reward computation and early stopping
   - Integrates with video recording and visualization

3. **Canvas System (wanderline/canvas.py)**
   - `apply_stroke()` - Core drawing function
   - `apply_stroke_vectorized()` - Batch processing for multi-step
   - Supports opacity and line width control

4. **Configuration (wanderline/config_manager.py)**
   - JSON-based configuration with CLI overrides
   - Manages complex parameter combinations
   - Validation and default value handling

5. **Reward Functions (wanderline/reward.py)**
   - L2 distance (standard)
   - L2 with white penalty (penalizes drawing on white areas)
   - Vectorized versions for performance

### Agent Types

- **Greedy Agent**: Single-step optimization using vectorized angle evaluation
- **Multi-step Agent**: 2+ step lookahead with memory optimization
- **Transformer Agent**: Neural network-based (experimental, not production-ready)

### Key Design Patterns

1. **Vectorization**: Critical for performance - most operations support batch processing
2. **Memory Management**: Coordinate-only recording for long runs to prevent crashes
3. **Configuration-driven**: Extensive use of JSON configs with CLI overrides
4. **Modular Architecture**: Clear separation between agents, canvas, rewards, and orchestration

## Performance Considerations

### Memory Optimization
- **Memory-Efficient Mode**: Use `--memory-efficient` for runs >1000 steps to prevent crashes
  - Saves stroke coordinates instead of video frames during computation (15,000x+ memory reduction)
  - Reconstructs video after calculation completes
  - Enabled by default in `configs/long_run.json`

### Speed Optimization  
- **Ultra-Fast Mode**: Use `--ultra-fast` for maximum speed (3.5x speedup)
  - Progressive refinement: starts with 8 cardinal angles, refines best regions
  - Memory-efficient canvas operations (36x less memory than standard vectorized)
  - Uses 16 samples max vs 36 for standard mode
  - Automatically disables real-time visualization for maximum performance
  - 97%+ quality retention compared to standard mode
- **Fast Agent**: Use `--fast-agent` for balanced optimization (1.5x speedup)
  - Adaptive sampling with early termination
  - Memory-efficient operations
  - Better quality than ultra-fast mode

### Sampling Strategy
- **Standard**: 36 samples for 1-step, 12-16 for 2-step lookahead
- **Fast modes**: Only work with 1-step lookahead (lookahead_depth=1)
- **Early Stopping**: Use patience and min_delta to prevent infinite runs

## Configuration Files

Located in `configs/`:
- `default.json` - Standard settings
- `quick_test_2step.json` - Multi-step lookahead test configuration
- `long_run.json` - Extended run settings with memory-efficient mode (10,000+ steps)

## Output Structure

Each run creates a timestamped directory in `outputs/` containing:
- `drawing.mp4` - Drawing process video
- `final_canvas.png` - Final result
- `motif.png` - Target image
- `distance_curve.png` - Convergence plot
- `run_summary.json` - Run metadata

## Development Guidelines

- Use `uv run` for all Python execution
- Follow the test-and-commit workflow via `scripts/test_and_commit.sh`
- Keep functions under 150 lines; modularize larger code
- Use vectorized operations for performance-critical paths
- English-only comments and documentation
- Clear, descriptive variable names