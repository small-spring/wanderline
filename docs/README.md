# Wanderline

Wanderline is an agent that draws images in a single-stroke style based on a given motif image. It outputs drawing angles for each stroke, which can later be used to control devices like robotic arms.

## Documentation

### For Users
- **[Configuration Guide](config_guide.md)** - How to configure Wanderline with config files and CLI options

### For Developers
- **[Coding Rules](coding_rules.md)** - Development guidelines and code standards
- **[TODO](TODO.md)** - Task tracking and development roadmap

## Project Structure

```
wanderline/
├── configs/          # Configuration files
│   ├── default.json              # Default configuration
│   ├── quick_test_2step.json     # Multi-step lookahead test
│   └── long_run.json             # Long-duration run configuration
├── debug/            # Debugging and analysis scripts (development only)
│   └── README.md           # Guidelines for debug scripts
├── docs/             # Documentation
├── wanderline/       # Main Python package
├── tests/            # Unit tests
├── assets/           # Sample images and resources
├── outputs/          # Generated outputs (timestamped folders)
└── scripts/          # Utility scripts
```

---

## Prerequisites

- Python 3.12
- uv (https://github.com/jaz303/uv)

## Setup

```zsh
uv init
uv env create 3.12
uv add numpy Pillow opencv-python pytest
```

## Configuration

Wanderline can be configured in two ways:

1. **Configuration File**: Use one of the pre-configured files or create your own
   ```zsh
   # Use default configuration
   uv run python run_test.py
   
   # Use specific configuration
   uv run python run_test.py --config configs/quick_test_2step.json
   ```

2. **Command-line Arguments**: Override any setting directly
   ```zsh
   uv run python run_test.py --steps 500 --greedy --opacity 0.3
   ```

For detailed configuration options, see the [Configuration Guide](config_guide.md).

## Development Notes

### Debug Scripts

The `debug/` folder is available for temporary debugging and analysis scripts during development. These scripts are not part of the main codebase and can be used for:

- Testing specific functionality
- Analyzing algorithm behavior
- Quick experiments and prototyping

See `debug/README.md` for guidelines on creating debug scripts.

## Usage

Run the main script to generate a drawing.

```zsh
uv run python run_test.py [MOTIF_PATH] [OPTIONS]
```

### Positional Arguments
- `MOTIF_PATH`: (Optional) Path to the target motif image file. If not provided, the agent will draw on a blank canvas.

### Options
- `--ratio <float>`: Length of each stroke as a ratio of the canvas's smaller dimension. Default: `0.2`.
- `--steps <int>`: Total number of strokes to draw. Default: `100`.
- `--duration <float>`: Desired duration of the output video in seconds. Default: `15.0`.
- `--greedy`: If set, use a greedy algorithm to choose the next stroke angle. Otherwise, angles are chosen randomly.
- `--agent-type <str>`: Type of agent to use. Options: `greedy` (default), `transformer`. The transformer agent uses neural networks for multi-step planning.
- `--opacity <float>`: Opacity of each stroke (from `0.0` to `1.0`). Default: `1.0`.
- `--patience <int>`: Early stopping patience. The number of steps with no significant improvement before stopping. Set to `0` to disable. Default: `10`.
- `--min-delta <float>`: The minimum change in distance to be considered an improvement for early stopping. Default: `1e-4`.
- `--resume_from <path>`: Path to a previous output directory (e.g., `outputs/20231027_123456`) to resume a run.
- `--line-width <int>`: Stroke thickness (line width) for drawing. Default: from config.json or 3.
- `--reward-type <str>`: Reward/loss function type to use. Options: `l2`, `l2_white_penalty`. Default: `l2`.
- `--white-penalty-alpha <float>`: Alpha value for white penalty (only used if `--reward-type` is `l2_white_penalty`).
- `--memory-efficient`: Enable memory-efficient mode for long runs. Saves stroke coordinates instead of video frames during computation.
- `--fast-agent`: Enable optimized fast greedy agent for 1.5x speedup. Uses adaptive sampling and early termination.
- `--ultra-fast`: Enable ultra-fast mode for maximum speed (2-3.5x speedup). Uses progressive refinement with configurable sample count.
- `--n-samples <int>`: Number of angle candidates to consider per step. Configures speed/quality tradeoff for ultra-fast mode.

### Example Commands

1.  **Basic run with a motif:**
    ```zsh
    uv run python run_test.py assets/sample.png
    ```

2.  **A longer, more detailed drawing using the greedy agent:**
    ```zsh
    uv run python run_test.py assets/sample.png --steps 500 --greedy --opacity 0.5 --patience 20
    ```

3.  **Run with white penalty reward function:**
    ```zsh
    uv run python run_test.py assets/sample.png --reward-type l2_white_penalty --white-penalty-alpha 0.1
    ```

4.  **Resume a previous run and add 500 more steps:**
    ```zsh
    uv run python run_test.py --resume_from outputs/20231105_103000 --steps 500
    ```

5.  **Basic run with a motif and custom line width:**
    ```zsh
    uv run python run_test.py assets/sample.png --line-width 5
    ```

6.  **Using Transformer agent for multi-step planning:**
    ```zsh
    uv run python run_test.py assets/sample.png --agent-type transformer --steps 100 --opacity 0.3
    ```

7.  **Performance optimization modes:**
    ```zsh
    # Ultra-fast mode for maximum speed (2.2x speedup with 16 samples)
    uv run python run_test.py assets/sample.png --ultra-fast --n-samples 16 --greedy --memory-efficient
    
    # Ultra-fast mode with standard quality (same speed as standard, memory efficient)
    uv run python run_test.py assets/sample.png --ultra-fast --n-samples 36 --greedy --memory-efficient
    
    # Fast agent for balanced speed and quality (1.5x speedup)  
    uv run python run_test.py assets/sample.png --fast-agent --greedy
    
    # Pre-configured ultra-fast setup
    uv run python run_test.py --config configs/ultra_fast_run.json
    ```

8.  **Long runs with memory optimization:**
    ```zsh
    # Memory-efficient mode for 10,000+ steps
    uv run python run_test.py assets/sample.png --steps 10000 --memory-efficient --greedy
    
    # Pre-configured long run setup
    uv run python run_test.py --config configs/long_run_memory_efficient.json
    ```
## Run all tests
```zsh
uv run pytest
```

## Automated Test & Commit

You can use the following script to automatically run all tests and commit your changes if they pass:

```zsh
scripts/test_and_commit.sh "your commit message here"
```

- Replace `"your commit message here"` with a short description of your changes.
- If tests fail, no commit will be made.

## Project Structure

- `wanderline/`: Core Python source code.
  - `agent.py`: Contains the logic for choosing drawing angles.
  - `canvas.py`: Manages the drawing canvas and the `apply_stroke` function.
  - `config_manager.py`: Handles configuration loading and argument parsing.
  - `run_manager.py`: Manages run setup, resumption, and output handling.
  - `drawing_engine.py`: Handles the main drawing loop and reward computation.
  - `image_utils.py`: Utilities for loading and handling images.
  - `reward.py`: Calculates the reward (distance) between the canvas and the motif.
  - `video_recorder.py`: Handles recording the drawing process to a video file.
  - `plot_utils.py`: Utility for plotting the distance curve.
- `tests/`: Unit tests for the core modules.
- `scripts/`: Automation scripts (e.g., `test_and_commit.sh`).
- `docs/`: Project specifications, design documents, and this README.
- `outputs/`: Default directory for all generated outputs (images, videos, summaries). Each run is saved in a timestamped subfolder.
- `assets/`: Sample images for testing.

