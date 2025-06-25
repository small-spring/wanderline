# Wanderline

## Documentation

This folder contains the following documentation files:

for coding agent
- [TODO](TODO.md)
- [Coding Rules](coding_rules.md) 
    - First, please review and restate the coding rules!

for people
- [Overview](README.md)
- [Specification](specification.md)
- [Configuration Guide](config_guide.md) - How to configure Wanderline with config files and CLI options




---

Wanderline is an agent that draws images in a one-stroke style based on a given motif image. It outputs drawing angles for each stroke, which can later be used to control devices like robotic arms.

## Prerequisites

- Python 3.12
- uv (https://github.com/jaz303/uv)

## Setup

```zsh
uv init
uv env create 3.12
uv add numpy Pillow opencv-python pytest
```

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
- `--opacity <float>`: Opacity of each stroke (from `0.0` to `1.0`). Default: `1.0`.
- `--patience <int>`: Early stopping patience. The number of steps with no significant improvement before stopping. Set to `0` to disable. Default: `10`.
- `--min-delta <float>`: The minimum change in distance to be considered an improvement for early stopping. Default: `1e-4`.
- `--resume_from <path>`: Path to a previous output directory (e.g., `outputs/20231027_123456`) to resume a run.
- `--line-width <int>`: Stroke thickness (line width) for drawing. Default: from config.json or 3.
- `--reward-type <str>`: Reward/loss function type to use. Options: `l2`, `l2_white_penalty`. Default: `l2`.
- `--white-penalty-alpha <float>`: Alpha value for white penalty (only used if `--reward-type` is `l2_white_penalty`).

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

