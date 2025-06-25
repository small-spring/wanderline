# Configuration Guide

This document explains how to configure Wanderline using configuration files and command-line options.

## Configuration File

Wanderline can be configured using a JSON configuration file. By default, it looks for `configs/default.json` in the current directory, but you can specify a different file with the `--config` option.

### Creating a Configuration File

1. Copy an existing configuration file:
   ```bash
   cp configs/quick_test_l2.json configs/my_config.json
   ```

2. Edit `configs/my_config.json` to suit your needs.

### Available Configuration Files

The `configs/` directory contains several pre-configured files:

- **`default.json`**: Default configuration used when no `--config` is specified
- **`quick_test_l2.json`**: Quick test with standard L2 distance (100 steps)
- **`quick_test_white_penalty.json`**: Quick test with white penalty (100 steps)  
- **`long_run.json`**: Long-duration run configuration (5000 steps)

### Configuration Options

#### Basic Drawing Parameters

- **`motif_path`** (string): Path to the target motif image file
  - Example: `"assets/sample.png"`
  - Default: `null` (blank canvas)

- **`ratio`** (float): Length of each stroke as a ratio of the canvas's smaller dimension
  - Range: 0.0 to 1.0
  - Example: `0.1`
  - Default: `0.2`

- **`steps`** (integer): Total number of strokes to draw
  - Example: `500`
  - Default: `100`

- **`duration`** (float): Desired duration of the output video in seconds
  - Example: `15.0`
  - Default: `15.0`

- **`opacity`** (float): Opacity of each stroke
  - Range: 0.0 (transparent) to 1.0 (opaque)
  - Example: `0.1`
  - Default: `1.0`

- **`line_width`** (integer): Stroke thickness
  - Example: `1`
  - Default: `3`

#### Agent Behavior

- **`greedy`** (boolean): Use greedy algorithm for angle selection
  - `true`: Choose angles that minimize immediate distance to motif
  - `false`: Choose angles randomly
  - Default: `false`

- **`lookahead_depth`** (integer): Number of steps ahead to search when using greedy mode
  - Range: 1 to 3 (higher values exponentially increase computation)
  - `1`: Greedy (immediate reward only)
  - `2`: 2-step lookahead (recommended)
  - `3+`: Not recommended (very slow)
  - Default: `1`

- **`n_samples`** (integer): Number of angle candidates to consider per step
  - For 1-step lookahead: up to 36 (recommended: 24-36)
  - For 2-step lookahead: up to 16 (recommended: 12-16)
  - For 3+ steps: up to 8 (not recommended)
  - Default: Auto-selected based on lookahead_depth

#### Early Stopping

- **`patience`** (integer): Number of steps with no improvement before stopping
  - Set to `0` to disable early stopping
  - Example: `20`
  - Default: `10`

- **`min_delta`** (float): Minimum change in distance to be considered improvement
  - Example: `0.0001`
  - Default: `1e-4`

#### Reward/Loss Function

- **`reward_type`** (string): Type of reward/loss function to use
  - Available options:
    - `"l2"`: Standard L2 distance (mean squared error)
    - `"l2_white_penalty"`: L2 distance with penalty for white pixels
  - Example: `"l2"`
  - Default: `"l2"`

##### White Penalty Settings

When using `"l2_white_penalty"`, you must specify the `white_penalty` parameter.

- **`white_penalty`** (float): Penalty strength (scale-invariant mode)
  - Formula: `loss = l2_distance * (1 + white_penalty * white_ratio)`
  - Penalty scales with L2 distance, making it scale-invariant
  - Recommended range: 0.05-0.5
  - Example: `0.1` (10% penalty when canvas is fully white)

##### White Penalty Example

```json
{
  "reward_type": "l2_white_penalty",
  "white_penalty": 0.1
}
```

#### Run Management

- **`resume_from`** (string): Path to a previous output directory to resume from
  - Example: `"outputs/20231027_123456"`
  - Default: `null` (new run)

## Command-Line Options

All configuration options can also be specified via command-line arguments, which will override the configuration file values.

### Examples

```bash
# Use configuration file
uv run python run_test.py

# Override specific options
uv run python run_test.py assets/sample.png --steps 1000 --greedy

# Use 2-step lookahead with optimized settings
uv run python run_test.py --greedy --lookahead_depth 2 --n_samples 12

# Use white penalty reward function
uv run python run_test.py --reward-type l2_white_penalty --white-penalty 0.1

# Use a specific config file
uv run python run_test.py --config configs/long_run.json

# Resume from previous run
uv run python run_test.py --resume_from outputs/20231027_123456
```

### Getting Help

For a complete list of command-line options, run:
```bash
uv run python run_test.py --help
```

## Tips

1. **Start with an existing configuration**: Copy one of the provided configs (e.g., `configs/quick_test_l2.json`) and modify as needed.

2. **Experiment with different reward functions**: The `l2_white_penalty` function can help avoid drawing on white areas.

3. **Use early stopping**: Set appropriate `patience` and `min_delta` values to avoid overfitting.

4. **Adjust opacity for layered effects**: Lower opacity values (e.g., 0.1) can create interesting layered drawing effects.

5. **Use greedy mode for better results**: The `--greedy` flag often produces better results than random angle selection.

6. **Optimize multi-step lookahead**: 
   - For fast results: Use `lookahead_depth=1` with `n_samples=36`
   - For better quality: Use `lookahead_depth=2` with `n_samples=12-16`
   - Avoid `lookahead_depth>=3` unless necessary (very slow)

7. **Memory considerations**: The vectorized implementation can use significant memory with large n_samples and lookahead_depth. Monitor memory usage during long runs.
