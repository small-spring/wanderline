# Configuration Guide

This document explains how to configure Wanderline using configuration files and command-line options.

## Configuration File

Wanderline can be configured using a JSON configuration file. By default, it looks for `configs/default.json` in the current directory, but you can specify a different file with the `--config` option.

### Creating a Configuration File

1. Copy an existing configuration file:
   ```bash
   cp configs/default.json configs/my_config.json
   ```

2. Edit `configs/my_config.json` to suit your needs.

### Available Configuration Files

The `configs/` directory contains several pre-configured files:

#### Available Configuration Files
- **`default.json`**: Default configuration used when no `--config` is specified
- **`quick_test_2step.json`**: Multi-step lookahead test configuration (2-step greedy)
- **`long_run.json`**: Long-duration run configuration (10,000 steps) with memory-efficient mode

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
  - Ultra-fast mode: configurable (default 16, can be increased for better quality)
  - Default: 36 for 1-step, 12 for 2+ step lookahead

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

#### Performance Optimization

- **`memory_efficient`** (boolean): Enable memory-efficient mode for long runs
  - Saves stroke coordinates instead of video frames during computation
  - Reconstructs video after calculation completes
  - Essential for runs >1000 steps to prevent crashes
  - Example: `true`
  - Default: `false`

- **`fast_agent`** (boolean): Enable optimized fast greedy agent
  - Provides 1.5x speedup with adaptive sampling and early termination
  - Only works with 1-step lookahead (lookahead_depth=1)
  - Better quality than ultra_fast mode
  - Example: `true`
  - Default: `false`

- **`ultra_fast`** (boolean): Enable ultra-fast mode for maximum speed
  - Provides 3.5x speedup with progressive refinement
  - Sample count is configurable via `n_samples` (default 16 for ultra-fast)
  - 97%+ quality retention compared to standard mode
  - Only works with 1-step lookahead (lookahead_depth=1)
  - Example: `true`
  - Default: `false`

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

# Use performance optimization modes
uv run python run_test.py --ultra-fast --greedy --memory-efficient
uv run python run_test.py --fast-agent --greedy

# Configure ultra-fast mode sample count for speed/quality tradeoff
uv run python run_test.py --ultra-fast --n-samples 16 --greedy  # Maximum speed (2.2x)
uv run python run_test.py --ultra-fast --n-samples 36 --greedy  # Same speed as standard but memory efficient

# Use pre-configured performance setups
uv run python run_test.py --config configs/ultra_fast_run.json
uv run python run_test.py --config configs/long_run_memory_efficient.json

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

7. **Performance optimization modes**:
   - Use `--ultra-fast` for maximum speed (2-3.5x speedup) with 97%+ quality retention
   - Use `--fast-agent` for balanced speed and quality (1.5x speedup)
   - Always combine with `--memory-efficient` for runs >1000 steps
   - Fast modes only work with 1-step lookahead (lookahead_depth=1)
   - Configure `n_samples` to control ultra-fast speed/quality tradeoff

8. **Memory considerations**: 
   - Always use `--memory-efficient` for runs >1000 steps to prevent crashes
   - Standard vectorized implementation can use 16MB+ per angle selection
   - Memory-efficient mode reduces usage by 36x (36 canvas copies → 1 at a time)
