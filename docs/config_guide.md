# Configuration Guide

This document explains how to configure Wanderline using configuration files and command-line options.

## Configuration File

Wanderline can be configured using a JSON configuration file. By default, it looks for `config.json` in the current directory, but you can specify a different file with the `--config` option.

### Creating a Configuration File

1. Copy the sample configuration file:
   ```bash
   cp config.sample.json config.json
   ```

2. Edit `config.json` to suit your needs.

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
    - `"l2_white_penalty"`: L2 distance with penalty for drawing on white areas
  - Example: `"l2"`
  - Default: `"l2"`

- **`white_penalty_alpha`** (float): Alpha value for white penalty
  - Required only when `reward_type` is `"l2_white_penalty"`
  - Controls the strength of the white penalty
  - Example: `0.1`
  - Default: `null`

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

# Use white penalty reward function
uv run python run_test.py --reward-type l2_white_penalty --white-penalty-alpha 0.1

# Resume from previous run
uv run python run_test.py --resume_from outputs/20231027_123456
```

### Getting Help

For a complete list of command-line options, run:
```bash
uv run python run_test.py --help
```

## Tips

1. **Start with the sample configuration**: Copy `config.sample.json` to `config.json` and modify as needed.

2. **Experiment with different reward functions**: The `l2_white_penalty` function can help avoid drawing on white areas.

3. **Use early stopping**: Set appropriate `patience` and `min_delta` values to avoid overfitting.

4. **Adjust opacity for layered effects**: Lower opacity values (e.g., 0.1) can create interesting layered drawing effects.

5. **Use greedy mode for better results**: The `--greedy` flag often produces better results than random angle selection.
