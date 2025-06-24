# Wanderline

## Documentation

This folder contains the following documentation files:

- [Overview](README.md)
- [Coding Rules](coding_rules.md)
- [Specification](specification.md)
- [Detailed Design](detailed_design.md)

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

Run the agent script:
```zsh
uv run python run_test.py
```

Run all tests:
```zsh
uv run pytest
```

## Project Structure

- wanderline/
  - canvas.py: Canvas representation and `apply_stroke` function
  - image_utils.py: Image loading and preprocessing
  - reward.py: Reward calculation based on L2 distance
- tests/: Unit tests for canvas and reward modules
- scripts/: Automation scripts (e.g., `test_and_commit.sh`)
- docs/: Project specifications and design documents


